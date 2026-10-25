#include "control/cruise_controller.hpp"

#include <algorithm>
#include <cmath>

namespace vecu::control {

namespace {

constexpr double kKphPerMps = 3.6;

}  // namespace

const char* toString(CruiseState state) noexcept {
  switch (state) {
    case CruiseState::Off:
      return "Off";
    case CruiseState::Standby:
      return "Standby";
    case CruiseState::Active:
      return "Active";
    case CruiseState::Override:
      return "Override";
    case CruiseState::Fault:
      return "Fault";
  }
  return "Unknown";
}

const char* toString(CruiseFault fault) noexcept {
  switch (fault) {
    case CruiseFault::None:
      return "None";
    case CruiseFault::SpeedSignalTimeout:
      return "SpeedSignalTimeout";
    case CruiseFault::DriverInputTimeout:
      return "DriverInputTimeout";
    case CruiseFault::SpeedInvalid:
      return "SpeedInvalid";
    case CruiseFault::InputsStale:
      return "InputsStale";
    case CruiseFault::ControllerFault:
      return "ControllerFault";
    case CruiseFault::FunctionInhibited:
      return "FunctionInhibited";
  }
  return "Unknown";
}

bool isValid(const CruiseConfig& config) noexcept {
  return isValid(config.pid) && (config.min_engage_speed_kph > 0.0) &&
         (config.max_set_speed_kph > config.min_engage_speed_kph) && (config.override_pedal_threshold_pct >= 0.0) &&
         (config.fault_recovery_time_s >= 0.0) && (config.setpoint_ramp_kph_per_s > 0.0) &&
         (config.vehicle_mass_kg > 0.0) && (config.inertia_factor >= 1.0) && (config.max_drive_force_n > 0.0);
}

CruiseController::CruiseController(const CruiseConfig& config) noexcept
    : config_(config), config_valid_(isValid(config)), pid_(config.pid) {}

CruiseFault CruiseController::detectFault(const CruiseInputs& inputs) const noexcept {
  if (!config_valid_ || controller_fault_) {
    return CruiseFault::ControllerFault;
  }
  if (inputs.inputs_stale) {
    return CruiseFault::InputsStale;
  }
  if (inputs.driver_inputs_timeout) {
    return CruiseFault::DriverInputTimeout;
  }
  if (inputs.speed_timeout) {
    return CruiseFault::SpeedSignalTimeout;
  }
  if (!inputs.speed_valid || !std::isfinite(inputs.vehicle_speed_kph)) {
    return CruiseFault::SpeedInvalid;
  }
  if (inputs.function_inhibited) {
    return CruiseFault::FunctionInhibited;  // most specific reasons are reported first
  }
  return CruiseFault::None;
}

bool CruiseController::canEngage(const CruiseInputs& inputs) const noexcept {
  return !inputs.brake_pressed && !inputs.cancel_request &&
         (inputs.vehicle_speed_kph >= config_.min_engage_speed_kph) &&
         (inputs.vehicle_speed_kph <= config_.max_set_speed_kph);
}

double CruiseController::clampTarget(double requested_kph) const noexcept {
  return std::clamp(requested_kph, config_.min_engage_speed_kph, config_.max_set_speed_kph);
}

void CruiseController::engage(const CruiseInputs& inputs) noexcept {
  // Desired speed: the driver's request if plausible, otherwise the current speed.
  const bool request_valid = std::isfinite(inputs.driver_set_speed_kph) &&
                             (inputs.driver_set_speed_kph >= config_.min_engage_speed_kph);
  target_kph_ = clampTarget(request_valid ? inputs.driver_set_speed_kph : inputs.vehicle_speed_kph);
  // The ramped setpoint starts at the current speed: no step in the control error.
  setpoint_kph_ = inputs.vehicle_speed_kph;
  // Bumpless transfer: start from the driver's current pedal position.
  pid_.reset(inputs.accelerator_pct);
}

double CruiseController::runSpeedControl(const CruiseInputs& inputs, double dt_s) noexcept {
  // Driver may change the desired speed while engaged.
  if (std::isfinite(inputs.driver_set_speed_kph) && (inputs.driver_set_speed_kph >= config_.min_engage_speed_kph)) {
    target_kph_ = clampTarget(inputs.driver_set_speed_kph);
  }

  // Ramp the setpoint towards the target (comfort limit on acceleration).
  const double max_step = config_.setpoint_ramp_kph_per_s * dt_s;
  const double previous_setpoint = setpoint_kph_;
  setpoint_kph_ += std::clamp(target_kph_ - setpoint_kph_, -max_step, max_step);
  const double setpoint_rate_kph_s = (setpoint_kph_ - previous_setpoint) / dt_s;

  // Feedforward: throttle needed to produce the setpoint acceleration (road load is left to
  // the integrator, which also absorbs mass and grade errors).
  const double accel_mps2 = setpoint_rate_kph_s / kKphPerMps;
  const double feedforward_pct =
      100.0 * (config_.vehicle_mass_kg * config_.inertia_factor * accel_mps2) / config_.max_drive_force_n;

  const double throttle = pid_.update(setpoint_kph_, inputs.vehicle_speed_kph, dt_s, feedforward_pct);
  if (pid_.faulted()) {
    controller_fault_ = true;
    return 0.0;
  }
  return throttle;
}

CruiseOutputs CruiseController::step(const CruiseInputs& inputs, double dt_s) noexcept {
  if (!std::isfinite(dt_s) || (dt_s <= 0.0)) {
    controller_fault_ = true;
    dt_s = 0.0;
  }
  const bool set_edge = inputs.set_request && !previous_set_request_;
  previous_set_request_ = inputs.set_request;

  const CruiseFault detected = detectFault(inputs);
  // Driver-input faults matter in every state (driver intent unknown). Speed faults matter
  // only while the function is switched on.
  const bool relevant_fault =
      (detected != CruiseFault::None) &&
      ((state_ != CruiseState::Off) || (detected == CruiseFault::DriverInputTimeout) ||
       (detected == CruiseFault::InputsStale) || (detected == CruiseFault::ControllerFault) || inputs.main_switch);
  // Note: FunctionInhibited while Off and switched off is not relevant (nothing to block).

  double throttle = 0.0;
  if (relevant_fault) {
    if (state_ != CruiseState::Fault) {
      fault_ = detected;
    }
    state_ = CruiseState::Fault;
    healthy_time_s_ = 0.0;
  } else {
    switch (state_) {
      case CruiseState::Off:
        if (inputs.main_switch) {
          state_ = CruiseState::Standby;
        }
        break;
      case CruiseState::Standby:
        if (!inputs.main_switch) {
          state_ = CruiseState::Off;
        } else if (set_edge && canEngage(inputs)) {
          engage(inputs);
          state_ = CruiseState::Active;
          throttle = runSpeedControl(inputs, dt_s);  // control from the very first step (bumpless)
        } else {
          // stay
        }
        break;
      case CruiseState::Active:
        if (!inputs.main_switch) {
          state_ = CruiseState::Off;
        } else if (inputs.brake_pressed || inputs.cancel_request) {
          state_ = CruiseState::Standby;  // brake override disengages
        } else if (inputs.accelerator_pct > config_.override_pedal_threshold_pct) {
          state_ = CruiseState::Override;  // driver accelerates above the cruise command
        } else {
          throttle = runSpeedControl(inputs, dt_s);
        }
        break;
      case CruiseState::Override:
        if (!inputs.main_switch) {
          state_ = CruiseState::Off;
        } else if (inputs.brake_pressed || inputs.cancel_request) {
          state_ = CruiseState::Standby;
        } else if (inputs.accelerator_pct <= config_.override_pedal_threshold_pct) {
          // Resume from the current speed. The integrator was held during the override and still
          // contains the road-load estimate, so restart from it (the released pedal reads ~0 %).
          setpoint_kph_ = inputs.vehicle_speed_kph;
          pid_.reset(pid_.lastTerms().integral);
          state_ = CruiseState::Active;
          throttle = runSpeedControl(inputs, dt_s);
        } else {
          // driver keeps control
        }
        break;
      case CruiseState::Fault:
        healthy_time_s_ += dt_s;
        if (healthy_time_s_ >= config_.fault_recovery_time_s) {
          // Never resume control automatically: back to Standby (or Off).
          state_ = inputs.main_switch ? CruiseState::Standby : CruiseState::Off;
          fault_ = CruiseFault::None;
        }
        break;
    }
  }

  if (controller_fault_ && (state_ != CruiseState::Fault)) {
    fault_ = CruiseFault::ControllerFault;
    state_ = CruiseState::Fault;
    throttle = 0.0;
  }

  CruiseOutputs outputs;
  outputs.state = state_;
  outputs.fault = (state_ == CruiseState::Fault) ? fault_ : CruiseFault::None;
  const bool engaged = (state_ == CruiseState::Active) || (state_ == CruiseState::Override);
  outputs.throttle_pct = (state_ == CruiseState::Active) ? throttle : 0.0;
  outputs.target_speed_kph = engaged ? target_kph_ : 0.0;
  outputs.setpoint_kph = engaged ? setpoint_kph_ : 0.0;
  outputs.pid = pid_.lastTerms();
  return outputs;
}

}  // namespace vecu::control
