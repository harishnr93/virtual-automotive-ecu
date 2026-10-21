#include "control/pid_controller.hpp"

#include <algorithm>
#include <cmath>

namespace vecu::control {

namespace {

bool finiteNonNegative(double value) noexcept { return std::isfinite(value) && (value >= 0.0); }

}  // namespace

bool isValid(const PidConfig& config) noexcept {
  return finiteNonNegative(config.kp) && finiteNonNegative(config.ki) && finiteNonNegative(config.kd) &&
         std::isfinite(config.output_min) && std::isfinite(config.output_max) &&
         (config.output_min < config.output_max) && finiteNonNegative(config.derivative_filter_tau_s) &&
         finiteNonNegative(config.max_output_rate) && finiteNonNegative(config.back_calculation_gain);
}

PidController::PidController(const PidConfig& config) noexcept
    : config_(config), config_valid_(isValid(config)), fault_(!config_valid_) {
  reset(0.0);  // clamped into the output range; no-op (stays faulted) for an invalid config
}

void PidController::reset(double output) noexcept {
  if (!config_valid_) {
    return;
  }
  const double initial = std::isfinite(output) ? std::clamp(output, config_.output_min, config_.output_max)
                                               : config_.output_min;
  integral_ = initial;
  has_previous_measurement_ = false;
  filtered_derivative_ = 0.0;
  terms_ = PidTerms{};
  terms_.integral = initial;
  terms_.unlimited = initial;
  terms_.output = initial;
  fault_ = false;
}

double PidController::limit(double value, double dt_s) const noexcept {
  double limited = std::clamp(value, config_.output_min, config_.output_max);
  if (config_.max_output_rate > 0.0) {
    const double max_step = config_.max_output_rate * dt_s;
    limited = std::clamp(limited, terms_.output - max_step, terms_.output + max_step);
  }
  return limited;
}

double PidController::update(double setpoint, double measurement, double dt_s, double feedforward) noexcept {
  if (!config_valid_ || !std::isfinite(setpoint) || !std::isfinite(measurement) || !std::isfinite(feedforward) ||
      !std::isfinite(dt_s) || (dt_s <= 0.0)) {
    fault_ = true;
    terms_.output = config_valid_ ? config_.output_min : 0.0;
    return terms_.output;
  }

  const double error = setpoint - measurement;
  const double proportional = config_.kp * error;

  // Derivative on measurement: a setpoint step does not cause a derivative "kick".
  if (has_previous_measurement_) {
    const double raw = -(measurement - previous_measurement_) / dt_s;
    if (config_.derivative_filter_tau_s > 0.0) {
      const double alpha = dt_s / (config_.derivative_filter_tau_s + dt_s);
      filtered_derivative_ += alpha * (raw - filtered_derivative_);
    } else {
      filtered_derivative_ = raw;
    }
  }
  previous_measurement_ = measurement;
  has_previous_measurement_ = true;
  const double derivative = config_.kd * filtered_derivative_;

  // Tentative integration, then anti-windup decides whether to keep it.
  double integral = integral_ + (config_.ki * error * dt_s);
  double unlimited = proportional + integral + derivative + feedforward;
  double output = limit(unlimited, dt_s);

  switch (config_.anti_windup) {
    case AntiWindup::Clamping: {
      // Freeze the integrator if the output is limited in the direction the error pushes.
      const bool pushing_up = (unlimited > output) && (error > 0.0);
      const bool pushing_down = (unlimited < output) && (error < 0.0);
      if (pushing_up || pushing_down) {
        integral = integral_;
        unlimited = proportional + integral + derivative + feedforward;
        output = limit(unlimited, dt_s);
      }
      break;
    }
    case AntiWindup::BackCalculation:
      integral += config_.back_calculation_gain * (output - unlimited) * dt_s;
      break;
    case AntiWindup::None:
      break;
  }

  integral_ = integral;
  terms_.proportional = proportional;
  terms_.integral = integral;
  terms_.derivative = derivative;
  terms_.feedforward = feedforward;
  terms_.unlimited = unlimited;
  terms_.output = output;
  terms_.limited = (output != unlimited);
  fault_ = false;
  return output;
}

}  // namespace vecu::control
