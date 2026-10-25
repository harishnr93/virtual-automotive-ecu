#pragma once

// Cruise control application logic: state machine + speed controller.
//
// Independent of CAN, threads and clocks: the ECU calls step() every 10 ms with validated
// inputs and dt, and transmits the outputs. Everything here is deterministic and unit-tested.
//
//                main switch on                  set (rising edge), 30..180 km/h,
//        ┌─────┐ ───────────────► ┌─────────┐    no brake          ┌────────┐
//        │ Off │                  │ Standby │ ───────────────────► │ Active │◄──┐ pedal released
//        └─────┘ ◄─────────────── └─────────┘ ◄─────────────────── └────────┘   │
//              main switch off         ▲    brake / cancel (override)  │ pedal pressed
//                                      │                            ┌──▼───────┐
//                     inputs healthy   │                            │ Override │
//                     for 500 ms       │                            └──────────┘
//                                 ┌────┴──┐   any input fault (timeout, invalid speed, stale
//                                 │ Fault │◄─ data, controller fault, function inhibited)
//                                 └───────┘
//
// Safety-oriented rules (educational, inspired by ISO 26262 practice — not a certified design):
//  - Throttle is commanded only in Active; every other state outputs 0 %.
//  - Fault recovery never returns directly to Active: the driver must engage again.
//  - Brake or cancel always disengage, in Active and in Override.

#include <cstdint>

#include "control/pid_controller.hpp"

namespace vecu::control {

/// Numeric values are identical to the DBC signal CruiseState (checked by static_assert in the ECU).
enum class CruiseState : std::uint8_t { Off = 0U, Standby = 1U, Active = 2U, Override = 3U, Fault = 4U };

/// Numeric values are identical to the DBC signal CruiseFaultReason.
enum class CruiseFault : std::uint8_t {
  None = 0U,
  SpeedSignalTimeout = 1U,
  DriverInputTimeout = 2U,
  SpeedInvalid = 3U,
  InputsStale = 4U,
  ControllerFault = 5U,
  FunctionInhibited = 6U  ///< blocked by the fault manager (qualified fault not yet recovered)
};

[[nodiscard]] const char* toString(CruiseState state) noexcept;
[[nodiscard]] const char* toString(CruiseFault fault) noexcept;

struct CruiseConfig {
  double min_engage_speed_kph{30.0};
  double max_set_speed_kph{180.0};
  double override_pedal_threshold_pct{5.0};  ///< accelerator above this -> Override
  double fault_recovery_time_s{0.5};         ///< inputs must be healthy this long to leave Fault
  double setpoint_ramp_kph_per_s{3.0};       ///< comfort: max. rate of the internal setpoint
  double vehicle_mass_kg{1500.0};            ///< nominal mass for acceleration feedforward
  double inertia_factor{1.05};               ///< rotating-mass factor
  double max_drive_force_n{4000.0};          ///< force at 100 % throttle (feedforward scaling)
  PidConfig pid{10.0, 1.5, 0.0, 0.0, 60.0, 0.0, 0.0, AntiWindup::Clamping, 0.0};
};

[[nodiscard]] bool isValid(const CruiseConfig& config) noexcept;

/// Validated inputs, prepared by the communication layer.
struct CruiseInputs {
  double vehicle_speed_kph{0.0};
  bool speed_valid{false};             ///< signal present, in range, validity bit set
  bool speed_timeout{false};           ///< VehicleDynamics message timed out
  bool driver_inputs_timeout{false};   ///< DriverInputs message timed out
  bool inputs_stale{false};            ///< COM layer stopped updating the inputs
  bool function_inhibited{false};      ///< fault manager / watchdog block the function
  bool main_switch{false};
  bool set_request{false};             ///< level; the controller detects the rising edge
  bool cancel_request{false};
  bool brake_pressed{false};
  double accelerator_pct{0.0};
  double driver_set_speed_kph{0.0};    ///< desired speed chosen by the driver
};

struct CruiseOutputs {
  CruiseState state{CruiseState::Off};
  CruiseFault fault{CruiseFault::None};
  double throttle_pct{0.0};
  double target_speed_kph{0.0};   ///< driver's desired speed (0 if not engaged)
  double setpoint_kph{0.0};       ///< ramped setpoint actually tracked by the PID
  PidTerms pid{};
};

class CruiseController {
 public:
  explicit CruiseController(const CruiseConfig& config = CruiseConfig{}) noexcept;

  /// One 10 ms control step.
  [[nodiscard]] CruiseOutputs step(const CruiseInputs& inputs, double dt_s) noexcept;

  [[nodiscard]] CruiseState state() const noexcept { return state_; }
  [[nodiscard]] bool configValid() const noexcept { return config_valid_; }

 private:
  [[nodiscard]] CruiseFault detectFault(const CruiseInputs& inputs) const noexcept;
  [[nodiscard]] bool canEngage(const CruiseInputs& inputs) const noexcept;
  [[nodiscard]] double clampTarget(double requested_kph) const noexcept;
  void engage(const CruiseInputs& inputs) noexcept;
  [[nodiscard]] double runSpeedControl(const CruiseInputs& inputs, double dt_s) noexcept;

  CruiseConfig config_;
  bool config_valid_;
  PidController pid_;
  CruiseState state_{CruiseState::Off};
  CruiseFault fault_{CruiseFault::None};
  double target_kph_{0.0};
  double setpoint_kph_{0.0};
  double healthy_time_s_{0.0};
  bool previous_set_request_{false};
  bool controller_fault_{false};
};

}  // namespace vecu::control
