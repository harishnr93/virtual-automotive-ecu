#pragma once

// Discrete PID controller for embedded use.
//
//   u = Kp·e + I + Kd·D + feedforward            e = setpoint − measurement
//   I_k = I_{k−1} + Ki·e·dt                      (stored in output units)
//   D   = −d(measurement)/dt, low-pass filtered   ("derivative on measurement")
//
// Features: output saturation, optional output-rate limit, anti-windup (clamping or
// back-calculation), bumpless reset, protection against non-finite inputs.
// Pure computation: no allocation, no I/O, no clock — dt is passed in. This keeps it
// deterministic and testable, and independent of CAN and scheduling.

#include <cstdint>

namespace vecu::control {

enum class AntiWindup : std::uint8_t {
  None,             ///< integrator always integrates (for comparison/tests only)
  Clamping,         ///< conditional integration: freeze while the limited output and the error agree
  BackCalculation   ///< bleed the integrator by Kt·(u_limited − u_unlimited)
};

struct PidConfig {
  double kp{0.0};
  double ki{0.0};                       ///< per second
  double kd{0.0};                       ///< seconds
  double output_min{0.0};
  double output_max{0.0};
  double derivative_filter_tau_s{0.0};  ///< low-pass on the D term; 0 = unfiltered
  double max_output_rate{0.0};          ///< output units per second; 0 = unlimited
  AntiWindup anti_windup{AntiWindup::Clamping};
  double back_calculation_gain{0.0};    ///< Kt [1/s], used with BackCalculation
};

/// True if gains are finite and non-negative, limits are ordered, rates/time constants >= 0.
[[nodiscard]] bool isValid(const PidConfig& config) noexcept;

/// Individual terms of the most recent update (for logging, tuning and tests).
struct PidTerms {
  double proportional{0.0};
  double integral{0.0};
  double derivative{0.0};
  double feedforward{0.0};
  double unlimited{0.0};  ///< sum before saturation / rate limit
  double output{0.0};     ///< final output
  bool limited{false};    ///< output was saturated or rate-limited
};

class PidController {
 public:
  /// An invalid configuration puts the controller in the fault state (output_min forever).
  explicit PidController(const PidConfig& config) noexcept;

  /// One control step. Returns the limited output. Non-finite setpoint/measurement/feedforward
  /// or dt <= 0 set the fault flag and return output_min (safe value) without changing state.
  [[nodiscard]] double update(double setpoint, double measurement, double dt_s, double feedforward = 0.0) noexcept;

  /// Bumpless (re)initialisation: the next output starts from `output` (plus P/FF action),
  /// derivative history is cleared and the fault flag is reset.
  void reset(double output = 0.0) noexcept;

  [[nodiscard]] const PidTerms& lastTerms() const noexcept { return terms_; }
  [[nodiscard]] double lastOutput() const noexcept { return terms_.output; }
  [[nodiscard]] bool faulted() const noexcept { return fault_; }
  [[nodiscard]] const PidConfig& config() const noexcept { return config_; }

 private:
  [[nodiscard]] double limit(double value, double dt_s) const noexcept;

  PidConfig config_;
  bool config_valid_;
  double integral_{0.0};
  double previous_measurement_{0.0};
  bool has_previous_measurement_{false};
  double filtered_derivative_{0.0};
  PidTerms terms_{};
  bool fault_{false};
};

}  // namespace vecu::control
