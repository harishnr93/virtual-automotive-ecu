#pragma once

// Test-only copy of the longitudinal vehicle model of simulator/vecu_sim/vehicle_model.py,
// used for fast, deterministic closed-loop tests of the controller (no threads, no sleeping).
// tests/unit/python/test_vehicle_model.py checks that both implementations agree.

#include <algorithm>
#include <cmath>

namespace vecu::test {

struct PlantParams {
  double mass_kg{1500.0};
  double inertia_factor{1.05};
  double drag_coefficient{0.30};
  double frontal_area_m2{2.2};
  double air_density{1.225};
  double rolling_resistance{0.010};
  double max_drive_force_n{4000.0};
  double max_power_w{90000.0};
  double max_brake_force_n{9000.0};
  double drive_time_constant_s{0.4};
  double gravity{9.81};
};

class LongitudinalPlant {
 public:
  explicit LongitudinalPlant(PlantParams params = {}, double speed_kph = 0.0)
      : p_(params), speed_mps_(speed_kph / 3.6) {}

  /// Sets the drive force to steady state for the current speed (vehicle cruising).
  void settleDriveForce(double throttle_pct) { drive_force_n_ = availableForce() * throttle_pct / 100.0; }

  void step(double throttle_pct, double brake_pct, double grade_pct, double dt) {
    const double throttle = std::clamp(throttle_pct, 0.0, 100.0) / 100.0;
    const double brake = std::clamp(brake_pct, 0.0, 100.0) / 100.0;
    drive_force_n_ += (throttle * availableForce() - drive_force_n_) * dt / p_.drive_time_constant_s;
    const double theta = std::atan(grade_pct / 100.0);
    const double aero = 0.5 * p_.air_density * p_.drag_coefficient * p_.frontal_area_m2 * speed_mps_ * speed_mps_;
    const double rolling = (speed_mps_ > 0.0) ? p_.mass_kg * p_.gravity * p_.rolling_resistance * std::cos(theta) : 0.0;
    const double grade = p_.mass_kg * p_.gravity * std::sin(theta);
    const double braking = (speed_mps_ > 0.0) ? brake * p_.max_brake_force_n : 0.0;
    const double net = drive_force_n_ - aero - rolling - grade - braking;
    speed_mps_ = std::max(0.0, speed_mps_ + net / (p_.mass_kg * p_.inertia_factor) * dt);
  }

  [[nodiscard]] double speedKph() const { return speed_mps_ * 3.6; }

  /// Throttle needed to hold `speed_kph` on flat road (road load / available force).
  [[nodiscard]] double roadLoadThrottlePct(double speed_kph) const {
    const double v = speed_kph / 3.6;
    const double load = 0.5 * p_.air_density * p_.drag_coefficient * p_.frontal_area_m2 * v * v +
                        p_.mass_kg * p_.gravity * p_.rolling_resistance;
    return 100.0 * load / std::min(p_.max_drive_force_n, p_.max_power_w / std::max(v, 1.0));
  }

 private:
  [[nodiscard]] double availableForce() const {
    return std::min(p_.max_drive_force_n, p_.max_power_w / std::max(speed_mps_, 1.0));
  }

  PlantParams p_;
  double speed_mps_;
  double drive_force_n_{0.0};
};

}  // namespace vecu::test
