// Deterministic closed-loop validation of the cruise controller against the vehicle model.
//
// Runs 10 ms steps faster than real time. The acceptance criteria below are the controller's
// design targets; the measured values are printed so they can be recorded in the docs.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "control/cruise_controller.hpp"
#include "support/longitudinal_plant.hpp"

namespace vecu::control {
namespace {

constexpr double kDt = 0.01;

struct Sample {
  double t;
  double speed;
  double throttle;
  CruiseState state;
};

struct StepMetrics {
  double overshoot_kph;   ///< beyond the target in the direction of the step
  double settling_s;      ///< time until speed stays within ±0.5 km/h of the target
  double final_error_kph;
  double max_throttle;
};

StepMetrics analyse(const std::vector<Sample>& samples, double start_kph, double target_kph) {
  StepMetrics m{0.0, -1.0, 0.0, 0.0};
  const double direction = (target_kph >= start_kph) ? 1.0 : -1.0;
  for (const Sample& s : samples) {
    m.overshoot_kph = std::max(m.overshoot_kph, direction * (s.speed - target_kph));
    m.max_throttle = std::max(m.max_throttle, s.throttle);
  }
  for (std::size_t i = samples.size(); i-- > 0U;) {
    if (std::fabs(samples[i].speed - target_kph) > 0.5) {
      m.settling_s = (i + 1U < samples.size()) ? samples[i + 1U].t : -1.0;
      break;
    }
    m.settling_s = samples[i].t;
  }
  m.final_error_kph = std::fabs(samples.back().speed - target_kph);
  return m;
}

/// Vehicle cruising at `start_kph` (road-load throttle), cruise engaged with `target_kph`.
std::vector<Sample> simulate(double start_kph, double target_kph, double duration_s,
                             test::PlantParams plant_params = {}, double grade_from_s = -1.0,
                             double grade_pct = 0.0) {
  test::LongitudinalPlant plant(plant_params, start_kph);
  const double hold = plant.roadLoadThrottlePct(start_kph);
  plant.settleDriveForce(hold);

  CruiseController cruise;
  CruiseInputs in;
  in.main_switch = true;
  in.speed_valid = true;
  in.driver_set_speed_kph = target_kph;
  in.accelerator_pct = hold;  // driver holds the speed with the pedal while pressing SET

  std::vector<Sample> samples;
  CruiseOutputs out;
  for (int k = 0; k < static_cast<int>(duration_s / kDt); ++k) {
    const double t = k * kDt;
    in.vehicle_speed_kph = plant.speedKph();
    in.set_request = (k >= 1) && (k < 20);  // press SET for 190 ms
    if (k == 2) {
      in.accelerator_pct = 0.0;  // ... and releases the pedal once engaged
    }
    out = cruise.step(in, kDt);
    const double driver_throttle = in.accelerator_pct;
    const double throttle = std::max(out.throttle_pct, driver_throttle);
    const double grade = ((grade_from_s >= 0.0) && (t >= grade_from_s)) ? grade_pct : 0.0;
    plant.step(throttle, 0.0, grade, kDt);
    samples.push_back({t, plant.speedKph(), out.throttle_pct, out.state});
  }
  return samples;
}

void print(const char* name, const StepMetrics& m) {
  std::printf("[ closed-loop ] %-34s overshoot=%.2f km/h settling=%.2f s final_error=%.3f km/h max_throttle=%.1f %%\n",
              name, m.overshoot_kph, m.settling_s, m.final_error_kph, m.max_throttle);
}

TEST(CruiseClosedLoop, PlantMatchesPythonGoldenValues) {
  // Same values as tests/unit/python/test_vehicle_model.py: keeps the C++ test plant and
  // the Python simulator model in sync.
  test::LongitudinalPlant accelerating({}, 0.0);
  for (int k = 0; k < 1000; ++k) {
    accelerating.step(100.0, 0.0, 0.0, kDt);
  }
  EXPECT_NEAR(accelerating.speedKph(), 82.90919684950765, 1e-9);
  test::LongitudinalPlant coasting({}, 100.0);
  for (int k = 0; k < 1000; ++k) {
    coasting.step(0.0, 0.0, 0.0, kDt);
  }
  EXPECT_NEAR(coasting.speedKph(), 90.19733428509716, 1e-9);
}

TEST(CruiseClosedLoop, AccelerationStep60To80) {
  const auto samples = simulate(60.0, 80.0, 40.0);
  ASSERT_EQ(samples.back().state, CruiseState::Active);
  const StepMetrics m = analyse(samples, 60.0, 80.0);
  print("step 60 -> 80 km/h", m);
  EXPECT_LT(m.overshoot_kph, 1.0);
  EXPECT_GT(m.settling_s, 0.0);
  EXPECT_LT(m.settling_s, 12.0);
  EXPECT_LT(m.final_error_kph, 0.1);
  EXPECT_LE(m.max_throttle, 60.0 + 1e-9);  // saturation respected
}

TEST(CruiseClosedLoop, DecelerationStep100To80CoastsWithoutUndershoot) {
  const auto samples = simulate(100.0, 80.0, 60.0);
  const StepMetrics m = analyse(samples, 100.0, 80.0);
  print("step 100 -> 80 km/h (coasting)", m);
  EXPECT_LT(m.overshoot_kph, 0.5);  // undershoot below 80
  EXPECT_GT(m.settling_s, 0.0);
  EXPECT_LT(m.settling_s, 30.0);    // no brake actuator: limited by drag
  EXPECT_LT(m.final_error_kph, 0.1);
}

TEST(CruiseClosedLoop, RejectsUphillGradeDisturbance) {
  // Cruise at 90 km/h, 4 % uphill grade from t = 10 s.
  const auto samples = simulate(90.0, 90.0, 40.0, {}, 10.0, 4.0);
  double min_speed = 1e9;
  double recovered_at = -1.0;
  for (std::size_t i = 0U; i < samples.size(); ++i) {
    if (samples[i].t >= 10.0) {
      min_speed = std::min(min_speed, samples[i].speed);
    }
  }
  const auto after_dip = std::find_if(samples.begin(), samples.end(),
                                      [min_speed](const Sample& s) { return s.speed == min_speed; });
  for (auto it = after_dip; it != samples.end(); ++it) {
    if (std::all_of(it, samples.end(), [](const Sample& s) { return std::fabs(s.speed - 90.0) <= 0.5; })) {
      recovered_at = it->t - 10.0;
      break;
    }
  }
  std::printf("[ closed-loop ] 4%% grade at 90 km/h: max speed drop=%.2f km/h, back within 0.5 km/h after %.2f s\n",
              90.0 - min_speed, recovered_at);
  EXPECT_LT(90.0 - min_speed, 2.0);
  EXPECT_GT(recovered_at, 0.0);
  EXPECT_LT(recovered_at, 15.0);
  EXPECT_LT(std::fabs(samples.back().speed - 90.0), 0.1);  // integral action removes steady-state error
}

TEST(CruiseClosedLoop, RobustToTwentyPercentHeavierVehicle) {
  // The feedforward assumes 1500 kg; the integrator must absorb the model error.
  test::PlantParams heavy;
  heavy.mass_kg = 1800.0;
  const auto samples = simulate(60.0, 80.0, 50.0, heavy);
  const StepMetrics m = analyse(samples, 60.0, 80.0);
  print("step 60 -> 80 km/h, mass 1800 kg", m);
  EXPECT_LT(m.overshoot_kph, 2.0);
  EXPECT_LT(m.final_error_kph, 0.1);
}

TEST(CruiseClosedLoop, ThrottleCutToZeroWithinOneStepOnSpeedFault) {
  test::LongitudinalPlant plant({}, 80.0);
  CruiseController cruise;
  CruiseInputs in;
  in.main_switch = true;
  in.speed_valid = true;
  in.driver_set_speed_kph = 100.0;
  for (int k = 0; k < 300; ++k) {
    in.vehicle_speed_kph = plant.speedKph();
    in.set_request = (k >= 1) && (k < 10);
    const CruiseOutputs out = cruise.step(in, kDt);
    plant.step(out.throttle_pct, 0.0, 0.0, kDt);
  }
  ASSERT_EQ(cruise.state(), CruiseState::Active);
  in.speed_timeout = true;
  const CruiseOutputs out = cruise.step(in, kDt);
  EXPECT_EQ(out.state, CruiseState::Fault);
  EXPECT_DOUBLE_EQ(out.throttle_pct, 0.0);
}

}  // namespace
}  // namespace vecu::control
