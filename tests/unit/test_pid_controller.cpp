#include "control/pid_controller.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace vecu::control {
namespace {

constexpr double kDt = 0.01;

PidConfig config(double kp, double ki, double kd = 0.0) {
  PidConfig c;
  c.kp = kp;
  c.ki = ki;
  c.kd = kd;
  c.output_min = -100.0;
  c.output_max = 100.0;
  return c;
}

TEST(PidConfig, Validation) {
  EXPECT_TRUE(isValid(config(1.0, 0.5)));
  PidConfig c = config(1.0, 0.5);
  c.kp = -1.0;
  EXPECT_FALSE(isValid(c));
  c = config(1.0, 0.5);
  c.output_min = c.output_max;
  EXPECT_FALSE(isValid(c));
  c = config(1.0, 0.5);
  c.ki = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(isValid(c));
  c = config(1.0, 0.5);
  c.max_output_rate = -1.0;
  EXPECT_FALSE(isValid(c));
}

TEST(PidController, ProportionalOnly) {
  PidController pid(config(2.0, 0.0));
  EXPECT_DOUBLE_EQ(pid.update(10.0, 7.0, kDt), 6.0);
  EXPECT_DOUBLE_EQ(pid.lastTerms().proportional, 6.0);
  EXPECT_DOUBLE_EQ(pid.lastTerms().integral, 0.0);
}

TEST(PidController, IntegralAccumulatesErrorOverTime) {
  PidController pid(config(0.0, 0.5));
  double out = 0.0;
  for (int i = 0; i < 100; ++i) {  // 1 s with constant error 2
    out = pid.update(2.0, 0.0, kDt);
  }
  EXPECT_NEAR(out, 0.5 * 2.0 * 1.0, 1e-9);  // Ki * e * t
}

TEST(PidController, DerivativeActsOnMeasurementNotSetpoint) {
  PidController pid(config(0.0, 0.0, 1.0));
  (void)pid.update(0.0, 0.0, kDt);
  // Setpoint step: no derivative kick.
  EXPECT_DOUBLE_EQ(pid.update(50.0, 0.0, kDt), 0.0);
  // Measurement rises by 0.1 in 10 ms -> D = -Kd * 10/s.
  EXPECT_NEAR(pid.update(50.0, 0.1, kDt), -10.0, 1e-9);
}

TEST(PidController, DerivativeFilterSmoothsSteps) {
  PidConfig c = config(0.0, 0.0, 1.0);
  c.derivative_filter_tau_s = 0.09;  // alpha = 0.01 / 0.1 = 0.1
  PidController pid(c);
  (void)pid.update(0.0, 0.0, kDt);
  EXPECT_NEAR(pid.update(0.0, 0.1, kDt), -1.0, 1e-9);  // 10% of the raw -10
}

TEST(PidController, FeedforwardIsAddedDirectly) {
  PidController pid(config(1.0, 0.0));
  EXPECT_DOUBLE_EQ(pid.update(1.0, 1.0, kDt, 12.5), 12.5);
  EXPECT_DOUBLE_EQ(pid.lastTerms().feedforward, 12.5);
}

TEST(PidController, OutputIsSaturated) {
  PidConfig c = config(10.0, 0.0);
  c.output_min = 0.0;
  c.output_max = 60.0;
  PidController pid(c);
  EXPECT_DOUBLE_EQ(pid.update(100.0, 0.0, kDt), 60.0);
  EXPECT_TRUE(pid.lastTerms().limited);
  EXPECT_DOUBLE_EQ(pid.update(-100.0, 0.0, kDt), 0.0);
}

TEST(PidController, OutputRateLimit) {
  PidConfig c = config(10.0, 0.0);
  c.max_output_rate = 50.0;  // 0.5 per 10 ms step
  PidController pid(c);
  EXPECT_NEAR(pid.update(100.0, 0.0, kDt), 0.5, 1e-12);
  EXPECT_NEAR(pid.update(100.0, 0.0, kDt), 1.0, 1e-12);
}

/// Runs `steps` updates with a large positive error (output saturated), then returns how many
/// steps it takes for the output to leave saturation after the error becomes negative.
int recoverySteps(AntiWindup mode) {
  PidConfig c = config(1.0, 2.0);
  c.output_min = 0.0;
  c.output_max = 10.0;
  c.anti_windup = mode;
  c.back_calculation_gain = 5.0;
  PidController pid(c);
  for (int i = 0; i < 500; ++i) {  // 5 s saturated with error 20
    (void)pid.update(20.0, 0.0, kDt);
  }
  for (int i = 1; i < 10000; ++i) {
    if (pid.update(0.0, 1.0, kDt) < 10.0) {  // error -1
      return i;
    }
  }
  return 10000;
}

TEST(PidController, AntiWindupShortensRecoveryFromSaturation) {
  const int none = recoverySteps(AntiWindup::None);
  const int clamping = recoverySteps(AntiWindup::Clamping);
  const int back_calculation = recoverySteps(AntiWindup::BackCalculation);
  EXPECT_GT(none, 1000);           // wound-up integrator (~200) needs > 10 s to unwind
  EXPECT_LE(clamping, 2);          // integrator frozen at the limit: leaves saturation at once
  EXPECT_LT(back_calculation, 50);
}

TEST(PidController, ClampingStillIntegratesWhenErrorReversesInSaturation) {
  PidConfig c = config(0.0, 1.0);
  c.output_min = 0.0;
  c.output_max = 1.0;
  PidController pid(c);
  pid.reset(1.0);
  // At the upper limit with negative error the integrator must be allowed to decrease.
  const double out = pid.update(0.0, 10.0, kDt);
  EXPECT_LT(out, 1.0);
}

TEST(PidController, BumplessResetStartsFromGivenOutput) {
  PidConfig c = config(5.0, 1.0);
  c.output_min = 0.0;
  c.output_max = 60.0;
  PidController pid(c);
  pid.reset(12.0);
  EXPECT_DOUBLE_EQ(pid.lastOutput(), 12.0);
  EXPECT_NEAR(pid.update(80.0, 80.0, kDt), 12.0, 1e-12);  // zero error: output unchanged
  pid.reset(500.0);                                       // clamped into the output range
  EXPECT_DOUBLE_EQ(pid.lastOutput(), 60.0);
}

TEST(PidController, NonFiniteInputsGiveSafeOutputAndFault) {
  PidConfig c = config(1.0, 1.0);
  c.output_min = 0.0;
  PidController pid(c);
  (void)pid.update(10.0, 0.0, kDt);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_DOUBLE_EQ(pid.update(10.0, nan, kDt), 0.0);
  EXPECT_TRUE(pid.faulted());
  EXPECT_DOUBLE_EQ(pid.update(10.0, 0.0, 0.0), 0.0);  // dt = 0
  EXPECT_TRUE(pid.faulted());
  EXPECT_DOUBLE_EQ(pid.update(nan, 0.0, kDt, 0.0), 0.0);
  pid.reset(0.0);
  EXPECT_FALSE(pid.faulted());
  EXPECT_GT(pid.update(10.0, 0.0, kDt), 0.0);
}

TEST(PidController, InvalidConfigurationIsPermanentlyFaulted) {
  PidConfig c = config(1.0, 1.0);
  c.output_max = c.output_min;
  PidController pid(c);
  EXPECT_TRUE(pid.faulted());
  EXPECT_DOUBLE_EQ(pid.update(10.0, 0.0, kDt), 0.0);
  pid.reset(5.0);
  EXPECT_TRUE(pid.faulted());
}

}  // namespace
}  // namespace vecu::control
