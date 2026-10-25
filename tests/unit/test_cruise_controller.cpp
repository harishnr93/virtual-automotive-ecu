// State-machine tests of the cruise controller (no vehicle dynamics involved).

#include "control/cruise_controller.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace vecu::control {
namespace {

constexpr double kDt = 0.01;

class CruiseControllerTest : public ::testing::Test {
 protected:
  CruiseControllerTest() {
    in.vehicle_speed_kph = 80.0;
    in.speed_valid = true;
    in.main_switch = true;
    in.driver_set_speed_kph = 90.0;
  }

  CruiseOutputs step(int n = 1) {
    CruiseOutputs out;
    for (int i = 0; i < n; ++i) {
      out = cruise.step(in, kDt);
    }
    return out;
  }

  /// Press and release the SET button.
  CruiseOutputs pressSet() {
    in.set_request = true;
    const CruiseOutputs out = step();
    in.set_request = false;
    return out;
  }

  CruiseController cruise;
  CruiseInputs in;
};

TEST_F(CruiseControllerTest, DefaultConfigurationIsValid) { EXPECT_TRUE(cruise.configValid()); }

TEST_F(CruiseControllerTest, MainSwitchTogglesOffAndStandby) {
  in.main_switch = false;
  EXPECT_EQ(step().state, CruiseState::Off);
  in.main_switch = true;
  EXPECT_EQ(step().state, CruiseState::Standby);
  in.main_switch = false;
  EXPECT_EQ(step().state, CruiseState::Off);
}

TEST_F(CruiseControllerTest, EngagesOnSetEdgeWithDriverTarget) {
  step();
  const CruiseOutputs out = pressSet();
  EXPECT_EQ(out.state, CruiseState::Active);
  EXPECT_DOUBLE_EQ(out.target_speed_kph, 90.0);
  EXPECT_NEAR(out.setpoint_kph, 80.0 + 3.0 * kDt, 1e-9);  // ramp starts at current speed
  EXPECT_GT(out.throttle_pct, 0.0);
}

TEST_F(CruiseControllerTest, HoldingSetDoesNotReEngageAfterCancel) {
  step();
  in.set_request = true;
  EXPECT_EQ(step().state, CruiseState::Active);
  in.cancel_request = true;
  EXPECT_EQ(step().state, CruiseState::Standby);
  in.cancel_request = false;
  EXPECT_EQ(step(5).state, CruiseState::Standby);  // SET still held: no new rising edge
}

TEST_F(CruiseControllerTest, UsesCurrentSpeedWhenDriverRequestImplausible) {
  in.driver_set_speed_kph = 0.0;
  step();
  EXPECT_DOUBLE_EQ(pressSet().target_speed_kph, 80.0);
}

TEST_F(CruiseControllerTest, TargetIsClampedToConfiguredRange) {
  in.driver_set_speed_kph = 250.0;
  step();
  EXPECT_DOUBLE_EQ(pressSet().target_speed_kph, 180.0);
}

TEST_F(CruiseControllerTest, DoesNotEngageBelowMinimumSpeedOrWithBrake) {
  in.vehicle_speed_kph = 20.0;
  step();
  EXPECT_EQ(pressSet().state, CruiseState::Standby);
  in.vehicle_speed_kph = 80.0;
  in.brake_pressed = true;
  EXPECT_EQ(pressSet().state, CruiseState::Standby);
}

TEST_F(CruiseControllerTest, BrakeOverrideDisengagesImmediately) {
  step();
  pressSet();
  in.brake_pressed = true;
  const CruiseOutputs out = step();
  EXPECT_EQ(out.state, CruiseState::Standby);
  EXPECT_DOUBLE_EQ(out.throttle_pct, 0.0);
}

TEST_F(CruiseControllerTest, AcceleratorOverrideAndResume) {
  step();
  in.accelerator_pct = 4.0;  // below the override threshold: bumpless start from 4 %
  pressSet();
  in.accelerator_pct = 0.0;
  const double integral_before = step(50).pid.integral;
  in.accelerator_pct = 30.0;
  CruiseOutputs out = step();
  EXPECT_EQ(out.state, CruiseState::Override);
  EXPECT_DOUBLE_EQ(step(100).pid.integral, integral_before);  // integrator held during override
  EXPECT_DOUBLE_EQ(out.throttle_pct, 0.0);       // driver pedal is in control
  EXPECT_DOUBLE_EQ(out.target_speed_kph, 90.0);  // target remembered
  in.vehicle_speed_kph = 95.0;
  in.accelerator_pct = 0.0;
  out = step();
  EXPECT_EQ(out.state, CruiseState::Active);
  EXPECT_NEAR(out.setpoint_kph, 95.0 - 3.0 * kDt, 1e-9);  // ramps down from current speed
  EXPECT_NEAR(out.pid.integral, integral_before, 0.1);     // resumes from the held integrator
}

TEST_F(CruiseControllerTest, BrakeInOverrideDisengages) {
  step();
  pressSet();
  in.accelerator_pct = 30.0;
  step();
  in.brake_pressed = true;
  EXPECT_EQ(step().state, CruiseState::Standby);
}

TEST_F(CruiseControllerTest, TargetFollowsDriverChangesWhileActive) {
  step();
  pressSet();
  in.driver_set_speed_kph = 100.0;
  EXPECT_DOUBLE_EQ(step().target_speed_kph, 100.0);
}

TEST_F(CruiseControllerTest, SpeedTimeoutFaultCutsThrottleAndRecoversToStandbyOnly) {
  step();
  pressSet();
  ASSERT_GT(step(10).throttle_pct, 0.0);

  in.speed_timeout = true;
  in.speed_valid = false;
  CruiseOutputs out = step();
  EXPECT_EQ(out.state, CruiseState::Fault);
  EXPECT_EQ(out.fault, CruiseFault::SpeedSignalTimeout);
  EXPECT_DOUBLE_EQ(out.throttle_pct, 0.0);  // safe state within one 10 ms step

  in.speed_timeout = false;
  in.speed_valid = true;
  EXPECT_EQ(step(49).state, CruiseState::Fault);  // 0.49 s healthy: not yet
  out = step();
  EXPECT_EQ(out.state, CruiseState::Standby);     // 0.50 s healthy: recovered, NOT Active
  EXPECT_EQ(out.fault, CruiseFault::None);
}

TEST_F(CruiseControllerTest, FaultRecoveryTimerRestartsOnIntermittentFault) {
  step();
  in.speed_valid = false;
  step();
  in.speed_valid = true;
  step(30);
  in.speed_valid = false;  // fault returns before the 0.5 s debounce elapsed
  step();
  in.speed_valid = true;
  EXPECT_EQ(step(49).state, CruiseState::Fault);
  EXPECT_EQ(step().state, CruiseState::Standby);
}

TEST_F(CruiseControllerTest, FaultReasonsArePrioritised) {
  step();
  in.speed_valid = false;
  EXPECT_EQ(step().fault, CruiseFault::SpeedInvalid);
  in.speed_valid = true;
  step(100);
  in.driver_inputs_timeout = true;
  in.speed_timeout = true;
  EXPECT_EQ(step().fault, CruiseFault::DriverInputTimeout);
  in = CruiseInputs{};
  in.inputs_stale = true;
  EXPECT_EQ(step(100).fault, CruiseFault::DriverInputTimeout);  // first reason is kept while in Fault
}

TEST_F(CruiseControllerTest, DriverInputFaultIsRelevantEvenWhenOff) {
  in.main_switch = false;
  step();
  in.driver_inputs_timeout = true;
  EXPECT_EQ(step().state, CruiseState::Fault);
}

TEST_F(CruiseControllerTest, SpeedFaultIgnoredWhileSwitchedOff) {
  in.main_switch = false;
  in.speed_valid = false;
  EXPECT_EQ(step().state, CruiseState::Off);
}

TEST_F(CruiseControllerTest, FunctionInhibitionForcesFaultUntilReleasedPlusRecovery) {
  step();
  pressSet();
  ASSERT_EQ(cruise.state(), CruiseState::Active);
  in.function_inhibited = true;
  CruiseOutputs out = step();
  EXPECT_EQ(out.state, CruiseState::Fault);
  EXPECT_EQ(out.fault, CruiseFault::FunctionInhibited);
  EXPECT_DOUBLE_EQ(out.throttle_pct, 0.0);
  EXPECT_EQ(step(100).state, CruiseState::Fault);  // held as long as inhibited
  in.function_inhibited = false;
  EXPECT_EQ(step(49).state, CruiseState::Fault);   // then the normal 500 ms debounce
  EXPECT_EQ(step().state, CruiseState::Standby);
}

TEST_F(CruiseControllerTest, SpecificFaultReasonWinsOverInhibition) {
  step();
  in.function_inhibited = true;
  in.speed_timeout = true;
  EXPECT_EQ(step().fault, CruiseFault::SpeedSignalTimeout);
}

TEST_F(CruiseControllerTest, InvalidTimeStepLatchesControllerFault) {
  step();
  const CruiseOutputs out = cruise.step(in, std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(out.state, CruiseState::Fault);
  EXPECT_EQ(out.fault, CruiseFault::ControllerFault);
  EXPECT_EQ(step(200).state, CruiseState::Fault);  // latched until ECU restart
}

TEST(CruiseConfig, InvalidConfigurationAlwaysFaults) {
  CruiseConfig config;
  config.max_set_speed_kph = 10.0;  // below min engage speed
  CruiseController cruise(config);
  EXPECT_FALSE(cruise.configValid());
  CruiseInputs in;
  in.main_switch = true;
  in.speed_valid = true;
  EXPECT_EQ(cruise.step(in, kDt).state, CruiseState::Fault);
}

}  // namespace
}  // namespace vecu::control
