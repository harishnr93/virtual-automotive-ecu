// Deterministic watchdog supervision tests (synthetic time).

#include "watchdog/watchdog_supervisor.hpp"

#include <gtest/gtest.h>

namespace vecu::watchdog {
namespace {

using std::chrono::milliseconds;

core::TimePoint at(int ms) { return core::TimePoint{} + milliseconds{ms}; }

class WatchdogTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // 10 ms task: heartbeat timeout 30 ms, max execution 20 ms, tolerance 2 cycles
    ASSERT_TRUE(wd.addEntity({"Control_10ms", milliseconds{30}, milliseconds{20}, 2U}));
    ASSERT_TRUE(wd.addEntity({"Com_20ms", milliseconds{60}, milliseconds{40}, 2U}));
    wd.activate(at(0));
  }

  /// Simulates a task that runs `exec` ms every `period` ms in [from, to).
  void runTask(std::size_t task, int period, int exec, int from, int to) {
    for (int t = from; t < to; t += period) {
      wd.onActivationStart(task, at(t));
      wd.onActivationEnd(task, at(t + exec));
    }
  }

  WatchdogSupervisor wd;
};

TEST_F(WatchdogTest, HealthyTasksStayOk) {
  for (int cycle = 1; cycle <= 50; ++cycle) {
    runTask(0U, 10, 1, (cycle - 1) * 20, cycle * 20);
    runTask(1U, 20, 2, (cycle - 1) * 20, cycle * 20);
    const CheckResult r = wd.check(at(cycle * 20 + 3));
    ASSERT_EQ(r.global, SupervisionStatus::Ok) << "cycle " << cycle;
    ASSERT_FALSE(r.newly_expired);
  }
  EXPECT_EQ(wd.expirations(), 0U);
}

TEST_F(WatchdogTest, MissingHeartbeatFailsThenExpiresAfterTolerance) {
  runTask(1U, 20, 2, 0, 400);  // Com stays healthy
  runTask(0U, 10, 1, 0, 100);  // Control stops after its activation at 90 ms (heartbeat 91 ms)
  EXPECT_EQ(wd.check(at(120)).global, SupervisionStatus::Ok);  // 29 ms silence: within timeout
  CheckResult r = wd.check(at(140));                           // 49 ms > 30 ms
  EXPECT_EQ(r.global, SupervisionStatus::Failed);
  EXPECT_EQ(wd.lastViolation(0U), Violation::MissingHeartbeat);
  r = wd.check(at(160));
  EXPECT_EQ(r.global, SupervisionStatus::Failed);  // 2nd failed cycle: tolerated
  r = wd.check(at(180));                            // 3rd failed cycle: tolerance exceeded
  EXPECT_EQ(r.global, SupervisionStatus::Expired);
  ASSERT_TRUE(r.newly_expired);
  EXPECT_EQ(r.entity, 0U);
  EXPECT_EQ(r.violation, Violation::MissingHeartbeat);
  EXPECT_EQ(r.detail, milliseconds{89});
  EXPECT_FALSE(wd.check(at(200)).newly_expired);  // reported once
  EXPECT_EQ(wd.expirations(), 1U);
}

TEST_F(WatchdogTest, ExecutionTimeoutDetectsStuckActivation) {
  runTask(1U, 20, 2, 0, 400);
  runTask(0U, 10, 1, 0, 100);         // normal operation up to 100 ms (last heartbeat 91 ms)
  wd.onActivationStart(0U, at(100));  // then one activation starts and never finishes
  EXPECT_EQ(wd.check(at(115)).global, SupervisionStatus::Ok);
  EXPECT_EQ(wd.check(at(125)).global, SupervisionStatus::Failed);  // 25 ms > 20 ms
  EXPECT_EQ(wd.lastViolation(0U), Violation::ExecutionTimeout);
  (void)wd.check(at(145));
  const CheckResult r = wd.check(at(165));
  ASSERT_TRUE(r.newly_expired);
  EXPECT_EQ(r.violation, Violation::ExecutionTimeout);
}

TEST_F(WatchdogTest, TransientViolationRecoversWithinTolerance) {
  runTask(1U, 20, 2, 0, 400);
  runTask(0U, 10, 1, 0, 100);
  EXPECT_EQ(wd.check(at(140)).global, SupervisionStatus::Failed);  // short hiccup
  runTask(0U, 10, 1, 140, 300);                                    // task is back
  EXPECT_EQ(wd.check(at(160)).global, SupervisionStatus::Ok);
  EXPECT_EQ(wd.status(0U), SupervisionStatus::Ok);
  EXPECT_EQ(wd.expirations(), 0U);
}

TEST_F(WatchdogTest, StalledApplicationExpiresImmediately) {
  runTask(0U, 10, 1, 0, 100);
  runTask(1U, 20, 2, 0, 100);
  const CheckResult r = wd.check(at(170));  // both silent: whole application stalled
  EXPECT_EQ(r.global, SupervisionStatus::Expired);
  ASSERT_TRUE(r.newly_expired);
  EXPECT_EQ(r.violation, Violation::ApplicationStalled);
}

TEST_F(WatchdogTest, ExpiredIsLatchedUntilReset) {
  runTask(1U, 20, 2, 0, 400);  // simulated time only moves forward
  runTask(0U, 10, 1, 0, 100);
  for (int t = 140; t <= 200; t += 20) {
    (void)wd.check(at(t));
  }
  ASSERT_EQ(wd.globalStatus(), SupervisionStatus::Expired);
  runTask(0U, 10, 1, 200, 400);  // task recovers by itself...
  EXPECT_EQ(wd.check(at(400)).global, SupervisionStatus::Expired);  // ...but Expired stays latched
  wd.reset(at(400));
  runTask(0U, 10, 1, 400, 600);
  runTask(1U, 20, 2, 400, 600);
  EXPECT_EQ(wd.check(at(600)).global, SupervisionStatus::Ok);
}

TEST_F(WatchdogTest, ConfigurationValidation) {
  WatchdogSupervisor other;
  EXPECT_FALSE(other.addEntity({"bad", milliseconds{0}, milliseconds{10}, 1U}));
  EXPECT_FALSE(other.addEntity({"bad", milliseconds{10}, milliseconds{0}, 1U}));
  for (std::size_t i = 0U; i < WatchdogSupervisor::kMaxEntities; ++i) {
    EXPECT_TRUE(other.addEntity({"t" + std::to_string(i), milliseconds{10}, milliseconds{10}, 1U}));
  }
  EXPECT_FALSE(other.addEntity({"overflow", milliseconds{10}, milliseconds{10}, 1U}));
  EXPECT_EQ(other.name(1U), "t1");
}

}  // namespace
}  // namespace vecu::watchdog
