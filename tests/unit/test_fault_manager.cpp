// Deterministic fault-injection tests of the fault manager (synthetic time, no sleeping).

#include "faults/fault_manager.hpp"

#include <gtest/gtest.h>

#include <array>
#include <vector>

namespace vecu::faults {
namespace {

using std::chrono::milliseconds;
using S = FaultState;

core::TimePoint at(int ms) { return core::TimePoint{} + milliseconds{ms}; }

constexpr FaultId kTimeFault = 0U;
constexpr FaultId kCounterFault = 1U;
constexpr FaultId kLatchedFault = 2U;
constexpr FaultId kInfoFault = 3U;
constexpr DtcCode kDtcTime = 0xC12187U;
constexpr DtcCode kDtcCounter = 0xF00048U;
constexpr DtcCode kDtcLatched = 0xF00047U;

class FaultManagerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ASSERT_TRUE(fm.addFault(kTimeFault, {"TimeFault", kDtcTime, DebounceConfig::time(milliseconds{40}, milliseconds{500}),
                                         false, true}));
    ASSERT_TRUE(fm.addFault(kCounterFault, {"CounterFault", kDtcCounter, DebounceConfig::counter(50, 10, 100, 0),
                                            false, true}));
    ASSERT_TRUE(fm.addFault(kLatchedFault, {"LatchedFault", kDtcLatched, DebounceConfig::time(milliseconds{0}, milliseconds{0}),
                                            true, true}));
    ASSERT_TRUE(fm.addFault(kInfoFault, {"InfoFault", 0U, DebounceConfig::time(milliseconds{0}, milliseconds{0}),
                                         false, false}));
  }

  /// Reports `failed` every 10 ms in [from, to).
  void run(FaultId id, bool failed, int from, int to) {
    for (int t = from; t < to; t += 10) {
      fm.report(id, failed, at(t));
    }
  }

  std::uint8_t dtcStatus(DtcCode code) {
    std::uint8_t status = 0U;
    EXPECT_TRUE(dtcs.status(code, status));
    return status;
  }

  DtcStore dtcs;
  FaultManager fm{dtcs};
};

TEST_F(FaultManagerTest, RegistrationRules) {
  EXPECT_FALSE(fm.addFault(kTimeFault, {"Dup", 0U, {}, false, false}));          // id in use
  EXPECT_FALSE(fm.addFault(FaultManager::kMaxFaults, {"Big", 0U, {}, false, false}));
  EXPECT_FALSE(fm.addFault(9U, {"", 0U, {}, false, false}));                     // no name
  EXPECT_FALSE(fm.addFault(9U, {"BadCounter", 0U, DebounceConfig::counter(0, 1, 10, 0), false, false}));
  EXPECT_TRUE(dtcs.isSupported(kDtcTime));                                       // DTC registered
  EXPECT_EQ(fm.state(kTimeFault), S::NotTested);
  EXPECT_EQ(fm.name(kCounterFault), "CounterFault");
}

TEST_F(FaultManagerTest, TimeDebounceQualifiesAfterContinuousFailure) {
  fm.report(kTimeFault, true, at(0));
  EXPECT_EQ(fm.state(kTimeFault), S::PreFailed);
  fm.report(kTimeFault, true, at(30));
  EXPECT_EQ(fm.state(kTimeFault), S::PreFailed);
  EXPECT_EQ(dtcStatus(kDtcTime), dtc_status::kInitial);  // not qualified: DTC untouched
  fm.report(kTimeFault, true, at(40));
  EXPECT_EQ(fm.state(kTimeFault), S::Failed);
  EXPECT_NE(dtcStatus(kDtcTime) & dtc_status::kConfirmedDtc, 0U);
  EXPECT_TRUE(fm.isActive(kTimeFault));
  EXPECT_TRUE(fm.functionInhibited());
}

TEST_F(FaultManagerTest, ShortGlitchIsDebouncedAway) {
  fm.report(kTimeFault, true, at(0));
  fm.report(kTimeFault, true, at(20));
  fm.report(kTimeFault, false, at(30));  // glitch shorter than 40 ms
  EXPECT_EQ(fm.state(kTimeFault), S::Passed);
  fm.report(kTimeFault, true, at(40));   // the streak restarts from here
  fm.report(kTimeFault, true, at(70));
  EXPECT_EQ(fm.state(kTimeFault), S::PreFailed);
  EXPECT_EQ(dtcStatus(kDtcTime) & dtc_status::kTestFailed, 0U);
}

TEST_F(FaultManagerTest, RecoveryConditionRequiresContinuousPassTime) {
  run(kTimeFault, true, 0, 100);
  ASSERT_EQ(fm.state(kTimeFault), S::Failed);
  fm.report(kTimeFault, false, at(100));
  EXPECT_EQ(fm.state(kTimeFault), S::PrePassed);
  EXPECT_TRUE(fm.functionInhibited());  // still active while healing
  run(kTimeFault, false, 110, 400);
  fm.report(kTimeFault, true, at(400));  // relapse during recovery
  EXPECT_EQ(fm.state(kTimeFault), S::Failed);
  fm.report(kTimeFault, false, at(410));
  run(kTimeFault, false, 420, 910);
  EXPECT_EQ(fm.state(kTimeFault), S::PrePassed);  // 500 ms not yet complete (410 -> 900)
  fm.report(kTimeFault, false, at(910));
  EXPECT_EQ(fm.state(kTimeFault), S::Passed);
  EXPECT_FALSE(fm.functionInhibited());
  const std::uint8_t status = dtcStatus(kDtcTime);
  EXPECT_EQ(status & dtc_status::kTestFailed, 0U);       // currently passing
  EXPECT_NE(status & dtc_status::kConfirmedDtc, 0U);     // history kept
}

TEST_F(FaultManagerTest, CounterDebounceToleratesSporadicFailures) {
  fm.report(kCounterFault, true, at(0));    // +50
  EXPECT_EQ(fm.state(kCounterFault), S::PreFailed);
  for (int i = 0; i < 5; ++i) {
    fm.report(kCounterFault, false, at(10 + i));  // -50 -> back to 0
  }
  EXPECT_EQ(fm.state(kCounterFault), S::Passed);
  fm.report(kCounterFault, true, at(20));
  fm.report(kCounterFault, true, at(30));   // 100: qualified
  EXPECT_EQ(fm.state(kCounterFault), S::Failed);
  fm.report(kCounterFault, false, at(40));  // 90
  EXPECT_EQ(fm.state(kCounterFault), S::PrePassed);
  for (int i = 0; i < 9; ++i) {
    fm.report(kCounterFault, false, at(50 + i));
  }
  EXPECT_EQ(fm.state(kCounterFault), S::Passed);  // 10 passing reports to heal
}

TEST_F(FaultManagerTest, LatchedFaultStaysUntilNextOperationCycle) {
  fm.report(kLatchedFault, true, at(0));
  ASSERT_EQ(fm.state(kLatchedFault), S::Failed);
  run(kLatchedFault, false, 10, 2000);
  EXPECT_EQ(fm.state(kLatchedFault), S::Failed);
  EXPECT_TRUE(fm.functionInhibited());
  fm.startOperationCycle();
  EXPECT_EQ(fm.state(kLatchedFault), S::NotTested);
  fm.report(kLatchedFault, false, at(2000));
  EXPECT_EQ(fm.state(kLatchedFault), S::Passed);
  EXPECT_FALSE(fm.functionInhibited());
}

TEST_F(FaultManagerTest, NonInhibitingFaultDoesNotBlockFunction) {
  fm.report(kInfoFault, true, at(0));
  EXPECT_EQ(fm.state(kInfoFault), S::Failed);
  EXPECT_FALSE(fm.functionInhibited());
  EXPECT_EQ(fm.activeMask(), 1U << kInfoFault);
}

TEST_F(FaultManagerTest, DtcClearRestartsTestingAndActiveFaultRefails) {
  run(kTimeFault, true, 0, 100);
  ASSERT_TRUE(dtcs.clear(kDtcGroupAll));
  fm.onDtcCleared(kDtcGroupAll);
  EXPECT_EQ(fm.state(kTimeFault), S::NotTested);
  run(kTimeFault, true, 100, 150);  // still failing: re-qualified after the debounce time
  EXPECT_EQ(fm.state(kTimeFault), S::Failed);
  EXPECT_NE(dtcStatus(kDtcTime) & dtc_status::kTestFailed, 0U);
}

TEST_F(FaultManagerTest, EventMemoryRecordsTransitionsInOrder) {
  fm.report(kTimeFault, true, at(0));
  fm.report(kTimeFault, true, at(40));
  fm.report(kTimeFault, false, at(50));
  std::array<FaultEvent, 8U> events{};
  ASSERT_EQ(fm.eventsSince(0U, events.data(), events.size()), 3U);
  EXPECT_EQ(events[0].to, S::PreFailed);
  EXPECT_EQ(events[1].to, S::Failed);
  EXPECT_EQ(events[1].time, at(40));
  EXPECT_EQ(events[2].from, S::Failed);
  EXPECT_EQ(events[2].to, S::PrePassed);
  EXPECT_EQ(fm.eventsSince(3U, events.data(), events.size()), 0U);
}

TEST_F(FaultManagerTest, EventMemoryKeepsMostRecentWhenFull) {
  for (int i = 0; i < 40; ++i) {  // 40 transitions Failed <-> Passed
    fm.report(kInfoFault, (i % 2) == 0, at(i));
  }
  EXPECT_EQ(fm.eventCount(), 40U);
  std::array<FaultEvent, FaultManager::kEventMemory> events{};
  const std::size_t n = fm.eventsSince(0U, events.data(), events.size());
  ASSERT_EQ(n, FaultManager::kEventMemory);
  EXPECT_EQ(events.front().sequence, 9U);  // 1..8 were overwritten
  EXPECT_EQ(events.back().sequence, 40U);
}

TEST_F(FaultManagerTest, UnknownFaultIsIgnored) {
  EXPECT_FALSE(fm.report(12U, true, at(0)));
  EXPECT_EQ(fm.state(12U), S::NotTested);
}

}  // namespace
}  // namespace vecu::faults
