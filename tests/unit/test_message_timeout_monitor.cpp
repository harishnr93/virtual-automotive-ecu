#include "can/message_timeout_monitor.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace vecu::can {
namespace {

using std::chrono::milliseconds;

// Deterministic time: tests construct time points directly instead of sleeping.
core::TimePoint at(int ms) { return core::TimePoint{} + milliseconds{ms}; }

class MessageTimeoutMonitorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ASSERT_TRUE(monitor.addMessage(0x100U, milliseconds{100}, milliseconds{1000}));
    ASSERT_TRUE(monitor.addMessage(0x110U, milliseconds{250}, milliseconds{1000}));
    monitor.start(at(0));
  }

  auto recorder() {
    return [this](const RxTimeoutEvent& event) { events.push_back(event); };
  }

  MessageTimeoutMonitor<4U> monitor;
  std::vector<RxTimeoutEvent> events;
};

TEST_F(MessageTimeoutMonitorTest, RejectsDuplicatesZeroTimeoutsAndOverflow) {
  EXPECT_FALSE(monitor.addMessage(0x100U, milliseconds{100}, milliseconds{100}));
  EXPECT_FALSE(monitor.addMessage(0x120U, milliseconds{0}, milliseconds{100}));
  EXPECT_TRUE(monitor.addMessage(0x120U, milliseconds{10}, milliseconds{10}));
  EXPECT_TRUE(monitor.addMessage(0x130U, milliseconds{10}, milliseconds{10}));
  EXPECT_FALSE(monitor.addMessage(0x140U, milliseconds{10}, milliseconds{10}));  // capacity 4
}

TEST_F(MessageTimeoutMonitorTest, FirstTimeoutAppliesBeforeFirstReception) {
  monitor.update(at(999), recorder());
  EXPECT_TRUE(events.empty());
  EXPECT_EQ(monitor.state(0x100U), RxTimeoutState::WaitingFirst);
  monitor.update(at(1000), recorder());
  ASSERT_EQ(events.size(), 2U);
  EXPECT_EQ(events[0].previous, RxTimeoutState::WaitingFirst);
  EXPECT_EQ(events[0].current, RxTimeoutState::TimedOut);
}

TEST_F(MessageTimeoutMonitorTest, DetectsTimeoutExactlyAtDeadline) {
  ASSERT_TRUE(monitor.notifyReceived(0x100U, at(10), recorder()));
  ASSERT_EQ(events.size(), 1U);  // WaitingFirst -> Ok
  events.clear();

  monitor.update(at(109), recorder());
  EXPECT_TRUE(events.empty());
  monitor.update(at(110), recorder());
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events[0].id, 0x100U);
  EXPECT_EQ(events[0].current, RxTimeoutState::TimedOut);
  EXPECT_EQ(events[0].silence, milliseconds{100});
  EXPECT_TRUE(monitor.anyTimedOut());
  EXPECT_EQ(monitor.timeoutCount(0x100U), 1U);
}

TEST_F(MessageTimeoutMonitorTest, PeriodicReceptionPreventsTimeout) {
  for (int t = 0; t <= 2000; t += 20) {
    ASSERT_TRUE(monitor.notifyReceived(0x100U, at(t), recorder()));
    ASSERT_TRUE(monitor.notifyReceived(0x110U, at(t), recorder()));
    monitor.update(at(t + 5), recorder());
  }
  ASSERT_EQ(events.size(), 2U);  // only the two "first reception" events
  EXPECT_FALSE(monitor.anyTimedOut());
}

TEST_F(MessageTimeoutMonitorTest, TimeoutIsReportedOnceThenRecovers) {
  (void)monitor.notifyReceived(0x100U, at(0), recorder());
  events.clear();
  monitor.update(at(150), recorder());
  monitor.update(at(300), recorder());  // still timed out: no duplicate event
  ASSERT_EQ(events.size(), 1U);

  (void)monitor.notifyReceived(0x100U, at(320), recorder());
  ASSERT_EQ(events.size(), 2U);
  EXPECT_EQ(events[1].previous, RxTimeoutState::TimedOut);
  EXPECT_EQ(events[1].current, RxTimeoutState::Ok);
  EXPECT_EQ(events[1].silence, milliseconds{320});
  EXPECT_EQ(monitor.state(0x100U), RxTimeoutState::Ok);
}

TEST_F(MessageTimeoutMonitorTest, UnknownIdentifierIsIgnoredAndFailsSafe) {
  EXPECT_FALSE(monitor.notifyReceived(0x7FFU, at(0), recorder()));
  EXPECT_TRUE(events.empty());
  EXPECT_EQ(monitor.state(0x7FFU), RxTimeoutState::TimedOut);
}

}  // namespace
}  // namespace vecu::can
