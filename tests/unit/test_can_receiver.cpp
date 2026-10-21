#include "can/can_receiver.hpp"

#include <gtest/gtest.h>

#include <chrono>

#include "can/virtual_can_bus.hpp"

namespace vecu::can {
namespace {

using std::chrono::milliseconds;

TEST(CanRxQueue, FifoAndOverrunCounting) {
  CanRxQueue queue;
  RxFrame entry;
  for (std::size_t i = 0U; i < CanRxQueue::kDepth; ++i) {
    entry.frame.id = static_cast<std::uint32_t>(i);
    EXPECT_TRUE(queue.push(entry));
  }
  entry.frame.id = 0x7FFU;
  EXPECT_FALSE(queue.push(entry));
  EXPECT_EQ(queue.overruns(), 1U);
  EXPECT_EQ(queue.size(), CanRxQueue::kDepth);

  RxFrame out;
  ASSERT_TRUE(queue.pop(out));
  EXPECT_EQ(out.frame.id, 0U);
}

TEST(CanReceiver, ForwardsTimestampedFramesFromChannelToQueue) {
  VirtualCanBus bus;
  auto tx = bus.createChannel("tx");
  auto rx = bus.createChannel("rx");
  ASSERT_EQ(tx->open(), CanStatus::Ok);
  ASSERT_EQ(rx->open(), CanStatus::Ok);

  CanRxQueue queue;
  CanReceiver receiver(*rx, queue);
  os::ThreadConfig config;
  config.name = "TestRx";
  ASSERT_TRUE(receiver.start(config));
  EXPECT_FALSE(receiver.start(config));  // already running

  const core::TimePoint before = core::now();
  CanFrame frame;
  frame.id = 0x100U;
  frame.dlc = 1U;
  for (std::uint8_t i = 0U; i < 5U; ++i) {
    frame.data[0] = i;
    ASSERT_EQ(tx->send(frame), CanStatus::Ok);
  }

  // Wait (bounded) until all frames arrived.
  const core::TimePoint deadline = core::now() + milliseconds{2000};
  while ((queue.size() < 5U) && (core::now() < deadline)) {
    os::sleepUntil(core::now() + milliseconds{1});
  }
  receiver.stop();

  EXPECT_EQ(receiver.receivedFrames(), 5U);
  for (std::uint8_t i = 0U; i < 5U; ++i) {
    RxFrame entry;
    ASSERT_TRUE(queue.pop(entry));
    EXPECT_EQ(entry.frame.data[0], i);  // order preserved
    EXPECT_GE(entry.rx_time, before);
    EXPECT_GT(entry.wall_time_us, 0);
  }
}

TEST(CanReceiver, StopReturnsPromptlyWithoutTraffic) {
  VirtualCanBus bus;
  auto rx = bus.createChannel("rx");
  ASSERT_EQ(rx->open(), CanStatus::Ok);
  CanRxQueue queue;
  CanReceiver receiver(*rx, queue);
  ASSERT_TRUE(receiver.start(os::ThreadConfig{}));
  const core::TimePoint before = core::now();
  receiver.stop();
  EXPECT_LT(core::now() - before, CanReceiver::kPollInterval * 5);
}

}  // namespace
}  // namespace vecu::can
