#include "can/virtual_can_bus.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

namespace vecu::can {
namespace {

using std::chrono::microseconds;
using std::chrono::milliseconds;

CanFrame makeTestFrame(std::uint32_t id, std::uint8_t first_byte) {
  CanFrame frame;
  frame.id = id;
  frame.dlc = 1U;
  frame.data[0] = first_byte;
  return frame;
}

TEST(VirtualCanBus, BroadcastsToAllOtherChannelsButNotSender) {
  VirtualCanBus bus;
  auto a = bus.createChannel("a");
  auto b = bus.createChannel("b");
  auto c = bus.createChannel("c");
  ASSERT_EQ(a->open(), CanStatus::Ok);
  ASSERT_EQ(b->open(), CanStatus::Ok);
  ASSERT_EQ(c->open(), CanStatus::Ok);
  EXPECT_EQ(bus.openChannelCount(), 3U);

  const CanFrame sent = makeTestFrame(0x100U, 0x42U);
  ASSERT_EQ(a->send(sent), CanStatus::Ok);

  CanFrame received;
  ASSERT_EQ(b->receive(received, microseconds{0}), CanStatus::Ok);
  EXPECT_EQ(received, sent);
  ASSERT_EQ(c->receive(received, microseconds{0}), CanStatus::Ok);
  EXPECT_EQ(received, sent);
  EXPECT_EQ(a->receive(received, microseconds{0}), CanStatus::Timeout);  // no self-reception
}

TEST(VirtualCanBus, ClosedChannelsDoNotSendOrReceive) {
  VirtualCanBus bus;
  auto a = bus.createChannel("a");
  auto b = bus.createChannel("b");
  CanFrame frame = makeTestFrame(0x100U, 1U);
  EXPECT_EQ(a->send(frame), CanStatus::NotOpen);
  ASSERT_EQ(a->open(), CanStatus::Ok);
  ASSERT_EQ(a->send(frame), CanStatus::Ok);  // b not open: frame is simply not delivered to it
  ASSERT_EQ(b->open(), CanStatus::Ok);
  EXPECT_EQ(b->receive(frame, microseconds{0}), CanStatus::Timeout);
  b->close();
  EXPECT_EQ(b->receive(frame, microseconds{0}), CanStatus::NotOpen);
  EXPECT_EQ(bus.openChannelCount(), 1U);
}

TEST(VirtualCanBus, RejectsInvalidFrames) {
  VirtualCanBus bus;
  auto a = bus.createChannel("a");
  ASSERT_EQ(a->open(), CanStatus::Ok);
  CanFrame frame = makeTestFrame(0x800U, 0U);  // > 11 bit
  EXPECT_EQ(a->send(frame), CanStatus::InvalidFrame);
  EXPECT_EQ(a->stats().tx_errors, 1U);
}

TEST(VirtualCanBus, CountsOverrunWhenReceiveQueueIsFull) {
  VirtualCanBus bus;
  auto tx = bus.createChannel("tx");
  auto rx = bus.createChannel("rx");
  ASSERT_EQ(tx->open(), CanStatus::Ok);
  ASSERT_EQ(rx->open(), CanStatus::Ok);
  for (std::size_t i = 0U; i < VirtualCanBus::kRxQueueDepth + 5U; ++i) {
    ASSERT_EQ(tx->send(makeTestFrame(0x100U, static_cast<std::uint8_t>(i))), CanStatus::Ok);
  }
  EXPECT_EQ(rx->stats().rx_overruns, 5U);
  CanFrame frame;
  ASSERT_EQ(rx->receive(frame, microseconds{0}), CanStatus::Ok);
  EXPECT_EQ(frame.data[0], 0U);  // oldest frames kept, newest dropped
}

TEST(VirtualCanBus, ReceiveWaitsForFrameFromAnotherThread) {
  VirtualCanBus bus;
  auto tx = bus.createChannel("tx");
  auto rx = bus.createChannel("rx");
  ASSERT_EQ(tx->open(), CanStatus::Ok);
  ASSERT_EQ(rx->open(), CanStatus::Ok);
  std::thread sender([&tx] {
    std::this_thread::sleep_for(milliseconds{20});
    (void)tx->send(makeTestFrame(0x321U, 7U));
  });
  CanFrame frame;
  EXPECT_EQ(rx->receive(frame, milliseconds{2000}), CanStatus::Ok);
  EXPECT_EQ(frame.id, 0x321U);
  sender.join();
}

TEST(VirtualCanBus, ChannelOutlivingBusIsSafe) {
  std::unique_ptr<ICanChannel> channel;
  {
    VirtualCanBus bus;
    channel = bus.createChannel("orphan");
    ASSERT_EQ(channel->open(), CanStatus::Ok);
  }
  EXPECT_EQ(channel->send(makeTestFrame(0x100U, 0U)), CanStatus::Ok);  // shared core keeps state alive
}

}  // namespace
}  // namespace vecu::can
