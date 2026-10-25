// Integration tests of the UDP software CAN bus over real OS sockets.

#include "can/udp_can_channel.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <thread>

#include "can/can_channel_factory.hpp"

namespace vecu::can {
namespace {

using std::chrono::microseconds;
using std::chrono::milliseconds;

/// Each test process uses its own port, so parallel test runs never share a bus.
UdpBusConfig isolatedBus() {
  UdpBusConfig config;
  config.port = static_cast<std::uint16_t>(40000 + (::getpid() % 20000));
  return config;
}

CanFrame testFrame(std::uint32_t id) {
  CanFrame frame;
  frame.id = id;
  frame.dlc = 8U;
  for (std::uint8_t i = 0U; i < 8U; ++i) {
    frame.data[i] = static_cast<std::uint8_t>(0x10U + i);
  }
  return frame;
}

TEST(UdpCanChannel, TwoNodesExchangeFramesBothWays) {
  UdpCanChannel ecu_a(isolatedBus());
  UdpCanChannel ecu_b(isolatedBus());
  ASSERT_EQ(ecu_a.open(), CanStatus::Ok);
  ASSERT_EQ(ecu_b.open(), CanStatus::Ok);

  CanFrame received;
  ASSERT_EQ(ecu_a.send(testFrame(0x100U)), CanStatus::Ok);
  ASSERT_EQ(ecu_b.receive(received, milliseconds{500}), CanStatus::Ok);
  EXPECT_EQ(received, testFrame(0x100U));

  ASSERT_EQ(ecu_b.send(testFrame(0x200U)), CanStatus::Ok);
  ASSERT_EQ(ecu_a.receive(received, milliseconds{500}), CanStatus::Ok);
  EXPECT_EQ(received, testFrame(0x200U));

  EXPECT_EQ(ecu_a.stats().tx_frames, 1U);
  EXPECT_EQ(ecu_a.stats().rx_frames, 1U);
}

TEST(UdpCanChannel, BroadcastReachesEveryOtherNode) {
  UdpCanChannel tx(isolatedBus());
  UdpCanChannel rx1(isolatedBus());
  UdpCanChannel rx2(isolatedBus());
  ASSERT_EQ(tx.open(), CanStatus::Ok);
  ASSERT_EQ(rx1.open(), CanStatus::Ok);
  ASSERT_EQ(rx2.open(), CanStatus::Ok);
  ASSERT_EQ(tx.send(testFrame(0x123U)), CanStatus::Ok);
  CanFrame frame;
  EXPECT_EQ(rx1.receive(frame, milliseconds{500}), CanStatus::Ok);
  EXPECT_EQ(rx2.receive(frame, milliseconds{500}), CanStatus::Ok);
}

TEST(UdpCanChannel, SenderDoesNotReceiveItsOwnFrames) {
  UdpCanChannel node(isolatedBus());
  ASSERT_EQ(node.open(), CanStatus::Ok);
  ASSERT_EQ(node.send(testFrame(0x100U)), CanStatus::Ok);
  CanFrame frame;
  EXPECT_EQ(node.receive(frame, milliseconds{50}), CanStatus::Timeout);
}

TEST(UdpCanChannel, PreservesOrderFromOneSender) {
  UdpCanChannel tx(isolatedBus());
  UdpCanChannel rx(isolatedBus());
  ASSERT_EQ(tx.open(), CanStatus::Ok);
  ASSERT_EQ(rx.open(), CanStatus::Ok);
  constexpr std::uint8_t kCount = 100U;
  for (std::uint8_t i = 0U; i < kCount; ++i) {
    CanFrame frame = testFrame(0x100U);
    frame.data[0] = i;
    ASSERT_EQ(tx.send(frame), CanStatus::Ok);
  }
  for (std::uint8_t i = 0U; i < kCount; ++i) {
    CanFrame frame;
    ASSERT_EQ(rx.receive(frame, milliseconds{500}), CanStatus::Ok);
    EXPECT_EQ(frame.data[0], i);
  }
}

TEST(UdpCanChannel, ConcurrentSendersAreSafe) {
  // Since Phase 4 two ECU tasks (Com_20ms, Dcm_10ms) transmit on the same channel.
  UdpCanChannel tx(isolatedBus());
  UdpCanChannel rx(isolatedBus());
  ASSERT_EQ(tx.open(), CanStatus::Ok);
  ASSERT_EQ(rx.open(), CanStatus::Ok);
  constexpr int kPerThread = 300;
  // Paced at one frame per 250 us per sender: together twice the frame rate of a fully
  // loaded 500 kbit/s CAN bus (~4000 frames/s), i.e. realistic but demanding.
  auto sender = [&tx](std::uint32_t id) {
    for (int i = 0; i < kPerThread; ++i) {
      CanFrame frame = testFrame(id);
      frame.data[0] = static_cast<std::uint8_t>(i & 0xFF);
      frame.data[7] = static_cast<std::uint8_t>(~frame.data[0]);  // integrity check byte
      while (tx.send(frame) == CanStatus::TxBufferFull) {
      }
      std::this_thread::sleep_for(microseconds{250});
    }
  };
  std::thread a(sender, 0x100U);
  std::thread b(sender, 0x7E8U);
  int count_a = 0;
  int count_b = 0;
  CanFrame frame;
  while ((count_a + count_b) < 2 * kPerThread && rx.receive(frame, milliseconds{500}) == CanStatus::Ok) {
    ASSERT_EQ(frame.data[7], static_cast<std::uint8_t>(~frame.data[0]));  // never corrupted
    (frame.id == 0x100U ? count_a : count_b)++;
  }
  a.join();
  b.join();
  EXPECT_EQ(tx.stats().tx_frames, 2U * kPerThread);
  // Every frame is accounted for: received, or counted as a receive-queue drop.
  EXPECT_EQ(static_cast<std::uint64_t>(count_a + count_b) + rx.stats().rx_overruns, 2U * kPerThread);
  EXPECT_EQ(rx.stats().rx_overruns, 0U) << "receiver could not keep up at a realistic bus load";
}

TEST(UdpCanChannel, KernelQueueDropsAreCountedAsOverruns) {
  // A receiver that does not read for a while loses frames when its socket queue is full,
  // like a CAN controller with full mailboxes. The loss must be visible, never silent.
  UdpCanChannel tx(isolatedBus());
  UdpCanChannel rx(isolatedBus());
  ASSERT_EQ(tx.open(), CanStatus::Ok);
  ASSERT_EQ(rx.open(), CanStatus::Ok);
  constexpr std::uint64_t kSent = 20000U;
  for (std::uint64_t i = 0U; i < kSent; ++i) {
    while (tx.send(testFrame(0x123U)) == CanStatus::TxBufferFull) {
    }
  }
  std::this_thread::sleep_for(milliseconds{100});
  std::uint64_t received = 0U;
  CanFrame frame;
  while (rx.receive(frame, milliseconds{50}) == CanStatus::Ok) {
    ++received;
  }
  // The kernel reports drops together with the next frame that is queued (SO_RXQ_OVFL
  // semantics); with periodic traffic that is at most one cycle later.
  ASSERT_EQ(tx.send(testFrame(0x123U)), CanStatus::Ok);
  ASSERT_EQ(rx.receive(frame, milliseconds{500}), CanStatus::Ok);
  ++received;
  EXPECT_GT(rx.stats().rx_overruns, 0U);
  EXPECT_EQ(received + rx.stats().rx_overruns, kSent + 1U);  // nothing lost silently
}

TEST(UdpCanChannel, ReceiveTimesOutWithoutTraffic) {
  UdpCanChannel node(isolatedBus());
  ASSERT_EQ(node.open(), CanStatus::Ok);
  CanFrame frame;
  const auto start = std::chrono::steady_clock::now();
  EXPECT_EQ(node.receive(frame, milliseconds{30}), CanStatus::Timeout);
  EXPECT_GE(std::chrono::steady_clock::now() - start, milliseconds{30});
}

TEST(UdpCanChannel, ValidatesConfigurationAndState) {
  UdpBusConfig not_multicast = isolatedBus();
  not_multicast.group_address = "127.0.0.1";
  UdpCanChannel bad_group(not_multicast);
  EXPECT_EQ(bad_group.open(), CanStatus::InvalidArgument);

  UdpBusConfig garbage = isolatedBus();
  garbage.group_address = "not-an-ip";
  UdpCanChannel bad_text(garbage);
  EXPECT_EQ(bad_text.open(), CanStatus::InvalidArgument);

  UdpCanChannel closed(isolatedBus());
  CanFrame frame;
  EXPECT_EQ(closed.send(testFrame(0x100U)), CanStatus::NotOpen);
  EXPECT_EQ(closed.receive(frame, milliseconds{0}), CanStatus::NotOpen);

  ASSERT_EQ(closed.open(), CanStatus::Ok);
  CanFrame invalid = testFrame(0x900U);
  EXPECT_EQ(closed.send(invalid), CanStatus::InvalidFrame);
  closed.close();
  EXPECT_FALSE(closed.isOpen());
}

TEST(CanChannelFactory, ExplicitUdpBackendOpens) {
  CanChannelConfig config;
  config.backend = CanBackend::Udp;
  config.udp = isolatedBus();
  CanStatus status = CanStatus::IoError;
  const auto channel = createCanChannel(config, status);
  ASSERT_NE(channel, nullptr);
  EXPECT_EQ(status, CanStatus::Ok);
  EXPECT_EQ(channel->name(), "vbus0");
}

TEST(CanChannelFactory, AutoFallsBackWhenInterfaceMissing) {
  CanChannelConfig config;
  config.backend = CanBackend::Auto;
  config.socketcan_interface = "vecu_none0";  // never exists
  config.udp = isolatedBus();
  CanStatus status = CanStatus::IoError;
  const auto channel = createCanChannel(config, status);
  ASSERT_NE(channel, nullptr);
  EXPECT_EQ(channel->name(), "vbus0");
}

TEST(CanChannelFactory, StrictSocketCanFailsWhenInterfaceMissing) {
  CanChannelConfig config;
  config.backend = CanBackend::SocketCan;
  config.socketcan_interface = "vecu_none0";
  CanStatus status = CanStatus::Ok;
  EXPECT_EQ(createCanChannel(config, status), nullptr);
  EXPECT_EQ(status, CanStatus::NotAvailable);
}

TEST(CanChannelFactory, ParsesBackendNames) {
  CanBackend backend = CanBackend::Auto;
  EXPECT_TRUE(parseBackend("udp", backend));
  EXPECT_EQ(backend, CanBackend::Udp);
  EXPECT_TRUE(parseBackend("socketcan", backend));
  EXPECT_EQ(backend, CanBackend::SocketCan);
  EXPECT_FALSE(parseBackend("serial", backend));
}

}  // namespace
}  // namespace vecu::can
