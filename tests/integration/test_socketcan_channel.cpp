// Integration tests of the Linux SocketCAN backend.
//
// Tests that need a virtual CAN interface are SKIPPED (not failed) when vcan0 does not
// exist, e.g. on a stock WSL2 kernel. Set it up with scripts/setup_vcan.sh where the
// kernel supports it (native Ubuntu, GitHub Actions runners, custom WSL2 kernel).

#include "can/socketcan_channel.hpp"

#include <gtest/gtest.h>
#include <net/if.h>

#include <chrono>

namespace vecu::can {
namespace {

using std::chrono::milliseconds;

constexpr const char* kInterface = "vcan0";

bool vcanAvailable() { return ::if_nametoindex(kInterface) != 0U; }

TEST(SocketCanChannel, MissingInterfaceIsReportedAsNotAvailable) {
  SocketCanChannel channel("vecu_none0");
  EXPECT_EQ(channel.open(), CanStatus::NotAvailable);
  EXPECT_FALSE(channel.isOpen());
  EXPECT_NE(channel.lastErrno(), 0);
}

TEST(SocketCanChannel, RejectsInvalidInterfaceName) {
  SocketCanChannel empty("");
  EXPECT_EQ(empty.open(), CanStatus::InvalidArgument);
  SocketCanChannel too_long("this_interface_name_is_too_long");
  EXPECT_EQ(too_long.open(), CanStatus::InvalidArgument);
}

TEST(SocketCanChannel, NonCanInterfaceIsRejected) {
  SocketCanChannel loopback("lo");  // exists, but is not a CAN device
  EXPECT_NE(loopback.open(), CanStatus::Ok);
}

TEST(SocketCanChannel, TwoSocketsExchangeFramesOnVcan) {
  if (!vcanAvailable()) {
    GTEST_SKIP() << "vcan0 not present (kernel without vcan support?)";
  }
  SocketCanChannel ecu_a(kInterface);
  SocketCanChannel ecu_b(kInterface);
  ASSERT_EQ(ecu_a.open(), CanStatus::Ok);
  ASSERT_EQ(ecu_b.open(), CanStatus::Ok);

  CanFrame sent;
  sent.id = 0x123U;
  sent.dlc = 3U;
  sent.data = {0xAAU, 0xBBU, 0xCCU, 0U, 0U, 0U, 0U, 0U};
  ASSERT_EQ(ecu_a.send(sent), CanStatus::Ok);
  CanFrame received;
  ASSERT_EQ(ecu_b.receive(received, milliseconds{500}), CanStatus::Ok);
  EXPECT_EQ(received, sent);

  CanFrame extended;
  extended.id = 0x18DAF110U;
  extended.extended = true;
  extended.dlc = 8U;
  ASSERT_EQ(ecu_b.send(extended), CanStatus::Ok);
  ASSERT_EQ(ecu_a.receive(received, milliseconds{500}), CanStatus::Ok);
  EXPECT_EQ(received, extended);
}

TEST(SocketCanChannel, SenderDoesNotReceiveOwnFrameOnVcan) {
  if (!vcanAvailable()) {
    GTEST_SKIP() << "vcan0 not present (kernel without vcan support?)";
  }
  SocketCanChannel node(kInterface);
  ASSERT_EQ(node.open(), CanStatus::Ok);
  CanFrame frame;
  frame.id = 0x7AAU;
  ASSERT_EQ(node.send(frame), CanStatus::Ok);
  EXPECT_EQ(node.receive(frame, milliseconds{50}), CanStatus::Timeout);
}

}  // namespace
}  // namespace vecu::can
