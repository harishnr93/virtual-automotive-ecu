#include "can/udp_wire_format.hpp"

#include <gtest/gtest.h>

namespace vecu::can::udp_wire {
namespace {

// Same reference vector as tests/unit/python/test_udp_can_bus.py — guarantees that the
// C++ ECUs and the Python tools agree on the byte layout.
constexpr Datagram kReference{0x56U, 0x43U, 0x41U, 0x4EU, 0x01U, 0x00U, 0x03U, 0x00U,
                              0xD4U, 0xC3U, 0xB2U, 0xA1U, 0x23U, 0x01U, 0x00U, 0x00U,
                              0x01U, 0x02U, 0x03U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};

CanFrame referenceFrame() {
  CanFrame frame;
  frame.id = 0x123U;
  frame.dlc = 3U;
  frame.data = {0x01U, 0x02U, 0x03U, 0U, 0U, 0U, 0U, 0U};
  return frame;
}

TEST(UdpWireFormat, EncodesReferenceVector) {
  Datagram out{};
  encode(referenceFrame(), 0xA1B2C3D4U, out);
  EXPECT_EQ(out, kReference);
}

TEST(UdpWireFormat, DecodesReferenceVector) {
  CanFrame frame;
  std::uint32_t sender = 0U;
  ASSERT_EQ(decode(kReference.data(), kReference.size(), frame, sender), DecodeResult::Ok);
  EXPECT_EQ(frame, referenceFrame());
  EXPECT_EQ(sender, 0xA1B2C3D4U);
}

TEST(UdpWireFormat, RoundTripsExtendedFrame) {
  CanFrame in;
  in.id = 0x18DAF110U;
  in.extended = true;
  in.dlc = 8U;
  in.data = {1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U};
  Datagram buffer{};
  encode(in, 42U, buffer);
  CanFrame out;
  std::uint32_t sender = 0U;
  ASSERT_EQ(decode(buffer.data(), buffer.size(), out, sender), DecodeResult::Ok);
  EXPECT_EQ(out, in);
  EXPECT_TRUE(out.extended);
}

TEST(UdpWireFormat, RejectsMalformedDatagrams) {
  CanFrame frame;
  std::uint32_t sender = 0U;
  EXPECT_EQ(decode(kReference.data(), kReference.size() - 1U, frame, sender), DecodeResult::BadSize);
  EXPECT_EQ(decode(nullptr, kDatagramSize, frame, sender), DecodeResult::BadSize);

  Datagram bad = kReference;
  bad[0] = 'X';
  EXPECT_EQ(decode(bad.data(), bad.size(), frame, sender), DecodeResult::BadMagic);

  bad = kReference;
  bad[4] = 2U;
  EXPECT_EQ(decode(bad.data(), bad.size(), frame, sender), DecodeResult::BadVersion);

  bad = kReference;
  bad[6] = 9U;  // DLC > 8
  EXPECT_EQ(decode(bad.data(), bad.size(), frame, sender), DecodeResult::InvalidFrame);

  bad = kReference;
  bad[13] = 0x08U;  // id 0x823 > 0x7FF for an 11-bit frame
  EXPECT_EQ(decode(bad.data(), bad.size(), frame, sender), DecodeResult::InvalidFrame);

  bad = kReference;
  bad[5] = 0x80U;  // unknown flag
  EXPECT_EQ(decode(bad.data(), bad.size(), frame, sender), DecodeResult::InvalidFrame);
}

}  // namespace
}  // namespace vecu::can::udp_wire
