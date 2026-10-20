#include "can/can_frame.hpp"

#include <gtest/gtest.h>

namespace vecu::can {
namespace {

CanFrame frame(std::uint32_t id, bool extended, std::uint8_t dlc) {
  CanFrame f;
  f.id = id;
  f.extended = extended;
  f.dlc = dlc;
  return f;
}

TEST(CanFrame, ValidatesStandardIdentifierRange) {
  EXPECT_TRUE(isValid(frame(0x000U, false, 0U)));
  EXPECT_TRUE(isValid(frame(0x7FFU, false, 8U)));
  EXPECT_FALSE(isValid(frame(0x800U, false, 8U)));
}

TEST(CanFrame, ValidatesExtendedIdentifierRange) {
  EXPECT_TRUE(isValid(frame(0x1FFFFFFFU, true, 8U)));
  EXPECT_FALSE(isValid(frame(0x20000000U, true, 8U)));
}

TEST(CanFrame, RejectsDlcAboveEight) { EXPECT_FALSE(isValid(frame(0x100U, false, 9U))); }

TEST(CanFrame, EqualityIgnoresBytesBeyondDlc) {
  CanFrame a = frame(0x123U, false, 2U);
  CanFrame b = a;
  a.data = {1U, 2U, 0xAAU, 0U, 0U, 0U, 0U, 0U};
  b.data = {1U, 2U, 0xBBU, 0U, 0U, 0U, 0U, 0U};
  EXPECT_EQ(a, b);
  b.data[1] = 3U;
  EXPECT_NE(a, b);
}

TEST(CanFrame, FormatsCandumpNotation) {
  CanFrame f = frame(0x123U, false, 4U);
  f.data = {0xDEU, 0xADU, 0xBEU, 0xEFU, 0U, 0U, 0U, 0U};
  EXPECT_EQ(toCandumpString(f), "123#DEADBEEF");
  EXPECT_EQ(toCandumpString(frame(0x1ABCDEFU, true, 0U)), "01ABCDEF#");
}

TEST(CanFrame, FormatsIdentifier) {
  EXPECT_EQ(formatId(0x100U), "0x100");
  EXPECT_EQ(formatId(0x18DAF110U), "0x18DAF110");
}

}  // namespace
}  // namespace vecu::can
