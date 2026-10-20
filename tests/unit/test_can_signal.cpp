#include "can/can_signal.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <random>

namespace vecu::can {
namespace {

SignalDef intel(std::uint16_t start, std::uint8_t length, ValueType type = ValueType::Unsigned) {
  return SignalDef{"S", start, length, ByteOrder::LittleEndian, type, 1.0, 0.0, -1e9, 1e9, ""};
}

SignalDef motorola(std::uint16_t start, std::uint8_t length, ValueType type = ValueType::Unsigned) {
  return SignalDef{"S", start, length, ByteOrder::BigEndian, type, 1.0, 0.0, -1e9, 1e9, ""};
}

CanFrame emptyFrame() {
  CanFrame frame;
  frame.id = 0x100U;
  frame.dlc = 8U;
  return frame;
}

// ---------------------------------------------------------------- raw bit layout ----

TEST(CanSignalRaw, IntelByteAligned16Bit) {
  std::array<std::uint8_t, 8U> data{};
  ASSERT_EQ(encodeRaw(intel(0U, 16U), 0x1234U, data.data(), data.size()), SignalStatus::Ok);
  EXPECT_EQ(data[0], 0x34U);  // LSB first
  EXPECT_EQ(data[1], 0x12U);
}

TEST(CanSignalRaw, IntelUnalignedCrossesByteBoundary) {
  std::array<std::uint8_t, 8U> data{};
  ASSERT_EQ(encodeRaw(intel(4U, 12U), 0xABCU, data.data(), data.size()), SignalStatus::Ok);
  EXPECT_EQ(data[0], 0xC0U);
  EXPECT_EQ(data[1], 0xABU);
}

TEST(CanSignalRaw, MotorolaByteAligned16Bit) {
  // DBC "7|16@0+": MSB is bit 7 of byte 0 -> big-endian byte order.
  std::array<std::uint8_t, 8U> data{};
  ASSERT_EQ(encodeRaw(motorola(7U, 16U), 0x1234U, data.data(), data.size()), SignalStatus::Ok);
  EXPECT_EQ(data[0], 0x12U);
  EXPECT_EQ(data[1], 0x34U);
}

TEST(CanSignalRaw, MotorolaUnalignedFollowsSawtoothNumbering) {
  // DBC "3|8@0+": bits 3..0 of byte 0 hold the high nibble, bits 7..4 of byte 1 the low nibble.
  std::array<std::uint8_t, 8U> data{};
  ASSERT_EQ(encodeRaw(motorola(3U, 8U), 0xABU, data.data(), data.size()), SignalStatus::Ok);
  EXPECT_EQ(data[0], 0x0AU);
  EXPECT_EQ(data[1], 0xB0U);
}

TEST(CanSignalRaw, EncodingLeavesOtherBitsUntouched) {
  std::array<std::uint8_t, 8U> data{};
  data.fill(0xFFU);
  ASSERT_EQ(encodeRaw(intel(4U, 8U), 0x00U, data.data(), data.size()), SignalStatus::Ok);
  EXPECT_EQ(data[0], 0x0FU);
  EXPECT_EQ(data[1], 0xF0U);
  for (std::size_t i = 2U; i < data.size(); ++i) {
    EXPECT_EQ(data[i], 0xFFU);
  }
}

TEST(CanSignalRaw, RoundTripAllPositionsAndLengths) {
  std::mt19937_64 rng(12345U);  // fixed seed: deterministic
  for (const ByteOrder order : {ByteOrder::LittleEndian, ByteOrder::BigEndian}) {
    for (std::uint8_t length = 1U; length <= 64U; ++length) {
      for (std::uint16_t start = 0U; start < 64U; ++start) {
        SignalDef signal = intel(start, length);
        signal.byte_order = order;
        if (!isValidDefinition(signal, 8U)) {
          continue;
        }
        const std::uint64_t mask = (length == 64U) ? ~0ULL : ((1ULL << length) - 1ULL);
        const std::uint64_t value = rng() & mask;
        std::array<std::uint8_t, 8U> data{};
        ASSERT_EQ(encodeRaw(signal, value, data.data(), data.size()), SignalStatus::Ok);
        std::uint64_t decoded = 0U;
        ASSERT_EQ(decodeRaw(signal, data.data(), data.size(), decoded), SignalStatus::Ok);
        EXPECT_EQ(decoded, value) << "start=" << start << " length=" << int{length};
      }
    }
  }
}

TEST(CanSignalRaw, RejectsSignalsOutsidePayload) {
  EXPECT_FALSE(isValidDefinition(intel(60U, 8U), 8U));
  EXPECT_FALSE(isValidDefinition(intel(0U, 16U), 1U));
  EXPECT_FALSE(isValidDefinition(intel(0U, 0U), 8U));
  // Motorola: bit 56 is the LSB of byte 7, so a 2-bit signal would continue into byte 8.
  EXPECT_FALSE(isValidDefinition(motorola(56U, 2U), 8U));
  EXPECT_TRUE(isValidDefinition(motorola(56U, 1U), 8U));
  EXPECT_TRUE(isValidDefinition(motorola(63U, 8U), 8U));  // exactly byte 7
  std::array<std::uint8_t, 8U> data{};
  EXPECT_EQ(encodeRaw(intel(60U, 8U), 1U, data.data(), data.size()), SignalStatus::InvalidDefinition);
  EXPECT_EQ(encodeRaw(intel(0U, 8U), 1U, nullptr, 8U), SignalStatus::InvalidDefinition);
}

// ---------------------------------------------------------------- physical values ----

const SignalDef kSpeed{"Speed", 0U, 16U, ByteOrder::LittleEndian, ValueType::Unsigned, 0.01, 0.0, 0.0, 300.0, "km/h"};
const SignalDef kAccel{"Accel", 16U, 16U, ByteOrder::LittleEndian, ValueType::Signed, 0.001, 0.0, -20.0, 20.0, "m/s2"};
const SignalDef kTemp{"Temp", 32U, 8U, ByteOrder::LittleEndian, ValueType::Unsigned, 1.0, -40.0, -40.0, 215.0, "degC"};

TEST(CanSignalPhysical, ScalesAndRoundTrips) {
  CanFrame frame = emptyFrame();
  ASSERT_EQ(encodePhysical(kSpeed, 123.45, frame), SignalStatus::Ok);
  EXPECT_EQ(frame.data[0], 0x39U);  // raw 12345 = 0x3039
  EXPECT_EQ(frame.data[1], 0x30U);
  double value = 0.0;
  ASSERT_EQ(decodePhysical(kSpeed, frame, value), SignalStatus::Ok);
  EXPECT_NEAR(value, 123.45, 0.005);
}

TEST(CanSignalPhysical, SignedValuesUseTwosComplement) {
  CanFrame frame = emptyFrame();
  ASSERT_EQ(encodePhysical(kAccel, -1.234, frame), SignalStatus::Ok);
  EXPECT_EQ(frame.data[2], 0x2EU);  // -1234 = 0xFB2E
  EXPECT_EQ(frame.data[3], 0xFBU);
  double value = 0.0;
  ASSERT_EQ(decodePhysical(kAccel, frame, value), SignalStatus::Ok);
  EXPECT_NEAR(value, -1.234, 0.0005);
}

TEST(CanSignalPhysical, AppliesOffset) {
  CanFrame frame = emptyFrame();
  ASSERT_EQ(encodePhysical(kTemp, 20.0, frame), SignalStatus::Ok);
  EXPECT_EQ(frame.data[4], 60U);  // (20 - (-40)) / 1
  double value = 0.0;
  ASSERT_EQ(decodePhysical(kTemp, frame, value), SignalStatus::Ok);
  EXPECT_DOUBLE_EQ(value, 20.0);
}

TEST(CanSignalPhysical, ClampsToPhysicalRange) {
  CanFrame frame = emptyFrame();
  EXPECT_EQ(encodePhysical(kSpeed, 400.0, frame), SignalStatus::Clamped);
  double value = 0.0;
  ASSERT_EQ(decodePhysical(kSpeed, frame, value), SignalStatus::Ok);
  EXPECT_NEAR(value, 300.0, 0.005);
  EXPECT_EQ(encodePhysical(kSpeed, -5.0, frame), SignalStatus::Clamped);
  ASSERT_EQ(decodePhysical(kSpeed, frame, value), SignalStatus::Ok);
  EXPECT_DOUBLE_EQ(value, 0.0);
}

TEST(CanSignalPhysical, RejectsNonFiniteValuesWithoutWriting) {
  CanFrame frame = emptyFrame();
  frame.data.fill(0xA5U);
  EXPECT_EQ(encodePhysical(kSpeed, std::numeric_limits<double>::quiet_NaN(), frame), SignalStatus::InvalidValue);
  EXPECT_EQ(encodePhysical(kSpeed, std::numeric_limits<double>::infinity(), frame), SignalStatus::InvalidValue);
  EXPECT_EQ(frame.data[0], 0xA5U);
}

TEST(CanSignalPhysical, ReportsOutOfRangeOnDecode) {
  CanFrame frame = emptyFrame();
  frame.data[0] = 0xFFU;  // raw 0xFFFF = 655.35 km/h > 300 km/h
  frame.data[1] = 0xFFU;
  double value = 0.0;
  EXPECT_EQ(decodePhysical(kSpeed, frame, value), SignalStatus::OutOfRange);
  EXPECT_NEAR(value, 655.35, 0.005);  // value still delivered for diagnostics
}

TEST(CanSignalPhysical, RespectsFrameDlc) {
  CanFrame frame = emptyFrame();
  frame.dlc = 1U;  // signal needs 2 bytes
  double value = 0.0;
  EXPECT_EQ(encodePhysical(kSpeed, 1.0, frame), SignalStatus::InvalidDefinition);
  EXPECT_EQ(decodePhysical(kSpeed, frame, value), SignalStatus::InvalidDefinition);
}

TEST(CanSignalPhysical, RejectsZeroFactorAndOversizedSignals) {
  SignalDef zero_factor = kSpeed;
  zero_factor.factor = 0.0;
  SignalDef wide = intel(0U, 40U);
  CanFrame frame = emptyFrame();
  EXPECT_EQ(encodePhysical(zero_factor, 1.0, frame), SignalStatus::InvalidDefinition);
  EXPECT_EQ(encodePhysical(wide, 1.0, frame), SignalStatus::InvalidDefinition);
}

TEST(CanSignalPhysical, MakeFrameAndFindSignal) {
  const std::array<SignalDef, 2U> signals{kSpeed, kAccel};
  const MessageDef message{"Msg", 0x321U, 8U, 20U, 100U, "Node", signals.data(), signals.size()};
  const CanFrame frame = makeFrame(message);
  EXPECT_EQ(frame.id, 0x321U);
  EXPECT_FALSE(frame.extended);
  EXPECT_EQ(frame.dlc, 8U);
  ASSERT_NE(findSignal(message, "Accel"), nullptr);
  EXPECT_EQ(findSignal(message, "Accel")->start_bit, 16U);
  EXPECT_EQ(findSignal(message, "Missing"), nullptr);
}

}  // namespace
}  // namespace vecu::can
