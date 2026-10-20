// Verifies that the compiled-in CAN matrix (vehicle_signals.hpp) is identical to the DBC
// file in config/. This prevents the classic integration bug where two ECUs are built from
// different versions of the communication matrix.

#include "can/vehicle_signals.hpp"

#include <gtest/gtest.h>

#include <fstream>
#include <map>
#include <regex>
#include <string>
#include <vector>

namespace vecu::can::vehicle {
namespace {

struct DbcSignal {
  std::string name;
  unsigned start_bit{};
  unsigned length{};
  bool little_endian{};
  bool is_signed{};
  double factor{};
  double offset{};
  double minimum{};
  double maximum{};
  std::string unit;
};

struct DbcMessage {
  std::string name;
  unsigned dlc{};
  std::string transmitter;
  std::vector<DbcSignal> signals;
  long cycle_time_ms{-1};
  long timeout_ms{-1};
};

/// Minimal DBC reader: supports exactly the subset used by config/vehicle_can.dbc.
std::map<std::uint32_t, DbcMessage> parseDbc(const std::string& path) {
  std::ifstream file(path);
  EXPECT_TRUE(file.is_open()) << "cannot open " << path;
  const std::regex bo(R"(^BO_\s+(\d+)\s+(\w+)\s*:\s*(\d+)\s+(\w+))");
  const std::regex sg(
      R"(^\s+SG_\s+(\w+)\s*:\s*(\d+)\|(\d+)@([01])([+-])\s*\(([^,]+),([^)]+)\)\s*\[([^|]+)\|([^\]]+)\]\s*\"([^\"]*)\")");
  const std::regex ba(R"(^BA_\s+\"(\w+)\"\s+BO_\s+(\d+)\s+(\d+)\s*;)");

  std::map<std::uint32_t, DbcMessage> messages;
  DbcMessage* current = nullptr;
  std::string line;
  std::smatch m;
  while (std::getline(file, line)) {
    if (std::regex_search(line, m, bo)) {
      DbcMessage& message = messages[static_cast<std::uint32_t>(std::stoul(m[1]))];
      message.name = m[2];
      message.dlc = static_cast<unsigned>(std::stoul(m[3]));
      message.transmitter = m[4];
      current = &message;
    } else if ((current != nullptr) && std::regex_search(line, m, sg)) {
      DbcSignal s;
      s.name = m[1];
      s.start_bit = static_cast<unsigned>(std::stoul(m[2]));
      s.length = static_cast<unsigned>(std::stoul(m[3]));
      s.little_endian = (m[4] == "1");
      s.is_signed = (m[5] == "-");
      s.factor = std::stod(m[6]);
      s.offset = std::stod(m[7]);
      s.minimum = std::stod(m[8]);
      s.maximum = std::stod(m[9]);
      s.unit = m[10];
      current->signals.push_back(s);
    } else if (std::regex_search(line, m, ba)) {
      DbcMessage& message = messages[static_cast<std::uint32_t>(std::stoul(m[2]))];
      if (m[1] == "GenMsgCycleTime") {
        message.cycle_time_ms = std::stol(m[3]);
      } else if (m[1] == "VECU_RxTimeoutMs") {
        message.timeout_ms = std::stol(m[3]);
      }
    } else if (line.empty()) {
      current = nullptr;
    }
  }
  return messages;
}

TEST(VehicleSignals, MatchesDbcFile) {
  const auto dbc = parseDbc(VECU_DBC_PATH);
  ASSERT_EQ(dbc.size(), kAllMessages.size());

  for (const MessageDef* message : kAllMessages) {
    SCOPED_TRACE(std::string{message->name});
    const auto it = dbc.find(message->id);
    ASSERT_NE(it, dbc.end()) << "message missing in DBC";
    const DbcMessage& expected = it->second;
    EXPECT_EQ(expected.name, message->name);
    EXPECT_EQ(expected.dlc, message->dlc);
    EXPECT_EQ(expected.transmitter, message->transmitter);
    EXPECT_EQ(expected.cycle_time_ms, static_cast<long>(message->cycle_time_ms));
    EXPECT_EQ(expected.timeout_ms, static_cast<long>(message->timeout_ms));
    ASSERT_EQ(expected.signals.size(), message->signal_count);

    for (std::size_t i = 0U; i < message->signal_count; ++i) {
      const SignalDef& actual = message->signals[i];
      const DbcSignal& ref = expected.signals[i];
      SCOPED_TRACE(std::string{actual.name});
      EXPECT_EQ(ref.name, actual.name);
      EXPECT_EQ(ref.start_bit, actual.start_bit);
      EXPECT_EQ(ref.length, actual.length);
      EXPECT_EQ(ref.little_endian, actual.byte_order == ByteOrder::LittleEndian);
      EXPECT_EQ(ref.is_signed, actual.value_type == ValueType::Signed);
      EXPECT_DOUBLE_EQ(ref.factor, actual.factor);
      EXPECT_DOUBLE_EQ(ref.offset, actual.offset);
      EXPECT_DOUBLE_EQ(ref.minimum, actual.minimum);
      EXPECT_DOUBLE_EQ(ref.maximum, actual.maximum);
      EXPECT_EQ(ref.unit, actual.unit);
    }
  }
}

TEST(VehicleSignals, AllSignalsFitTheirMessageAndDoNotOverlap) {
  for (const MessageDef* message : kAllMessages) {
    std::uint64_t used_bits = 0U;
    for (std::size_t i = 0U; i < message->signal_count; ++i) {
      const SignalDef& signal = message->signals[i];
      ASSERT_TRUE(isValidDefinition(signal, message->dlc)) << signal.name;
      ASSERT_EQ(signal.byte_order, ByteOrder::LittleEndian);  // matrix uses Intel only
      const std::uint64_t bits = ((1ULL << signal.length) - 1ULL) << signal.start_bit;
      EXPECT_EQ(used_bits & bits, 0U) << "overlap at " << signal.name;
      used_bits |= bits;
    }
  }
}

TEST(VehicleSignals, CycleTimesAreShorterThanTimeouts) {
  for (const MessageDef* message : kAllMessages) {
    EXPECT_GT(message->cycle_time_ms, 0U);
    EXPECT_GE(message->timeout_ms, 3U * message->cycle_time_ms) << message->name;
  }
}

TEST(VehicleSignals, FindMessageById) {
  EXPECT_EQ(findMessage(0x100U), &kVehicleDynamics);
  EXPECT_EQ(findMessage(0x200U), &kCruiseControlStatus);
  EXPECT_EQ(findMessage(0x7FFU), nullptr);
}

}  // namespace
}  // namespace vecu::can::vehicle
