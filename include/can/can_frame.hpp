#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace vecu::can {

inline constexpr std::uint32_t kMaxStandardId = 0x7FFU;      ///< 11-bit identifier
inline constexpr std::uint32_t kMaxExtendedId = 0x1FFFFFFFU; ///< 29-bit identifier
inline constexpr std::uint8_t kMaxClassicDlc = 8U;           ///< Classic CAN payload

/// Application-level Classic CAN 2.0 data frame (no CAN FD, no remote frames).
///
/// Deliberately independent of <linux/can.h>, so that application code never depends on
/// the transport. Each backend converts to/from its own representation.
struct CanFrame {
  std::uint32_t id{0U};  ///< identifier without any flag bits
  bool extended{false};  ///< false = 11-bit, true = 29-bit identifier
  std::uint8_t dlc{0U};  ///< number of payload bytes, 0..8
  std::array<std::uint8_t, kMaxClassicDlc> data{};
};

/// True if identifier and DLC are within the Classic CAN limits.
[[nodiscard]] bool isValid(const CanFrame& frame) noexcept;

/// Compares id, format, DLC and the first `dlc` payload bytes (unused bytes are ignored).
[[nodiscard]] bool operator==(const CanFrame& lhs, const CanFrame& rhs) noexcept;
[[nodiscard]] inline bool operator!=(const CanFrame& lhs, const CanFrame& rhs) noexcept {
  return !(lhs == rhs);
}

/// Formats a frame in can-utils notation: "123#DEADBEEF" (11-bit) / "12345678#00" (29-bit).
[[nodiscard]] std::string toCandumpString(const CanFrame& frame);

/// Formats an identifier as "0x100".
[[nodiscard]] std::string formatId(std::uint32_t id);

}  // namespace vecu::can
