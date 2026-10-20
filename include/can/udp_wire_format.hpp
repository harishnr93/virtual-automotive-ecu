#pragma once

// Datagram format of the UDP software CAN bus (fallback when vcan is unavailable).
//
// One datagram carries exactly one Classic CAN frame. All multi-byte fields are
// little-endian. The Python mirror is simulator/vecu_sim/udp_can_bus.py
// (struct format "<4sBBBBII8s"); both sides are tested against the same byte vector.
//
//   offset size field
//   0      4    magic "VCAN"
//   4      1    version (1)
//   5      1    flags (bit0 = 29-bit identifier)
//   6      1    dlc (0..8)
//   7      1    reserved (0)
//   8      4    sender_id (random per channel; used to suppress own frames)
//   12     4    can_id (without flag bits)
//   16     8    data (bytes beyond dlc are zero)

#include <array>
#include <cstddef>
#include <cstdint>

#include "can/can_frame.hpp"

namespace vecu::can::udp_wire {

inline constexpr std::array<std::uint8_t, 4U> kMagic{'V', 'C', 'A', 'N'};
inline constexpr std::uint8_t kVersion = 1U;
inline constexpr std::uint8_t kFlagExtended = 0x01U;
inline constexpr std::size_t kDatagramSize = 24U;

using Datagram = std::array<std::uint8_t, kDatagramSize>;

enum class DecodeResult : std::uint8_t { Ok, BadSize, BadMagic, BadVersion, InvalidFrame };

void encode(const CanFrame& frame, std::uint32_t sender_id, Datagram& out) noexcept;

[[nodiscard]] DecodeResult decode(const std::uint8_t* buffer, std::size_t length, CanFrame& frame,
                                  std::uint32_t& sender_id) noexcept;

}  // namespace vecu::can::udp_wire
