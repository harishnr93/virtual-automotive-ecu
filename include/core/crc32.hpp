#pragma once

// CRC-32 (IEEE 802.3 / ISO-HDLC: polynomial 0x04C11DB7 reflected = 0xEDB88320,
// init 0xFFFFFFFF, final XOR 0xFFFFFFFF). Check value: crc32("123456789") = 0xCBF43926.
//
// Detects accidental corruption (bit flips, truncation, torn writes). It is NOT a
// cryptographic checksum: anyone can compute a matching CRC for modified data, so it
// provides no authentication. Used by the NVM simulation (Phase 5) and image checks (Phase 6).

#include <array>
#include <cstddef>
#include <cstdint>

namespace vecu::core {

namespace detail {
constexpr std::array<std::uint32_t, 256U> makeCrc32Table() noexcept {
  std::array<std::uint32_t, 256U> table{};
  for (std::uint32_t i = 0U; i < 256U; ++i) {
    std::uint32_t value = i;
    for (int bit = 0; bit < 8; ++bit) {
      value = ((value & 1U) != 0U) ? ((value >> 1U) ^ 0xEDB88320U) : (value >> 1U);
    }
    table[i] = value;
  }
  return table;
}
inline constexpr std::array<std::uint32_t, 256U> kCrc32Table = makeCrc32Table();  // built at compile time
}  // namespace detail

/// Incremental use: crc = crc32Update(crc32Init(), data...); result = crc32Final(crc).
[[nodiscard]] constexpr std::uint32_t crc32Init() noexcept { return 0xFFFFFFFFU; }

[[nodiscard]] inline std::uint32_t crc32Update(std::uint32_t crc, const std::uint8_t* data, std::size_t length) noexcept {
  for (std::size_t i = 0U; i < length; ++i) {
    crc = detail::kCrc32Table[(crc ^ data[i]) & 0xFFU] ^ (crc >> 8U);
  }
  return crc;
}

[[nodiscard]] constexpr std::uint32_t crc32Final(std::uint32_t crc) noexcept { return crc ^ 0xFFFFFFFFU; }

[[nodiscard]] inline std::uint32_t crc32(const std::uint8_t* data, std::size_t length) noexcept {
  return crc32Final(crc32Update(crc32Init(), data, length));
}

}  // namespace vecu::core
