#pragma once

// Non-volatile memory (NVM) simulation for fault memory: a binary file that survives ECU
// restarts, like the EEPROM/data-flash block an ECU uses for DTCs.
//
// Layout (little-endian, version 1):
//   0  "VNVM" magic            4
//   4  version (1)             2
//   6  record count N          2
//   8  operation cycles        4
//  12  reset counter           4
//  16  last reset reason       1
//  17  reserved                3
//  20  N x { code u32, status u8, occurrence u16, aging u8 }   (8 bytes each)
//  ..  CRC-32 over all preceding bytes                           4
//
// Robustness: writes go to "<file>.tmp", are flushed with fsync() and then atomically renamed
// over the old file, so a crash mid-write leaves either the old or the new block, never a torn
// one. On load, size, magic, version and CRC are checked; a corrupt block is reported (the
// ECU then starts with an empty fault memory and records an NVM fault).

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "faults/dtc_store.hpp"

namespace vecu::faults {

enum class ResetReason : std::uint8_t {
  PowerOn = 0U,       ///< first start or orderly shutdown before
  UdsHardReset = 1U,
  UdsSoftReset = 2U,
  Watchdog = 3U       ///< software watchdog expired
};

[[nodiscard]] const char* toString(ResetReason reason) noexcept;

struct NvmData {
  std::uint32_t operation_cycles{0U};
  std::uint32_t reset_count{0U};
  ResetReason last_reset_reason{ResetReason::PowerOn};
  std::array<DtcRecord, DtcStore::kCapacity> records{};
  std::size_t record_count{0U};
};

enum class NvmStatus : std::uint8_t { Ok, NotFound, Corrupted, IoError };
[[nodiscard]] const char* toString(NvmStatus status) noexcept;

[[nodiscard]] NvmStatus saveNvm(const std::string& path, const NvmData& data);
[[nodiscard]] NvmStatus loadNvm(const std::string& path, NvmData& data);

}  // namespace vecu::faults
