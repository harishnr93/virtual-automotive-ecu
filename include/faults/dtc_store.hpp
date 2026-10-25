#pragma once

// Diagnostic Trouble Code (DTC) memory with ISO 14229-1 status bits.
//
// A DTC is a 3-byte code: 2 bytes SAE J2012 code (e.g. U0121 -> 0xC121) + 1 failure-type
// byte (e.g. 0x87 "missing message"). Each DTC has a status byte (ISO 14229-1, Annex D):
//
//   bit 0 testFailed                         bit 4 testNotCompletedSinceLastClear
//   bit 1 testFailedThisOperationCycle       bit 5 testFailedSinceLastClear
//   bit 2 pendingDTC                         bit 6 testNotCompletedThisOperationCycle
//   bit 3 confirmedDTC                       bit 7 warningIndicatorRequested (not supported)
//
// Test results come qualified (debounced) from the FaultManager (Phase 5). A DTC is confirmed
// on its first qualified failure (confirmation threshold 1). Aging: a confirmed DTC that did
// not fail for `aging_threshold` complete operation cycles loses its confirmed state.
// Persistence across ECU restarts: snapshot(..., 0) / importRecords() + faults/nvm_storage.hpp.
// Thread-safe: monitoring tasks report results while the diagnostic task reads/clears.

#include <array>
#include <cstddef>
#include <cstdint>

#include "core/os_port.hpp"

namespace vecu::faults {

using DtcCode = std::uint32_t;  ///< 24-bit value

namespace dtc_status {
inline constexpr std::uint8_t kTestFailed = 0x01U;
inline constexpr std::uint8_t kTestFailedThisOperationCycle = 0x02U;
inline constexpr std::uint8_t kPendingDtc = 0x04U;
inline constexpr std::uint8_t kConfirmedDtc = 0x08U;
inline constexpr std::uint8_t kTestNotCompletedSinceLastClear = 0x10U;
inline constexpr std::uint8_t kTestFailedSinceLastClear = 0x20U;
inline constexpr std::uint8_t kTestNotCompletedThisOperationCycle = 0x40U;
inline constexpr std::uint8_t kWarningIndicatorRequested = 0x80U;
/// Status bits this implementation supports (reported as DTCStatusAvailabilityMask).
inline constexpr std::uint8_t kAvailabilityMask = 0x7FU;
/// Status after registration or clearing: no test completed yet.
inline constexpr std::uint8_t kInitial = kTestNotCompletedSinceLastClear | kTestNotCompletedThisOperationCycle;
}  // namespace dtc_status

inline constexpr DtcCode kDtcGroupAll = 0xFFFFFFU;

struct DtcRecord {
  DtcCode code{0U};
  std::uint8_t status{dtc_status::kInitial};
  std::uint16_t occurrence_count{0U};  ///< transitions of testFailed 0 -> 1 since last clear
  std::uint8_t aging_counter{0U};      ///< completed operation cycles without failure while confirmed
};

class DtcStore {
 public:
  static constexpr std::size_t kCapacity = 16U;

  explicit DtcStore(std::uint8_t aging_threshold = 3U) noexcept : aging_threshold_(aging_threshold) {}

  /// Registers a supported DTC (initialisation phase). False if full, duplicate or > 24 bit.
  [[nodiscard]] bool registerDtc(DtcCode code);

  /// Reports the result of the monitor ("test") behind a DTC. False if the code is unknown.
  bool reportTestResult(DtcCode code, bool failed) noexcept;

  /// Clears one DTC, or all with kDtcGroupAll. False if the code is unknown.
  bool clear(DtcCode code_or_group) noexcept;

  /// Starts a new operation cycle (e.g. ignition on / ECU hard reset); also performs aging.
  void startOperationCycle() noexcept;

  /// Restores status, occurrence and aging data of registered DTCs (unknown codes are ignored,
  /// e.g. from an older software version). Returns the number of records restored.
  std::size_t importRecords(const DtcRecord* records, std::size_t count) noexcept;

  [[nodiscard]] bool status(DtcCode code, std::uint8_t& status) const noexcept;
  [[nodiscard]] bool isSupported(DtcCode code) const noexcept;

  /// Copies all records (in registration order) whose status matches `mask` (0xFF... any bit;
  /// mask 0 is treated as "all supported DTCs"). Returns the number of records written.
  std::size_t snapshot(DtcRecord* out, std::size_t capacity, std::uint8_t mask) const noexcept;
  [[nodiscard]] std::size_t countMatching(std::uint8_t mask) const noexcept;
  [[nodiscard]] std::size_t size() const noexcept;

 private:
  [[nodiscard]] DtcRecord* find(DtcCode code) noexcept;
  [[nodiscard]] const DtcRecord* find(DtcCode code) const noexcept;
  [[nodiscard]] static bool matches(const DtcRecord& record, std::uint8_t mask) noexcept;

  std::uint8_t aging_threshold_;
  mutable os::PiMutex mutex_;
  std::array<DtcRecord, kCapacity> records_{};
  std::size_t count_{0U};
};

}  // namespace vecu::faults
