#pragma once

// Fault manager: qualification (debouncing), state machine, recovery conditions, DTC update,
// event memory and function inhibition — comparable to AUTOSAR DEM + FiM, simplified.
//
// Monitors ("tests") report raw results: failed / passed. The fault manager debounces them:
//
//        failed reports                         passed reports
//   NotTested ──────► PreFailed ──(debounce)──► Failed ──────► PrePassed ──(recovery)──► Passed
//                        │  ▲ passed: back           ▲  failed: back  │
//                        ▼  │ (never qualified)       └───────────────┘
//
//   Time-based debounce:    failed for >= fail_time continuously      -> Failed
//                           passed for >= pass_time continuously      -> Passed (recovery condition)
//   Counter-based debounce: counter += increment per failed report, -= decrement per passed
//                           report; >= fail_threshold -> Failed, <= pass_threshold -> Passed
//
// Qualified results drive the DTC status (DtcStore). A *latched* fault stays Failed until the
// next operation cycle (ECU reset) or a DTC clear. Faults configured with `inhibits_function`
// block the supervised function while they are active (Failed or PrePassed).
// Thread-safe: monitors in different tasks report concurrently; a PI mutex protects the state.

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string_view>

#include "core/clock.hpp"
#include "core/os_port.hpp"
#include "faults/dtc_store.hpp"

namespace vecu::faults {

using FaultId = std::uint8_t;

enum class FaultState : std::uint8_t { NotTested, PreFailed, Failed, PrePassed, Passed };
[[nodiscard]] const char* toString(FaultState state) noexcept;

enum class DebounceType : std::uint8_t { Time, Counter };

struct DebounceConfig {
  DebounceType type{DebounceType::Time};
  std::chrono::milliseconds fail_time{0};  ///< Time: continuous failure needed to qualify
  std::chrono::milliseconds pass_time{0};  ///< Time: continuous pass needed to recover
  std::int16_t increment{1};               ///< Counter: added per failed report
  std::int16_t decrement{1};               ///< Counter: subtracted per passed report
  std::int16_t fail_threshold{1};          ///< Counter: qualified failed at or above
  std::int16_t pass_threshold{0};          ///< Counter: qualified passed at or below

  [[nodiscard]] static constexpr DebounceConfig time(std::chrono::milliseconds fail,
                                                     std::chrono::milliseconds pass) noexcept {
    DebounceConfig config;
    config.type = DebounceType::Time;
    config.fail_time = fail;
    config.pass_time = pass;
    return config;
  }
  [[nodiscard]] static constexpr DebounceConfig counter(std::int16_t inc, std::int16_t dec, std::int16_t fail_at,
                                                        std::int16_t pass_at) noexcept {
    DebounceConfig config;
    config.type = DebounceType::Counter;
    config.increment = inc;
    config.decrement = dec;
    config.fail_threshold = fail_at;
    config.pass_threshold = pass_at;
    return config;
  }
};

struct FaultConfig {
  std::string_view name;
  DtcCode dtc{0U};
  DebounceConfig debounce{};
  bool latched{false};            ///< stays Failed until the next operation cycle / clear
  bool inhibits_function{false};  ///< blocks the supervised function while active
};

/// One state change, kept in a small event memory for logging and diagnostics.
struct FaultEvent {
  std::uint32_t sequence{0U};
  FaultId id{0U};
  FaultState from{FaultState::NotTested};
  FaultState to{FaultState::NotTested};
  core::TimePoint time{};
};

class FaultManager {
 public:
  static constexpr std::size_t kMaxFaults = 16U;
  static constexpr std::uint32_t kEventMemory = 32U;

  explicit FaultManager(DtcStore& dtcs) noexcept : dtcs_(dtcs) {}

  /// Registers a fault and its DTC (initialisation phase). Returns false if full or invalid.
  [[nodiscard]] bool addFault(FaultId id, const FaultConfig& config);

  /// Reports a raw monitor result. Unknown ids are ignored (returns false).
  bool report(FaultId id, bool failed, core::TimePoint now) noexcept;

  /// New operation cycle (ECU start/reset): latched faults are released, all faults are
  /// re-tested from NotTested, and the DTC store starts a new cycle (pending/aging).
  void startOperationCycle() noexcept;

  /// After a UDS DTC clear: debounce and state of the matching faults restart (group 0xFFFFFF = all).
  void onDtcCleared(DtcCode code_or_group) noexcept;

  [[nodiscard]] FaultState state(FaultId id) const noexcept;
  [[nodiscard]] bool isActive(FaultId id) const noexcept;  ///< Failed or PrePassed
  [[nodiscard]] bool functionInhibited() const noexcept;   ///< any active inhibiting fault
  [[nodiscard]] std::uint16_t activeMask() const noexcept; ///< bit n = fault id n active
  [[nodiscard]] std::string_view name(FaultId id) const noexcept;

  /// Copies events with sequence > `after` (oldest first). Returns the number copied.
  std::size_t eventsSince(std::uint32_t after, FaultEvent* out, std::size_t capacity) const noexcept;
  [[nodiscard]] std::uint32_t eventCount() const noexcept;

 private:
  struct Entry {
    bool used{false};
    FaultConfig config{};
    FaultState state{FaultState::NotTested};
    std::int32_t counter{0};
    core::TimePoint since{};  ///< start of the current failing/passing streak (time debounce)
  };

  void transition(FaultId id, Entry& entry, FaultState to, core::TimePoint now) noexcept;
  void updateTime(FaultId id, Entry& entry, bool failed, core::TimePoint now) noexcept;
  void updateCounter(FaultId id, Entry& entry, bool failed, core::TimePoint now) noexcept;
  [[nodiscard]] static bool active(FaultState state) noexcept {
    return (state == FaultState::Failed) || (state == FaultState::PrePassed);
  }

  DtcStore& dtcs_;
  mutable os::PiMutex mutex_;
  std::array<Entry, kMaxFaults> faults_{};
  std::array<FaultEvent, kEventMemory> events_{};
  std::uint32_t event_sequence_{0U};
};

}  // namespace vecu::faults
