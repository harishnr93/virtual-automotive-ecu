#pragma once

// Software watchdog supervisor (comparable to AUTOSAR WdgM alive/deadline supervision).
//
// This is NOT a hardware watchdog: it is a supervision thread inside the same process. It
// detects
//   - a missing task heartbeat   (no completed activation for `heartbeat_timeout`),
//   - a task execution timeout   (one activation running longer than `max_execution`,
//                                 e.g. stuck in a loop or blocked),
//   - a stalled application       (all supervised tasks violating at the same time).
// A violation makes an entity Failed; more than `failed_tolerance` consecutive failed
// supervision cycles make it Expired (latched until reset()). The owner reacts to an
// expiration (safe state, simulated ECU reset or process restart).
//
// Tasks report through lock-free atomics (never blocked by the supervisor); check() runs in
// its own thread. Time is passed in, so the logic is unit-tested with synthetic time.

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>

#include "core/clock.hpp"
#include "core/periodic_scheduler.hpp"

namespace vecu::watchdog {

enum class SupervisionStatus : std::uint8_t { Ok = 0U, Failed = 1U, Expired = 2U };
enum class Violation : std::uint8_t { None = 0U, MissingHeartbeat = 1U, ExecutionTimeout = 2U, ApplicationStalled = 3U };

[[nodiscard]] const char* toString(SupervisionStatus status) noexcept;
[[nodiscard]] const char* toString(Violation violation) noexcept;

struct EntityConfig {
  std::string name;
  std::chrono::milliseconds heartbeat_timeout{100};
  std::chrono::milliseconds max_execution{50};
  std::uint8_t failed_tolerance{2U};  ///< failed cycles tolerated before Expired
};

/// Result of one supervision cycle.
struct CheckResult {
  SupervisionStatus global{SupervisionStatus::Ok};
  bool newly_expired{false};                ///< global status became Expired in this cycle
  std::size_t entity{0U};                   ///< first expired entity (if newly_expired)
  Violation violation{Violation::None};     ///< its violation (ApplicationStalled if all)
  std::chrono::nanoseconds detail{0};       ///< time since heartbeat / execution start
};

class WatchdogSupervisor final : public sched::ITaskObserver {
 public:
  static constexpr std::size_t kMaxEntities = 8U;

  /// Registers an entity (initialisation phase). Entity index = registration order, which
  /// matches the scheduler's task index when tasks are registered in the same order.
  [[nodiscard]] bool addEntity(const EntityConfig& config);

  /// Starts supervision: every entity counts as "just seen" at `now`.
  void activate(core::TimePoint now) noexcept;

  // Called from the task threads (via the scheduler observer interface): lock-free.
  void onActivationStart(std::size_t task_index, core::TimePoint now) noexcept override;
  void onActivationEnd(std::size_t task_index, core::TimePoint now) noexcept override;

  /// One supervision cycle. Call periodically (e.g. every 20 ms) from the watchdog thread.
  CheckResult check(core::TimePoint now) noexcept;

  /// After recovery: clears Expired/Failed states and restarts all timers at `now`.
  void reset(core::TimePoint now) noexcept;

  [[nodiscard]] SupervisionStatus globalStatus() const noexcept;
  [[nodiscard]] SupervisionStatus status(std::size_t entity) const noexcept;
  [[nodiscard]] Violation lastViolation(std::size_t entity) const noexcept;
  [[nodiscard]] std::string name(std::size_t entity) const;
  /// Last completed activation of an entity, and whether an activation is running now.
  [[nodiscard]] core::TimePoint lastHeartbeat(std::size_t entity) const noexcept;
  [[nodiscard]] bool isExecuting(std::size_t entity) const noexcept;
  [[nodiscard]] std::size_t size() const noexcept { return count_; }
  [[nodiscard]] std::uint32_t expirations() const noexcept { return expirations_.load(); }

 private:
  struct Entity {
    EntityConfig config{};
    // written by the task thread
    std::atomic<std::int64_t> last_heartbeat_ns{0};
    std::atomic<std::int64_t> execution_start_ns{0};
    std::atomic<bool> executing{false};
    // owned by check()/reset() (protected by mutex_)
    SupervisionStatus status{SupervisionStatus::Ok};
    Violation violation{Violation::None};
    std::uint32_t failed_cycles{0U};
  };

  static std::int64_t ns(core::TimePoint t) noexcept {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
  }
  [[nodiscard]] SupervisionStatus globalLocked() const noexcept;

  std::array<Entity, kMaxEntities> entities_{};
  std::size_t count_{0U};
  mutable std::mutex mutex_;
  SupervisionStatus last_global_{SupervisionStatus::Ok};
  std::atomic<std::uint32_t> expirations_{0U};
};

}  // namespace vecu::watchdog
