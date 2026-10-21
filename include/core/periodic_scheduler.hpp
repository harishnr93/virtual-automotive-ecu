#pragma once

// Periodic, fixed-priority task scheduler for the virtual ECUs.
//
// Model (close to an OSEK/AUTOSAR OS with basic tasks and alarms):
//  - every task is a thread with its own priority, released on a fixed time grid
//    (epoch + offset + k * period) using absolute monotonic wake-ups;
//  - all tasks share one epoch, set when start() releases them through a condition-variable
//    barrier, so their phases are well-defined;
//  - each activation is measured (start latency, execution time, response time, period
//    jitter) and checked against its deadline and optional execution budget;
//  - overruns follow an activation limit of 1 (see computeNextRelease()).
//
// IMPORTANT: on Linux / WSL2 this is soft real-time. Timing is measured and violations are
// detected, but nothing here can *guarantee* deadlines (no PREEMPT_RT kernel, no isolated CPU,
// and real-time priorities may be unavailable to the user).

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

#include "core/clock.hpp"
#include "core/os_port.hpp"
#include "core/timing_stats.hpp"

namespace vecu::sched {

inline constexpr std::size_t kMaxTasks = 8U;

struct TaskConfig {
  std::string name;
  std::chrono::microseconds period{0};
  std::chrono::microseconds offset{0};    ///< phase shift within the period (spreads load)
  std::chrono::microseconds deadline{0};  ///< relative deadline; 0 = implicit (= period)
  std::chrono::microseconds budget{0};    ///< execution-time budget; 0 = not monitored
  int priority{os::kMinPriority};         ///< 1..99, higher = more urgent
};

/// Passed to the task function on every activation.
struct ActivationInfo {
  std::uint64_t activation{0U};  ///< 0, 1, 2, ...
  core::TimePoint release{};     ///< planned release time of this activation
};

using TaskFunction = std::function<void(const ActivationInfo&)>;

enum class SchedulerStatus : std::uint8_t {
  Ok,
  InvalidConfig,
  AlreadyRunning,
  TooManyTasks,
  NoTasks,
  ThreadStartFailed
};

[[nodiscard]] const char* toString(SchedulerStatus status) noexcept;

struct SchedulerConfig {
  bool request_realtime{false};  ///< try SCHED_FIFO with the task priorities
  bool lock_memory{false};       ///< try mlockall() to avoid page-fault delays
  std::chrono::milliseconds start_delay{20};  ///< time for all threads to reach the barrier
};

/// Snapshot of one task: configuration plus measured timing.
struct TaskStatsSnapshot {
  std::string_view name;  ///< valid while the scheduler exists
  std::chrono::microseconds period{0};
  std::chrono::microseconds offset{0};
  std::chrono::microseconds deadline{0};
  std::chrono::microseconds budget{0};
  int priority{0};
  os::SchedulingClass scheduling_class{os::SchedulingClass::Normal};
  TaskTimingSummary timing{};
};

/// Validates a task configuration; returns an explanation if invalid.
[[nodiscard]] bool validateTaskConfig(const TaskConfig& config, std::string& reason);

/// Rate-monotonic priority assignment (shorter period -> higher priority) is optimal for
/// fixed-priority scheduling. Returns one warning per violating task pair.
[[nodiscard]] std::vector<std::string> checkRateMonotonic(const std::vector<TaskConfig>& tasks);

class PeriodicScheduler {
 public:
  explicit PeriodicScheduler(SchedulerConfig config = {});
  ~PeriodicScheduler();  // stops and joins all tasks

  PeriodicScheduler(const PeriodicScheduler&) = delete;
  PeriodicScheduler& operator=(const PeriodicScheduler&) = delete;
  PeriodicScheduler(PeriodicScheduler&&) = delete;
  PeriodicScheduler& operator=(PeriodicScheduler&&) = delete;

  /// Registers a task. Only allowed before start() (all allocation happens at init time).
  [[nodiscard]] SchedulerStatus addTask(TaskConfig config, TaskFunction function);

  /// Creates the task threads and releases them at a common epoch.
  [[nodiscard]] SchedulerStatus start();

  /// Requests all tasks to finish their current activation and joins them.
  /// Returns within about one period of the slowest task.
  void stop() noexcept;

  [[nodiscard]] bool isRunning() const noexcept { return running_.load(); }
  [[nodiscard]] std::size_t taskCount() const noexcept { return tasks_.size(); }
  [[nodiscard]] bool memoryLocked() const noexcept { return memory_locked_; }
  [[nodiscard]] core::TimePoint epoch() const noexcept { return epoch_; }

  /// Lock-free counters, cheap enough to poll from a task (e.g. a diagnostic task).
  [[nodiscard]] std::uint64_t activations(std::size_t index) const noexcept;
  [[nodiscard]] std::uint64_t deadlineMisses(std::size_t index) const noexcept;
  [[nodiscard]] std::string_view taskName(std::size_t index) const noexcept;

  /// Full statistics (copies and sorts sample windows: reporting context only).
  [[nodiscard]] TaskStatsSnapshot taskStats(std::size_t index) const;
  [[nodiscard]] std::vector<TaskStatsSnapshot> allTaskStats() const;

  /// Rate-monotonic warnings for the registered tasks.
  [[nodiscard]] std::vector<std::string> configurationWarnings() const;

 private:
  struct Task;
  void runTask(Task& task);

  SchedulerConfig config_;
  std::vector<std::unique_ptr<Task>> tasks_;
  std::atomic<bool> running_{false};
  std::atomic<bool> stop_requested_{false};

  std::mutex start_mutex_;
  std::condition_variable start_cv_;
  bool released_{false};
  core::TimePoint epoch_{};
  bool memory_locked_{false};
};

}  // namespace vecu::sched
