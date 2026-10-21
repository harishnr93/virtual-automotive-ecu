#pragma once

// OS abstraction layer ("OS port") — POSIX / Linux implementation.
//
// The scheduler and the ECU tasks use only the primitives declared here, never pthreads
// directly. Porting the platform to an RTOS means re-implementing this header and
// src/core/os_port_posix.cpp, e.g. for FreeRTOS:
//   Thread::start  -> xTaskCreate (priority from ThreadConfig)
//   sleepUntil     -> vTaskDelayUntil
//   PiMutex        -> xSemaphoreCreateMutex (FreeRTOS mutexes use priority inheritance)
//   lockProcessMemory -> no-op (no virtual memory on an MCU)

#include <pthread.h>

#include <cstdint>
#include <functional>
#include <string>

#include "core/clock.hpp"

namespace vecu::os {

inline constexpr int kMinPriority = 1;   ///< lowest real-time priority (SCHED_FIFO)
inline constexpr int kMaxPriority = 99;  ///< highest real-time priority

enum class SchedulingClass : std::uint8_t {
  RealTimeFifo,  ///< SCHED_FIFO: fixed priority, preemptive (needs CAP_SYS_NICE / rtprio limit)
  Normal         ///< SCHED_OTHER: Linux fair scheduler, priority value has no effect
};

[[nodiscard]] const char* toString(SchedulingClass scheduling_class) noexcept;

struct ThreadConfig {
  std::string name;               ///< shown in top/htop/gdb (truncated to 15 characters)
  int priority{kMinPriority};     ///< 1 (lowest) .. 99 (highest); used with SCHED_FIFO only
  bool request_realtime{false};   ///< try SCHED_FIFO, fall back to Normal if not permitted
};

/// RAII thread. The destructor joins, so a thread can never outlive its owner.
class Thread {
 public:
  using Entry = std::function<void()>;

  Thread() = default;
  ~Thread();

  Thread(const Thread&) = delete;
  Thread& operator=(const Thread&) = delete;
  Thread(Thread&&) = delete;
  Thread& operator=(Thread&&) = delete;

  /// Creates the thread. If real-time scheduling was requested but is not permitted, the
  /// thread is created with normal scheduling; schedulingClass() and realtimeError() report it.
  [[nodiscard]] bool start(const ThreadConfig& config, Entry entry);
  void join() noexcept;
  [[nodiscard]] bool joinable() const noexcept { return started_; }

  [[nodiscard]] SchedulingClass schedulingClass() const noexcept { return scheduling_class_; }
  /// Error code of the failed SCHED_FIFO request (e.g. EPERM), 0 if not requested or granted.
  [[nodiscard]] int realtimeError() const noexcept { return realtime_error_; }

 private:
  pthread_t handle_{};
  bool started_{false};
  SchedulingClass scheduling_class_{SchedulingClass::Normal};
  int realtime_error_{0};
};

/// Sleeps until an absolute point on the monotonic clock. Absolute wake-up times prevent
/// the cumulative drift that relative sleeps ("sleep 10 ms") would add every period.
void sleepUntil(core::TimePoint wake_time) noexcept;

/// Mutex with the priority-inheritance protocol: while a low-priority thread holds the lock,
/// it temporarily runs at the priority of the highest waiter. This bounds "priority
/// inversion" (the failure mode of the Mars Pathfinder mission). Satisfies the standard
/// Lockable requirements, so it works with std::lock_guard / std::unique_lock.
class PiMutex {
 public:
  PiMutex() noexcept;
  ~PiMutex();

  PiMutex(const PiMutex&) = delete;
  PiMutex& operator=(const PiMutex&) = delete;
  PiMutex(PiMutex&&) = delete;
  PiMutex& operator=(PiMutex&&) = delete;

  void lock() noexcept;
  void unlock() noexcept;
  [[nodiscard]] bool try_lock() noexcept;  // NOLINT(readability-identifier-naming): std Lockable API

  /// True if the OS accepted the priority-inheritance protocol.
  [[nodiscard]] bool priorityInheritance() const noexcept { return priority_inheritance_; }

 private:
  pthread_mutex_t mutex_{};
  bool priority_inheritance_{false};
};

/// Locks all current and future pages in RAM (mlockall) so page faults cannot delay tasks.
/// Returns false (and leaves memory unlocked) if not permitted.
[[nodiscard]] bool lockProcessMemory() noexcept;

}  // namespace vecu::os
