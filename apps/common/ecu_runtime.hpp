#pragma once

// Runtime shared by the virtual ECU executables (Phase 2).
//
// Execution model of every ECU:
//
//   CAN RX thread (prio 90, event-driven)  ──► CanRxQueue
//   PeriodicScheduler tasks (rate-monotonic priorities), e.g.
//     Control_10ms (80) · Dcm_10ms (75) · Com_20ms (70) · Diag_100ms (50)
//   Watchdog thread (prio 95, 20 ms): software watchdog supervision of all tasks
//   main thread: start-up, checks that the watchdog thread itself is alive, shutdown, reports
//
// Priority values only take effect when real-time scheduling is granted (--realtime and
// rtprio permission); otherwise all threads use SCHED_OTHER and this is reported.

#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <cstdint>
#include <memory>
#include <string>

#include "can/can_frame.hpp"
#include "can/can_log_writer.hpp"
#include "can/can_receiver.hpp"
#include "can/i_can_channel.hpp"
#include "can/message_timeout_monitor.hpp"
#include "common/app_options.hpp"
#include "common/fault_injector.hpp"
#include "core/logger.hpp"
#include "core/os_port.hpp"
#include "core/periodic_scheduler.hpp"
#include "watchdog/watchdog_supervisor.hpp"

namespace vecu::app {

inline constexpr int kPriorityWatchdog = 95;
inline constexpr int kPriorityCanRx = 90;
inline constexpr int kPriority10ms = 80;
inline constexpr int kPriorityDcm10ms = 75;  ///< diagnostics, 10 ms (below control)
inline constexpr int kPriority20ms = 70;
inline constexpr int kPriority100ms = 50;

/// Transmit path with counting and tracing. Thread-safe: since Phase 4 both Com_20ms and
/// Dcm_10ms transmit (channel send() is concurrency-safe, counters are atomic, the trace
/// writer locks a mutex).
class CanIo {
 public:
  CanIo(can::ICanChannel& channel, can::CanLogWriter& trace) : channel_(channel), trace_(trace) {}

  bool send(const can::CanFrame& frame);
  void traceReceived(const can::RxFrame& entry);

  [[nodiscard]] std::uint64_t txOk() const noexcept { return tx_ok_.load(std::memory_order_relaxed); }
  [[nodiscard]] std::uint64_t txFailed() const noexcept { return tx_failed_.load(std::memory_order_relaxed); }

 private:
  can::ICanChannel& channel_;
  can::CanLogWriter& trace_;
  std::atomic<std::uint64_t> tx_ok_{0U};
  std::atomic<std::uint64_t> tx_failed_{0U};
};

/// Callbacks for the watchdog recovery sequence (all called from the watchdog thread):
///   on_expired  - supervision expired: enter the safe state immediately
///   on_resumed  - the stalled task(s) run again: perform the recovery action (e.g. reset)
///   on_healthy  - 200 ms of healthy supervision after recovery: leave the safe state
struct WatchdogHandlers {
  std::function<void(const watchdog::CheckResult&, const std::string& entity)> on_expired;
  std::function<void()> on_resumed;
  std::function<void()> on_healthy;
};

/// Owns the platform services of one virtual ECU: CAN channel, trace, RX thread + queue,
/// task scheduler, software watchdog and (test-only) fault injection. The ECU application
/// registers its tasks, then calls run().
class EcuPlatform {
 public:
  EcuPlatform(std::string ecu_name, const AppOptions& options);
  ~EcuPlatform();

  EcuPlatform(const EcuPlatform&) = delete;
  EcuPlatform& operator=(const EcuPlatform&) = delete;
  EcuPlatform(EcuPlatform&&) = delete;
  EcuPlatform& operator=(EcuPlatform&&) = delete;

  /// Opens the CAN channel and trace file. Returns false (after logging) on failure.
  [[nodiscard]] bool init();

  /// Registers a periodic task and its watchdog supervision (heartbeat timeout = 3 periods,
  /// max. execution = 2 periods, tolerance = 2 supervision cycles); false on invalid config.
  [[nodiscard]] bool addTask(sched::TaskConfig config, sched::TaskFunction function);

  /// Starts the RX thread and the scheduler, blocks until Ctrl+C / SIGTERM or the configured
  /// duration, stops everything and prints the timing report. Returns the process exit code.
  [[nodiscard]] int run();

  [[nodiscard]] CanIo& io() noexcept { return *io_; }
  [[nodiscard]] can::CanRxQueue& rxQueue() noexcept { return rx_queue_; }
  [[nodiscard]] can::CanRxQueue& diagRxQueue() noexcept { return diag_rx_queue_; }

  /// Routes a CAN identifier to diagRxQueue() (call after init(), before run()).
  [[nodiscard]] bool routeToDiagnostics(std::uint32_t id);
  [[nodiscard]] const sched::PeriodicScheduler& scheduler() const noexcept { return scheduler_; }
  [[nodiscard]] std::string_view busName() const noexcept;
  [[nodiscard]] const core::Logger& log() const noexcept { return log_; }

  /// Called from a diagnostic task: logs "DEADLINE_MISS" for tasks whose miss counter grew.
  void reportNewDeadlineMisses();

  void setWatchdogHandlers(WatchdogHandlers handlers) { watchdog_handlers_ = std::move(handlers); }
  /// Called on shutdown (emergency = true when terminating after a watchdog expiration).
  void setShutdownHook(std::function<void(bool emergency)> hook) { shutdown_hook_ = std::move(hook); }
  /// Terminates immediately like a hardware reset: runs the shutdown hook (e.g. NVM write),
  /// then std::_Exit without joining threads (a hung task thread cannot be joined).
  [[noreturn]] void emergencyExit(int exit_code);

  [[nodiscard]] const watchdog::WatchdogSupervisor& watchdog() const noexcept { return watchdog_; }
  [[nodiscard]] FaultInjector& injector() noexcept { return injector_; }
  [[nodiscard]] const AppOptions& options() const noexcept { return options_; }
  /// Seconds since the scheduler epoch for a task activation (used for injections).
  [[nodiscard]] double secondsSinceEpoch(core::TimePoint t) const noexcept;

  static constexpr std::chrono::milliseconds kWatchdogPeriod{20};
  static constexpr std::chrono::milliseconds kWatchdogProbation{200};

 private:
  std::string ecu_name_;
  AppOptions options_;
  core::Logger log_;
  std::unique_ptr<can::ICanChannel> channel_;
  can::CanLogWriter trace_;
  std::unique_ptr<CanIo> io_;
  can::CanRxQueue rx_queue_;
  can::CanRxQueue diag_rx_queue_;
  std::unique_ptr<can::CanReceiver> receiver_;
  sched::PeriodicScheduler scheduler_;
  std::array<std::uint64_t, sched::kMaxTasks> reported_misses_{};

  void runWatchdog();
  [[nodiscard]] bool resumedAfter(const watchdog::CheckResult& expired, core::TimePoint since) const;

  watchdog::WatchdogSupervisor watchdog_;
  WatchdogHandlers watchdog_handlers_;
  os::Thread watchdog_thread_;
  std::atomic<bool> watchdog_stop_{false};
  std::atomic<std::int64_t> watchdog_last_check_ns_{0};
  std::function<void(bool)> shutdown_hook_;
  FaultInjector injector_;
};

/// Logs a reception-supervision event in a stable, machine-parseable form, e.g.
///   RX_TIMEOUT id=0x100 silence_ms=101.3
void logRxTimeoutEvent(const core::Logger& log, const can::RxTimeoutEvent& event);

/// Formats a value with a fixed number of decimals.
[[nodiscard]] std::string fixed(double value, int decimals);

}  // namespace vecu::app
