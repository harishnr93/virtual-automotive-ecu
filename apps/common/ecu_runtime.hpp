#pragma once

// Runtime shared by the virtual ECU executables (Phase 2).
//
// Execution model of every ECU:
//
//   CAN RX thread (prio 90, event-driven)  ──► CanRxQueue
//   PeriodicScheduler tasks (rate-monotonic priorities), e.g.
//     Control_10ms (80) · Com_20ms (70) · Diag_100ms (50)
//   main thread: start-up, waits for shutdown, prints the timing report
//
// Priority values only take effect when real-time scheduling is granted (--realtime and
// rtprio permission); otherwise all threads use SCHED_OTHER and this is reported.

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include "can/can_frame.hpp"
#include "can/can_log_writer.hpp"
#include "can/can_receiver.hpp"
#include "can/i_can_channel.hpp"
#include "can/message_timeout_monitor.hpp"
#include "common/app_options.hpp"
#include "core/logger.hpp"
#include "core/periodic_scheduler.hpp"

namespace vecu::app {

inline constexpr int kPriorityCanRx = 90;
inline constexpr int kPriority10ms = 80;
inline constexpr int kPriority20ms = 70;
inline constexpr int kPriority100ms = 50;

/// Transmit path with counting and tracing. send() is called from one task only;
/// the counters are atomic so other tasks can read them.
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

/// Owns the platform services of one virtual ECU: CAN channel, trace, RX thread + queue,
/// and the task scheduler. The ECU application registers its tasks, then calls run().
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

  /// Registers a periodic task; logs and returns false on invalid configuration.
  [[nodiscard]] bool addTask(sched::TaskConfig config, sched::TaskFunction function);

  /// Starts the RX thread and the scheduler, blocks until Ctrl+C / SIGTERM or the configured
  /// duration, stops everything and prints the timing report. Returns the process exit code.
  [[nodiscard]] int run();

  [[nodiscard]] CanIo& io() noexcept { return *io_; }
  [[nodiscard]] can::CanRxQueue& rxQueue() noexcept { return rx_queue_; }
  [[nodiscard]] const sched::PeriodicScheduler& scheduler() const noexcept { return scheduler_; }
  [[nodiscard]] std::string_view busName() const noexcept;
  [[nodiscard]] const core::Logger& log() const noexcept { return log_; }

  /// Called from a diagnostic task: logs "DEADLINE_MISS" for tasks whose miss counter grew.
  void reportNewDeadlineMisses();

 private:
  std::string ecu_name_;
  AppOptions options_;
  core::Logger log_;
  std::unique_ptr<can::ICanChannel> channel_;
  can::CanLogWriter trace_;
  std::unique_ptr<CanIo> io_;
  can::CanRxQueue rx_queue_;
  std::unique_ptr<can::CanReceiver> receiver_;
  sched::PeriodicScheduler scheduler_;
  std::array<std::uint64_t, sched::kMaxTasks> reported_misses_{};
};

/// Logs a reception-supervision event in a stable, machine-parseable form, e.g.
///   RX_TIMEOUT id=0x100 silence_ms=101.3
void logRxTimeoutEvent(const core::Logger& log, const can::RxTimeoutEvent& event);

/// Formats a value with a fixed number of decimals.
[[nodiscard]] std::string fixed(double value, int decimals);

}  // namespace vecu::app
