#pragma once

// Phase 1 ECU runtime helpers.
//
// runCooperativeLoop() is a deliberately simple single-threaded "super loop": it services
// CAN reception until the next 10 ms tick, then runs the cyclic function. Phase 2 replaces
// it with the multi-rate ECU scheduler (10/20/100 ms tasks with timing measurement).

#include <chrono>
#include <cstdint>
#include <functional>
#include <string>

#include "can/can_frame.hpp"
#include "can/can_log_writer.hpp"
#include "can/i_can_channel.hpp"
#include "can/message_timeout_monitor.hpp"
#include "core/clock.hpp"
#include "core/logger.hpp"

namespace vecu::app {

/// Wraps a channel and an optional trace: every transmitted and received frame is
/// counted and written to the candump log.
class CanIo {
 public:
  CanIo(can::ICanChannel& channel, can::CanLogWriter& trace) : channel_(channel), trace_(trace) {}

  /// Sends a frame; failures are counted, not fatal (a real ECU keeps running).
  bool send(const can::CanFrame& frame);
  void traceReceived(const can::CanFrame& frame);

  [[nodiscard]] std::uint64_t txOk() const noexcept { return tx_ok_; }
  [[nodiscard]] std::uint64_t txFailed() const noexcept { return tx_failed_; }
  [[nodiscard]] can::ICanChannel& channel() noexcept { return channel_; }

 private:
  can::ICanChannel& channel_;
  can::CanLogWriter& trace_;
  std::uint64_t tx_ok_{0U};
  std::uint64_t tx_failed_{0U};
};

/// Measured behaviour of the cooperative loop (real measurements, not guarantees).
struct LoopTiming {
  std::uint64_t ticks{0U};
  std::uint64_t late_ticks{0U};  ///< tick started more than one period late
  std::chrono::nanoseconds max_lateness{0};
  std::uint64_t rx_errors{0U};
};

using RxHandler = std::function<void(const can::CanFrame&, core::TimePoint)>;
using TickHandler = std::function<void(core::TimePoint, std::uint64_t)>;

/// Runs until shutdown is requested or `duration_s` (> 0) elapses.
[[nodiscard]] LoopTiming runCooperativeLoop(can::ICanChannel& channel,
                                            std::chrono::milliseconds tick_period, double duration_s,
                                            const RxHandler& on_rx, const TickHandler& on_tick);

/// Logs a reception-supervision event in a stable, machine-parseable form, e.g.
///   RX_TIMEOUT id=0x100 silence_ms=101.3
void logRxTimeoutEvent(const core::Logger& log, const can::RxTimeoutEvent& event);

/// Formats a value with a fixed number of decimals.
[[nodiscard]] std::string fixed(double value, int decimals);

}  // namespace vecu::app
