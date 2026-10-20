#include "common/ecu_runtime.hpp"

#include <array>
#include <cstdio>
#include <thread>

#include "common/shutdown_signal.hpp"

namespace vecu::app {

bool CanIo::send(const can::CanFrame& frame) {
  const can::CanStatus status = channel_.send(frame);
  if (status != can::CanStatus::Ok) {
    ++tx_failed_;
    return false;
  }
  ++tx_ok_;
  trace_.write(frame, core::wallClockMicros());
  return true;
}

void CanIo::traceReceived(const can::CanFrame& frame) { trace_.write(frame, core::wallClockMicros()); }

LoopTiming runCooperativeLoop(can::ICanChannel& channel, std::chrono::milliseconds tick_period,
                              double duration_s, const RxHandler& on_rx, const TickHandler& on_tick) {
  LoopTiming timing;
  const core::TimePoint start = core::now();
  const bool bounded = duration_s > 0.0;
  const core::TimePoint end =
      start + std::chrono::duration_cast<core::MonotonicClock::duration>(std::chrono::duration<double>(duration_s));
  core::TimePoint next_tick = start;

  while (!shutdownRequested()) {
    core::TimePoint now = core::now();
    if (bounded && (now >= end)) {
      break;
    }

    if (now >= next_tick) {
      const auto lateness = now - next_tick;
      if (lateness > timing.max_lateness) {
        timing.max_lateness = lateness;
      }
      if (lateness > tick_period) {
        ++timing.late_ticks;
      }
      on_tick(now, timing.ticks);
      ++timing.ticks;
      next_tick += tick_period;
      // After a long stall, resynchronise instead of running a burst of catch-up ticks.
      if ((now - next_tick) > (tick_period * 5)) {
        next_tick = now + tick_period;
      }
      continue;
    }

    // Service CAN reception until the next tick is due.
    can::CanFrame frame;
    const auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(next_tick - now);
    const can::CanStatus status = channel.receive(frame, remaining);
    if (status == can::CanStatus::Ok) {
      on_rx(frame, core::now());
    } else if ((status != can::CanStatus::Timeout) && (status != can::CanStatus::InvalidFrame)) {
      ++timing.rx_errors;
      std::this_thread::sleep_until(next_tick);  // avoid a busy loop on a persistent error
    } else {
      // timeout or ignored frame: nothing to do
    }
  }
  return timing;
}

void logRxTimeoutEvent(const core::Logger& log, const can::RxTimeoutEvent& event) {
  const std::string silence_ms = fixed(core::toMillis(event.silence), 1);
  if (event.current == can::RxTimeoutState::TimedOut) {
    log.warning(core::strCat("RX_TIMEOUT id=", can::formatId(event.id), " silence_ms=", silence_ms,
                             " previous=", can::toString(event.previous)));
  } else if (event.previous == can::RxTimeoutState::TimedOut) {
    log.info(core::strCat("RX_RECOVERED id=", can::formatId(event.id), " gap_ms=", silence_ms));
  } else {
    log.info(core::strCat("RX_FIRST id=", can::formatId(event.id), " after_ms=", silence_ms));
  }
}

std::string fixed(double value, int decimals) {
  std::array<char, 32U> buffer{};
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
  (void)std::snprintf(buffer.data(), buffer.size(), "%.*f", decimals, value);
  return std::string{buffer.data()};
}

}  // namespace vecu::app
