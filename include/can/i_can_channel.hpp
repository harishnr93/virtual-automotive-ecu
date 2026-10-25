#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string_view>

#include "can/can_frame.hpp"
#include "can/can_status.hpp"

namespace vecu::can {

/// Snapshot of channel counters (useful for diagnostics and bus-load analysis).
struct CanChannelStats {
  std::uint64_t tx_frames{0U};    ///< frames successfully handed to the bus
  std::uint64_t tx_errors{0U};    ///< send attempts that failed
  std::uint64_t rx_frames{0U};    ///< frames delivered to the application
  std::uint64_t rx_rejected{0U};  ///< received data discarded (malformed, error/RTR frame)
  std::uint64_t rx_overruns{0U};  ///< frames lost because the receive queue was full
                                  ///< (socket backends: kernel drop counter via SO_RXQ_OVFL,
                                  ///<  updated when the next frame is received)
};

/// Application-facing CAN interface (a "CAN driver" abstraction, comparable to the
/// AUTOSAR CanIf/Can driver boundary).
///
/// Everything above this interface (signal packing, COM supervision, diagnostics,
/// applications) is identical regardless of whether frames travel over Linux SocketCAN,
/// the UDP software bus, or the in-memory test bus.
///
/// Thread-safety: send() may be called concurrently from several threads, also while another
/// thread is in receive() (in send(), all backends use only local buffers, one system call per
/// frame, atomic counters and, for the in-memory bus, a mutex). open()/close() must not race
/// with send()/receive(). Covered by the test UdpCanChannel.ConcurrentSendersAreSafe.
class ICanChannel {
 public:
  virtual ~ICanChannel() = default;

  ICanChannel(const ICanChannel&) = delete;
  ICanChannel& operator=(const ICanChannel&) = delete;
  ICanChannel(ICanChannel&&) = delete;
  ICanChannel& operator=(ICanChannel&&) = delete;

  [[nodiscard]] virtual CanStatus open() = 0;
  virtual void close() noexcept = 0;
  [[nodiscard]] virtual bool isOpen() const noexcept = 0;

  /// Non-blocking transmit. Returns TxBufferFull instead of waiting when the bus is busy.
  [[nodiscard]] virtual CanStatus send(const CanFrame& frame) = 0;

  /// Waits up to `timeout` for one frame. A zero timeout polls without waiting.
  /// Like SocketCAN's default, a channel never receives frames that it sent itself.
  [[nodiscard]] virtual CanStatus receive(CanFrame& frame, std::chrono::microseconds timeout) = 0;

  /// Short bus label used in logs, e.g. "vcan0" or "vbus0".
  [[nodiscard]] virtual std::string_view name() const noexcept = 0;

  [[nodiscard]] virtual CanChannelStats stats() const noexcept = 0;

 protected:
  ICanChannel() = default;
};

/// Lock-free counters shared by the channel implementations.
struct ChannelCounters {
  std::atomic<std::uint64_t> tx_frames{0U};
  std::atomic<std::uint64_t> tx_errors{0U};
  std::atomic<std::uint64_t> rx_frames{0U};
  std::atomic<std::uint64_t> rx_rejected{0U};
  std::atomic<std::uint64_t> rx_overruns{0U};

  [[nodiscard]] CanChannelStats snapshot() const noexcept {
    CanChannelStats stats;
    stats.tx_frames = tx_frames.load(std::memory_order_relaxed);
    stats.tx_errors = tx_errors.load(std::memory_order_relaxed);
    stats.rx_frames = rx_frames.load(std::memory_order_relaxed);
    stats.rx_rejected = rx_rejected.load(std::memory_order_relaxed);
    stats.rx_overruns = rx_overruns.load(std::memory_order_relaxed);
    return stats;
  }
};

}  // namespace vecu::can
