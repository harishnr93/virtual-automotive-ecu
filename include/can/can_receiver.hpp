#pragma once

// Event-driven CAN reception, decoupled from the periodic tasks.
//
// On a microcontroller, a CAN receive interrupt copies each frame from the controller's
// mailbox into a software queue; a periodic COM task later processes the queue.
// CanReceiver reproduces that split with a high-priority thread:
//
//   bus ──► CanReceiver thread ──push──► CanRxQueue ──pop──► COM task (20 ms)
//            (ISR analogue,             (bounded,           (decode, supervise)
//             timestamps frames)         overrun counted)

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "can/i_can_channel.hpp"
#include "core/clock.hpp"
#include "core/os_port.hpp"
#include "core/ring_buffer.hpp"

namespace vecu::can {

/// A received frame with its reception timestamps.
struct RxFrame {
  CanFrame frame{};
  core::TimePoint rx_time{};      ///< monotonic, used for timeout supervision
  std::int64_t wall_time_us{0};   ///< wall clock, used for trace files
};

/// Bounded, thread-safe FIFO between the reception thread and the COM task.
class CanRxQueue {
 public:
  static constexpr std::size_t kDepth = 128U;

  /// Returns false (and counts an overrun) if the queue is full; the frame is dropped.
  bool push(const RxFrame& frame) noexcept;
  [[nodiscard]] bool pop(RxFrame& frame) noexcept;
  [[nodiscard]] std::size_t size() const noexcept;
  [[nodiscard]] std::uint64_t overruns() const noexcept { return overruns_.load(std::memory_order_relaxed); }

 private:
  mutable os::PiMutex mutex_;
  core::RingBuffer<RxFrame, kDepth> buffer_;
  std::atomic<std::uint64_t> overruns_{0U};
};

/// Owns a thread that receives frames from a channel and pushes them into a CanRxQueue.
class CanReceiver {
 public:
  CanReceiver(ICanChannel& channel, CanRxQueue& queue) noexcept : channel_(channel), queue_(queue) {}
  ~CanReceiver() { stop(); }

  CanReceiver(const CanReceiver&) = delete;
  CanReceiver& operator=(const CanReceiver&) = delete;
  CanReceiver(CanReceiver&&) = delete;
  CanReceiver& operator=(CanReceiver&&) = delete;

  [[nodiscard]] bool start(const os::ThreadConfig& config);
  /// Returns within one receive poll interval (kPollInterval).
  void stop() noexcept;

  [[nodiscard]] std::uint64_t receivedFrames() const noexcept { return received_.load(std::memory_order_relaxed); }
  [[nodiscard]] std::uint64_t receiveErrors() const noexcept { return errors_.load(std::memory_order_relaxed); }
  [[nodiscard]] os::SchedulingClass schedulingClass() const noexcept { return thread_.schedulingClass(); }

  static constexpr std::chrono::milliseconds kPollInterval{20};

 private:
  void run();

  ICanChannel& channel_;
  CanRxQueue& queue_;
  os::Thread thread_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<std::uint64_t> received_{0U};
  std::atomic<std::uint64_t> errors_{0U};
};

}  // namespace vecu::can
