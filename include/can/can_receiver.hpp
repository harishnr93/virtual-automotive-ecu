#pragma once

// Event-driven CAN reception, decoupled from the periodic tasks.
//
// On a microcontroller, a CAN receive interrupt copies each frame from the controller's
// mailbox into a software queue; a periodic COM task later processes the queue.
// CanReceiver reproduces that split with a high-priority thread:
//
//   bus ──► CanReceiver thread ──push──► CanRxQueue ──pop──► COM task (20 ms)
//            (ISR analogue,             (bounded,           (decode, supervise)
//             timestamps frames,         overrun counted)
//             routes by CAN ID) ───────► diagnostic queue ──► Dcm task (10 ms)

#include <array>
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

  /// Routes frames with identifier `id` to `queue` instead of the default queue (like AUTOSAR
  /// CanIf routing, e.g. diagnostic requests to the diagnostic task). Before start() only.
  [[nodiscard]] bool addRoute(std::uint32_t id, CanRxQueue& queue);

  [[nodiscard]] bool start(const os::ThreadConfig& config);
  /// Returns within one receive poll interval (kPollInterval).
  void stop() noexcept;

  [[nodiscard]] std::uint64_t receivedFrames() const noexcept { return received_.load(std::memory_order_relaxed); }
  [[nodiscard]] std::uint64_t receiveErrors() const noexcept { return errors_.load(std::memory_order_relaxed); }
  [[nodiscard]] os::SchedulingClass schedulingClass() const noexcept { return thread_.schedulingClass(); }

  static constexpr std::chrono::milliseconds kPollInterval{20};
  static constexpr std::size_t kMaxRoutes = 4U;

 private:
  struct Route {
    std::uint32_t id{0U};
    CanRxQueue* queue{nullptr};
  };

  void run();
  [[nodiscard]] CanRxQueue& queueFor(std::uint32_t id) noexcept;

  ICanChannel& channel_;
  CanRxQueue& queue_;
  std::array<Route, kMaxRoutes> routes_{};
  std::size_t route_count_{0U};
  os::Thread thread_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<std::uint64_t> received_{0U};
  std::atomic<std::uint64_t> errors_{0U};
};

}  // namespace vecu::can
