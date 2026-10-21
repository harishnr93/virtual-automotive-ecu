#include "can/can_receiver.hpp"

#include <mutex>

namespace vecu::can {

bool CanRxQueue::push(const RxFrame& frame) noexcept {
  bool queued = false;
  {
    const std::lock_guard<os::PiMutex> lock(mutex_);
    queued = buffer_.push(frame);
  }
  if (!queued) {
    overruns_.fetch_add(1U, std::memory_order_relaxed);
  }
  return queued;
}

bool CanRxQueue::pop(RxFrame& frame) noexcept {
  const std::lock_guard<os::PiMutex> lock(mutex_);
  return buffer_.pop(frame);
}

std::size_t CanRxQueue::size() const noexcept {
  const std::lock_guard<os::PiMutex> lock(mutex_);
  return buffer_.size();
}

bool CanReceiver::start(const os::ThreadConfig& config) {
  if (thread_.joinable()) {
    return false;
  }
  stop_requested_ = false;
  return thread_.start(config, [this] { run(); });
}

void CanReceiver::stop() noexcept {
  stop_requested_ = true;
  thread_.join();
}

void CanReceiver::run() {
  while (!stop_requested_.load(std::memory_order_acquire)) {
    RxFrame entry;
    const CanStatus status = channel_.receive(entry.frame, kPollInterval);
    if (status == CanStatus::Ok) {
      entry.rx_time = core::now();
      entry.wall_time_us = core::wallClockMicros();
      received_.fetch_add(1U, std::memory_order_relaxed);
      (void)queue_.push(entry);  // overruns are counted by the queue
    } else if ((status != CanStatus::Timeout) && (status != CanStatus::InvalidFrame)) {
      errors_.fetch_add(1U, std::memory_order_relaxed);
      os::sleepUntil(core::now() + kPollInterval);  // avoid a busy loop on a persistent error
    } else {
      // timeout or ignored frame
    }
  }
}

}  // namespace vecu::can
