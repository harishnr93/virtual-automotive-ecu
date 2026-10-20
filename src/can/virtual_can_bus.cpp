#include "can/virtual_can_bus.hpp"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <utility>
#include <vector>

#include "core/ring_buffer.hpp"

namespace vecu::can {

namespace detail {

class VirtualChannel;

/// State shared by the bus and all its channels.
/// Lock order (to prevent deadlock): bus mutex first, then a channel's queue mutex.
struct VirtualBusCore {
  mutable std::mutex mutex;
  std::vector<VirtualChannel*> open_channels;
};

class VirtualChannel final : public ICanChannel {
 public:
  VirtualChannel(std::shared_ptr<VirtualBusCore> bus, std::string name)
      : bus_(std::move(bus)), name_(std::move(name)) {}

  ~VirtualChannel() override { detach(); }

  VirtualChannel(const VirtualChannel&) = delete;
  VirtualChannel& operator=(const VirtualChannel&) = delete;
  VirtualChannel(VirtualChannel&&) = delete;
  VirtualChannel& operator=(VirtualChannel&&) = delete;

  CanStatus open() override {
    const std::lock_guard<std::mutex> bus_lock(bus_->mutex);
    if (!open_) {
      bus_->open_channels.push_back(this);
      open_ = true;
    }
    return CanStatus::Ok;
  }

  void close() noexcept override { detach(); }

  [[nodiscard]] bool isOpen() const noexcept override { return open_.load(); }

  [[nodiscard]] CanStatus send(const CanFrame& frame) override {
    if (!open_) {
      return CanStatus::NotOpen;
    }
    if (!isValid(frame)) {
      counters_.tx_errors.fetch_add(1U, std::memory_order_relaxed);
      return CanStatus::InvalidFrame;
    }
    {
      const std::lock_guard<std::mutex> bus_lock(bus_->mutex);
      for (VirtualChannel* receiver : bus_->open_channels) {
        if (receiver != this) {
          receiver->deliver(frame);
        }
      }
    }
    counters_.tx_frames.fetch_add(1U, std::memory_order_relaxed);
    return CanStatus::Ok;
  }

  [[nodiscard]] CanStatus receive(CanFrame& frame, std::chrono::microseconds timeout) override {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    if (!open_) {
      return CanStatus::NotOpen;
    }
    (void)queue_cv_.wait_for(lock, timeout, [this] { return !queue_.empty() || !open_; });
    if (queue_.pop(frame)) {
      counters_.rx_frames.fetch_add(1U, std::memory_order_relaxed);
      return CanStatus::Ok;
    }
    return open_ ? CanStatus::Timeout : CanStatus::NotOpen;
  }

  [[nodiscard]] std::string_view name() const noexcept override { return name_; }

  [[nodiscard]] CanChannelStats stats() const noexcept override { return counters_.snapshot(); }

 private:
  /// Non-virtual, so it is safe to call from the destructor.
  void detach() noexcept {
    {
      const std::lock_guard<std::mutex> bus_lock(bus_->mutex);
      auto& channels = bus_->open_channels;
      channels.erase(std::remove(channels.begin(), channels.end(), this), channels.end());
      const std::lock_guard<std::mutex> queue_lock(queue_mutex_);
      open_ = false;
      queue_.clear();
    }
    queue_cv_.notify_all();  // wake a receiver blocked in receive()
  }

  /// Called by the sender with the bus mutex held.
  void deliver(const CanFrame& frame) {
    bool queued = false;
    {
      const std::lock_guard<std::mutex> lock(queue_mutex_);
      queued = queue_.push(frame);
    }
    if (queued) {
      queue_cv_.notify_one();
    } else {
      counters_.rx_overruns.fetch_add(1U, std::memory_order_relaxed);
    }
  }

  std::shared_ptr<VirtualBusCore> bus_;
  std::string name_;
  std::atomic<bool> open_{false};
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  core::RingBuffer<CanFrame, VirtualCanBus::kRxQueueDepth> queue_;
  ChannelCounters counters_;
};

}  // namespace detail

VirtualCanBus::VirtualCanBus() : core_(std::make_shared<detail::VirtualBusCore>()) {}

VirtualCanBus::~VirtualCanBus() = default;

std::unique_ptr<ICanChannel> VirtualCanBus::createChannel(std::string name) {
  return std::make_unique<detail::VirtualChannel>(core_, std::move(name));
}

std::size_t VirtualCanBus::openChannelCount() const {
  const std::lock_guard<std::mutex> lock(core_->mutex);
  return core_->open_channels.size();
}

}  // namespace vecu::can
