#pragma once

#include <array>
#include <cstddef>

namespace vecu::core {

/// Fixed-capacity FIFO queue with storage allocated inside the object.
///
/// No heap allocation after construction, so push/pop have bounded, predictable cost —
/// the usual choice for message queues in embedded software. When full, push() fails
/// and the caller decides the policy (here: drop the new element and count an overrun,
/// like a CAN controller whose receive mailboxes are full).
///
/// Not thread-safe by itself; protect with a mutex when shared between threads.
template <typename T, std::size_t Capacity>
class RingBuffer {
  static_assert(Capacity > 0U, "RingBuffer capacity must be greater than zero");

 public:
  [[nodiscard]] bool push(const T& item) noexcept {
    if (full()) {
      return false;
    }
    storage_[tail_] = item;
    tail_ = (tail_ + 1U) % Capacity;
    ++size_;
    return true;
  }

  [[nodiscard]] bool pop(T& out) noexcept {
    if (empty()) {
      return false;
    }
    out = storage_[head_];
    head_ = (head_ + 1U) % Capacity;
    --size_;
    return true;
  }

  void clear() noexcept {
    head_ = 0U;
    tail_ = 0U;
    size_ = 0U;
  }

  [[nodiscard]] std::size_t size() const noexcept { return size_; }
  [[nodiscard]] bool empty() const noexcept { return size_ == 0U; }
  [[nodiscard]] bool full() const noexcept { return size_ == Capacity; }
  [[nodiscard]] static constexpr std::size_t capacity() noexcept { return Capacity; }

 private:
  std::array<T, Capacity> storage_{};
  std::size_t head_{0U};
  std::size_t tail_{0U};
  std::size_t size_{0U};
};

}  // namespace vecu::core
