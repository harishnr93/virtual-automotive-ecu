#pragma once

#include <cstdint>
#include <mutex>
#include <type_traits>

#include "core/clock.hpp"
#include "core/os_port.hpp"

namespace vecu::core {

/// Thread-safe "latest value" buffer for data exchanged between tasks of different rates
/// (comparable to an AUTOSAR RTE sender-receiver port with last-is-best semantics).
///
/// - The writer replaces the value; readers always get a complete, consistent copy
///   (never half old / half new — "torn" data).
/// - Each sample carries a timestamp and a sequence number, so a reader can detect stale
///   data (age) or missed updates (sequence gaps).
/// - Critical sections are a few bytes of copying; the mutex uses priority inheritance.
/// - Restricted to trivially copyable types: copying cannot allocate or throw.
template <typename T>
class SharedSignal {
  static_assert(std::is_trivially_copyable_v<T>, "SharedSignal requires a trivially copyable type");

 public:
  struct Sample {
    T value{};
    TimePoint timestamp{};
    std::uint64_t sequence{0U};  ///< 0 = never written
  };

  SharedSignal() = default;
  explicit SharedSignal(const T& initial) { sample_.value = initial; }

  void write(const T& value, TimePoint now) noexcept {
    const std::lock_guard<os::PiMutex> lock(mutex_);
    sample_.value = value;
    sample_.timestamp = now;
    ++sample_.sequence;
  }

  [[nodiscard]] Sample read() const noexcept {
    const std::lock_guard<os::PiMutex> lock(mutex_);
    return sample_;
  }

 private:
  mutable os::PiMutex mutex_;
  Sample sample_{};
};

}  // namespace vecu::core
