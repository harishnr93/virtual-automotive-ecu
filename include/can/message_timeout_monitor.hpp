#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>

#include "core/clock.hpp"

namespace vecu::can {

/// Reception state of one periodic message (comparable to AUTOSAR COM "deadline monitoring").
enum class RxTimeoutState : std::uint8_t {
  WaitingFirst,  ///< not yet received since start; the (longer) first-timeout applies
  Ok,            ///< received within its timeout
  TimedOut       ///< no reception for longer than its timeout
};

[[nodiscard]] constexpr const char* toString(RxTimeoutState state) noexcept {
  switch (state) {
    case RxTimeoutState::WaitingFirst:
      return "WaitingFirst";
    case RxTimeoutState::Ok:
      return "Ok";
    case RxTimeoutState::TimedOut:
      return "TimedOut";
  }
  return "Unknown";
}

/// Reported on every state change.
struct RxTimeoutEvent {
  std::uint32_t id;
  RxTimeoutState previous;
  RxTimeoutState current;
  std::chrono::nanoseconds silence;  ///< time since last reception (or since start)
};

/// Supervises the reception of up to `MaxMessages` periodic CAN messages.
///
/// Pure logic: time is passed in by the caller, which makes the class deterministic and
/// unit-testable without sleeping. Storage is a fixed array (no heap allocation).
/// Events are delivered through a callable template parameter (no std::function).
/// Not thread-safe: call from one task only.
template <std::size_t MaxMessages>
class MessageTimeoutMonitor {
 public:
  /// Registers a message. `first_timeout` applies until the first reception after start()
  /// (gives other ECUs time to boot). Returns false if full, duplicate or zero timeouts.
  [[nodiscard]] bool addMessage(std::uint32_t id, std::chrono::milliseconds timeout,
                                std::chrono::milliseconds first_timeout) noexcept {
    if ((count_ >= MaxMessages) || (timeout.count() <= 0) || (first_timeout.count() <= 0) ||
        (find(id) != nullptr)) {
      return false;
    }
    entries_[count_] = Entry{id, timeout, first_timeout, core::TimePoint{}, RxTimeoutState::WaitingFirst, 0U};
    ++count_;
    return true;
  }

  /// (Re)starts supervision: every message waits for its first reception.
  void start(core::TimePoint now) noexcept {
    for (std::size_t i = 0U; i < count_; ++i) {
      entries_[i].last_rx = now;
      entries_[i].state = RxTimeoutState::WaitingFirst;
    }
  }

  /// Call for every received frame. Returns false if the id is not supervised.
  template <typename OnEvent>
  bool notifyReceived(std::uint32_t id, core::TimePoint now, OnEvent&& on_event) {
    Entry* entry = find(id);
    if (entry == nullptr) {
      return false;
    }
    const auto silence = now - entry->last_rx;
    entry->last_rx = now;
    if (entry->state != RxTimeoutState::Ok) {
      const RxTimeoutState previous = entry->state;
      entry->state = RxTimeoutState::Ok;
      on_event(RxTimeoutEvent{id, previous, RxTimeoutState::Ok, silence});
    }
    return true;
  }

  /// Call periodically (e.g. every 10 ms) to detect timeouts.
  template <typename OnEvent>
  void update(core::TimePoint now, OnEvent&& on_event) {
    for (std::size_t i = 0U; i < count_; ++i) {
      Entry& entry = entries_[i];
      if (entry.state == RxTimeoutState::TimedOut) {
        continue;
      }
      const auto limit = (entry.state == RxTimeoutState::WaitingFirst) ? entry.first_timeout : entry.timeout;
      const auto silence = now - entry.last_rx;
      if (silence >= limit) {
        const RxTimeoutState previous = entry.state;
        entry.state = RxTimeoutState::TimedOut;
        ++entry.timeout_count;
        on_event(RxTimeoutEvent{entry.id, previous, RxTimeoutState::TimedOut, silence});
      }
    }
  }

  /// State of a message; unknown ids report TimedOut (fail-safe default).
  [[nodiscard]] RxTimeoutState state(std::uint32_t id) const noexcept {
    const Entry* entry = find(id);
    return (entry != nullptr) ? entry->state : RxTimeoutState::TimedOut;
  }

  [[nodiscard]] bool anyTimedOut() const noexcept {
    for (std::size_t i = 0U; i < count_; ++i) {
      if (entries_[i].state == RxTimeoutState::TimedOut) {
        return true;
      }
    }
    return false;
  }

  [[nodiscard]] std::uint32_t timeoutCount(std::uint32_t id) const noexcept {
    const Entry* entry = find(id);
    return (entry != nullptr) ? entry->timeout_count : 0U;
  }

  [[nodiscard]] std::size_t size() const noexcept { return count_; }

 private:
  struct Entry {
    std::uint32_t id{0U};
    std::chrono::milliseconds timeout{0};
    std::chrono::milliseconds first_timeout{0};
    core::TimePoint last_rx{};
    RxTimeoutState state{RxTimeoutState::WaitingFirst};
    std::uint32_t timeout_count{0U};
  };

  [[nodiscard]] Entry* find(std::uint32_t id) noexcept {
    for (std::size_t i = 0U; i < count_; ++i) {
      if (entries_[i].id == id) {
        return &entries_[i];
      }
    }
    return nullptr;
  }
  [[nodiscard]] const Entry* find(std::uint32_t id) const noexcept {
    for (std::size_t i = 0U; i < count_; ++i) {
      if (entries_[i].id == id) {
        return &entries_[i];
      }
    }
    return nullptr;
  }

  std::array<Entry, MaxMessages> entries_{};
  std::size_t count_{0U};
};

}  // namespace vecu::can
