#pragma once

#include <chrono>
#include <cstdint>

namespace vecu::core {

/// All control and supervision timing uses the monotonic clock: it never jumps when the
/// wall-clock time is adjusted (NTP, user changes), which is essential for timeouts.
using MonotonicClock = std::chrono::steady_clock;
using TimePoint = MonotonicClock::time_point;

[[nodiscard]] inline TimePoint now() noexcept { return MonotonicClock::now(); }

/// Wall-clock time in microseconds since the Unix epoch.
/// Used only to timestamp log files (so they can be correlated with other tools),
/// never for timing decisions.
[[nodiscard]] inline std::int64_t wallClockMicros() noexcept {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

/// Converts a duration to floating-point milliseconds (for reporting only).
template <typename Rep, typename Period>
[[nodiscard]] constexpr double toMillis(std::chrono::duration<Rep, Period> duration) noexcept {
  return std::chrono::duration<double, std::milli>(duration).count();
}

}  // namespace vecu::core
