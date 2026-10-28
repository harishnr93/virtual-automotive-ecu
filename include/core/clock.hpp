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

/// Wall-clock time that never jumps: the wall clock read once (at the first call) plus the
/// monotonic time elapsed since then. Use it for timestamps from which intervals are later
/// measured (CAN traces, aligning processes): the real wall clock can be stepped by NTP, a user
/// or WSL2's time re-sync with Windows (a 0.85 s forward step was observed during a 17 s run),
/// which would appear as a gap in a trace. Price: over a long run it may drift from the real
/// wall clock by the corrections it ignores.
[[nodiscard]] inline std::int64_t steadyWallMicros() noexcept {
  static const std::int64_t wall_anchor = wallClockMicros();
  static const TimePoint monotonic_anchor = now();
  return wall_anchor + std::chrono::duration_cast<std::chrono::microseconds>(now() - monotonic_anchor).count();
}

/// Converts a duration to floating-point milliseconds (for reporting only).
template <typename Rep, typename Period>
[[nodiscard]] constexpr double toMillis(std::chrono::duration<Rep, Period> duration) noexcept {
  return std::chrono::duration<double, std::milli>(duration).count();
}

}  // namespace vecu::core
