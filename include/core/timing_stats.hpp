#pragma once

// Timing analysis for periodic tasks. Pure logic: all timestamps are passed in, so every
// formula here is unit-tested with synthetic time (no sleeping, no flaky tests).
//
// Per activation k of a task with period T and deadline D:
//
//   release r_k ──── start s_k ═════ execution ═════ end e_k
//   |<-- start latency -->|<---- execution time ---->|
//   |<---------------- response time --------------->|   deadline miss if > D
//
//   start latency   = s_k - r_k        (how late the OS started the task: "release jitter")
//   execution time  = e_k - s_k        (includes preemption by other threads)
//   response time   = e_k - r_k
//   period jitter   = |(s_k - s_{k-1}) - (r_k - r_{k-1})|  (variation of the start interval)

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "core/clock.hpp"

namespace vecu::sched {

using Nanos = std::chrono::nanoseconds;

/// Running min/max/mean/standard deviation (Welford's algorithm: O(1), numerically stable,
/// no storage per sample).
class DurationStats {
 public:
  void add(Nanos sample) noexcept;

  [[nodiscard]] std::uint64_t count() const noexcept { return count_; }
  [[nodiscard]] Nanos min() const noexcept { return Nanos{min_ns_}; }
  [[nodiscard]] Nanos max() const noexcept { return Nanos{max_ns_}; }
  [[nodiscard]] double meanNs() const noexcept { return mean_ns_; }
  [[nodiscard]] double stddevNs() const noexcept;

 private:
  std::uint64_t count_{0U};
  std::int64_t min_ns_{0};
  std::int64_t max_ns_{0};
  double mean_ns_{0.0};
  double m2_{0.0};
};

/// Keeps the most recent N samples (fixed storage) for percentile estimation.
template <std::size_t N>
class SampleWindow {
  static_assert(N > 0U, "window must not be empty");

 public:
  void add(Nanos sample) noexcept {
    samples_[next_] = sample.count();
    next_ = (next_ + 1U) % N;
    if (size_ < N) {
      ++size_;
    }
  }

  [[nodiscard]] std::size_t size() const noexcept { return size_; }

  /// Nearest-rank percentile, p in [0, 100]. Allocates and sorts a copy: call from a
  /// reporting context, never from a time-critical task.
  [[nodiscard]] Nanos percentile(double p) const {
    if (size_ == 0U) {
      return Nanos{0};
    }
    std::vector<std::int64_t> sorted(samples_.begin(), samples_.begin() + static_cast<std::ptrdiff_t>(size_));
    std::sort(sorted.begin(), sorted.end());
    const double clamped = std::clamp(p, 0.0, 100.0);
    auto rank = static_cast<std::size_t>((clamped / 100.0) * static_cast<double>(size_) + 0.999999);
    rank = std::clamp<std::size_t>(rank, 1U, size_);
    return Nanos{sorted[rank - 1U]};
  }

 private:
  std::array<std::int64_t, N> samples_{};
  std::size_t next_{0U};
  std::size_t size_{0U};
};

inline constexpr std::size_t kTimingWindow = 8192U;  ///< samples kept for percentiles

struct ActivationRecord {
  core::TimePoint release;
  core::TimePoint start;
  core::TimePoint end;
};

struct ActivationResult {
  bool deadline_missed{false};
  bool budget_exceeded{false};
};

/// Summary of one task's timing (plain data, safe to copy between threads).
struct TaskTimingSummary {
  std::uint64_t activations{0U};
  std::uint64_t deadline_misses{0U};
  std::uint64_t budget_overruns{0U};
  std::uint64_t skipped_activations{0U};
  DurationStats execution_time{};
  DurationStats start_latency{};
  DurationStats response_time{};
  DurationStats period_jitter{};
  Nanos execution_p50{0};
  Nanos execution_p99{0};
  Nanos start_latency_p50{0};
  Nanos start_latency_p99{0};
};

/// Collects the timing of every activation of one task and detects violations.
/// Not thread-safe; the scheduler protects it with a mutex.
class TaskTimingMonitor {
 public:
  /// `budget` = 0 disables the execution-time budget check.
  TaskTimingMonitor(Nanos deadline, Nanos budget) noexcept;

  ActivationResult record(const ActivationRecord& activation) noexcept;
  void addSkipped(std::uint64_t count) noexcept { summary_.skipped_activations += count; }

  /// Summary including percentiles (allocates; reporting context only).
  [[nodiscard]] TaskTimingSummary summary() const;

 private:
  Nanos deadline_;
  Nanos budget_;
  TaskTimingSummary summary_{};
  bool has_previous_{false};
  core::TimePoint previous_release_{};
  core::TimePoint previous_start_{};
  SampleWindow<kTimingWindow> execution_samples_{};
  SampleWindow<kTimingWindow> latency_samples_{};
};

/// Result of the activation policy after one execution.
struct NextRelease {
  core::TimePoint release{};
  std::uint64_t skipped{0U};  ///< activations lost because the task was still running
};

/// OSEK/AUTOSAR-like policy with an activation limit of 1: if the task overran its period,
/// one pending activation is kept (it starts immediately, late), any further activations
/// that fell into the overrun are dropped and counted. The period grid (phase) is kept.
[[nodiscard]] NextRelease computeNextRelease(core::TimePoint current_release, core::TimePoint end,
                                             Nanos period) noexcept;

}  // namespace vecu::sched
