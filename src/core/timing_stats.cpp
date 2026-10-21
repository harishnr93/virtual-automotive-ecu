#include "core/timing_stats.hpp"

#include <cmath>

namespace vecu::sched {

void DurationStats::add(Nanos sample) noexcept {
  const std::int64_t value = sample.count();
  if ((count_ == 0U) || (value < min_ns_)) {
    min_ns_ = value;
  }
  if ((count_ == 0U) || (value > max_ns_)) {
    max_ns_ = value;
  }
  ++count_;
  const double delta = static_cast<double>(value) - mean_ns_;
  mean_ns_ += delta / static_cast<double>(count_);
  m2_ += delta * (static_cast<double>(value) - mean_ns_);
}

double DurationStats::stddevNs() const noexcept {
  return (count_ < 2U) ? 0.0 : std::sqrt(m2_ / static_cast<double>(count_ - 1U));
}

TaskTimingMonitor::TaskTimingMonitor(Nanos deadline, Nanos budget) noexcept
    : deadline_(deadline), budget_(budget) {}

ActivationResult TaskTimingMonitor::record(const ActivationRecord& activation) noexcept {
  ActivationResult result;
  const Nanos latency = activation.start - activation.release;
  const Nanos execution = activation.end - activation.start;
  const Nanos response = activation.end - activation.release;

  ++summary_.activations;
  summary_.start_latency.add(latency);
  summary_.execution_time.add(execution);
  summary_.response_time.add(response);
  latency_samples_.add(latency);
  execution_samples_.add(execution);

  if (has_previous_) {
    const Nanos actual_interval = activation.start - previous_start_;
    const Nanos planned_interval = activation.release - previous_release_;
    const Nanos deviation = actual_interval - planned_interval;
    summary_.period_jitter.add((deviation.count() < 0) ? -deviation : deviation);
  }
  has_previous_ = true;
  previous_release_ = activation.release;
  previous_start_ = activation.start;

  if (response > deadline_) {
    ++summary_.deadline_misses;
    result.deadline_missed = true;
  }
  if ((budget_.count() > 0) && (execution > budget_)) {
    ++summary_.budget_overruns;
    result.budget_exceeded = true;
  }
  return result;
}

TaskTimingSummary TaskTimingMonitor::summary() const {
  TaskTimingSummary result = summary_;
  result.execution_p50 = execution_samples_.percentile(50.0);
  result.execution_p99 = execution_samples_.percentile(99.0);
  result.start_latency_p50 = latency_samples_.percentile(50.0);
  result.start_latency_p99 = latency_samples_.percentile(99.0);
  return result;
}

NextRelease computeNextRelease(core::TimePoint current_release, core::TimePoint end, Nanos period) noexcept {
  NextRelease next{current_release + period, 0U};
  if ((period.count() > 0) && (end > next.release)) {
    // Releases at next, next+T, ... up to `end` happened while the task was running.
    // Keep the latest one (pending activation), drop the others.
    const auto overdue = end - next.release;
    next.skipped = static_cast<std::uint64_t>(overdue / period);
    next.release += period * static_cast<std::int64_t>(next.skipped);
  }
  return next;
}

}  // namespace vecu::sched
