// Deterministic tests of the timing formulas, using synthetic timestamps.

#include "core/timing_stats.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace vecu::sched {
namespace {

using std::chrono::microseconds;
using std::chrono::milliseconds;

core::TimePoint at(microseconds t) { return core::TimePoint{} + t; }
core::TimePoint atUs(long long us) { return at(microseconds{us}); }

TEST(DurationStats, ComputesMinMaxMeanStddev) {
  DurationStats stats;
  for (const long long value : {2, 4, 4, 4, 5, 5, 7, 9}) {
    stats.add(Nanos{value});
  }
  EXPECT_EQ(stats.count(), 8U);
  EXPECT_EQ(stats.min(), Nanos{2});
  EXPECT_EQ(stats.max(), Nanos{9});
  EXPECT_DOUBLE_EQ(stats.meanNs(), 5.0);
  EXPECT_NEAR(stats.stddevNs(), std::sqrt(32.0 / 7.0), 1e-12);  // sample standard deviation
}

TEST(DurationStats, EmptyAndSingleSample) {
  DurationStats stats;
  EXPECT_EQ(stats.count(), 0U);
  EXPECT_DOUBLE_EQ(stats.stddevNs(), 0.0);
  stats.add(Nanos{-5});  // negative values are valid (e.g. differences)
  EXPECT_EQ(stats.min(), Nanos{-5});
  EXPECT_EQ(stats.max(), Nanos{-5});
  EXPECT_DOUBLE_EQ(stats.stddevNs(), 0.0);
}

TEST(SampleWindow, NearestRankPercentiles) {
  SampleWindow<100U> window;
  EXPECT_EQ(window.percentile(50.0), Nanos{0});
  for (int i = 100; i >= 1; --i) {  // insertion order must not matter
    window.add(Nanos{i});
  }
  EXPECT_EQ(window.percentile(0.0), Nanos{1});
  EXPECT_EQ(window.percentile(50.0), Nanos{50});
  EXPECT_EQ(window.percentile(99.0), Nanos{99});
  EXPECT_EQ(window.percentile(100.0), Nanos{100});
}

TEST(SampleWindow, KeepsOnlyMostRecentSamples) {
  SampleWindow<4U> window;
  for (int i = 1; i <= 10; ++i) {
    window.add(Nanos{i});
  }
  EXPECT_EQ(window.size(), 4U);
  EXPECT_EQ(window.percentile(0.0), Nanos{7});
  EXPECT_EQ(window.percentile(100.0), Nanos{10});
}

class TaskTimingMonitorTest : public ::testing::Test {
 protected:
  // deadline 8 ms, budget 2 ms (period 10 ms in the scenarios below)
  TaskTimingMonitor monitor{milliseconds{8}, milliseconds{2}};
};

TEST_F(TaskTimingMonitorTest, MeasuresLatencyExecutionAndResponse) {
  const ActivationResult r = monitor.record({atUs(0), atUs(300), atUs(1300)});
  EXPECT_FALSE(r.deadline_missed);
  EXPECT_FALSE(r.budget_exceeded);
  const TaskTimingSummary s = monitor.summary();
  EXPECT_EQ(s.activations, 1U);
  EXPECT_EQ(s.start_latency.max(), microseconds{300});
  EXPECT_EQ(s.execution_time.max(), microseconds{1000});
  EXPECT_EQ(s.response_time.max(), microseconds{1300});
  EXPECT_EQ(s.period_jitter.count(), 0U);  // needs two activations
}

TEST_F(TaskTimingMonitorTest, PeriodJitterIsChangeOfStartLatency) {
  (void)monitor.record({atUs(0), atUs(100), atUs(200)});
  (void)monitor.record({atUs(10000), atUs(10600), atUs(10700)});  // started 500 us "later"
  (void)monitor.record({atUs(20000), atUs(20200), atUs(20300)});  // 400 us "earlier"
  const TaskTimingSummary s = monitor.summary();
  EXPECT_EQ(s.period_jitter.count(), 2U);
  EXPECT_EQ(s.period_jitter.max(), microseconds{500});
  EXPECT_EQ(s.period_jitter.min(), microseconds{400});
}

TEST_F(TaskTimingMonitorTest, DetectsDeadlineMissOnResponseTime) {
  // Execution is short, but the task started late: the response time decides.
  EXPECT_FALSE(monitor.record({atUs(0), atUs(7000), atUs(8000)}).deadline_missed);  // exactly 8 ms: ok
  EXPECT_TRUE(monitor.record({atUs(10000), atUs(17500), atUs(18001)}).deadline_missed);
  EXPECT_EQ(monitor.summary().deadline_misses, 1U);
}

TEST_F(TaskTimingMonitorTest, DetectsBudgetOverrun) {
  EXPECT_FALSE(monitor.record({atUs(0), atUs(0), atUs(2000)}).budget_exceeded);
  EXPECT_TRUE(monitor.record({atUs(10000), atUs(10000), atUs(12001)}).budget_exceeded);
  EXPECT_EQ(monitor.summary().budget_overruns, 1U);
}

TEST(TaskTimingMonitor, ZeroBudgetDisablesBudgetCheck) {
  TaskTimingMonitor monitor{milliseconds{10}, Nanos{0}};
  EXPECT_FALSE(monitor.record({atUs(0), atUs(0), atUs(9000)}).budget_exceeded);
}

TEST_F(TaskTimingMonitorTest, PercentilesInSummary) {
  for (int i = 1; i <= 100; ++i) {
    const long long base = static_cast<long long>(i) * 10000;
    (void)monitor.record({atUs(base), atUs(base + i), atUs(base + i + 10 * i)});
  }
  const TaskTimingSummary s = monitor.summary();
  EXPECT_EQ(s.start_latency_p50, microseconds{50});
  EXPECT_EQ(s.start_latency_p99, microseconds{99});
  EXPECT_EQ(s.execution_p99, microseconds{990});
}

// ----------------------------------------------------------------- activation policy ----

TEST(ComputeNextRelease, NormalCaseAdvancesOnePeriod) {
  const NextRelease next = computeNextRelease(atUs(0), atUs(3000), milliseconds{10});
  EXPECT_EQ(next.release, atUs(10000));
  EXPECT_EQ(next.skipped, 0U);
}

TEST(ComputeNextRelease, EndingExactlyAtNextReleaseIsNotAnOverrun) {
  const NextRelease next = computeNextRelease(atUs(0), atUs(10000), milliseconds{10});
  EXPECT_EQ(next.release, atUs(10000));
  EXPECT_EQ(next.skipped, 0U);
}

TEST(ComputeNextRelease, SmallOverrunKeepsOnePendingActivation) {
  // Ran 15 ms: release at 10 ms is pending and starts immediately (late), none is lost.
  const NextRelease next = computeNextRelease(atUs(0), atUs(15000), milliseconds{10});
  EXPECT_EQ(next.release, atUs(10000));
  EXPECT_EQ(next.skipped, 0U);
}

TEST(ComputeNextRelease, LargeOverrunDropsExtraActivationsAndKeepsPhase) {
  // Ran 35 ms: releases at 10, 20, 30 ms occurred; 30 is kept, 10 and 20 are lost.
  const NextRelease next = computeNextRelease(atUs(0), atUs(35000), milliseconds{10});
  EXPECT_EQ(next.release, atUs(30000));
  EXPECT_EQ(next.skipped, 2U);
}

}  // namespace
}  // namespace vecu::sched
