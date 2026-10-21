// Integration tests of the scheduler with real threads and real time.
//
// Assertions are chosen to be robust on a non-real-time OS: event counts get a tolerance,
// planned (computed) release times are checked exactly, and overload scenarios are made
// unambiguous (e.g. 25 ms of work in a 10 ms task always misses its deadline).

#include "core/periodic_scheduler.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <thread>
#include <vector>

#include "core/shared_signal.hpp"

namespace vecu::sched {
namespace {

using std::chrono::microseconds;
using std::chrono::milliseconds;

TaskConfig task(const char* name, long long period_us, int priority, long long offset_us = 0) {
  TaskConfig config;
  config.name = name;
  config.period = microseconds{period_us};
  config.offset = microseconds{offset_us};
  config.priority = priority;
  return config;
}

void busyWait(microseconds duration) {
  const core::TimePoint end = core::now() + duration;
  while (core::now() < end) {
  }
}

void runFor(milliseconds duration) { std::this_thread::sleep_for(duration); }

TEST(PeriodicScheduler, ActivationCountsMatchPeriods) {
  PeriodicScheduler scheduler;
  ASSERT_EQ(scheduler.addTask(task("T10", 10000, 80), [](const ActivationInfo&) {}), SchedulerStatus::Ok);
  ASSERT_EQ(scheduler.addTask(task("T20", 20000, 70), [](const ActivationInfo&) {}), SchedulerStatus::Ok);
  ASSERT_EQ(scheduler.addTask(task("T100", 100000, 50), [](const ActivationInfo&) {}), SchedulerStatus::Ok);
  ASSERT_EQ(scheduler.start(), SchedulerStatus::Ok);
  EXPECT_TRUE(scheduler.isRunning());
  EXPECT_EQ(scheduler.start(), SchedulerStatus::AlreadyRunning);
  runFor(milliseconds{1020});  // includes the 20 ms start delay
  scheduler.stop();
  EXPECT_FALSE(scheduler.isRunning());

  // Expected ~100 / 50 / 10 activations in 1 s; allow scheduling noise.
  EXPECT_NEAR(static_cast<double>(scheduler.activations(0U)), 100.0, 3.0);
  EXPECT_NEAR(static_cast<double>(scheduler.activations(1U)), 50.0, 2.0);
  EXPECT_NEAR(static_cast<double>(scheduler.activations(2U)), 10.0, 1.0);
  for (const TaskStatsSnapshot& stats : scheduler.allTaskStats()) {
    EXPECT_EQ(stats.timing.activations, stats.timing.execution_time.count());
    EXPECT_GE(stats.timing.start_latency.min().count(), 0);  // never started before release
  }
}

TEST(PeriodicScheduler, ReleasesFollowExactGridWithOffsetsFromCommonEpoch) {
  PeriodicScheduler scheduler;
  std::vector<core::TimePoint> releases_a;
  std::vector<core::TimePoint> releases_b;
  releases_a.reserve(64U);
  releases_b.reserve(64U);
  ASSERT_EQ(scheduler.addTask(task("A", 10000, 80, 0),
                              [&releases_a](const ActivationInfo& info) { releases_a.push_back(info.release); }),
            SchedulerStatus::Ok);
  ASSERT_EQ(scheduler.addTask(task("B", 20000, 70, 5000),
                              [&releases_b](const ActivationInfo& info) { releases_b.push_back(info.release); }),
            SchedulerStatus::Ok);
  ASSERT_EQ(scheduler.start(), SchedulerStatus::Ok);
  runFor(milliseconds{300});
  scheduler.stop();

  ASSERT_GE(releases_a.size(), 10U);
  ASSERT_GE(releases_b.size(), 5U);
  EXPECT_EQ(releases_a.front(), scheduler.epoch());
  EXPECT_EQ(releases_b.front(), scheduler.epoch() + microseconds{5000});
  for (std::size_t i = 1U; i < releases_a.size(); ++i) {
    EXPECT_EQ(releases_a[i] - releases_a[i - 1U], milliseconds{10});  // no drift, by construction
  }
  for (std::size_t i = 1U; i < releases_b.size(); ++i) {
    EXPECT_EQ(releases_b[i] - releases_b[i - 1U], milliseconds{20});
  }
}

TEST(PeriodicScheduler, DetectsDeadlineMissesAndSkipsActivationsOfOverloadedTask) {
  PeriodicScheduler scheduler;
  // 25 ms of work every 10 ms: every activation misses, and per activation one release is lost.
  ASSERT_EQ(scheduler.addTask(task("Overload", 10000, 80), [](const ActivationInfo&) { busyWait(milliseconds{25}); }),
            SchedulerStatus::Ok);
  ASSERT_EQ(scheduler.addTask(task("Healthy", 100000, 50), [](const ActivationInfo&) {}), SchedulerStatus::Ok);
  ASSERT_EQ(scheduler.start(), SchedulerStatus::Ok);
  runFor(milliseconds{500});
  scheduler.stop();

  const TaskStatsSnapshot overload = scheduler.taskStats(0U);
  ASSERT_GT(overload.timing.activations, 5U);
  EXPECT_EQ(overload.timing.deadline_misses, overload.timing.activations);
  EXPECT_EQ(scheduler.deadlineMisses(0U), overload.timing.deadline_misses);
  EXPECT_GE(overload.timing.skipped_activations, overload.timing.activations - 1U);
  // Activations + skipped ~ number of 10 ms releases in the run.
  EXPECT_NEAR(static_cast<double>(overload.timing.activations + overload.timing.skipped_activations), 50.0, 5.0);

  // The overloaded task runs in its own thread, so the other task is unaffected on a multi-core host.
  EXPECT_EQ(scheduler.taskStats(1U).timing.deadline_misses, 0U);
}

TEST(PeriodicScheduler, DetectsExecutionBudgetOverrun) {
  PeriodicScheduler scheduler;
  TaskConfig config = task("Budget", 20000, 70);
  config.budget = microseconds{1000};
  std::atomic<int> count{0};
  ASSERT_EQ(scheduler.addTask(config,
                              [&count](const ActivationInfo&) {
                                // every 2nd activation exceeds the 1 ms budget (but not the deadline)
                                busyWait(((count++ % 2) == 0) ? microseconds{3000} : microseconds{100});
                              }),
            SchedulerStatus::Ok);
  ASSERT_EQ(scheduler.start(), SchedulerStatus::Ok);
  runFor(milliseconds{300});
  scheduler.stop();
  const TaskTimingSummary timing = scheduler.taskStats(0U).timing;
  ASSERT_GE(timing.activations, 8U);
  EXPECT_NEAR(static_cast<double>(timing.budget_overruns), static_cast<double>(timing.activations) / 2.0, 1.0);
}

TEST(PeriodicScheduler, StopIsPromptAndRestartIsPossible) {
  PeriodicScheduler scheduler;
  ASSERT_EQ(scheduler.addTask(task("T100", 100000, 50), [](const ActivationInfo&) {}), SchedulerStatus::Ok);
  ASSERT_EQ(scheduler.start(), SchedulerStatus::Ok);
  runFor(milliseconds{50});
  const core::TimePoint before = core::now();
  scheduler.stop();
  EXPECT_LT(core::now() - before, milliseconds{150});  // at most about one period
  EXPECT_EQ(scheduler.addTask(task("Late", 10000, 80), [](const ActivationInfo&) {}), SchedulerStatus::Ok);
  ASSERT_EQ(scheduler.start(), SchedulerStatus::Ok);
  scheduler.stop();
}

TEST(PeriodicScheduler, StatsCanBeReadWhileRunning) {
  PeriodicScheduler scheduler;
  ASSERT_EQ(scheduler.addTask(task("T10", 10000, 80), [](const ActivationInfo&) {}), SchedulerStatus::Ok);
  ASSERT_EQ(scheduler.start(), SchedulerStatus::Ok);
  std::uint64_t previous = 0U;
  for (int i = 0; i < 20; ++i) {
    const TaskStatsSnapshot stats = scheduler.taskStats(0U);
    EXPECT_GE(stats.timing.activations, previous);
    previous = stats.timing.activations;
    runFor(milliseconds{10});
  }
  scheduler.stop();
  EXPECT_GT(previous, 5U);
}

TEST(PeriodicScheduler, InterTaskDataExchangeIsConsistent) {
  // Producer (10 ms) and consumer (20 ms) exchange data only through a SharedSignal.
  struct Data {
    std::uint64_t counter;
    std::uint64_t counter_squared;
  };
  core::SharedSignal<Data> channel{Data{0U, 0U}};
  std::atomic<std::uint64_t> inconsistent{0U};
  std::atomic<std::uint64_t> received{0U};
  std::uint64_t last_counter = 0U;  // consumer-owned

  PeriodicScheduler scheduler;
  ASSERT_EQ(scheduler.addTask(task("Producer", 10000, 80),
                              [&channel](const ActivationInfo& info) {
                                const std::uint64_t n = info.activation + 1U;
                                channel.write(Data{n, n * n}, core::now());
                              }),
            SchedulerStatus::Ok);
  ASSERT_EQ(scheduler.addTask(task("Consumer", 20000, 70, 5000),
                              [&](const ActivationInfo&) {
                                const auto sample = channel.read();
                                if ((sample.value.counter_squared != sample.value.counter * sample.value.counter) ||
                                    (sample.value.counter < last_counter)) {
                                  inconsistent.fetch_add(1U);
                                }
                                last_counter = sample.value.counter;
                                received.fetch_add(1U);
                              }),
            SchedulerStatus::Ok);
  ASSERT_EQ(scheduler.start(), SchedulerStatus::Ok);
  runFor(milliseconds{400});
  scheduler.stop();
  EXPECT_GT(received.load(), 10U);
  EXPECT_EQ(inconsistent.load(), 0U);
  EXPECT_GT(last_counter, 20U);
}

}  // namespace
}  // namespace vecu::sched
