// Configuration checks of the scheduler (no timing involved).

#include <gtest/gtest.h>

#include <string>

#include "core/periodic_scheduler.hpp"
#include "core/timing_report.hpp"

namespace vecu::sched {
namespace {

using std::chrono::microseconds;

TaskConfig valid(std::string name = "Task", long long period_us = 10000, int priority = 50) {
  TaskConfig config;
  config.name = std::move(name);
  config.period = microseconds{period_us};
  config.priority = priority;
  return config;
}

void noop(const ActivationInfo& /*unused*/) {}

TEST(SchedulerConfig, AcceptsValidConfiguration) {
  std::string reason;
  EXPECT_TRUE(validateTaskConfig(valid(), reason)) << reason;
}

TEST(SchedulerConfig, RejectsInvalidFields) {
  std::string reason;
  TaskConfig c = valid();
  c.name.clear();
  EXPECT_FALSE(validateTaskConfig(c, reason));

  c = valid();
  c.period = microseconds{0};
  EXPECT_FALSE(validateTaskConfig(c, reason));

  c = valid();
  c.offset = c.period;  // offset must be < period
  EXPECT_FALSE(validateTaskConfig(c, reason));

  c = valid();
  c.deadline = c.period + microseconds{1};  // only constrained deadlines (<= period)
  EXPECT_FALSE(validateTaskConfig(c, reason));

  c = valid();
  c.priority = 0;
  EXPECT_FALSE(validateTaskConfig(c, reason));
  c.priority = 100;
  EXPECT_FALSE(validateTaskConfig(c, reason));

  c = valid();
  c.budget = microseconds{-1};
  EXPECT_FALSE(validateTaskConfig(c, reason));
  EXPECT_FALSE(reason.empty());
}

TEST(SchedulerConfig, AddTaskEnforcesRules) {
  PeriodicScheduler scheduler;
  EXPECT_EQ(scheduler.start(), SchedulerStatus::NoTasks);
  EXPECT_EQ(scheduler.addTask(valid("A"), TaskFunction{}), SchedulerStatus::InvalidConfig);  // no function
  EXPECT_EQ(scheduler.addTask(valid("A"), noop), SchedulerStatus::Ok);
  EXPECT_EQ(scheduler.addTask(valid("A"), noop), SchedulerStatus::InvalidConfig);  // duplicate name
  for (std::size_t i = 1U; i < kMaxTasks; ++i) {
    EXPECT_EQ(scheduler.addTask(valid("T" + std::to_string(i)), noop), SchedulerStatus::Ok);
  }
  EXPECT_EQ(scheduler.addTask(valid("Overflow"), noop), SchedulerStatus::TooManyTasks);
  EXPECT_EQ(scheduler.taskCount(), kMaxTasks);
}

TEST(SchedulerConfig, ImplicitDeadlineEqualsPeriod) {
  PeriodicScheduler scheduler;
  ASSERT_EQ(scheduler.addTask(valid("A", 20000), noop), SchedulerStatus::Ok);
  EXPECT_EQ(scheduler.taskStats(0U).deadline, microseconds{20000});
  EXPECT_EQ(scheduler.taskStats(0U).timing.activations, 0U);
  EXPECT_EQ(scheduler.taskName(5U), "");  // out-of-range index is safe
}

TEST(SchedulerConfig, RateMonotonicCheck) {
  EXPECT_TRUE(checkRateMonotonic({valid("Fast", 10000, 80), valid("Slow", 100000, 50)}).empty());
  const auto warnings = checkRateMonotonic({valid("Fast", 10000, 40), valid("Slow", 100000, 50)});
  ASSERT_EQ(warnings.size(), 1U);
  EXPECT_NE(warnings[0].find("Fast"), std::string::npos);
}

TEST(TimingReport, FormatsSummaryLineAndJson) {
  TaskStatsSnapshot task;
  task.name = "Control_10ms";
  task.period = microseconds{10000};
  task.priority = 80;
  task.timing.activations = 100U;
  task.timing.deadline_misses = 2U;
  task.timing.execution_time.add(std::chrono::microseconds{500});
  const std::string line = formatTimingSummaryLine(task);
  EXPECT_NE(line.find("SCHED_SUMMARY task=Control_10ms period_ms=10.000 prio=80"), std::string::npos);
  EXPECT_NE(line.find("deadline_misses=2"), std::string::npos);
  EXPECT_NEAR(estimatedUtilization({task}), 0.05, 1e-9);

  const std::string json = formatTimingJson({task}, "unit\"test", false);
  EXPECT_NE(json.find("\"label\": \"unit\\\"test\""), std::string::npos);
  EXPECT_NE(json.find("\"name\": \"Control_10ms\""), std::string::npos);
}

}  // namespace
}  // namespace vecu::sched
