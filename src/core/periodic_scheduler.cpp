#include "core/periodic_scheduler.hpp"

#include <algorithm>
#include <iterator>
#include <utility>

namespace vecu::sched {

struct PeriodicScheduler::Task {
  Task(std::size_t task_index, TaskConfig task_config, TaskFunction task_function)
      : index(task_index),
        config(std::move(task_config)),
        function(std::move(task_function)),
        monitor(config.deadline, config.budget) {}

  std::size_t index;
  TaskConfig config;
  TaskFunction function;
  os::Thread thread;
  mutable os::PiMutex stats_mutex;  // protects monitor
  TaskTimingMonitor monitor;
  std::atomic<std::uint64_t> activations{0U};
  std::atomic<std::uint64_t> deadline_misses{0U};
};

const char* toString(SchedulerStatus status) noexcept {
  switch (status) {
    case SchedulerStatus::Ok:
      return "Ok";
    case SchedulerStatus::InvalidConfig:
      return "InvalidConfig";
    case SchedulerStatus::AlreadyRunning:
      return "AlreadyRunning";
    case SchedulerStatus::TooManyTasks:
      return "TooManyTasks";
    case SchedulerStatus::NoTasks:
      return "NoTasks";
    case SchedulerStatus::ThreadStartFailed:
      return "ThreadStartFailed";
  }
  return "Unknown";
}

bool validateTaskConfig(const TaskConfig& config, std::string& reason) {
  if (config.name.empty()) {
    reason = "task name must not be empty";
  } else if (config.period.count() <= 0) {
    reason = "period must be > 0";
  } else if ((config.offset.count() < 0) || (config.offset >= config.period)) {
    reason = "offset must be in [0, period)";
  } else if ((config.deadline.count() < 0) || (config.deadline > config.period)) {
    reason = "deadline must be in (0, period] (0 = implicit)";
  } else if (config.budget.count() < 0) {
    reason = "budget must be >= 0";
  } else if ((config.priority < os::kMinPriority) || (config.priority > os::kMaxPriority)) {
    reason = "priority must be in [1, 99]";
  } else {
    return true;
  }
  return false;
}

std::vector<std::string> checkRateMonotonic(const std::vector<TaskConfig>& tasks) {
  std::vector<std::string> warnings;
  for (const TaskConfig& a : tasks) {
    for (const TaskConfig& b : tasks) {
      if ((a.period < b.period) && (a.priority < b.priority)) {
        warnings.push_back("task '" + a.name + "' has a shorter period but lower priority than '" + b.name +
                           "' (not rate-monotonic)");
      }
    }
  }
  return warnings;
}

PeriodicScheduler::PeriodicScheduler(SchedulerConfig config) : config_(config) { tasks_.reserve(kMaxTasks); }

PeriodicScheduler::~PeriodicScheduler() { stop(); }

SchedulerStatus PeriodicScheduler::addTask(TaskConfig config, TaskFunction function) {
  if (running_) {
    return SchedulerStatus::AlreadyRunning;
  }
  if (tasks_.size() >= kMaxTasks) {
    return SchedulerStatus::TooManyTasks;
  }
  std::string reason;
  if (!function || !validateTaskConfig(config, reason)) {
    return SchedulerStatus::InvalidConfig;
  }
  const bool duplicate = std::any_of(tasks_.begin(), tasks_.end(),
                                     [&config](const auto& existing) { return existing->config.name == config.name; });
  if (duplicate) {
    return SchedulerStatus::InvalidConfig;
  }
  if (config.deadline.count() == 0) {
    config.deadline = config.period;
  }
  tasks_.push_back(std::make_unique<Task>(tasks_.size(), std::move(config), std::move(function)));
  return SchedulerStatus::Ok;
}

SchedulerStatus PeriodicScheduler::setObserver(ITaskObserver* observer) noexcept {
  if (running_) {
    return SchedulerStatus::AlreadyRunning;
  }
  observer_ = observer;
  return SchedulerStatus::Ok;
}

SchedulerStatus PeriodicScheduler::start() {
  if (running_) {
    return SchedulerStatus::AlreadyRunning;
  }
  if (tasks_.empty()) {
    return SchedulerStatus::NoTasks;
  }
  if (config_.lock_memory) {
    memory_locked_ = os::lockProcessMemory();
  }

  stop_requested_ = false;
  {
    const std::lock_guard<std::mutex> lock(start_mutex_);
    released_ = false;
  }
  for (auto& task : tasks_) {
    os::ThreadConfig thread_config;
    thread_config.name = task->config.name;
    thread_config.priority = task->config.priority;
    thread_config.request_realtime = config_.request_realtime;
    Task* raw = task.get();
    if (!task->thread.start(thread_config, [this, raw] { runTask(*raw); })) {
      stop_requested_ = true;
      start_cv_.notify_all();
      for (auto& started : tasks_) {
        started->thread.join();
      }
      return SchedulerStatus::ThreadStartFailed;
    }
  }

  // Release all tasks with a common epoch (condition-variable barrier).
  {
    const std::lock_guard<std::mutex> lock(start_mutex_);
    epoch_ = core::now() + config_.start_delay;
    released_ = true;
  }
  start_cv_.notify_all();
  running_ = true;
  return SchedulerStatus::Ok;
}

void PeriodicScheduler::stop() noexcept {
  stop_requested_ = true;
  start_cv_.notify_all();
  for (auto& task : tasks_) {
    task->thread.join();
  }
  running_ = false;
}

void PeriodicScheduler::runTask(Task& task) {
  core::TimePoint epoch{};
  {
    std::unique_lock<std::mutex> lock(start_mutex_);
    start_cv_.wait(lock, [this] { return released_ || stop_requested_.load(); });
    epoch = epoch_;
  }

  const Nanos period = task.config.period;
  core::TimePoint release = epoch + task.config.offset;
  std::uint64_t activation = 0U;

  while (!stop_requested_.load(std::memory_order_acquire)) {
    os::sleepUntil(release);
    if (stop_requested_.load(std::memory_order_acquire)) {
      break;
    }
    const core::TimePoint start = core::now();
    if (observer_ != nullptr) {
      observer_->onActivationStart(task.index, start);
    }
    task.function(ActivationInfo{activation, release});
    const core::TimePoint end = core::now();
    if (observer_ != nullptr) {
      observer_->onActivationEnd(task.index, end);
    }

    const NextRelease next = computeNextRelease(release, end, period);
    ActivationResult result;
    {
      const std::lock_guard<os::PiMutex> lock(task.stats_mutex);
      result = task.monitor.record(ActivationRecord{release, start, end});
      task.monitor.addSkipped(next.skipped);
    }
    task.activations.fetch_add(1U, std::memory_order_relaxed);
    if (result.deadline_missed) {
      task.deadline_misses.fetch_add(1U, std::memory_order_relaxed);
    }
    release = next.release;
    ++activation;
  }
}

std::uint64_t PeriodicScheduler::activations(std::size_t index) const noexcept {
  return (index < tasks_.size()) ? tasks_[index]->activations.load(std::memory_order_relaxed) : 0U;
}

std::uint64_t PeriodicScheduler::deadlineMisses(std::size_t index) const noexcept {
  return (index < tasks_.size()) ? tasks_[index]->deadline_misses.load(std::memory_order_relaxed) : 0U;
}

std::string_view PeriodicScheduler::taskName(std::size_t index) const noexcept {
  return (index < tasks_.size()) ? std::string_view{tasks_[index]->config.name} : std::string_view{};
}

TaskStatsSnapshot PeriodicScheduler::taskStats(std::size_t index) const {
  TaskStatsSnapshot snapshot;
  if (index >= tasks_.size()) {
    return snapshot;
  }
  const Task& task = *tasks_[index];
  snapshot.name = task.config.name;
  snapshot.period = task.config.period;
  snapshot.offset = task.config.offset;
  snapshot.deadline = task.config.deadline;
  snapshot.budget = task.config.budget;
  snapshot.priority = task.config.priority;
  snapshot.scheduling_class = task.thread.schedulingClass();

  // Copy under the lock (a short memcpy), compute percentiles outside it, so the task is
  // never blocked by sorting.
  auto copy = std::make_unique<TaskTimingMonitor>(task.config.deadline, task.config.budget);
  {
    const std::lock_guard<os::PiMutex> lock(task.stats_mutex);
    *copy = task.monitor;
  }
  snapshot.timing = copy->summary();
  return snapshot;
}

std::vector<TaskStatsSnapshot> PeriodicScheduler::allTaskStats() const {
  std::vector<TaskStatsSnapshot> result;
  result.reserve(tasks_.size());
  for (std::size_t i = 0U; i < tasks_.size(); ++i) {
    result.push_back(taskStats(i));
  }
  return result;
}

std::vector<std::string> PeriodicScheduler::configurationWarnings() const {
  std::vector<TaskConfig> configs;
  configs.reserve(tasks_.size());
  std::transform(tasks_.begin(), tasks_.end(), std::back_inserter(configs),
                 [](const auto& task) { return task->config; });
  return checkRateMonotonic(configs);
}

}  // namespace vecu::sched
