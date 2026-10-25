#include "watchdog/watchdog_supervisor.hpp"

namespace vecu::watchdog {

const char* toString(SupervisionStatus status) noexcept {
  switch (status) {
    case SupervisionStatus::Ok:
      return "Ok";
    case SupervisionStatus::Failed:
      return "Failed";
    case SupervisionStatus::Expired:
      return "Expired";
  }
  return "Unknown";
}

const char* toString(Violation violation) noexcept {
  switch (violation) {
    case Violation::None:
      return "None";
    case Violation::MissingHeartbeat:
      return "MissingHeartbeat";
    case Violation::ExecutionTimeout:
      return "ExecutionTimeout";
    case Violation::ApplicationStalled:
      return "ApplicationStalled";
  }
  return "Unknown";
}

bool WatchdogSupervisor::addEntity(const EntityConfig& config) {
  const std::lock_guard<std::mutex> lock(mutex_);
  if ((count_ >= kMaxEntities) || (config.heartbeat_timeout.count() <= 0) || (config.max_execution.count() <= 0)) {
    return false;
  }
  entities_[count_].config = config;
  ++count_;
  return true;
}

void WatchdogSupervisor::activate(core::TimePoint now) noexcept { reset(now); }

void WatchdogSupervisor::onActivationStart(std::size_t task_index, core::TimePoint now) noexcept {
  if (task_index < count_) {
    Entity& e = entities_[task_index];
    e.execution_start_ns.store(ns(now), std::memory_order_relaxed);
    e.executing.store(true, std::memory_order_release);
  }
}

void WatchdogSupervisor::onActivationEnd(std::size_t task_index, core::TimePoint now) noexcept {
  if (task_index < count_) {
    Entity& e = entities_[task_index];
    e.last_heartbeat_ns.store(ns(now), std::memory_order_relaxed);
    e.executing.store(false, std::memory_order_release);
  }
}

SupervisionStatus WatchdogSupervisor::globalLocked() const noexcept {
  SupervisionStatus global = SupervisionStatus::Ok;
  for (std::size_t i = 0U; i < count_; ++i) {
    if (entities_[i].status == SupervisionStatus::Expired) {
      return SupervisionStatus::Expired;
    }
    if (entities_[i].status == SupervisionStatus::Failed) {
      global = SupervisionStatus::Failed;
    }
  }
  return global;
}

CheckResult WatchdogSupervisor::check(core::TimePoint now) noexcept {
  const std::lock_guard<std::mutex> lock(mutex_);
  const std::int64_t now_ns = ns(now);
  CheckResult result;
  std::array<Violation, kMaxEntities> violations{};
  std::array<std::int64_t, kMaxEntities> details{};
  std::size_t violating = 0U;

  for (std::size_t i = 0U; i < count_; ++i) {
    Entity& e = entities_[i];
    const bool executing = e.executing.load(std::memory_order_acquire);
    const std::int64_t since_start = now_ns - e.execution_start_ns.load(std::memory_order_relaxed);
    const std::int64_t since_heartbeat = now_ns - e.last_heartbeat_ns.load(std::memory_order_relaxed);
    const auto max_exec = std::chrono::duration_cast<std::chrono::nanoseconds>(e.config.max_execution).count();
    const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(e.config.heartbeat_timeout).count();
    if (executing && (since_start > max_exec)) {
      violations[i] = Violation::ExecutionTimeout;
      details[i] = since_start;
    } else if (since_heartbeat > timeout) {
      violations[i] = Violation::MissingHeartbeat;
      details[i] = since_heartbeat;
    } else {
      violations[i] = Violation::None;
    }
    if (violations[i] != Violation::None) {
      ++violating;
    }
  }
  const bool application_stalled = (count_ > 0U) && (violating == count_);

  for (std::size_t i = 0U; i < count_; ++i) {
    Entity& e = entities_[i];
    if (e.status == SupervisionStatus::Expired) {
      continue;  // latched until reset()
    }
    if (violations[i] == Violation::None) {
      e.failed_cycles = 0U;
      e.status = SupervisionStatus::Ok;
      continue;
    }
    ++e.failed_cycles;
    e.violation = application_stalled ? Violation::ApplicationStalled : violations[i];
    // A stalled application expires immediately; a single entity after its tolerance.
    e.status = (application_stalled || (e.failed_cycles > e.config.failed_tolerance)) ? SupervisionStatus::Expired
                                                                                       : SupervisionStatus::Failed;
    if ((e.status == SupervisionStatus::Expired) && !result.newly_expired && (last_global_ != SupervisionStatus::Expired)) {
      result.newly_expired = true;
      result.entity = i;
      result.violation = e.violation;
      result.detail = std::chrono::nanoseconds{details[i]};
    }
  }
  result.global = globalLocked();
  if (result.newly_expired) {
    expirations_.fetch_add(1U);
  }
  last_global_ = result.global;
  return result;
}

void WatchdogSupervisor::reset(core::TimePoint now) noexcept {
  const std::lock_guard<std::mutex> lock(mutex_);
  for (std::size_t i = 0U; i < count_; ++i) {
    Entity& e = entities_[i];
    e.last_heartbeat_ns.store(ns(now), std::memory_order_relaxed);
    e.status = SupervisionStatus::Ok;
    e.violation = Violation::None;
    e.failed_cycles = 0U;
    // An activation still running keeps its start time: if it is really stuck, the
    // execution timeout is detected again.
  }
  last_global_ = SupervisionStatus::Ok;
}

SupervisionStatus WatchdogSupervisor::globalStatus() const noexcept {
  const std::lock_guard<std::mutex> lock(mutex_);
  return globalLocked();
}

SupervisionStatus WatchdogSupervisor::status(std::size_t entity) const noexcept {
  const std::lock_guard<std::mutex> lock(mutex_);
  return (entity < count_) ? entities_[entity].status : SupervisionStatus::Ok;
}

Violation WatchdogSupervisor::lastViolation(std::size_t entity) const noexcept {
  const std::lock_guard<std::mutex> lock(mutex_);
  return (entity < count_) ? entities_[entity].violation : Violation::None;
}

core::TimePoint WatchdogSupervisor::lastHeartbeat(std::size_t entity) const noexcept {
  if (entity >= count_) {
    return core::TimePoint{};
  }
  return core::TimePoint{std::chrono::duration_cast<core::MonotonicClock::duration>(
      std::chrono::nanoseconds{entities_[entity].last_heartbeat_ns.load(std::memory_order_relaxed)})};
}

bool WatchdogSupervisor::isExecuting(std::size_t entity) const noexcept {
  return (entity < count_) && entities_[entity].executing.load(std::memory_order_acquire);
}

std::string WatchdogSupervisor::name(std::size_t entity) const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return (entity < count_) ? entities_[entity].config.name : std::string{};
}

}  // namespace vecu::watchdog
