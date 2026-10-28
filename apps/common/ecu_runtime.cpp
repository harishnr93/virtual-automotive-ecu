#include "common/ecu_runtime.hpp"

#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <utility>

#include "can/can_channel_factory.hpp"
#include "common/shutdown_signal.hpp"
#include "core/os_port.hpp"
#include "core/timing_report.hpp"

#ifdef VECU_COVERAGE
extern "C" void __gcov_dump(void);  // NOLINT(bugprone-reserved-identifier): gcov runtime (coverage builds)
#endif

namespace vecu::app {

namespace {

sched::SchedulerConfig schedulerConfig(const AppOptions& options) {
  sched::SchedulerConfig config;
  config.request_realtime = options.realtime;
  config.lock_memory = options.lock_memory;
  return config;
}

}  // namespace

// ------------------------------------------------------------------------------- CanIo ----

bool CanIo::send(const can::CanFrame& frame) {
  if (channel_.send(frame) != can::CanStatus::Ok) {
    tx_failed_.fetch_add(1U, std::memory_order_relaxed);
    return false;
  }
  tx_ok_.fetch_add(1U, std::memory_order_relaxed);
  trace_.write(frame, core::steadyWallMicros());
  return true;
}

void CanIo::traceReceived(const can::RxFrame& entry) { trace_.write(entry.frame, entry.wall_time_us); }

// ------------------------------------------------------------------------- EcuPlatform ----

EcuPlatform::EcuPlatform(std::string ecu_name, const AppOptions& options)
    : ecu_name_(std::move(ecu_name)), options_(options), log_(ecu_name_), scheduler_(schedulerConfig(options)) {}

EcuPlatform::~EcuPlatform() {
  watchdog_stop_ = true;
  watchdog_thread_.join();
  injector_.release();
  scheduler_.stop();
  if (receiver_) {
    receiver_->stop();
  }
}

bool EcuPlatform::init() {
  can::CanStatus status = can::CanStatus::Ok;
  channel_ = can::createCanChannel(options_.can, status);
  if (!channel_) {
    log_.error(core::strCat("cannot open CAN channel: ", can::toString(status)));
    return false;
  }
  if (!options_.can_log_path.empty() &&
      (trace_.open(options_.can_log_path, std::string{channel_->name()}) != can::CanStatus::Ok)) {
    log_.error(core::strCat("cannot open CAN trace file ", options_.can_log_path));
    return false;
  }
  io_ = std::make_unique<CanIo>(*channel_, trace_);
  receiver_ = std::make_unique<can::CanReceiver>(*channel_, rx_queue_);
  for (const std::string& spec : options_.injections) {
    std::string error;
    if (!injector_.add(spec, error)) {
      log_.error(error);
      return false;
    }
  }
  if (!injector_.empty()) {
    log_.warning(core::strCat("TEST MODE: ", injector_.injections().size(), " fault injection(s) armed"));
  }
  return true;
}

bool EcuPlatform::addTask(sched::TaskConfig config, sched::TaskFunction function) {
  std::string reason;
  if (!sched::validateTaskConfig(config, reason)) {
    log_.error(core::strCat("invalid task '", config.name, "': ", reason));
    return false;
  }
  const std::string name = config.name;
  const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(config.period);
  // Test-only injections run inside the task, before the application function.
  sched::TaskFunction wrapped = [this, name, fn = std::move(function)](const sched::ActivationInfo& activation) {
    injector_.beforeTask(name, secondsSinceEpoch(activation.release));
    fn(activation);
  };
  const sched::SchedulerStatus status = scheduler_.addTask(std::move(config), std::move(wrapped));
  if (status != sched::SchedulerStatus::Ok) {
    log_.error(core::strCat("cannot add task '", name, "': ", sched::toString(status)));
    return false;
  }
  // Watchdog entity index == scheduler task index (both registered here, in order).
  watchdog::EntityConfig supervision;
  supervision.name = name;
  supervision.heartbeat_timeout = period * 3;
  supervision.max_execution = period * 2;
  supervision.failed_tolerance = 2U;
  if (!watchdog_.addEntity(supervision)) {
    log_.error(core::strCat("cannot supervise task '", name, "'"));
    return false;
  }
  return true;
}

double EcuPlatform::secondsSinceEpoch(core::TimePoint t) const noexcept {
  return core::toMillis(t - scheduler_.epoch()) / 1000.0;
}

bool EcuPlatform::resumedAfter(const watchdog::CheckResult& expired, core::TimePoint since) const {
  auto resumed = [this, since](std::size_t entity) {
    return (watchdog_.lastHeartbeat(entity) > since) && !watchdog_.isExecuting(entity);
  };
  if (expired.violation != watchdog::Violation::ApplicationStalled) {
    return resumed(expired.entity);
  }
  for (std::size_t i = 0U; i < watchdog_.size(); ++i) {
    if (!resumed(i)) {
      return false;
    }
  }
  return true;
}

void EcuPlatform::runWatchdog() {
  enum class Phase { Supervising, WaitingForResume, Probation };
  Phase phase = Phase::Supervising;
  watchdog::CheckResult expired;
  core::TimePoint expired_at{};
  core::TimePoint probation_end{};
  core::TimePoint next = core::now();

  while (!watchdog_stop_.load()) {
    next += kWatchdogPeriod;
    os::sleepUntil(next);
    const core::TimePoint now = core::now();
    if ((now - next) > (kWatchdogPeriod * 5)) {
      next = now;  // resynchronise after a host stall
    }
    watchdog_last_check_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    const watchdog::CheckResult result = watchdog_.check(now);

    if (result.newly_expired && (phase != Phase::WaitingForResume)) {
      expired = result;
      expired_at = now;
      const std::string entity = watchdog_.name(result.entity);
      log_.error(core::strCat("WATCHDOG_EXPIRED entity=", entity, " violation=", watchdog::toString(result.violation),
                              " detail_ms=", fixed(core::toMillis(result.detail), 1),
                              " t=", fixed(secondsSinceEpoch(now), 3), "s"));
      if (watchdog_handlers_.on_expired) {
        watchdog_handlers_.on_expired(result, entity);
      }
      phase = Phase::WaitingForResume;
      continue;
    }
    if ((phase == Phase::WaitingForResume) && resumedAfter(expired, expired_at)) {
      log_.warning(core::strCat("WATCHDOG_RESUMED entity=", watchdog_.name(expired.entity), " after_ms=",
                                fixed(core::toMillis(now - expired_at), 1)));
      if (watchdog_handlers_.on_resumed) {
        watchdog_handlers_.on_resumed();
      }
      watchdog_.reset(now);
      probation_end = now + kWatchdogProbation;
      phase = Phase::Probation;
    } else if ((phase == Phase::Probation) && (now >= probation_end) &&
               (result.global == watchdog::SupervisionStatus::Ok)) {
      log_.info("WATCHDOG_HEALTHY supervision ok after recovery");
      if (watchdog_handlers_.on_healthy) {
        watchdog_handlers_.on_healthy();
      }
      phase = Phase::Supervising;
    } else {
      // keep supervising / waiting
    }
  }
}

void EcuPlatform::emergencyExit(int exit_code) {
  log_.error(core::strCat("EMERGENCY_EXIT code=", exit_code, " (simulated hardware reset: no orderly shutdown)"));
  if (shutdown_hook_) {
    shutdown_hook_(true);
  }
  (void)std::fflush(nullptr);
#ifdef VECU_COVERAGE
  __gcov_dump();  // coverage builds only: _Exit skips the atexit handler that writes the counters
#endif
  std::_Exit(exit_code);
}

bool EcuPlatform::routeToDiagnostics(std::uint32_t id) {
  if (!receiver_ || !receiver_->addRoute(id, diag_rx_queue_)) {
    log_.error(core::strCat("cannot route ", can::formatId(id), " to diagnostics"));
    return false;
  }
  return true;
}

std::string_view EcuPlatform::busName() const noexcept {
  return channel_ ? channel_->name() : std::string_view{"none"};
}

int EcuPlatform::run() {
  if (!channel_ || !receiver_) {
    log_.error("run() called before successful init()");
    return 1;
  }
  for (const std::string& warning : scheduler_.configurationWarnings()) {
    log_.warning(warning);
  }

  os::ThreadConfig rx_config;
  rx_config.name = "CanRx";
  rx_config.priority = kPriorityCanRx;
  rx_config.request_realtime = options_.realtime;
  if (!receiver_->start(rx_config)) {
    log_.error("cannot start CAN reception thread");
    return 1;
  }
  (void)scheduler_.setObserver(&watchdog_);  // heartbeats / execution monitoring
  const sched::SchedulerStatus status = scheduler_.start();
  if (status != sched::SchedulerStatus::Ok) {
    log_.error(core::strCat("scheduler start failed: ", sched::toString(status)));
    receiver_->stop();
    return 1;
  }
  watchdog_.activate(scheduler_.epoch());
  // Wall-clock time of the scheduler epoch: lets tools align the "t=...s" of log events with
  // other processes (simulator, CAN traces) that record wall-clock time.
  log_.info(core::strCat("SCHED_EPOCH wall_us=",
                         core::steadyWallMicros() -
                             std::chrono::duration_cast<std::chrono::microseconds>(core::now() - scheduler_.epoch())
                                 .count()));
  os::ThreadConfig watchdog_config;
  watchdog_config.name = "Watchdog";
  watchdog_config.priority = kPriorityWatchdog;
  watchdog_config.request_realtime = options_.realtime;
  watchdog_last_check_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(core::now().time_since_epoch()).count();
  if (!watchdog_thread_.start(watchdog_config, [this] { runWatchdog(); })) {
    log_.error("cannot start watchdog thread");
    scheduler_.stop();
    receiver_->stop();
    return 1;
  }

  const os::SchedulingClass task_class = scheduler_.taskStats(0U).scheduling_class;
  log_.info(core::strCat("started: tasks=", scheduler_.taskCount(), " policy=", os::toString(task_class),
                         options_.realtime && (task_class != os::SchedulingClass::RealTimeFifo)
                             ? " (real-time requested but not permitted -> fallback)"
                             : "",
                         " rx_thread=", os::toString(receiver_->schedulingClass()),
                         " memory_locked=", scheduler_.memoryLocked(), " (Ctrl+C to stop)"));

  // The main thread supervises run time and the watchdog thread itself ("who watches the
  // watchdog": on a real ECU this is the hardware watchdog triggered by the supervisor).
  const core::TimePoint start = core::now();
  const auto limit = std::chrono::duration<double>(options_.duration_s);
  bool supervisor_stall_reported = false;
  while (!shutdownRequested() && ((options_.duration_s <= 0.0) || ((core::now() - start) < limit))) {
    os::sleepUntil(core::now() + std::chrono::milliseconds{10});
    const auto last_check = std::chrono::nanoseconds{watchdog_last_check_ns_.load()};
    const bool stalled = (core::now().time_since_epoch() - last_check) > std::chrono::milliseconds{500};
    if (stalled && !supervisor_stall_reported) {
      log_.error("WATCHDOG_SUPERVISOR_STALLED: no supervision cycle for > 500 ms");
    }
    supervisor_stall_reported = stalled;
  }

  watchdog_stop_ = true;
  watchdog_thread_.join();
  injector_.release();  // end injected hangs so the task threads can be joined
  scheduler_.stop();
  receiver_->stop();
  if (shutdown_hook_) {
    shutdown_hook_(false);
  }

  const std::vector<sched::TaskStatsSnapshot> stats = scheduler_.allTaskStats();
  log_.info(core::strCat("scheduler timing report (soft real-time, measured):\n", sched::formatTimingTable(stats)));
  for (const sched::TaskStatsSnapshot& task : stats) {
    log_.info(sched::formatTimingSummaryLine(task));
  }
  log_.info(core::strCat("CAN_RX_SUMMARY received=", receiver_->receivedFrames(),
                         " errors=", receiver_->receiveErrors(), " queue_overruns=", rx_queue_.overruns(),
                         " diag_queue_overruns=", diag_rx_queue_.overruns(),
                         " channel_drops=", channel_->stats().rx_overruns));
  log_.info(core::strCat("WATCHDOG_SUMMARY expirations=", watchdog_.expirations(), " entities=", watchdog_.size(),
                         " status=", watchdog::toString(watchdog_.globalStatus())));
  if (!options_.timing_report_path.empty()) {
    std::ofstream file(options_.timing_report_path);
    file << sched::formatTimingJson(stats, ecu_name_, scheduler_.memoryLocked());
    if (!file) {
      log_.error(core::strCat("cannot write timing report ", options_.timing_report_path));
      return 1;
    }
  }
  return 0;
}

void EcuPlatform::reportNewDeadlineMisses() {
  for (std::size_t i = 0U; i < scheduler_.taskCount(); ++i) {
    const std::uint64_t misses = scheduler_.deadlineMisses(i);
    if (misses > reported_misses_[i]) {
      log_.warning(core::strCat("DEADLINE_MISS task=", scheduler_.taskName(i), " new=", misses - reported_misses_[i],
                                " total=", misses));
      reported_misses_[i] = misses;
    }
  }
}

// --------------------------------------------------------------------------- helpers ----

void logRxTimeoutEvent(const core::Logger& log, const can::RxTimeoutEvent& event) {
  const std::string silence_ms = fixed(core::toMillis(event.silence), 1);
  if (event.current == can::RxTimeoutState::TimedOut) {
    log.warning(core::strCat("RX_TIMEOUT id=", can::formatId(event.id), " silence_ms=", silence_ms,
                             " previous=", can::toString(event.previous)));
  } else if (event.previous == can::RxTimeoutState::TimedOut) {
    log.info(core::strCat("RX_RECOVERED id=", can::formatId(event.id), " gap_ms=", silence_ms));
  } else {
    log.info(core::strCat("RX_FIRST id=", can::formatId(event.id), " after_ms=", silence_ms));
  }
}

std::string fixed(double value, int decimals) {
  std::array<char, 32U> buffer{};
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
  (void)std::snprintf(buffer.data(), buffer.size(), "%.*f", decimals, value);
  return std::string{buffer.data()};
}

}  // namespace vecu::app
