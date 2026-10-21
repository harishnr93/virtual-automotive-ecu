#include "common/ecu_runtime.hpp"

#include <array>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <utility>

#include "can/can_channel_factory.hpp"
#include "common/shutdown_signal.hpp"
#include "core/os_port.hpp"
#include "core/timing_report.hpp"

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
  trace_.write(frame, core::wallClockMicros());
  return true;
}

void CanIo::traceReceived(const can::RxFrame& entry) { trace_.write(entry.frame, entry.wall_time_us); }

// ------------------------------------------------------------------------- EcuPlatform ----

EcuPlatform::EcuPlatform(std::string ecu_name, const AppOptions& options)
    : ecu_name_(std::move(ecu_name)), options_(options), log_(ecu_name_), scheduler_(schedulerConfig(options)) {}

EcuPlatform::~EcuPlatform() {
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
  return true;
}

bool EcuPlatform::addTask(sched::TaskConfig config, sched::TaskFunction function) {
  std::string reason;
  if (!sched::validateTaskConfig(config, reason)) {
    log_.error(core::strCat("invalid task '", config.name, "': ", reason));
    return false;
  }
  const std::string name = config.name;
  const sched::SchedulerStatus status = scheduler_.addTask(std::move(config), std::move(function));
  if (status != sched::SchedulerStatus::Ok) {
    log_.error(core::strCat("cannot add task '", name, "': ", sched::toString(status)));
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
  const sched::SchedulerStatus status = scheduler_.start();
  if (status != sched::SchedulerStatus::Ok) {
    log_.error(core::strCat("scheduler start failed: ", sched::toString(status)));
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

  // The main thread only supervises run time; all ECU work happens in the tasks.
  const core::TimePoint start = core::now();
  const auto limit = std::chrono::duration<double>(options_.duration_s);
  while (!shutdownRequested() && ((options_.duration_s <= 0.0) || ((core::now() - start) < limit))) {
    os::sleepUntil(core::now() + std::chrono::milliseconds{10});
  }

  scheduler_.stop();
  receiver_->stop();

  const std::vector<sched::TaskStatsSnapshot> stats = scheduler_.allTaskStats();
  log_.info(core::strCat("scheduler timing report (soft real-time, measured):\n", sched::formatTimingTable(stats)));
  for (const sched::TaskStatsSnapshot& task : stats) {
    log_.info(sched::formatTimingSummaryLine(task));
  }
  log_.info(core::strCat("CAN_RX_SUMMARY received=", receiver_->receivedFrames(),
                         " errors=", receiver_->receiveErrors(), " queue_overruns=", rx_queue_.overruns()));
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
