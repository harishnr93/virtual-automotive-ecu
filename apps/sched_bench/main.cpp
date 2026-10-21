// Scheduler timing benchmark.
//
// Runs the three standard ECU task rates (10 / 20 / 100 ms) with configurable synthetic
// execution times and optional background CPU stress, then reports measured execution time,
// start latency, period jitter, deadline misses and skipped activations.
//
//   vecu_sched_bench --duration 10
//   vecu_sched_bench --duration 10 --load-10ms 2000 --stress 16
//   vecu_sched_bench --duration 10 --realtime --lock-memory --json timing.json

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "common/shutdown_signal.hpp"
#include "core/clock.hpp"
#include "core/os_port.hpp"
#include "core/periodic_scheduler.hpp"
#include "core/timing_report.hpp"

namespace {

namespace core = vecu::core;
namespace os = vecu::os;
namespace sched = vecu::sched;
using std::chrono::microseconds;

struct BenchOptions {
  double duration_s{5.0};
  bool realtime{false};
  bool lock_memory{false};
  long load_10ms_us{100};
  long load_20ms_us{200};
  long load_100ms_us{500};
  long stress_threads{0};
  std::string json_path{};
  std::string label{"sched_bench"};
};

/// Busy-waits for `duration`: a synthetic, CPU-bound task body.
void busyWait(microseconds duration) {
  const core::TimePoint end = core::now() + duration;
  while (core::now() < end) {
  }
}

bool parseLong(const char* text, long& value) {
  char* end = nullptr;
  errno = 0;
  const long parsed = std::strtol(text, &end, 10);
  if ((errno != 0) || (end == text) || (*end != '\0') || (parsed < 0)) {
    return false;
  }
  value = parsed;
  return true;
}

void printUsage(const char* program) {
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
  std::printf(
      "Scheduler timing benchmark\n\nUsage: %s [options]\n"
      "  --duration S        run time in seconds (default 5)\n"
      "  --load-10ms US      busy time of the 10 ms task in microseconds (default 100)\n"
      "  --load-20ms US      busy time of the 20 ms task (default 200)\n"
      "  --load-100ms US     busy time of the 100 ms task (default 500)\n"
      "  --stress N          N background threads spinning at normal priority (default 0)\n"
      "  --realtime          request SCHED_FIFO priorities\n"
      "  --lock-memory       mlockall()\n"
      "  --json FILE         write the timing report as JSON\n"
      "  --label TEXT        label stored in the JSON report\n",
      program);
}

int parse(int argc, char** argv, BenchOptions& options) {
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg{argv[i]};
    if ((arg == "-h") || (arg == "--help")) {
      printUsage(argv[0]);
      return 0;
    }
    if (arg == "--realtime") {
      options.realtime = true;
      continue;
    }
    if (arg == "--lock-memory") {
      options.lock_memory = true;
      continue;
    }
    if ((i + 1) >= argc) {
      (void)std::fprintf(stderr, "error: missing value for %s\n", argv[i]);  // NOLINT(cppcoreguidelines-pro-type-vararg)
      return 2;
    }
    const char* value = argv[++i];
    bool ok = true;
    if (arg == "--duration") {
      char* end = nullptr;
      options.duration_s = std::strtod(value, &end);
      ok = (end != value) && (*end == '\0') && (options.duration_s > 0.0);
    } else if (arg == "--load-10ms") {
      ok = parseLong(value, options.load_10ms_us);
    } else if (arg == "--load-20ms") {
      ok = parseLong(value, options.load_20ms_us);
    } else if (arg == "--load-100ms") {
      ok = parseLong(value, options.load_100ms_us);
    } else if (arg == "--stress") {
      ok = parseLong(value, options.stress_threads) && (options.stress_threads <= 256);
    } else if (arg == "--json") {
      options.json_path = value;
    } else if (arg == "--label") {
      options.label = value;
    } else {
      ok = false;
    }
    if (!ok) {
      (void)std::fprintf(stderr, "error: invalid option %s %s\n", argv[i - 1], value);  // NOLINT(cppcoreguidelines-pro-type-vararg)
      return 2;
    }
  }
  return -1;  // continue
}

}  // namespace

int main(int argc, char** argv) {
  BenchOptions options;
  const int parse_result = parse(argc, argv, options);
  if (parse_result >= 0) {
    return parse_result;
  }
  vecu::app::installShutdownHandlers();

  sched::SchedulerConfig config;
  config.request_realtime = options.realtime;
  config.lock_memory = options.lock_memory;
  sched::PeriodicScheduler scheduler(config);

  const microseconds load10{options.load_10ms_us};
  const microseconds load20{options.load_20ms_us};
  const microseconds load100{options.load_100ms_us};
  const bool added =
      (scheduler.addTask({"Task_10ms", microseconds{10000}, microseconds{0}, microseconds{0}, microseconds{0}, 80},
                         [load10](const sched::ActivationInfo&) { busyWait(load10); }) == sched::SchedulerStatus::Ok) &&
      (scheduler.addTask({"Task_20ms", microseconds{20000}, microseconds{1000}, microseconds{0}, microseconds{0}, 70},
                         [load20](const sched::ActivationInfo&) { busyWait(load20); }) == sched::SchedulerStatus::Ok) &&
      (scheduler.addTask({"Task_100ms", microseconds{100000}, microseconds{3000}, microseconds{0}, microseconds{0}, 50},
                         [load100](const sched::ActivationInfo&) { busyWait(load100); }) == sched::SchedulerStatus::Ok);
  if (!added) {
    (void)std::fprintf(stderr, "error: invalid task configuration\n");  // NOLINT(cppcoreguidelines-pro-type-vararg)
    return 1;
  }

  // Background load: spinning threads at normal priority compete for the CPUs.
  std::atomic<bool> stop_stress{false};
  std::vector<std::unique_ptr<os::Thread>> stressors;
  for (long i = 0; i < options.stress_threads; ++i) {
    auto thread = std::make_unique<os::Thread>();
    os::ThreadConfig thread_config;
    thread_config.name = "Stress" + std::to_string(i);
    if (thread->start(thread_config, [&stop_stress] {
          while (!stop_stress.load(std::memory_order_relaxed)) {
          }
        })) {
      stressors.push_back(std::move(thread));
    }
  }

  if (scheduler.start() != sched::SchedulerStatus::Ok) {
    stop_stress = true;
    (void)std::fprintf(stderr, "error: scheduler start failed\n");  // NOLINT(cppcoreguidelines-pro-type-vararg)
    return 1;
  }
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
  std::printf("running %.1f s: loads 10ms=%ldus 20ms=%ldus 100ms=%ldus, stress threads=%zu, policy=%s, "
              "memory_locked=%s\n",
              options.duration_s, options.load_10ms_us, options.load_20ms_us, options.load_100ms_us, stressors.size(),
              os::toString(scheduler.taskStats(0U).scheduling_class), scheduler.memoryLocked() ? "yes" : "no");
  (void)std::fflush(stdout);

  const core::TimePoint start = core::now();
  const auto limit = std::chrono::duration<double>(options.duration_s);
  while (!vecu::app::shutdownRequested() && ((core::now() - start) < limit)) {
    os::sleepUntil(core::now() + std::chrono::milliseconds{10});
  }
  scheduler.stop();
  stop_stress = true;
  stressors.clear();  // joins

  const std::vector<sched::TaskStatsSnapshot> stats = scheduler.allTaskStats();
  (void)std::fputs(sched::formatTimingTable(stats).c_str(), stdout);
  for (const sched::TaskStatsSnapshot& task : stats) {
    (void)std::puts(sched::formatTimingSummaryLine(task).c_str());
  }
  if (!options.json_path.empty()) {
    std::ofstream file(options.json_path);
    file << sched::formatTimingJson(stats, options.label, scheduler.memoryLocked());
    if (!file) {
      (void)std::fprintf(stderr, "error: cannot write %s\n", options.json_path.c_str());  // NOLINT(cppcoreguidelines-pro-type-vararg)
      return 1;
    }
  }
  return 0;
}
