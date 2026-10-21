#include "core/timing_report.hpp"

#include <iomanip>
#include <sstream>

namespace vecu::sched {

namespace {

double us(Nanos value) { return static_cast<double>(value.count()) / 1000.0; }
double usFromNs(double value_ns) { return value_ns / 1000.0; }

template <typename Duration>
double usOf(Duration value) {
  return static_cast<double>(std::chrono::duration_cast<Nanos>(value).count()) / 1000.0;
}

/// Type-safe fixed-point formatting helpers (no printf-style varargs).
struct Fixed {
  double value;
  int decimals;
};

std::ostream& operator<<(std::ostream& stream, const Fixed& f) {
  const auto flags = stream.flags();
  const auto precision = stream.precision();
  stream << std::fixed << std::setprecision(f.decimals) << f.value;
  stream.flags(flags);
  stream.precision(precision);
  return stream;
}

std::string escapeJson(std::string_view text) {
  std::string out;
  for (const char c : text) {
    if ((c == '"') || (c == '\\')) {
      out.push_back('\\');
    }
    out.push_back(c);
  }
  return out;
}

void jsonStats(std::ostream& out, const char* key, const DurationStats& stats) {
  out << '"' << key << R"(": {"min_us": )" << Fixed{us(stats.min()), 3} << R"(, "mean_us": )"
      << Fixed{usFromNs(stats.meanNs()), 3} << R"(, "max_us": )" << Fixed{us(stats.max()), 3}
      << R"(, "stddev_us": )" << Fixed{usFromNs(stats.stddevNs()), 3} << '}';
}

}  // namespace

std::string formatTimingTable(const std::vector<TaskStatsSnapshot>& tasks) {
  std::ostringstream out;
  out << std::left << std::setw(14) << "task" << std::right << std::setw(7) << "period" << std::setw(5) << "prio"
      << ' ' << std::left << std::setw(11) << "policy" << std::right << std::setw(7) << "acts" << std::setw(7)
      << "dlmiss" << std::setw(6) << "skip" << std::setw(7) << "budget" << " |" << std::setw(9) << "exec_avg"
      << std::setw(10) << "exec_p99" << std::setw(10) << "exec_max" << " |" << std::setw(9) << "lat_avg"
      << std::setw(10) << "lat_p99" << std::setw(10) << "lat_max" << " |" << std::setw(9) << "jit_max" << '\n';
  out << std::left << std::setw(14) << "" << std::right << std::setw(7) << "[ms]" << std::setw(46) << "|"
      << std::setw(9) << "[us]" << std::setw(10) << "[us]" << std::setw(10) << "[us]" << " |" << std::setw(9)
      << "[us]" << std::setw(10) << "[us]" << std::setw(10) << "[us]" << " |" << std::setw(9) << "[us]" << '\n';
  for (const TaskStatsSnapshot& task : tasks) {
    const TaskTimingSummary& t = task.timing;
    out << std::left << std::setw(14) << task.name.substr(0U, 13U) << std::right << std::setw(7)
        << Fixed{usOf(task.period) / 1000.0, 1} << std::setw(5) << task.priority << ' ' << std::left
        << std::setw(11) << os::toString(task.scheduling_class) << std::right << std::setw(7) << t.activations
        << std::setw(7) << t.deadline_misses << std::setw(6) << t.skipped_activations << std::setw(7)
        << t.budget_overruns << " |" << std::setw(9) << Fixed{usFromNs(t.execution_time.meanNs()), 1}
        << std::setw(10) << Fixed{us(t.execution_p99), 1} << std::setw(10) << Fixed{us(t.execution_time.max()), 1}
        << " |" << std::setw(9) << Fixed{usFromNs(t.start_latency.meanNs()), 1} << std::setw(10)
        << Fixed{us(t.start_latency_p99), 1} << std::setw(10) << Fixed{us(t.start_latency.max()), 1} << " |"
        << std::setw(9) << Fixed{us(t.period_jitter.max()), 1} << '\n';
  }
  out << "estimated CPU utilisation (sum of mean exec / period): " << Fixed{estimatedUtilization(tasks) * 100.0, 3}
      << " %\n";
  return out.str();
}

std::string formatTimingSummaryLine(const TaskStatsSnapshot& task) {
  const TaskTimingSummary& t = task.timing;
  std::ostringstream out;
  out << "SCHED_SUMMARY task=" << task.name << " period_ms=" << Fixed{usOf(task.period) / 1000.0, 3}
      << " prio=" << task.priority << " policy=" << os::toString(task.scheduling_class)
      << " activations=" << t.activations << " deadline_misses=" << t.deadline_misses
      << " skipped=" << t.skipped_activations << " budget_overruns=" << t.budget_overruns
      << " exec_mean_us=" << Fixed{usFromNs(t.execution_time.meanNs()), 1}
      << " exec_p99_us=" << Fixed{us(t.execution_p99), 1} << " exec_max_us=" << Fixed{us(t.execution_time.max()), 1}
      << " latency_mean_us=" << Fixed{usFromNs(t.start_latency.meanNs()), 1}
      << " latency_p99_us=" << Fixed{us(t.start_latency_p99), 1}
      << " latency_max_us=" << Fixed{us(t.start_latency.max()), 1}
      << " jitter_max_us=" << Fixed{us(t.period_jitter.max()), 1}
      << " response_max_us=" << Fixed{us(t.response_time.max()), 1};
  return out.str();
}

double estimatedUtilization(const std::vector<TaskStatsSnapshot>& tasks) {
  double utilization = 0.0;
  for (const TaskStatsSnapshot& task : tasks) {
    const double period_ns = static_cast<double>(std::chrono::duration_cast<Nanos>(task.period).count());
    if ((period_ns > 0.0) && (task.timing.activations > 0U)) {
      utilization += task.timing.execution_time.meanNs() / period_ns;
    }
  }
  return utilization;
}

std::string formatTimingJson(const std::vector<TaskStatsSnapshot>& tasks, std::string_view label,
                             bool memory_locked) {
  std::ostringstream out;
  out << "{\n  \"label\": \"" << escapeJson(label) << "\",\n";
  out << R"(  "memory_locked": )" << (memory_locked ? "true" : "false") << ",\n";
  out << R"(  "estimated_utilization": )" << Fixed{estimatedUtilization(tasks), 6} << ",\n";
  out << R"(  "tasks": [)" << '\n';
  for (std::size_t i = 0U; i < tasks.size(); ++i) {
    const TaskStatsSnapshot& task = tasks[i];
    const TaskTimingSummary& t = task.timing;
    out << R"(    {"name": ")" << escapeJson(task.name) << R"(", "period_us": )" << Fixed{usOf(task.period), 0}
        << R"(, "offset_us": )" << Fixed{usOf(task.offset), 0} << R"(, "deadline_us": )"
        << Fixed{usOf(task.deadline), 0} << R"(, "budget_us": )" << Fixed{usOf(task.budget), 0}
        << R"(, "priority": )" << task.priority << R"(, "policy": ")" << os::toString(task.scheduling_class)
        << R"(", "activations": )" << t.activations << R"(, "deadline_misses": )" << t.deadline_misses
        << R"(, "skipped_activations": )" << t.skipped_activations << R"(, "budget_overruns": )"
        << t.budget_overruns << ",\n     ";
    jsonStats(out, "execution_time", t.execution_time);
    out << ",\n     ";
    jsonStats(out, "start_latency", t.start_latency);
    out << ",\n     ";
    jsonStats(out, "response_time", t.response_time);
    out << ",\n     ";
    jsonStats(out, "period_jitter", t.period_jitter);
    out << ",\n     " << R"("execution_p50_us": )" << Fixed{us(t.execution_p50), 3} << R"(, "execution_p99_us": )"
        << Fixed{us(t.execution_p99), 3} << R"(, "start_latency_p50_us": )" << Fixed{us(t.start_latency_p50), 3}
        << R"(, "start_latency_p99_us": )" << Fixed{us(t.start_latency_p99), 3} << '}'
        << ((i + 1U < tasks.size()) ? ",\n" : "\n");
  }
  out << "  ]\n}\n";
  return out.str();
}

}  // namespace vecu::sched
