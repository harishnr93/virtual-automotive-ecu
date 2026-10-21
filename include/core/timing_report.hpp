#pragma once

// Human- and machine-readable reports of scheduler timing measurements.

#include <string>
#include <string_view>
#include <vector>

#include "core/periodic_scheduler.hpp"

namespace vecu::sched {

/// Fixed-width table for the console.
[[nodiscard]] std::string formatTimingTable(const std::vector<TaskStatsSnapshot>& tasks);

/// One "SCHED_SUMMARY key=value ..." line per task (parsed by the integration tests).
[[nodiscard]] std::string formatTimingSummaryLine(const TaskStatsSnapshot& task);

/// Estimated CPU utilisation U = sum(mean execution time / period), as a fraction.
[[nodiscard]] double estimatedUtilization(const std::vector<TaskStatsSnapshot>& tasks);

/// JSON document (no external library) for later analysis / plotting.
[[nodiscard]] std::string formatTimingJson(const std::vector<TaskStatsSnapshot>& tasks, std::string_view label,
                                           bool memory_locked);

}  // namespace vecu::sched
