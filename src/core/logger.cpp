#include "core/logger.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdio>
#include <mutex>

#include "core/clock.hpp"

namespace vecu::core {

namespace {

// Process-wide logger state. Internal linkage: not visible outside this file.
std::atomic<LogLevel> g_min_level{LogLevel::Info};  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
std::mutex g_output_mutex;                          // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
const TimePoint g_process_start = now();            // NOLINT(cert-err58-cpp)

constexpr std::array<const char*, 4U> kLevelNames{"DEBUG", "INFO ", "WARN ", "ERROR"};

}  // namespace

Logger::Logger(std::string component) : component_(std::move(component)) {}

void Logger::log(LogLevel level, std::string_view message) const {
  if (!isEnabled(level)) {
    return;
  }
  const double elapsed_s = toMillis(now() - g_process_start) / 1000.0;
  const std::lock_guard<std::mutex> lock(g_output_mutex);
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
  (void)std::fprintf(stderr, "[%11.6f] [%s] [%s] %.*s\n", elapsed_s,
                     kLevelNames.at(static_cast<std::size_t>(level)), component_.c_str(),
                     static_cast<int>(message.size()), message.data());
}

void Logger::setMinLevel(LogLevel level) noexcept { g_min_level.store(level); }

bool Logger::isEnabled(LogLevel level) noexcept {
  return static_cast<std::uint8_t>(level) >= static_cast<std::uint8_t>(g_min_level.load());
}

}  // namespace vecu::core
