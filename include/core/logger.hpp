#pragma once

#include <cstdint>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>

namespace vecu::core {

enum class LogLevel : std::uint8_t { Debug = 0U, Info = 1U, Warning = 2U, Error = 3U };

/// Small thread-safe console logger.
///
/// Intended for start-up messages, state changes and periodic summaries. It formats
/// strings (heap allocation), so it must not be called per frame from time-critical
/// code paths. Output goes to stderr; each line is written atomically under a mutex.
class Logger {
 public:
  explicit Logger(std::string component);

  void log(LogLevel level, std::string_view message) const;
  void debug(std::string_view message) const { log(LogLevel::Debug, message); }
  void info(std::string_view message) const { log(LogLevel::Info, message); }
  void warning(std::string_view message) const { log(LogLevel::Warning, message); }
  void error(std::string_view message) const { log(LogLevel::Error, message); }

  /// Process-wide threshold; messages below it are discarded cheaply.
  static void setMinLevel(LogLevel level) noexcept;
  [[nodiscard]] static bool isEnabled(LogLevel level) noexcept;

 private:
  std::string component_;
};

/// Concatenates streamable values into one string, e.g. strCat("speed=", 42.0, " km/h").
template <typename... Args>
[[nodiscard]] std::string strCat(Args&&... args) {
  std::ostringstream stream;
  (stream << ... << std::forward<Args>(args));
  return stream.str();
}

}  // namespace vecu::core
