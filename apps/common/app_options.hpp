#pragma once

#include <cstdint>
#include <string>
#include <string_view>

#include "can/can_channel_factory.hpp"

namespace vecu::app {

/// Fault-injection window [start_s, end_s) relative to ECU start, in seconds.
struct MuteWindow {
  double start_s{-1.0};
  double end_s{-1.0};

  [[nodiscard]] bool enabled() const noexcept { return start_s >= 0.0; }
  [[nodiscard]] bool contains(double t_s) const noexcept {
    return enabled() && (t_s >= start_s) && (t_s < end_s);
  }
};

/// Command-line options shared by the virtual ECU executables.
struct AppOptions {
  can::CanChannelConfig can{};
  double duration_s{0.0};     ///< 0 = run until Ctrl+C / SIGTERM
  std::string can_log_path{}; ///< candump-format trace file (empty = no trace)
  bool verbose{false};        ///< debug logging
  MuteWindow mute_tx{};       ///< sensor ECU: suppress VehicleDynamics transmission
};

enum class ParseOutcome : std::uint8_t { Run, ShowHelp, Error };

[[nodiscard]] ParseOutcome parseOptions(int argc, const char* const* argv, AppOptions& options,
                                        std::string& error);

void printUsage(std::string_view program, std::string_view description);

}  // namespace vecu::app
