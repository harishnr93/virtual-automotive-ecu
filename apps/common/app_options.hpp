#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

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

/// Reaction of the ECU when the software watchdog expires (after entering the safe state).
enum class WatchdogAction : std::uint8_t {
  Reset,  ///< simulated in-process ECU reset once the stalled task responds again (default)
  Exit,   ///< save NVM and terminate with exit code 75 so an external supervisor restarts the ECU
  None    ///< safe state and DTC only
};

/// Command-line options shared by the virtual ECU executables.
struct AppOptions {
  can::CanChannelConfig can{};
  double duration_s{0.0};     ///< 0 = run until Ctrl+C / SIGTERM
  std::string can_log_path{}; ///< candump-format trace file (empty = no trace)
  bool verbose{false};        ///< debug logging
  MuteWindow mute_tx{};       ///< sensor ECU: suppress VehicleDynamics transmission
  bool realtime{false};       ///< request SCHED_FIFO priorities for tasks and CAN RX thread
  bool lock_memory{false};    ///< request mlockall()
  std::string timing_report_path{};  ///< write scheduler timing as JSON
  std::string nvm_path{};            ///< fault memory file (NVM simulation); empty = volatile
  WatchdogAction watchdog_action{WatchdogAction::Reset};
  std::vector<std::string> injections{};  ///< test-only fault injections (--inject, repeatable)
};

enum class ParseOutcome : std::uint8_t { Run, ShowHelp, Error };

[[nodiscard]] ParseOutcome parseOptions(int argc, const char* const* argv, AppOptions& options,
                                        std::string& error);

void printUsage(std::string_view program, std::string_view description);

}  // namespace vecu::app
