#include "common/app_options.hpp"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace vecu::app {

namespace {

bool parseDouble(std::string_view text, double& value) {
  const std::string copy{text};
  char* end = nullptr;
  errno = 0;
  const double parsed = std::strtod(copy.c_str(), &end);
  if ((errno != 0) || copy.empty() || (end != (copy.c_str() + copy.size())) || !std::isfinite(parsed)) {
    return false;
  }
  value = parsed;
  return true;
}

bool parsePort(std::string_view text, std::uint16_t& port) {
  const std::string copy{text};
  char* end = nullptr;
  errno = 0;
  const unsigned long parsed = std::strtoul(copy.c_str(), &end, 10);
  if ((errno != 0) || copy.empty() || (end != (copy.c_str() + copy.size())) || (parsed == 0UL) ||
      (parsed > 65535UL)) {
    return false;
  }
  port = static_cast<std::uint16_t>(parsed);
  return true;
}

/// Parses "START:END" in seconds, e.g. "1.0:1.5".
bool parseWindow(std::string_view text, MuteWindow& window) {
  const std::size_t colon = text.find(':');
  if (colon == std::string_view::npos) {
    return false;
  }
  MuteWindow parsed;
  if (!parseDouble(text.substr(0U, colon), parsed.start_s) ||
      !parseDouble(text.substr(colon + 1U), parsed.end_s) || (parsed.start_s < 0.0) ||
      (parsed.end_s <= parsed.start_s)) {
    return false;
  }
  window = parsed;
  return true;
}

}  // namespace

ParseOutcome parseOptions(int argc, const char* const* argv, AppOptions& options, std::string& error) {
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg{argv[i]};
    if ((arg == "-h") || (arg == "--help")) {
      return ParseOutcome::ShowHelp;
    }
    if (arg == "--verbose") {
      options.verbose = true;
      continue;
    }
    if (arg == "--realtime") {
      options.realtime = true;
      continue;
    }
    if (arg == "--lock-memory") {
      options.lock_memory = true;
      continue;
    }
    // All remaining options take exactly one value.
    if ((i + 1) >= argc) {
      error = "missing value for option " + std::string{arg};
      return ParseOutcome::Error;
    }
    const std::string_view value{argv[++i]};
    bool ok = true;
    if (arg == "--bus") {
      ok = can::parseBackend(value, options.can.backend);
    } else if (arg == "--iface") {
      options.can.socketcan_interface = std::string{value};
    } else if (arg == "--udp-group") {
      options.can.udp.group_address = std::string{value};
    } else if (arg == "--udp-port") {
      ok = parsePort(value, options.can.udp.port);
    } else if (arg == "--duration") {
      ok = parseDouble(value, options.duration_s) && (options.duration_s >= 0.0);
    } else if (arg == "--log") {
      options.can_log_path = std::string{value};
    } else if (arg == "--timing-report") {
      options.timing_report_path = std::string{value};
    } else if (arg == "--mute-tx") {
      ok = parseWindow(value, options.mute_tx);
    } else {
      error = "unknown option " + std::string{arg};
      return ParseOutcome::Error;
    }
    if (!ok) {
      error = "invalid value '" + std::string{value} + "' for option " + std::string{arg};
      return ParseOutcome::Error;
    }
  }
  return ParseOutcome::Run;
}

void printUsage(std::string_view program, std::string_view description) {
  // NOLINTBEGIN(cppcoreguidelines-pro-type-vararg)
  std::printf(
      "%.*s\n\nUsage: %.*s [options]\n\n"
      "  --bus auto|socketcan|udp  CAN backend (default: auto = vcan0 if present, else UDP bus)\n"
      "  --iface NAME              SocketCAN interface (default: vcan0)\n"
      "  --udp-group ADDR          UDP bus multicast group (default: 239.255.0.1)\n"
      "  --udp-port PORT           UDP bus port (default: 47000)\n"
      "  --duration SECONDS        stop after this time (default: run until Ctrl+C)\n"
      "  --log FILE                write a candump-format CAN trace\n"
      "  --mute-tx START:END       sensor ECU fault injection: stop sending 0x100 in [START,END) s\n"
      "  --realtime                request SCHED_FIFO priorities (needs rtprio permission)\n"
      "  --lock-memory             lock process memory in RAM (mlockall)\n"
      "  --timing-report FILE      write scheduler timing statistics as JSON\n"
      "  --verbose                 debug logging\n"
      "  -h, --help                show this help\n",
      static_cast<int>(description.size()), description.data(), static_cast<int>(program.size()),
      program.data());
  // NOLINTEND(cppcoreguidelines-pro-type-vararg)
}

}  // namespace vecu::app
