// Virtual Cruise Control ECU — entry point. The application is in cruise_ecu.cpp, its fault
// configuration in cruise_faults.hpp.

#include <cstdio>
#include <string>

#include "common/app_options.hpp"
#include "common/ecu_runtime.hpp"
#include "common/shutdown_signal.hpp"
#include "core/logger.hpp"
#include "cruise_ecu/cruise_ecu.hpp"

int main(int argc, char** argv) {
  namespace app = vecu::app;
  app::AppOptions options;
  std::string error;
  switch (app::parseOptions(argc, argv, options, error)) {
    case app::ParseOutcome::ShowHelp:
      app::printUsage(argv[0], "Virtual Cruise Control ECU (PID control, UDS diagnostics, fault management, watchdog)");
      return 0;
    case app::ParseOutcome::Error:
      (void)std::fprintf(stderr, "error: %s (use --help)\n", error.c_str());  // NOLINT(cppcoreguidelines-pro-type-vararg)
      return 2;
    case app::ParseOutcome::Run:
      break;
  }
  vecu::core::Logger::setMinLevel(options.verbose ? vecu::core::LogLevel::Debug : vecu::core::LogLevel::Info);
  app::installShutdownHandlers();

  app::EcuPlatform platform("cruise_ecu", options);
  if (!platform.init()) {
    return 1;
  }
  vecu::cruise::CruiseEcu ecu(platform);
  if (!ecu.initialize()) {
    return 1;
  }
  const int exit_code = platform.run();
  ecu.logSummary();
  return exit_code;
}
