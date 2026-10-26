#pragma once

// TEST-ONLY fault injection for system tests and demonstrations.
//
// Injections are given on the command line (--inject SPEC) and executed inside the task
// threads at a defined time after the scheduler epoch:
//   stall:TASK:AT:DURATION          the task blocks once for DURATION seconds
//   overrun:TASK:AT:DURATION:EXTRA  each activation in [AT, AT+DURATION) runs EXTRA ms longer
//   hang:TASK:AT                    the task blocks until the ECU shuts down
//   controller-nan:AT               the next control step receives a NaN time step
// A production build would compile this out; here it is inactive unless options are given.

#include <array>
#include <atomic>
#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

#include "core/logger.hpp"

namespace vecu::app {

class FaultInjector {
 public:
  static constexpr std::size_t kMaxInjections = 16U;

  enum class Kind : std::uint8_t { Stall, Overrun, Hang, ControllerNan };

  struct Injection {
    Kind kind{Kind::Stall};
    std::string task{};
    double at_s{0.0};
    double duration_s{0.0};
    double extra_ms{0.0};
  };

  /// Parses and adds one specification (initialisation phase).
  [[nodiscard]] bool add(std::string_view spec, std::string& error);

  /// Executes due stall/overrun/hang injections for `task` (called at activation start).
  void beforeTask(std::string_view task, double t_s);

  /// True exactly once when a controller-nan injection is due.
  [[nodiscard]] bool takeControllerNan(double t_s);

  /// Ends all hangs (called during shutdown).
  void release() noexcept { released_ = true; }
  [[nodiscard]] bool released() const noexcept { return released_.load(); }

  [[nodiscard]] bool empty() const noexcept { return injections_.empty(); }
  [[nodiscard]] const std::vector<Injection>& injections() const noexcept { return injections_; }

 private:
  std::vector<Injection> injections_;
  std::array<std::atomic<bool>, kMaxInjections> fired_{};
  std::atomic<bool> released_{false};
  core::Logger log_{"inject"};
};

}  // namespace vecu::app
