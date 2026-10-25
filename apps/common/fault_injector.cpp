#include "common/fault_injector.hpp"

#include <chrono>
#include <cstdlib>

#include "common/ecu_runtime.hpp"
#include "core/clock.hpp"
#include "core/os_port.hpp"

namespace vecu::app {

namespace {

std::vector<std::string> split(std::string_view text, char separator) {
  std::vector<std::string> parts;
  std::size_t start = 0U;
  while (true) {
    const std::size_t end = text.find(separator, start);
    parts.emplace_back(text.substr(start, (end == std::string_view::npos) ? std::string_view::npos : end - start));
    if (end == std::string_view::npos) {
      return parts;
    }
    start = end + 1U;
  }
}

bool number(const std::string& text, double& value) {
  char* end = nullptr;
  value = std::strtod(text.c_str(), &end);
  return !text.empty() && (end == text.c_str() + text.size()) && (value >= 0.0);
}

void busyWait(double ms) {
  const core::TimePoint end =
      core::now() + std::chrono::duration_cast<core::MonotonicClock::duration>(std::chrono::duration<double, std::milli>(ms));
  while (core::now() < end) {
  }
}

}  // namespace

// cppcheck-suppress constParameter ; false positive: `error` is assigned below
bool FaultInjector::add(std::string_view spec, std::string& error) {
  const std::vector<std::string> f = split(spec, ':');
  Injection injection;
  bool ok = false;
  if ((f[0] == "stall") && (f.size() == 4U)) {
    injection.kind = Kind::Stall;
    ok = number(f[2], injection.at_s) && number(f[3], injection.duration_s);
  } else if ((f[0] == "overrun") && (f.size() == 5U)) {
    injection.kind = Kind::Overrun;
    ok = number(f[2], injection.at_s) && number(f[3], injection.duration_s) && number(f[4], injection.extra_ms);
  } else if ((f[0] == "hang") && (f.size() == 3U)) {
    injection.kind = Kind::Hang;
    ok = number(f[2], injection.at_s);
  } else if ((f[0] == "controller-nan") && (f.size() == 2U)) {
    injection.kind = Kind::ControllerNan;
    ok = number(f[1], injection.at_s);
  } else {
    ok = false;
  }
  if (injection.kind != Kind::ControllerNan && ok) {
    injection.task = f[1];
    ok = !injection.task.empty();
  }
  if (!ok || (injections_.size() >= kMaxInjections)) {
    error = "invalid fault injection '" + std::string{spec} + "'";
    return false;
  }
  injections_.push_back(injection);
  return true;
}

void FaultInjector::beforeTask(std::string_view task, double t_s) {
  for (std::size_t i = 0U; i < injections_.size(); ++i) {
    const Injection& inj = injections_[i];
    if ((inj.task != task) || (t_s < inj.at_s)) {
      continue;
    }
    switch (inj.kind) {
      case Kind::Stall:
        if (!fired_[i].exchange(true)) {
          log_.warning(core::strCat("FAULT_INJECTION stall task=", task, " t=", fixed(t_s, 3), "s duration_ms=",
                                    fixed(inj.duration_s * 1000.0, 0)));
          os::sleepUntil(core::now() + std::chrono::duration_cast<core::MonotonicClock::duration>(
                                           std::chrono::duration<double>(inj.duration_s)));
        }
        break;
      case Kind::Overrun:
        if (t_s < inj.at_s + inj.duration_s) {
          if (!fired_[i].exchange(true)) {
            log_.warning(core::strCat("FAULT_INJECTION overrun task=", task, " t=", fixed(t_s, 3), "s extra_ms=",
                                      fixed(inj.extra_ms, 1), " duration_s=", fixed(inj.duration_s, 1)));
          }
          busyWait(inj.extra_ms);
        }
        break;
      case Kind::Hang:
        if (!fired_[i].exchange(true)) {
          log_.warning(core::strCat("FAULT_INJECTION hang task=", task, " t=", fixed(t_s, 3), "s"));
          while (!released_.load()) {
            os::sleepUntil(core::now() + std::chrono::milliseconds{10});
          }
        }
        break;
      case Kind::ControllerNan:
        break;
    }
  }
}

bool FaultInjector::takeControllerNan(double t_s) {
  for (std::size_t i = 0U; i < injections_.size(); ++i) {
    if ((injections_[i].kind == Kind::ControllerNan) && (t_s >= injections_[i].at_s) && !fired_[i].exchange(true)) {
      log_.warning(core::strCat("FAULT_INJECTION controller-nan t=", fixed(t_s, 3), "s"));
      return true;
    }
  }
  return false;
}

}  // namespace vecu::app
