// Virtual Vehicle Sensor ECU.
//
// Publishes VehicleDynamics (0x100, 20 ms) and DriverInputs (0x110, 50 ms) from a synthetic
// speed profile and supervises CruiseControlStatus (0x200). In Phase 3 the Python vehicle
// simulator takes over this role; this ECU remains as a C++ traffic generator for tests.
//
// Phase 2 task structure:
//   CanRx thread  (prio 90) event-driven reception into the RX queue
//   Com_10ms      (prio 80) RX processing + supervision; TX with cycle dividers
//                           (0x100 every 2nd activation, 0x110 every 5th)
//   Diag_100ms    (prio 50) health monitoring and periodic status

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "can/alive_counter.hpp"
#include "can/can_receiver.hpp"
#include "can/can_signal.hpp"
#include "can/message_timeout_monitor.hpp"
#include "can/vehicle_signals.hpp"
#include "common/app_options.hpp"
#include "common/ecu_runtime.hpp"
#include "common/shutdown_signal.hpp"
#include "core/clock.hpp"
#include "core/logger.hpp"
#include "core/periodic_scheduler.hpp"

namespace {

namespace can = vecu::can;
namespace vehicle = vecu::can::vehicle;
namespace core = vecu::core;
namespace app = vecu::app;
namespace sched = vecu::sched;

using std::chrono::microseconds;
using std::chrono::milliseconds;

constexpr milliseconds kComPeriod{10};
constexpr milliseconds kFirstTimeout{1000};

constexpr std::uint64_t activationsPerCycle(const can::MessageDef& message) noexcept {
  return message.cycle_time_ms / static_cast<std::uint64_t>(kComPeriod.count());
}
static_assert(vehicle::kVehicleDynamics.cycle_time_ms % 10U == 0U, "cycle must be a multiple of the COM period");
static_assert(vehicle::kDriverInputs.cycle_time_ms % 10U == 0U, "cycle must be a multiple of the COM period");

struct VehicleSample {
  double speed_kph;
  double accel_mps2;
};

/// Synthetic profile: 0 -> 100 km/h in 10 s, back to 0 in 10 s, repeating.
VehicleSample syntheticProfile(double t_s) noexcept {
  constexpr double kPeriodS = 20.0;
  constexpr double kRateKphPerS = 10.0;
  const double phase = std::fmod(t_s, kPeriodS);
  if (phase < (kPeriodS / 2.0)) {
    return {kRateKphPerS * phase, kRateKphPerS / 3.6};
  }
  return {kRateKphPerS * (kPeriodS - phase), -kRateKphPerS / 3.6};
}

class SensorEcu {
 public:
  SensorEcu(app::EcuPlatform& platform, const app::MuteWindow& mute)
      : platform_(platform), log_(platform.log()), mute_(mute) {
    (void)monitor_.addMessage(vehicle::kCruiseControlStatus.id,
                              milliseconds{vehicle::kCruiseControlStatus.timeout_ms}, kFirstTimeout);
    monitor_.start(core::now());
  }

  [[nodiscard]] bool registerTasks() {
    sched::TaskConfig com{"Com_10ms", kComPeriod, microseconds{0}, microseconds{0}, microseconds{3000},
                          app::kPriority10ms};
    sched::TaskConfig diag{"Diag_100ms", microseconds{100000}, microseconds{5000}, microseconds{0},
                           microseconds{20000}, app::kPriority100ms};
    return platform_.addTask(com, [this](const sched::ActivationInfo& a) { taskCom(a); }) &&
           platform_.addTask(diag, [this](const sched::ActivationInfo& a) { taskDiag(a); });
  }

  void logSummary() const {
    log_.info(core::strCat("SUMMARY ecu=sensor_ecu bus=", platform_.busName(), " tx=", platform_.io().txOk(),
                           " tx_failed=", platform_.io().txFailed(), " muted=", muted_frames_.load(),
                           " rx_0x200=", rx_cruise_status_.load(), " invalid=", invalid_frames_.load(),
                           " alive_errors=", alive_errors_.load(), " timeouts=", timeout_events_.load(),
                           " recoveries=", recovery_events_.load()));
  }

 private:
  // ---------------------------------------------------------------- Com_10ms (prio 80) ----
  void taskCom(const sched::ActivationInfo& activation) {
    can::RxFrame entry;
    while (platform_.rxQueue().pop(entry)) {
      platform_.io().traceReceived(entry);
      if (entry.frame.id == vehicle::kCruiseControlStatus.id) {
        handleCruiseStatus(entry);
      }
    }
    monitor_.update(core::now(), [this](const can::RxTimeoutEvent& event) { onTimeoutEvent(event); });

    // Time base = planned release time, so the profile is independent of start jitter.
    const double t_s = core::toMillis(activation.release - platform_.scheduler().epoch()) / 1000.0;
    if ((activation.activation % activationsPerCycle(vehicle::kVehicleDynamics)) == 0U) {
      sendVehicleDynamics(t_s);
    }
    if ((activation.activation % activationsPerCycle(vehicle::kDriverInputs)) == 0U) {
      sendDriverInputs(t_s);
    }
  }

  void handleCruiseStatus(const can::RxFrame& entry) {
    const can::CanFrame& frame = entry.frame;
    if (frame.dlc != vehicle::kCruiseControlStatus.dlc) {
      invalid_frames_.fetch_add(1U);
      return;
    }
    rx_cruise_status_.fetch_add(1U);
    (void)monitor_.notifyReceived(frame.id, entry.rx_time, [this](const can::RxTimeoutEvent& e) { onTimeoutEvent(e); });

    double alive = 0.0;
    if (can::decodePhysical(vehicle::kCruiseStatusAlive, frame, alive) == can::SignalStatus::Ok) {
      const can::AliveCheckResult result = alive_checker_.check(static_cast<std::uint8_t>(alive));
      if ((result == can::AliveCheckResult::Repeated) || (result == can::AliveCheckResult::Skipped)) {
        alive_errors_.fetch_add(1U);
        log_.warning(core::strCat("ALIVE_ERROR id=0x200 result=", can::toString(result)));
      }
    }
    double state = 0.0;
    if (can::decodePhysical(vehicle::kCruiseState, frame, state) == can::SignalStatus::Ok) {
      cruise_state_.store(static_cast<std::uint8_t>(state));
    } else {
      invalid_frames_.fetch_add(1U);
    }
  }

  void sendVehicleDynamics(double t_s) {
    const VehicleSample sample = syntheticProfile(t_s);
    can::CanFrame frame = can::makeFrame(vehicle::kVehicleDynamics);
    (void)can::encodePhysical(vehicle::kVehicleSpeed, sample.speed_kph, frame);
    (void)can::encodePhysical(vehicle::kLongitudinalAccel, sample.accel_mps2, frame);
    (void)can::encodePhysical(vehicle::kVehicleSpeedValid, 1.0, frame);
    // The counter advances even while muted: muted frames count as "lost on the bus".
    (void)can::encodePhysical(vehicle::kVehicleDynamicsAlive, vehicle_dynamics_alive_.next(), frame);
    last_speed_kph_.store(sample.speed_kph);

    if (mute_.contains(t_s)) {
      if (muted_frames_.fetch_add(1U) == 0U) {
        log_.warning(core::strCat("FAULT_INJECTION mute 0x100 from t=", app::fixed(t_s, 3), "s"));
      }
      return;
    }
    (void)platform_.io().send(frame);
  }

  void sendDriverInputs(double t_s) {
    can::CanFrame frame = can::makeFrame(vehicle::kDriverInputs);
    const bool main_switch = t_s >= 1.0;
    (void)can::encodePhysical(vehicle::kAcceleratorPedal, 0.0, frame);
    (void)can::encodePhysical(vehicle::kBrakePedalPressed, 0.0, frame);
    (void)can::encodePhysical(vehicle::kCruiseMainSwitch, main_switch ? 1.0 : 0.0, frame);
    (void)can::encodePhysical(vehicle::kCruiseSetRequest, 0.0, frame);
    (void)can::encodePhysical(vehicle::kCruiseCancelRequest, 0.0, frame);
    (void)can::encodePhysical(vehicle::kDriverSetSpeed, 80.0, frame);
    (void)can::encodePhysical(vehicle::kDriverInputsAlive, driver_inputs_alive_.next(), frame);
    (void)platform_.io().send(frame);
  }

  void onTimeoutEvent(const can::RxTimeoutEvent& event) {
    app::logRxTimeoutEvent(log_, event);
    if (event.current == can::RxTimeoutState::TimedOut) {
      timeout_events_.fetch_add(1U);
      alive_checker_.reset();  // resynchronise the counter after a communication loss
    } else if (event.previous == can::RxTimeoutState::TimedOut) {
      recovery_events_.fetch_add(1U);
    } else {
      // first reception
    }
  }

  // -------------------------------------------------------------- Diag_100ms (prio 50) ----
  void taskDiag(const sched::ActivationInfo& activation) {
    platform_.reportNewDeadlineMisses();
    if ((activation.activation > 0U) && ((activation.activation % 10U) == 0U)) {
      const auto state = static_cast<vehicle::CruiseState>(cruise_state_.load());
      log_.info(core::strCat("speed_tx=", app::fixed(last_speed_kph_.load(), 2), "km/h cruise_state=",
                             vehicle::toString(state), " rx_0x200=", rx_cruise_status_.load(),
                             " tx=", platform_.io().txOk()));
    }
  }

  app::EcuPlatform& platform_;
  const core::Logger& log_;
  app::MuteWindow mute_;

  // Owned by Com_10ms only.
  can::MessageTimeoutMonitor<1U> monitor_{};
  can::AliveCounterGenerator vehicle_dynamics_alive_{};
  can::AliveCounterGenerator driver_inputs_alive_{};
  can::AliveCounterChecker alive_checker_{};

  // Written by Com_10ms, read by Diag_100ms and main.
  std::atomic<std::uint8_t> cruise_state_{0U};
  std::atomic<double> last_speed_kph_{0.0};
  std::atomic<std::uint64_t> rx_cruise_status_{0U};
  std::atomic<std::uint64_t> invalid_frames_{0U};
  std::atomic<std::uint64_t> alive_errors_{0U};
  std::atomic<std::uint64_t> muted_frames_{0U};
  std::atomic<std::uint32_t> timeout_events_{0U};
  std::atomic<std::uint32_t> recovery_events_{0U};
};

}  // namespace

int main(int argc, char** argv) {
  app::AppOptions options;
  std::string error;
  switch (app::parseOptions(argc, argv, options, error)) {
    case app::ParseOutcome::ShowHelp:
      app::printUsage(argv[0], "Virtual Vehicle Sensor ECU - publishes 0x100/0x110, supervises 0x200");
      return 0;
    case app::ParseOutcome::Error:
      (void)std::fprintf(stderr, "error: %s (use --help)\n", error.c_str());  // NOLINT(cppcoreguidelines-pro-type-vararg)
      return 2;
    case app::ParseOutcome::Run:
      break;
  }
  core::Logger::setMinLevel(options.verbose ? core::LogLevel::Debug : core::LogLevel::Info);
  app::installShutdownHandlers();

  app::EcuPlatform platform("sensor_ecu", options);
  if (!platform.init()) {
    return 1;
  }
  SensorEcu ecu(platform, options.mute_tx);
  if (!ecu.registerTasks()) {
    return 1;
  }
  const int exit_code = platform.run();
  ecu.logSummary();
  return exit_code;
}
