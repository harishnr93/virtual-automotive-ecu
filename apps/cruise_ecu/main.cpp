// Virtual Cruise Control ECU.
//
// Task structure (rate-monotonic priorities):
//   CanRx thread     (prio 90) event-driven reception into the RX queue
//   Control_10ms     (prio 80) CruiseController: state machine + PID speed control
//   Com_20ms         (prio 70) RX processing + supervision, publishes inputs, transmits 0x200
//   Diag_100ms       (prio 50) health monitoring: deadline misses, periodic status
//
// The control algorithm (src/control) knows nothing about CAN or threads; this file is the
// glue: it converts bus signals into CruiseInputs and CruiseOutputs into bus signals.

#include <atomic>
#include <chrono>
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
#include "control/cruise_controller.hpp"
#include "core/clock.hpp"
#include "core/logger.hpp"
#include "core/periodic_scheduler.hpp"
#include "core/shared_signal.hpp"

namespace {

namespace can = vecu::can;
namespace vehicle = vecu::can::vehicle;
namespace control = vecu::control;
namespace core = vecu::core;
namespace app = vecu::app;
namespace sched = vecu::sched;

using std::chrono::microseconds;
using std::chrono::milliseconds;

constexpr milliseconds kFirstTimeout{1000};
constexpr milliseconds kMaxInputAge{100};  // inputs older than this: COM task not running
constexpr double kControlPeriodS = 0.010;

// The control layer's enums are transmitted unchanged; guarantee they match the DBC.
static_assert(static_cast<int>(control::CruiseState::Off) == static_cast<int>(vehicle::CruiseState::Off));
static_assert(static_cast<int>(control::CruiseState::Standby) == static_cast<int>(vehicle::CruiseState::Standby));
static_assert(static_cast<int>(control::CruiseState::Active) == static_cast<int>(vehicle::CruiseState::Active));
static_assert(static_cast<int>(control::CruiseState::Override) == static_cast<int>(vehicle::CruiseState::Override));
static_assert(static_cast<int>(control::CruiseState::Fault) == static_cast<int>(vehicle::CruiseState::Fault));

class CruiseEcu {
 public:
  explicit CruiseEcu(app::EcuPlatform& platform) : platform_(platform), log_(platform.log()) {
    (void)monitor_.addMessage(vehicle::kVehicleDynamics.id, milliseconds{vehicle::kVehicleDynamics.timeout_ms},
                              kFirstTimeout);
    (void)monitor_.addMessage(vehicle::kDriverInputs.id, milliseconds{vehicle::kDriverInputs.timeout_ms},
                              kFirstTimeout);
    monitor_.start(core::now());
  }

  [[nodiscard]] bool registerTasks() {
    if (!controller_.configValid()) {
      log_.error("invalid cruise controller configuration");
      return false;
    }
    sched::TaskConfig control_task{"Control_10ms", microseconds{10000}, microseconds{0}, microseconds{0},
                                   microseconds{2000}, app::kPriority10ms};
    sched::TaskConfig com{"Com_20ms", microseconds{20000}, microseconds{1000}, microseconds{0},
                          microseconds{5000}, app::kPriority20ms};
    sched::TaskConfig diag{"Diag_100ms", microseconds{100000}, microseconds{3000}, microseconds{0},
                           microseconds{20000}, app::kPriority100ms};
    return platform_.addTask(control_task, [this](const sched::ActivationInfo& a) { taskControl(a); }) &&
           platform_.addTask(com, [this](const sched::ActivationInfo& a) { taskCom(a); }) &&
           platform_.addTask(diag, [this](const sched::ActivationInfo& a) { taskDiag(a); });
  }

  void logSummary() const {
    log_.info(core::strCat("SUMMARY ecu=cruise_ecu bus=", platform_.busName(), " tx=", platform_.io().txOk(),
                           " tx_failed=", platform_.io().txFailed(), " rx_0x100=", rx_vehicle_dynamics_.load(),
                           " rx_0x110=", rx_driver_inputs_.load(), " invalid=", invalid_frames_.load(),
                           " alive_errors=", alive_errors_.load(), " timeouts=", timeout_events_.load(),
                           " recoveries=", recovery_events_.load(), " engagements=", engagements_.load(),
                           " faults=", fault_entries_.load()));
  }

 private:
  // ------------------------------------------------------------ Control_10ms (prio 80) ----
  void taskControl(const sched::ActivationInfo& /*activation*/) {
    const core::TimePoint now = core::now();
    const auto sample = inputs_.read();
    control::CruiseInputs inputs = sample.value;
    // Stale-data protection: the COM task must refresh the inputs every 20 ms.
    inputs.inputs_stale = (sample.sequence != 0U) && ((now - sample.timestamp) > kMaxInputAge);

    const control::CruiseOutputs output = controller_.step(inputs, kControlPeriodS);
    if (output.state != last_state_) {
      log_.info(core::strCat("CRUISE_STATE ", control::toString(last_state_), " -> ", control::toString(output.state),
                             (output.state == control::CruiseState::Fault)
                                 ? core::strCat(" reason=", control::toString(output.fault))
                                 : std::string{},
                             (output.state == control::CruiseState::Active)
                                 ? core::strCat(" target_kph=", app::fixed(output.target_speed_kph, 1),
                                                " speed_kph=", app::fixed(inputs.vehicle_speed_kph, 1))
                                 : std::string{}));
      if (output.state == control::CruiseState::Active && last_state_ == control::CruiseState::Standby) {
        engagements_.fetch_add(1U);
      }
      if (output.state == control::CruiseState::Fault) {
        fault_entries_.fetch_add(1U);
      }
      last_state_ = output.state;
    }
    output_.write(output, now);
  }

  // ---------------------------------------------------------------- Com_20ms (prio 70) ----
  void taskCom(const sched::ActivationInfo& /*activation*/) {
    can::RxFrame entry;
    while (platform_.rxQueue().pop(entry)) {
      platform_.io().traceReceived(entry);
      if (entry.frame.id == vehicle::kVehicleDynamics.id) {
        handleVehicleDynamics(entry);
      } else if (entry.frame.id == vehicle::kDriverInputs.id) {
        handleDriverInputs(entry);
      } else {
        // not for this ECU (CAN is a broadcast bus)
      }
    }
    const core::TimePoint now = core::now();
    monitor_.update(now, [this](const can::RxTimeoutEvent& event) { onTimeoutEvent(event); });

    com_inputs_.speed_timeout = monitor_.state(vehicle::kVehicleDynamics.id) == can::RxTimeoutState::TimedOut;
    com_inputs_.driver_inputs_timeout = monitor_.state(vehicle::kDriverInputs.id) == can::RxTimeoutState::TimedOut;
    if (monitor_.state(vehicle::kVehicleDynamics.id) != can::RxTimeoutState::Ok) {
      com_inputs_.speed_valid = false;  // never trust a speed that is not being received
    }
    inputs_.write(com_inputs_, now);

    sendStatus(output_.read().value);
  }

  void handleVehicleDynamics(const can::RxFrame& entry) {
    const can::CanFrame& frame = entry.frame;
    if (frame.dlc != vehicle::kVehicleDynamics.dlc) {
      invalid_frames_.fetch_add(1U);
      return;
    }
    rx_vehicle_dynamics_.fetch_add(1U);
    (void)monitor_.notifyReceived(frame.id, entry.rx_time, [this](const can::RxTimeoutEvent& e) { onTimeoutEvent(e); });
    checkAlive(frame, vehicle::kVehicleDynamicsAlive, alive_vehicle_dynamics_);

    double speed = 0.0;
    double valid = 0.0;
    const bool speed_ok = can::decodePhysical(vehicle::kVehicleSpeed, frame, speed) == can::SignalStatus::Ok;
    (void)can::decodePhysical(vehicle::kVehicleSpeedValid, frame, valid);
    if (!speed_ok) {
      invalid_frames_.fetch_add(1U);  // out-of-range value
    }
    com_inputs_.speed_valid = speed_ok && (valid > 0.5);
    if (com_inputs_.speed_valid) {
      com_inputs_.vehicle_speed_kph = speed;
    }
  }

  void handleDriverInputs(const can::RxFrame& entry) {
    const can::CanFrame& frame = entry.frame;
    if (frame.dlc != vehicle::kDriverInputs.dlc) {
      invalid_frames_.fetch_add(1U);
      return;
    }
    rx_driver_inputs_.fetch_add(1U);
    (void)monitor_.notifyReceived(frame.id, entry.rx_time, [this](const can::RxTimeoutEvent& e) { onTimeoutEvent(e); });
    checkAlive(frame, vehicle::kDriverInputsAlive, alive_driver_inputs_);

    double accelerator = 0.0;
    double brake = 0.0;
    double main_switch = 0.0;
    double set_request = 0.0;
    double cancel_request = 0.0;
    double set_speed = 0.0;
    bool ok = can::decodePhysical(vehicle::kAcceleratorPedal, frame, accelerator) == can::SignalStatus::Ok;
    ok = (can::decodePhysical(vehicle::kBrakePedalPressed, frame, brake) == can::SignalStatus::Ok) && ok;
    ok = (can::decodePhysical(vehicle::kCruiseMainSwitch, frame, main_switch) == can::SignalStatus::Ok) && ok;
    ok = (can::decodePhysical(vehicle::kCruiseSetRequest, frame, set_request) == can::SignalStatus::Ok) && ok;
    ok = (can::decodePhysical(vehicle::kCruiseCancelRequest, frame, cancel_request) == can::SignalStatus::Ok) && ok;
    ok = (can::decodePhysical(vehicle::kDriverSetSpeed, frame, set_speed) == can::SignalStatus::Ok) && ok;
    if (!ok) {
      // Implausible driver input: ignore this frame and keep the last valid values.
      invalid_frames_.fetch_add(1U);
      return;
    }
    com_inputs_.accelerator_pct = accelerator;
    com_inputs_.brake_pressed = brake > 0.5;
    com_inputs_.main_switch = main_switch > 0.5;
    com_inputs_.set_request = set_request > 0.5;
    com_inputs_.cancel_request = cancel_request > 0.5;
    com_inputs_.driver_set_speed_kph = set_speed;
  }

  void sendStatus(const control::CruiseOutputs& output) {
    can::CanFrame frame = can::makeFrame(vehicle::kCruiseControlStatus);
    (void)can::encodePhysical(vehicle::kThrottleCommand, output.throttle_pct, frame);
    (void)can::encodePhysical(vehicle::kCruiseState, static_cast<double>(output.state), frame);
    (void)can::encodePhysical(vehicle::kCruiseTargetSpeed, output.target_speed_kph, frame);
    (void)can::encodePhysical(vehicle::kCruiseFaultReason, static_cast<double>(output.fault), frame);
    (void)can::encodePhysical(vehicle::kCruiseStatusAlive, alive_tx_.next(), frame);
    (void)platform_.io().send(frame);
  }

  void checkAlive(const can::CanFrame& frame, const can::SignalDef& signal, can::AliveCounterChecker& checker) {
    double value = 0.0;
    if (can::decodePhysical(signal, frame, value) != can::SignalStatus::Ok) {
      alive_errors_.fetch_add(1U);
      return;
    }
    const can::AliveCheckResult result = checker.check(static_cast<std::uint8_t>(value));
    if ((result == can::AliveCheckResult::Repeated) || (result == can::AliveCheckResult::Skipped) ||
        (result == can::AliveCheckResult::Invalid)) {
      alive_errors_.fetch_add(1U);
      log_.warning(core::strCat("ALIVE_ERROR id=", can::formatId(frame.id), " result=", can::toString(result)));
    }
  }

  void onTimeoutEvent(const can::RxTimeoutEvent& event) {
    app::logRxTimeoutEvent(log_, event);
    if (event.current == can::RxTimeoutState::TimedOut) {
      timeout_events_.fetch_add(1U);
      if (event.id == vehicle::kVehicleDynamics.id) {
        alive_vehicle_dynamics_.reset();
      } else {
        alive_driver_inputs_.reset();
      }
    } else if (event.previous == can::RxTimeoutState::TimedOut) {
      recovery_events_.fetch_add(1U);
    } else {
      // first reception
    }
  }

  // -------------------------------------------------------------- Diag_100ms (prio 50) ----
  void taskDiag(const sched::ActivationInfo& activation) {
    platform_.reportNewDeadlineMisses();
    if ((activation.activation > 0U) && ((activation.activation % 10U) == 0U)) {  // every 1 s
      const control::CruiseInputs inputs = inputs_.read().value;
      const control::CruiseOutputs output = output_.read().value;
      log_.info(core::strCat("state=", control::toString(output.state), " speed=", app::fixed(inputs.vehicle_speed_kph, 2),
                             "km/h target=", app::fixed(output.target_speed_kph, 0), "km/h setpoint=",
                             app::fixed(output.setpoint_kph, 2), "km/h throttle=", app::fixed(output.throttle_pct, 2),
                             "% pedal=", app::fixed(inputs.accelerator_pct, 1), "% brake=", inputs.brake_pressed,
                             " rx_0x100=", rx_vehicle_dynamics_.load(), " rxq_overruns=",
                             platform_.rxQueue().overruns()));
    }
  }

  app::EcuPlatform& platform_;
  const core::Logger& log_;

  // Shared between tasks (thread-safe buffers).
  core::SharedSignal<control::CruiseInputs> inputs_{};
  core::SharedSignal<control::CruiseOutputs> output_{};

  // Owned by Com_20ms only.
  can::MessageTimeoutMonitor<2U> monitor_{};
  can::AliveCounterChecker alive_vehicle_dynamics_{};
  can::AliveCounterChecker alive_driver_inputs_{};
  can::AliveCounterGenerator alive_tx_{};
  control::CruiseInputs com_inputs_{};

  // Owned by Control_10ms only.
  control::CruiseController controller_{};
  control::CruiseState last_state_{control::CruiseState::Off};

  // Counters: written by one task, read by Diag_100ms and main.
  std::atomic<std::uint64_t> rx_vehicle_dynamics_{0U};
  std::atomic<std::uint64_t> rx_driver_inputs_{0U};
  std::atomic<std::uint64_t> invalid_frames_{0U};
  std::atomic<std::uint64_t> alive_errors_{0U};
  std::atomic<std::uint32_t> timeout_events_{0U};
  std::atomic<std::uint32_t> recovery_events_{0U};
  std::atomic<std::uint32_t> engagements_{0U};
  std::atomic<std::uint32_t> fault_entries_{0U};
};

}  // namespace

int main(int argc, char** argv) {
  app::AppOptions options;
  std::string error;
  switch (app::parseOptions(argc, argv, options, error)) {
    case app::ParseOutcome::ShowHelp:
      app::printUsage(argv[0], "Virtual Cruise Control ECU (PID speed control, scheduled tasks)");
      return 0;
    case app::ParseOutcome::Error:
      (void)std::fprintf(stderr, "error: %s (use --help)\n", error.c_str());  // NOLINT(cppcoreguidelines-pro-type-vararg)
      return 2;
    case app::ParseOutcome::Run:
      break;
  }
  core::Logger::setMinLevel(options.verbose ? core::LogLevel::Debug : core::LogLevel::Info);
  app::installShutdownHandlers();

  app::EcuPlatform platform("cruise_ecu", options);
  if (!platform.init()) {
    return 1;
  }
  CruiseEcu ecu(platform);
  if (!ecu.registerTasks()) {
    return 1;
  }
  const int exit_code = platform.run();
  ecu.logSummary();
  return exit_code;
}
