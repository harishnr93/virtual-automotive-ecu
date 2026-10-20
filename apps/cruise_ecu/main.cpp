// Virtual Cruise Control ECU — Phase 1 communication skeleton.
//
// Receives VehicleDynamics (0x100) and DriverInputs (0x110), supervises them (timeout,
// alive counter, DLC, signal range) and publishes CruiseControlStatus (0x200) every 20 ms.
// No control algorithm yet: ThrottleCommand is always 0 %. The PID controller and the full
// cruise state machine are added in Phase 3, behind the same communication interface.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>

#include "can/alive_counter.hpp"
#include "can/can_channel_factory.hpp"
#include "can/can_log_writer.hpp"
#include "can/can_signal.hpp"
#include "can/message_timeout_monitor.hpp"
#include "can/vehicle_signals.hpp"
#include "common/app_options.hpp"
#include "common/ecu_runtime.hpp"
#include "common/shutdown_signal.hpp"
#include "core/clock.hpp"
#include "core/logger.hpp"

namespace {

namespace can = vecu::can;
namespace vehicle = vecu::can::vehicle;
namespace core = vecu::core;
namespace app = vecu::app;

constexpr std::chrono::milliseconds kTickPeriod{10};
constexpr std::chrono::milliseconds kFirstTimeout{1000};
constexpr std::uint64_t kStatusEveryTicks = 100U;

constexpr std::uint64_t ticksPerCycle(const can::MessageDef& message) noexcept {
  return message.cycle_time_ms / static_cast<std::uint64_t>(kTickPeriod.count());
}
static_assert(vehicle::kCruiseControlStatus.cycle_time_ms % 10U == 0U, "cycle must be a multiple of the tick");

/// Latest validated inputs from the bus (the ECU's "signal buffer").
struct Inputs {
  double speed_kph{0.0};
  double accel_mps2{0.0};
  bool speed_valid{false};
  bool brake_pressed{false};
  bool main_switch{false};
  double set_speed_kph{0.0};
};

class CruiseEcuComStub {
 public:
  explicit CruiseEcuComStub(app::CanIo& io) : io_(io) {
    (void)monitor_.addMessage(vehicle::kVehicleDynamics.id,
                              std::chrono::milliseconds{vehicle::kVehicleDynamics.timeout_ms}, kFirstTimeout);
    (void)monitor_.addMessage(vehicle::kDriverInputs.id,
                              std::chrono::milliseconds{vehicle::kDriverInputs.timeout_ms}, kFirstTimeout);
  }

  void onFrame(const can::CanFrame& frame, core::TimePoint now) {
    io_.traceReceived(frame);
    if (frame.id == vehicle::kVehicleDynamics.id) {
      handleVehicleDynamics(frame, now);
    } else if (frame.id == vehicle::kDriverInputs.id) {
      handleDriverInputs(frame, now);
    } else {
      // not for this ECU
    }
  }

  void onTick(core::TimePoint now, std::uint64_t tick) {
    if (tick == 0U) {
      start_ = now;
      monitor_.start(now);
    }
    monitor_.update(now, [this](const can::RxTimeoutEvent& event) { onTimeoutEvent(event); });

    const vehicle::CruiseState state = evaluateState();
    if (state != state_) {
      log_.info(core::strCat("CRUISE_STATE ", vehicle::toString(state_), " -> ", vehicle::toString(state)));
      state_ = state;
    }
    if ((tick % ticksPerCycle(vehicle::kCruiseControlStatus)) == 0U) {
      sendStatus();
    }
    if ((tick > 0U) && ((tick % kStatusEveryTicks) == 0U)) {
      log_.info(core::strCat("t=", app::fixed(core::toMillis(now - start_) / 1000.0, 1),
                             "s speed=", app::fixed(inputs_.speed_kph, 2), "km/h accel=",
                             app::fixed(inputs_.accel_mps2, 2), "m/s2 main=", inputs_.main_switch,
                             " set=", app::fixed(inputs_.set_speed_kph, 0), "km/h state=",
                             vehicle::toString(state_), " rx_0x100=", rx_vehicle_dynamics_,
                             " rx_0x110=", rx_driver_inputs_));
    }
  }

  void logSummary(const app::LoopTiming& timing, std::string_view bus) const {
    log_.info(core::strCat("SUMMARY ecu=cruise_ecu bus=", bus, " tx=", io_.txOk(),
                           " tx_failed=", io_.txFailed(), " rx_0x100=", rx_vehicle_dynamics_,
                           " rx_0x110=", rx_driver_inputs_, " invalid=", invalid_frames_,
                           " alive_errors=", alive_errors_, " timeouts=", timeout_events_,
                           " recoveries=", recovery_events_, " ticks=", timing.ticks,
                           " late_ticks=", timing.late_ticks,
                           " max_tick_lateness_ms=", app::fixed(core::toMillis(timing.max_lateness), 3)));
  }

 private:
  void handleVehicleDynamics(const can::CanFrame& frame, core::TimePoint now) {
    if (frame.dlc != vehicle::kVehicleDynamics.dlc) {
      ++invalid_frames_;
      return;
    }
    ++rx_vehicle_dynamics_;
    (void)monitor_.notifyReceived(frame.id, now, [this](const can::RxTimeoutEvent& e) { onTimeoutEvent(e); });
    checkAlive(frame, vehicle::kVehicleDynamicsAlive, alive_vehicle_dynamics_);

    double speed = 0.0;
    double accel = 0.0;
    double valid = 0.0;
    const bool speed_ok = can::decodePhysical(vehicle::kVehicleSpeed, frame, speed) == can::SignalStatus::Ok;
    const bool accel_ok = can::decodePhysical(vehicle::kLongitudinalAccel, frame, accel) == can::SignalStatus::Ok;
    (void)can::decodePhysical(vehicle::kVehicleSpeedValid, frame, valid);
    if (!speed_ok || !accel_ok) {
      ++invalid_frames_;  // out-of-range value: treat the sample as invalid (fault handling: Phase 5)
    }
    inputs_.speed_valid = speed_ok && (valid > 0.5);
    if (inputs_.speed_valid) {
      inputs_.speed_kph = speed;
      inputs_.accel_mps2 = accel;
    }
  }

  void handleDriverInputs(const can::CanFrame& frame, core::TimePoint now) {
    if (frame.dlc != vehicle::kDriverInputs.dlc) {
      ++invalid_frames_;
      return;
    }
    ++rx_driver_inputs_;
    (void)monitor_.notifyReceived(frame.id, now, [this](const can::RxTimeoutEvent& e) { onTimeoutEvent(e); });
    checkAlive(frame, vehicle::kDriverInputsAlive, alive_driver_inputs_);

    double brake = 0.0;
    double main_switch = 0.0;
    double set_speed = 0.0;
    (void)can::decodePhysical(vehicle::kBrakePedalPressed, frame, brake);
    (void)can::decodePhysical(vehicle::kCruiseMainSwitch, frame, main_switch);
    if (can::decodePhysical(vehicle::kDriverSetSpeed, frame, set_speed) != can::SignalStatus::Ok) {
      ++invalid_frames_;
      set_speed = 0.0;
    }
    inputs_.brake_pressed = brake > 0.5;
    inputs_.main_switch = main_switch > 0.5;
    inputs_.set_speed_kph = set_speed;
  }

  /// Phase 1 state logic (communication-driven only; full state machine in Phase 3).
  [[nodiscard]] vehicle::CruiseState evaluateState() const {
    if (monitor_.anyTimedOut()) {
      return vehicle::CruiseState::Fault;  // safe state: no valid inputs -> no control
    }
    const bool all_received = (monitor_.state(vehicle::kVehicleDynamics.id) == can::RxTimeoutState::Ok) &&
                              (monitor_.state(vehicle::kDriverInputs.id) == can::RxTimeoutState::Ok);
    if (!all_received || !inputs_.main_switch) {
      return vehicle::CruiseState::Off;
    }
    if (!inputs_.speed_valid) {
      return vehicle::CruiseState::Fault;
    }
    return vehicle::CruiseState::Standby;
  }

  void sendStatus() {
    can::CanFrame frame = can::makeFrame(vehicle::kCruiseControlStatus);
    const double target = (state_ == vehicle::CruiseState::Standby) ? inputs_.set_speed_kph : 0.0;
    (void)can::encodePhysical(vehicle::kThrottleCommand, 0.0, frame);  // no controller yet
    (void)can::encodePhysical(vehicle::kCruiseState, static_cast<double>(state_), frame);
    (void)can::encodePhysical(vehicle::kCruiseTargetSpeed, target, frame);
    (void)can::encodePhysical(vehicle::kCruiseStatusAlive, alive_tx_.next(), frame);
    (void)io_.send(frame);
  }

  void checkAlive(const can::CanFrame& frame, const can::SignalDef& signal, can::AliveCounterChecker& checker) {
    double value = 0.0;
    if (can::decodePhysical(signal, frame, value) != can::SignalStatus::Ok) {
      ++alive_errors_;
      return;
    }
    const can::AliveCheckResult result = checker.check(static_cast<std::uint8_t>(value));
    if ((result == can::AliveCheckResult::Repeated) || (result == can::AliveCheckResult::Skipped) ||
        (result == can::AliveCheckResult::Invalid)) {
      ++alive_errors_;
      log_.warning(core::strCat("ALIVE_ERROR id=", can::formatId(frame.id), " result=", can::toString(result)));
    }
  }

  void onTimeoutEvent(const can::RxTimeoutEvent& event) {
    app::logRxTimeoutEvent(log_, event);
    if (event.current == can::RxTimeoutState::TimedOut) {
      ++timeout_events_;
      if (event.id == vehicle::kVehicleDynamics.id) {
        alive_vehicle_dynamics_.reset();
        inputs_.speed_valid = false;
      } else {
        alive_driver_inputs_.reset();
      }
    } else if (event.previous == can::RxTimeoutState::TimedOut) {
      ++recovery_events_;
    } else {
      // first reception
    }
  }

  app::CanIo& io_;
  core::Logger log_{"cruise_ecu"};
  core::TimePoint start_{};
  can::MessageTimeoutMonitor<2U> monitor_{};
  can::AliveCounterChecker alive_vehicle_dynamics_{};
  can::AliveCounterChecker alive_driver_inputs_{};
  can::AliveCounterGenerator alive_tx_{};
  Inputs inputs_{};
  vehicle::CruiseState state_{vehicle::CruiseState::Off};
  std::uint64_t rx_vehicle_dynamics_{0U};
  std::uint64_t rx_driver_inputs_{0U};
  std::uint64_t invalid_frames_{0U};
  std::uint64_t alive_errors_{0U};
  std::uint32_t timeout_events_{0U};
  std::uint32_t recovery_events_{0U};
};

}  // namespace

int main(int argc, char** argv) {
  app::AppOptions options;
  std::string error;
  switch (app::parseOptions(argc, argv, options, error)) {
    case app::ParseOutcome::ShowHelp:
      app::printUsage(argv[0], "Virtual Cruise Control ECU (Phase 1 communication skeleton)");
      return 0;
    case app::ParseOutcome::Error:
      (void)std::fprintf(stderr, "error: %s (use --help)\n", error.c_str());  // NOLINT(cppcoreguidelines-pro-type-vararg)
      return 2;
    case app::ParseOutcome::Run:
      break;
  }

  core::Logger::setMinLevel(options.verbose ? core::LogLevel::Debug : core::LogLevel::Info);
  app::installShutdownHandlers();
  const core::Logger log("cruise_ecu");

  can::CanStatus status = can::CanStatus::Ok;
  const auto channel = can::createCanChannel(options.can, status);
  if (!channel) {
    log.error(core::strCat("cannot open CAN channel: ", can::toString(status)));
    return 1;
  }

  can::CanLogWriter trace;
  if (!options.can_log_path.empty() &&
      (trace.open(options.can_log_path, std::string{channel->name()}) != can::CanStatus::Ok)) {
    log.error(core::strCat("cannot open CAN trace file ", options.can_log_path));
    return 1;
  }

  app::CanIo io(*channel, trace);
  CruiseEcuComStub ecu(io);
  log.info("started (Ctrl+C to stop)");
  const app::LoopTiming timing = app::runCooperativeLoop(
      *channel, kTickPeriod, options.duration_s,
      [&ecu](const can::CanFrame& frame, core::TimePoint now) { ecu.onFrame(frame, now); },
      [&ecu](core::TimePoint now, std::uint64_t tick) { ecu.onTick(now, tick); });

  ecu.logSummary(timing, channel->name());
  return 0;
}
