// Virtual Vehicle Sensor ECU (Phase 1).
//
// Publishes VehicleDynamics (0x100, 20 ms) and DriverInputs (0x110, 50 ms) from a synthetic
// speed profile, and supervises CruiseControlStatus (0x200) from the cruise ECU.
// In Phase 3 the Python vehicle simulator takes over this role; this ECU remains as a
// lightweight C++ traffic generator for tests.

#include <chrono>
#include <cmath>
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
constexpr std::chrono::milliseconds kFirstTimeout{1000};  // start-up grace for the peer ECU
constexpr std::uint64_t kStatusEveryTicks = 100U;          // 1 s

constexpr std::uint64_t ticksPerCycle(const can::MessageDef& message) noexcept {
  return message.cycle_time_ms / static_cast<std::uint64_t>(kTickPeriod.count());
}
static_assert(vehicle::kVehicleDynamics.cycle_time_ms % 10U == 0U, "cycle must be a multiple of the tick");
static_assert(vehicle::kDriverInputs.cycle_time_ms % 10U == 0U, "cycle must be a multiple of the tick");

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
  SensorEcu(app::CanIo& io, const app::MuteWindow& mute) : io_(io), mute_(mute) {
    (void)monitor_.addMessage(vehicle::kCruiseControlStatus.id,
                              std::chrono::milliseconds{vehicle::kCruiseControlStatus.timeout_ms},
                              kFirstTimeout);
  }

  void onTick(core::TimePoint now, std::uint64_t tick) {
    if (tick == 0U) {
      start_ = now;
      monitor_.start(now);
    }
    const double t_s = core::toMillis(now - start_) / 1000.0;

    if ((tick % ticksPerCycle(vehicle::kVehicleDynamics)) == 0U) {
      sendVehicleDynamics(t_s);
    }
    if ((tick % ticksPerCycle(vehicle::kDriverInputs)) == 0U) {
      sendDriverInputs(t_s);
    }
    monitor_.update(now, [this](const can::RxTimeoutEvent& event) { onTimeoutEvent(event); });

    if ((tick > 0U) && ((tick % kStatusEveryTicks) == 0U)) {
      log_.info(core::strCat("t=", app::fixed(t_s, 1), "s speed_tx=", app::fixed(last_speed_kph_, 2),
                             "km/h cruise_state=", vehicle::toString(cruise_state_),
                             " rx_0x200=", rx_cruise_status_, " tx=", io_.txOk()));
    }
  }

  void onFrame(const can::CanFrame& frame, core::TimePoint now) {
    io_.traceReceived(frame);
    if (frame.id != vehicle::kCruiseControlStatus.id) {
      return;  // CAN is a broadcast bus: every node sees every frame and filters by ID
    }
    if (frame.dlc != vehicle::kCruiseControlStatus.dlc) {
      ++invalid_frames_;
      return;
    }
    ++rx_cruise_status_;
    (void)monitor_.notifyReceived(frame.id, now,
                                  [this](const can::RxTimeoutEvent& event) { onTimeoutEvent(event); });

    double alive = 0.0;
    if (can::decodePhysical(vehicle::kCruiseStatusAlive, frame, alive) == can::SignalStatus::Ok) {
      checkAlive(alive_checker_, static_cast<std::uint8_t>(alive));
    }
    double state = 0.0;
    if (can::decodePhysical(vehicle::kCruiseState, frame, state) == can::SignalStatus::Ok) {
      cruise_state_ = static_cast<vehicle::CruiseState>(static_cast<std::uint8_t>(state));
    } else {
      ++invalid_frames_;
    }
  }

  void logSummary(const app::LoopTiming& timing, std::string_view bus) const {
    log_.info(core::strCat("SUMMARY ecu=sensor_ecu bus=", bus, " tx=", io_.txOk(),
                           " tx_failed=", io_.txFailed(), " muted=", muted_frames_,
                           " rx_0x200=", rx_cruise_status_, " invalid=", invalid_frames_,
                           " alive_errors=", alive_errors_, " timeouts=", timeout_events_,
                           " recoveries=", recovery_events_, " ticks=", timing.ticks,
                           " late_ticks=", timing.late_ticks,
                           " max_tick_lateness_ms=", app::fixed(core::toMillis(timing.max_lateness), 3)));
  }

 private:
  void sendVehicleDynamics(double t_s) {
    const VehicleSample sample = syntheticProfile(t_s);
    can::CanFrame frame = can::makeFrame(vehicle::kVehicleDynamics);
    (void)can::encodePhysical(vehicle::kVehicleSpeed, sample.speed_kph, frame);
    (void)can::encodePhysical(vehicle::kLongitudinalAccel, sample.accel_mps2, frame);
    (void)can::encodePhysical(vehicle::kVehicleSpeedValid, 1.0, frame);
    // The counter advances even while muted: muted frames count as "lost on the bus".
    (void)can::encodePhysical(vehicle::kVehicleDynamicsAlive, vehicle_dynamics_alive_.next(), frame);
    last_speed_kph_ = sample.speed_kph;

    if (mute_.contains(t_s)) {
      if (muted_frames_ == 0U) {
        log_.warning(core::strCat("FAULT_INJECTION mute 0x100 from t=", app::fixed(t_s, 3), "s"));
      }
      ++muted_frames_;
      return;
    }
    (void)io_.send(frame);
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
    (void)io_.send(frame);
  }

  void checkAlive(can::AliveCounterChecker& checker, std::uint8_t value) {
    const can::AliveCheckResult result = checker.check(value);
    if ((result == can::AliveCheckResult::Repeated) || (result == can::AliveCheckResult::Skipped) ||
        (result == can::AliveCheckResult::Invalid)) {
      ++alive_errors_;
      log_.warning(core::strCat("ALIVE_ERROR id=0x200 result=", can::toString(result)));
    }
  }

  void onTimeoutEvent(const can::RxTimeoutEvent& event) {
    app::logRxTimeoutEvent(log_, event);
    if (event.current == can::RxTimeoutState::TimedOut) {
      ++timeout_events_;
      alive_checker_.reset();  // resynchronise the counter after a communication loss
    } else if (event.previous == can::RxTimeoutState::TimedOut) {
      ++recovery_events_;
    } else {
      // first reception: nothing else to do
    }
  }

  app::CanIo& io_;
  app::MuteWindow mute_;
  core::Logger log_{"sensor_ecu"};
  core::TimePoint start_{};
  can::MessageTimeoutMonitor<1U> monitor_{};
  can::AliveCounterGenerator vehicle_dynamics_alive_{};
  can::AliveCounterGenerator driver_inputs_alive_{};
  can::AliveCounterChecker alive_checker_{};
  vehicle::CruiseState cruise_state_{vehicle::CruiseState::Off};
  double last_speed_kph_{0.0};
  std::uint64_t rx_cruise_status_{0U};
  std::uint64_t invalid_frames_{0U};
  std::uint64_t alive_errors_{0U};
  std::uint64_t muted_frames_{0U};
  std::uint32_t timeout_events_{0U};
  std::uint32_t recovery_events_{0U};
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
  const core::Logger log("sensor_ecu");

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
  SensorEcu ecu(io, options.mute_tx);
  log.info("started (Ctrl+C to stop)");
  const app::LoopTiming timing = app::runCooperativeLoop(
      *channel, kTickPeriod, options.duration_s,
      [&ecu](const can::CanFrame& frame, core::TimePoint now) { ecu.onFrame(frame, now); },
      [&ecu](core::TimePoint now, std::uint64_t tick) { ecu.onTick(now, tick); });

  ecu.logSummary(timing, channel->name());
  return 0;
}
