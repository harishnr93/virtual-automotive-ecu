#include "cruise_ecu/cruise_ecu.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <string_view>

#include "can/can_signal.hpp"
#include "can/vehicle_signals.hpp"
#include "core/clock.hpp"
#include "core/logger.hpp"
#include "core/os_port.hpp"
#include "cruise_ecu/cruise_faults.hpp"
#include "diagnostics/diag_config.hpp"

namespace vecu::cruise {

namespace {

namespace can = vecu::can;
namespace vehicle = vecu::can::vehicle;
namespace control = vecu::control;
namespace core = vecu::core;
namespace app = vecu::app;
namespace diag = vecu::diag;

using std::chrono::microseconds;

constexpr milliseconds kFirstTimeout{1000};
constexpr milliseconds kMaxInputAge{100};  // inputs older than this: COM task not running
constexpr milliseconds kNvmMinWriteInterval{1000};
constexpr double kControlPeriodS = 0.010;
constexpr std::uint8_t kFaultReasonWatchdogSafeState = 7U;  // DBC CruiseFaultReason value table
constexpr std::uint8_t kFaultReasonControlOutputStale = 8U;
// The Control task refreshes its output every 10 ms; older than this means it stopped.
constexpr milliseconds kMaxOutputAge{40};
// A test image with the "simulated defect" flag hangs the Control task this long after boot.
constexpr milliseconds kDefectHangDelay{1000};
// Programming session only at standstill (the cruise function is inhibited while flashing).
constexpr double kMaxProgrammingSpeedKph = 1.0;

// The control layer's enums are transmitted unchanged; guarantee they match the DBC.
static_assert(static_cast<int>(control::CruiseState::Off) == static_cast<int>(vehicle::CruiseState::Off));
static_assert(static_cast<int>(control::CruiseState::Standby) == static_cast<int>(vehicle::CruiseState::Standby));
static_assert(static_cast<int>(control::CruiseState::Active) == static_cast<int>(vehicle::CruiseState::Active));
static_assert(static_cast<int>(control::CruiseState::Override) == static_cast<int>(vehicle::CruiseState::Override));
static_assert(static_cast<int>(control::CruiseState::Fault) == static_cast<int>(vehicle::CruiseState::Fault));

// Identification data (simulated ECU: clearly not a real vehicle/part).
constexpr std::string_view kVin{"VECUSIM0000000001"};  // 17 characters
constexpr std::string_view kSparePartNumber{"VECU-CC-0001"};
constexpr std::string_view kSerialNumber{"SIM-000001"};
static_assert(kVin.size() == 17U, "a VIN has 17 characters");

void putU16(std::uint8_t* out, double value) {
  const auto raw = static_cast<std::uint16_t>(std::clamp(value, 0.0, 65535.0));
  out[0] = static_cast<std::uint8_t>(raw >> 8U);
  out[1] = static_cast<std::uint8_t>(raw & 0xFFU);
}

void putU32(std::uint8_t* out, std::uint64_t value) {
  const auto raw = static_cast<std::uint32_t>(std::min<std::uint64_t>(value, 0xFFFFFFFFU));
  for (unsigned int i = 0U; i < 4U; ++i) {
    out[i] = static_cast<std::uint8_t>((raw >> (24U - 8U * i)) & 0xFFU);
  }
}

const char* resetName(faults::ResetReason reason) noexcept {
  switch (reason) {
    case faults::ResetReason::UdsHardReset:
      return "hard";
    case faults::ResetReason::UdsSoftReset:
      return "soft";
    case faults::ResetReason::Watchdog:
      return "watchdog";
    case faults::ResetReason::PowerOn:
      break;
  }
  return "power-on";
}

diag::IsoTpConfig isoTpConfig() {
  diag::IsoTpConfig config{diag::kPhysicalResponseId, diag::kPhysicalRequestId};
  // Receiver flow control: the diagnostic RX queue holds 128 frames and Dcm_10ms drains it
  // every 10 ms. Without a block size a tester may send all 585 consecutive frames of a
  // 4 KiB TransferData at once and overflow the queue; with BS = 32 it waits for our next
  // flow control frame after every 32 frames (sent when Dcm_10ms has consumed them).
  config.block_size = 32U;
  return config;
}

bool sameRecords(const faults::DtcRecord* a, const faults::DtcRecord* b, std::size_t n) {
  for (std::size_t i = 0U; i < n; ++i) {
    if ((a[i].code != b[i].code) || (a[i].status != b[i].status) || (a[i].occurrence_count != b[i].occurrence_count) ||
        (a[i].aging_counter != b[i].aging_counter)) {
      return false;
    }
  }
  return true;
}

}  // namespace

CruiseEcu::CruiseEcu(app::EcuPlatform& platform)
    : platform_(platform),
      log_(platform.log()),
      firmware_(platform.log()),
      uds_(diag::UdsServerConfig{}, dtcs_),
      isotp_(isoTpConfig(),
             [this](const can::CanFrame& frame) { return platform_.io().send(frame); }) {
  (void)monitor_.addMessage(vehicle::kVehicleDynamics.id, milliseconds{vehicle::kVehicleDynamics.timeout_ms},
                            kFirstTimeout);
  (void)monitor_.addMessage(vehicle::kDriverInputs.id, milliseconds{vehicle::kDriverInputs.timeout_ms}, kFirstTimeout);
  monitor_.start(core::now());
}

bool CruiseEcu::initialize() {
  if (!controller_.configValid()) {
    log_.error("invalid cruise controller configuration");
    return false;
  }
  const auto* failed = std::find_if(kFaultTable.begin(), kFaultTable.end(), [this](const FaultDefinition& d) {
    return !faults_.addFault(d.id, d.config);
  });
  if (failed != kFaultTable.end()) {
    log_.error(core::strCat("cannot register fault ", failed->config.name));
    return false;
  }

  // Fault memory: restore from NVM, then start a new operation cycle (ends the previous one:
  // pending/aging evaluation), then record start-up faults.
  loadNvm();
  operation_cycles_.fetch_add(1U);
  faults_.startOperationCycle();
  if ((nvm_load_status_ == faults::NvmStatus::Corrupted) || (nvm_load_status_ == faults::NvmStatus::IoError)) {
    (void)faults_.report(kFaultNvm, true, core::now());
  }
  if (!firmware_.open(platform_.options().flash_path)) {
    return false;
  }
  bootFirmware(core::now());
  if (!platform_.options().nvm_path.empty()) {
    saveNvm(last_reset_reason_.load());
  }

  uds_.setClearHandler([this](faults::DtcCode group) { faults_.onDtcCleared(group); });
  uds_.setSessionGuard([this](diag::Session requested) { return checkSessionChange(requested); });
  uds_.setSessionChangeHandler([this](diag::Session /*from*/, diag::Session to) {
    programming_session_ = (to == diag::Session::Programming);
    log_.info(core::strCat("UDS_SESSION ", diag::toString(to)));
  });
  registerDids();
  if (!firmware_.registerUds(uds_)) {
    log_.error("firmware DID/routine registration failed");
    return false;
  }

  platform_.setWatchdogHandlers(app::WatchdogHandlers{
      [this](const watchdog::CheckResult& r, const std::string& entity) { onWatchdogExpired(r, entity); },
      [this] { onWatchdogResumed(); }, [this] { onWatchdogHealthy(); }});
  platform_.setShutdownHook([this](bool emergency) {
    // Clean shutdown: the next start is a normal power-on. Emergency exit: watchdog reset.
    saveNvm(emergency ? faults::ResetReason::Watchdog : faults::ResetReason::PowerOn);
  });

  const sched::TaskConfig control_task{"Control_10ms", microseconds{10000}, microseconds{0}, microseconds{0},
                                       microseconds{2000}, app::kPriority10ms};
  const sched::TaskConfig dcm{"Dcm_10ms", microseconds{10000}, microseconds{500}, microseconds{0},
                              microseconds{3000}, app::kPriorityDcm10ms};
  const sched::TaskConfig com{"Com_20ms", microseconds{20000}, microseconds{1000}, microseconds{0},
                              microseconds{5000}, app::kPriority20ms};
  const sched::TaskConfig diag_task{"Diag_100ms", microseconds{100000}, microseconds{3000}, microseconds{0},
                                    microseconds{20000}, app::kPriority100ms};
  return platform_.routeToDiagnostics(diag::kPhysicalRequestId) &&
         platform_.routeToDiagnostics(diag::kFunctionalRequestId) &&
         platform_.addTask(control_task, [this](const sched::ActivationInfo& a) { taskControl(a); }) &&
         platform_.addTask(dcm, [this](const sched::ActivationInfo& a) { taskDcm(a); }) &&
         platform_.addTask(com, [this](const sched::ActivationInfo& a) { taskCom(a); }) &&
         platform_.addTask(diag_task, [this](const sched::ActivationInfo& a) { taskDiag(a); });
}

void CruiseEcu::logSummary() const {
  log_.info(core::strCat(
      "SUMMARY ecu=cruise_ecu bus=", platform_.busName(), " tx=", platform_.io().txOk(),
      " tx_failed=", platform_.io().txFailed(), " rx_0x100=", rx_vehicle_dynamics_.load(),
      " rx_0x110=", rx_driver_inputs_.load(), " invalid=", invalid_frames_.load(),
      " rejected_speed=", rejected_speed_samples_.load(), " alive_errors=", alive_errors_.load(),
      " timeouts=", timeout_events_.load(), " recoveries=", recovery_events_.load(),
      " engagements=", engagements_.load(), " faults=", fault_entries_.load(),
      " uds_requests=", uds_.statistics().requests, " uds_negative=", uds_.statistics().negative_responses,
      " resets=", reset_count_.load(), " isotp_errors=", isotp_.errorCount(),
      " watchdog_resets=", watchdog_resets_.load(), " active_faults=0x", std::hex, faults_.activeMask(), std::dec,
      " fault_events=", faults_.eventCount(), " nvm_writes=", nvm_writes_.load(),
      " stale_output_frames=", stale_output_frames_.load(),
      " operation_cycles=", operation_cycles_.load()));
  const FirmwareTimings t = firmware_.timings();
  log_.info(core::strCat("FW_SUMMARY version=", firmware_.runningVersion().toString(), " trial=",
                         firmware_.trialActive(), " boot_ms=", app::fixed(t.last_boot_ms, 3), " max_erase_ms=",
                         app::fixed(t.max_erase_ms, 3), " max_block_ms=", app::fixed(t.max_block_ms, 3),
                         " max_check_ms=", app::fixed(t.max_check_ms, 3), " last_download_bytes=",
                         t.last_download_bytes, " last_download_s=", app::fixed(t.last_download_s, 3)));
}

// ---------------------------------------------------------------- Control_10ms (prio 80) ----

void CruiseEcu::taskControl(const sched::ActivationInfo& activation) {
  const core::TimePoint now = core::now();
  if (reset_control_.exchange(false)) {
    controller_ = control::CruiseController{};  // simulated reset: application state re-initialised
    last_state_ = control::CruiseState::Off;
    defect_hang_at_.reset();
    if (firmware_.defectFlag()) {
      defect_hang_at_ = now + kDefectHangDelay;
    }
  }
  if (defect_hang_at_ && (now >= *defect_hang_at_)) {
    // Simulated defect of the running firmware image (test images only): a deadlock.
    log_.error("FW_DEFECT simulated firmware defect: Control task hangs");
    defect_hang_at_.reset();
    while (!platform_.injector().released()) {
      os::sleepUntil(core::now() + milliseconds{10});
    }
    return;
  }
  const auto sample = inputs_.read();
  control::CruiseInputs inputs = sample.value;
  // Stale-data protection: the COM task must refresh the inputs every 20 ms.
  inputs.inputs_stale = (sample.sequence != 0U) && ((now - sample.timestamp) > kMaxInputAge);
  // Function inhibition by qualified faults (FiM) or by the watchdog safe state.
  inputs.function_inhibited = faults_.functionInhibited() || safe_state_.load() || programming_session_.load() ||
                              bootloader_mode_.load();

  double dt = kControlPeriodS;
  if (platform_.injector().takeControllerNan(platform_.secondsSinceEpoch(activation.release))) {
    dt = std::numeric_limits<double>::quiet_NaN();  // test-only: corrupt one control step
  }
  const control::CruiseOutputs output = controller_.step(inputs, dt);
  if (output.state != last_state_) {
    log_.info(core::strCat("CRUISE_STATE ", control::toString(last_state_), " -> ", control::toString(output.state),
                           (output.state == control::CruiseState::Fault)
                               ? core::strCat(" reason=", control::toString(output.fault))
                               : std::string{},
                           (output.state == control::CruiseState::Active)
                               ? core::strCat(" target_kph=", app::fixed(output.target_speed_kph, 1),
                                              " speed_kph=", app::fixed(inputs.vehicle_speed_kph, 1))
                               : std::string{}));
    if ((output.state == control::CruiseState::Active) && (last_state_ == control::CruiseState::Standby)) {
      engagements_.fetch_add(1U);
    }
    if (output.state == control::CruiseState::Fault) {
      fault_entries_.fetch_add(1U);
    }
    last_state_ = output.state;
  }
  (void)faults_.report(kFaultController, output.fault == control::CruiseFault::ControllerFault, now);
  output_.write(output, now);
}

// -------------------------------------------------------------------- Com_20ms (prio 70) ----

void CruiseEcu::taskCom(const sched::ActivationInfo& /*activation*/) {
  if (reset_com_.exchange(false)) {
    monitor_.start(core::now());  // supervision restarts as after power-up
    alive_vehicle_dynamics_.reset();
    alive_driver_inputs_.reset();
    com_inputs_ = control::CruiseInputs{};
    has_accepted_speed_ = false;
    sender_invalid_ = false;
  }
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
  reportCommunicationFault(vehicle::kVehicleDynamics.id, kFaultSpeedTimeout, now);
  reportCommunicationFault(vehicle::kDriverInputs.id, kFaultDriverTimeout, now);

  com_inputs_.speed_timeout = monitor_.state(vehicle::kVehicleDynamics.id) == can::RxTimeoutState::TimedOut;
  com_inputs_.driver_inputs_timeout = monitor_.state(vehicle::kDriverInputs.id) == can::RxTimeoutState::TimedOut;
  // The speed is usable only if it is received, the sender marks it valid, and no
  // plausibility/staleness fault is qualified.
  com_inputs_.speed_valid = has_accepted_speed_ && !sender_invalid_ &&
                            (monitor_.state(vehicle::kVehicleDynamics.id) == can::RxTimeoutState::Ok) &&
                            !faults_.isActive(kFaultSpeedInvalid) && !faults_.isActive(kFaultSpeedStale);
  com_inputs_.vehicle_speed_kph = accepted_speed_kph_;
  inputs_.write(com_inputs_, now);

  // Stale-output protection (independent of the watchdog and faster than it): never keep
  // transmitting a frozen control output.
  const auto output = output_.read();
  const bool output_stale = (output.sequence != 0U) && ((now - output.timestamp) > kMaxOutputAge);
  if (output_stale) {
    stale_output_frames_.fetch_add(1U);
  }
  if (output_stale != output_was_stale_) {
    log_.warning(core::strCat(output_stale ? "CONTROL_OUTPUT_STALE age_ms=" : "CONTROL_OUTPUT_FRESH age_ms=",
                              app::fixed(core::toMillis(now - output.timestamp), 1),
                              " t=", app::fixed(platform_.secondsSinceEpoch(now), 3), "s"));
    output_was_stale_ = output_stale;
  }
  sendStatus(output.value, output_stale);
}

void CruiseEcu::handleVehicleDynamics(const can::RxFrame& entry) {
  const can::CanFrame& frame = entry.frame;
  if (frame.dlc != vehicle::kVehicleDynamics.dlc) {
    invalid_frames_.fetch_add(1U);
    return;
  }
  rx_vehicle_dynamics_.fetch_add(1U);
  (void)monitor_.notifyReceived(frame.id, entry.rx_time, [this](const can::RxTimeoutEvent& e) { onTimeoutEvent(e); });

  // Sensor signal timeout ("stale"): the alive counter did not advance -> old data repeated.
  const can::AliveCheckResult alive = checkAlive(frame, vehicle::kVehicleDynamicsAlive, alive_vehicle_dynamics_);
  (void)faults_.report(kFaultSpeedStale, alive == can::AliveCheckResult::Repeated, entry.rx_time);

  double speed = 0.0;
  double valid = 0.0;
  const bool in_range = can::decodePhysical(vehicle::kVehicleSpeed, frame, speed) == can::SignalStatus::Ok;
  (void)can::decodePhysical(vehicle::kVehicleSpeedValid, frame, valid);

  if (valid < 0.5) {
    // The sensor itself reports the value as invalid: do not use it, immediately.
    sender_invalid_ = true;
    (void)faults_.report(kFaultSpeedInvalid, true, entry.rx_time);
    return;
  }
  sender_invalid_ = false;
  // Gradient plausibility: the change since the last accepted sample must be physically possible.
  bool plausible = in_range;
  if (plausible && has_accepted_speed_) {
    const double dt_s = core::toMillis(entry.rx_time - accepted_speed_time_) / 1000.0;
    plausible = std::fabs(speed - accepted_speed_kph_) <= (kMaxSpeedRateKphPerS * dt_s) + kSpeedRateToleranceKph;
  }
  if (plausible) {
    accepted_speed_kph_ = speed;
    accepted_speed_time_ = entry.rx_time;
    has_accepted_speed_ = true;
  } else {
    rejected_speed_samples_.fetch_add(1U);  // keep the last plausible value (substitute value)
  }
  (void)faults_.report(kFaultSpeedInvalid, !plausible, entry.rx_time);
}

void CruiseEcu::handleDriverInputs(const can::RxFrame& entry) {
  const can::CanFrame& frame = entry.frame;
  if (frame.dlc != vehicle::kDriverInputs.dlc) {
    invalid_frames_.fetch_add(1U);
    return;
  }
  rx_driver_inputs_.fetch_add(1U);
  (void)monitor_.notifyReceived(frame.id, entry.rx_time, [this](const can::RxTimeoutEvent& e) { onTimeoutEvent(e); });
  (void)checkAlive(frame, vehicle::kDriverInputsAlive, alive_driver_inputs_);

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
    invalid_frames_.fetch_add(1U);  // implausible driver input: keep the last valid values
    return;
  }
  com_inputs_.accelerator_pct = accelerator;
  com_inputs_.brake_pressed = brake > 0.5;
  com_inputs_.main_switch = main_switch > 0.5;
  com_inputs_.set_request = set_request > 0.5;
  com_inputs_.cancel_request = cancel_request > 0.5;
  com_inputs_.driver_set_speed_kph = set_speed;
}

void CruiseEcu::reportCommunicationFault(std::uint32_t id, faults::FaultId fault, core::TimePoint now) {
  const can::RxTimeoutState state = monitor_.state(id);
  if (state != can::RxTimeoutState::WaitingFirst) {  // no result before the first reception
    (void)faults_.report(fault, state == can::RxTimeoutState::TimedOut, now);
  }
}

can::AliveCheckResult CruiseEcu::checkAlive(const can::CanFrame& frame, const can::SignalDef& signal,
                                            can::AliveCounterChecker& checker) {
  double value = 0.0;
  if (can::decodePhysical(signal, frame, value) != can::SignalStatus::Ok) {
    alive_errors_.fetch_add(1U);
    return can::AliveCheckResult::Invalid;
  }
  const can::AliveCheckResult result = checker.check(static_cast<std::uint8_t>(value));
  if ((result == can::AliveCheckResult::Repeated) || (result == can::AliveCheckResult::Skipped) ||
      (result == can::AliveCheckResult::Invalid)) {
    alive_errors_.fetch_add(1U);
    log_.debug(core::strCat("ALIVE_ERROR id=", can::formatId(frame.id), " result=", can::toString(result)));
  }
  return result;
}

void CruiseEcu::onTimeoutEvent(const can::RxTimeoutEvent& event) {
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

void CruiseEcu::sendStatus(const control::CruiseOutputs& output, bool output_stale) {
  can::CanFrame frame = can::makeFrame(vehicle::kCruiseControlStatus);
  const bool watchdog_safe = safe_state_.load();
  // In the safe state the COM task transmits Fault / 0 % itself: the Control task may be the
  // one that stalled, so its last output must not be trusted.
  const bool safe = watchdog_safe || output_stale;
  const double throttle = safe ? 0.0 : output.throttle_pct;
  const auto state = safe ? control::CruiseState::Fault : output.state;
  const double reason = watchdog_safe  ? kFaultReasonWatchdogSafeState
                        : output_stale ? kFaultReasonControlOutputStale
                                       : static_cast<double>(output.fault);
  (void)can::encodePhysical(vehicle::kThrottleCommand, throttle, frame);
  (void)can::encodePhysical(vehicle::kCruiseState, static_cast<double>(state), frame);
  (void)can::encodePhysical(vehicle::kCruiseTargetSpeed, safe ? 0.0 : output.target_speed_kph, frame);
  (void)can::encodePhysical(vehicle::kCruiseFaultReason, reason, frame);
  (void)can::encodePhysical(vehicle::kCruiseStatusAlive, alive_tx_.next(), frame);
  (void)platform_.io().send(frame);
}

// -------------------------------------------------------------------- Dcm_10ms (prio 75) ----

void CruiseEcu::taskDcm(const sched::ActivationInfo& /*activation*/) {
  const core::TimePoint now = core::now();
  can::RxFrame entry;
  while (platform_.diagRxQueue().pop(entry)) {
    platform_.io().traceReceived(entry);
    if (entry.frame.id == diag::kFunctionalRequestId) {
      handleFunctionalRequest(entry.frame, now);
    } else {
      isotp_.onFrame(entry.frame, now);
    }
  }
  if (isotp_.hasMessage()) {
    respond(isotp_.message(), isotp_.messageLength(), false, now);
    isotp_.consumeMessage();
  }
  isotp_.poll(now);
  uds_.poll(now);
  const diag::IsoTpError error = isotp_.takeError();
  if (error != diag::IsoTpError::None) {
    log_.warning(core::strCat("ISOTP_ERROR ", diag::toString(error)));
  }
  if (const auto reset = uds_.takePendingReset()) {
    // Executed after the positive response has been queued (ISO 14229-1).
    executeReset((*reset == diag::ResetType::HardReset) ? faults::ResetReason::UdsHardReset
                                                         : faults::ResetReason::UdsSoftReset);
  }
}

void CruiseEcu::handleFunctionalRequest(const can::CanFrame& frame, core::TimePoint now) {
  const std::size_t length = frame.data[0] & 0x0FU;
  if (((frame.data[0] >> 4U) != 0U) || (length == 0U) || (length > 7U) || (length + 1U > frame.dlc)) {
    return;  // functional requests must be single frames
  }
  respond(frame.data.data() + 1, length, true, now);
}

void CruiseEcu::respond(const std::uint8_t* request, std::size_t length, bool functional, core::TimePoint now) {
  const std::size_t response_length =
      uds_.handleRequest(request, length, functional, now, response_.data(), response_.size());
  log_.debug(core::strCat("UDS request sid=0x", std::hex, static_cast<unsigned int>(request[0]), std::dec,
                          " len=", length, " functional=", functional, " response_len=", response_length));
  if ((response_length != 0U) &&
      (isotp_.send(response_.data(), response_length, now) != diag::IsoTpSendResult::Ok)) {
    dropped_responses_.fetch_add(1U);  // previous multi-frame response still in progress
  }
}

void CruiseEcu::executeReset(faults::ResetReason reason) {
  reset_count_.fetch_add(1U);
  last_reset_reason_ = reason;
  log_.warning(core::strCat("ECU_RESET type=", resetName(reason),
                            " (simulated: application state re-initialised, process keeps running)"));
  if (reason != faults::ResetReason::UdsSoftReset) {
    // Hard reset / watchdog reset = power cycle = new operation cycle (releases latched faults).
    operation_cycles_.fetch_add(1U);
    faults_.startOperationCycle();
    bootFirmware(core::now());  // a hard reset passes through the bootloader
  }
  reset_control_ = true;
  reset_com_ = true;
}

void CruiseEcu::registerDids() {
  auto addText = [this](std::uint16_t id, std::string_view value, diag::SessionMask sessions) {
    return uds_.addDid(id, static_cast<std::uint16_t>(value.size()), sessions,
                       [value](std::uint8_t* out) { std::copy(value.begin(), value.end(), out); });
  };
  bool ok = addText(0xF190U, kVin, diag::kSessionAll);
  ok = addText(0xF187U, kSparePartNumber, diag::kSessionAll) && ok;
  ok = uds_.addDid(0xF189U, 8U, diag::kSessionAll, [this](std::uint8_t* out) {
    const std::string text = firmware_.runningVersion().toString();  // version of the booted image
    std::copy_n(text.begin(), std::min<std::size_t>(text.size(), 8U), out);
  }) && ok;
  ok = addText(0xF18CU, kSerialNumber, diag::kSessionExtended) && ok;
  ok = uds_.addDid(0xF186U, 1U, diag::kSessionAll,
                   [this](std::uint8_t* out) { out[0] = static_cast<std::uint8_t>(uds_.session()); }) && ok;
  ok = uds_.addDid(0x0100U, 2U, diag::kSessionAll,
                   [this](std::uint8_t* out) { putU16(out, inputs_.read().value.vehicle_speed_kph * 100.0); }) && ok;
  ok = uds_.addDid(0x0101U, 1U, diag::kSessionAll, [this](std::uint8_t* out) {
    out[0] = static_cast<std::uint8_t>(safe_state_.load() ? control::CruiseState::Fault : output_.read().value.state);
  }) && ok;
  ok = uds_.addDid(0x0102U, 1U, diag::kSessionAll, [this](std::uint8_t* out) {
    out[0] = static_cast<std::uint8_t>(std::clamp(output_.read().value.target_speed_kph, 0.0, 255.0));
  }) && ok;
  ok = uds_.addDid(0x0103U, 2U, diag::kSessionAll,
                   [this](std::uint8_t* out) { putU16(out, output_.read().value.throttle_pct * 100.0); }) && ok;
  ok = uds_.addDid(0x0104U, 1U, diag::kSessionAll, [this](std::uint8_t* out) {
    out[0] = safe_state_.load() ? kFaultReasonWatchdogSafeState
                                : static_cast<std::uint8_t>(output_.read().value.fault);
  }) && ok;
  ok = uds_.addDid(0x0200U, 4U, diag::kSessionExtended, [this](std::uint8_t* out) {
    std::uint64_t misses = 0U;
    for (std::size_t i = 0U; i < platform_.scheduler().taskCount(); ++i) {
      misses += platform_.scheduler().deadlineMisses(i);
    }
    putU32(out, misses);
  }) && ok;
  ok = uds_.addDid(0x0201U, 4U, diag::kSessionAll, [this](std::uint8_t* out) {
    putU32(out, static_cast<std::uint64_t>(std::max(0.0, platform_.secondsSinceEpoch(core::now()))));
  }) && ok;
  ok = uds_.addDid(0x0202U, 2U, diag::kSessionAll,
                   [this](std::uint8_t* out) { putU16(out, static_cast<double>(reset_count_.load())); }) && ok;
  // Phase 5: fault management and watchdog
  ok = uds_.addDid(0x0300U, 2U, diag::kSessionAll,
                   [this](std::uint8_t* out) { putU16(out, static_cast<double>(faults_.activeMask())); }) && ok;
  ok = uds_.addDid(0x0301U, 3U, diag::kSessionAll, [this](std::uint8_t* out) {
    out[0] = static_cast<std::uint8_t>(platform_.watchdog().globalStatus());
    putU16(out + 1, static_cast<double>(platform_.watchdog().expirations()));
  }) && ok;
  ok = uds_.addDid(0x0302U, 1U, diag::kSessionAll,
                   [this](std::uint8_t* out) { out[0] = static_cast<std::uint8_t>(last_reset_reason_.load()); }) && ok;
  ok = uds_.addDid(0x0303U, 4U, diag::kSessionAll,
                   [this](std::uint8_t* out) { putU32(out, operation_cycles_.load()); }) && ok;
  ok = uds_.addDid(0x0304U, 1U, diag::kSessionAll,
                   [this](std::uint8_t* out) { out[0] = static_cast<std::uint8_t>(nvm_load_status_); }) && ok;
  if (!ok) {
    log_.error("DID registration failed");
  }
}

// ------------------------------------------------------------------ Diag_100ms (prio 50) ----

void CruiseEcu::taskDiag(const sched::ActivationInfo& activation) {
  const core::TimePoint now = core::now();
  platform_.reportNewDeadlineMisses();
  // Task deadline-miss monitor: any new miss in this 100 ms window counts as a failed test.
  std::uint64_t total = 0U;
  for (std::size_t i = 0U; i < platform_.scheduler().taskCount(); ++i) {
    total += platform_.scheduler().deadlineMisses(i);
  }
  (void)faults_.report(kFaultDeadlineMiss, total > last_total_misses_, now);
  last_total_misses_ = total;

  logFaultEvents();
  saveNvmIfChanged(now);
  superviseTrialImage(now);

  if ((activation.activation > 0U) && ((activation.activation % 10U) == 0U)) {  // every 1 s
    const control::CruiseInputs inputs = inputs_.read().value;
    const control::CruiseOutputs output = output_.read().value;
    log_.info(core::strCat("state=", control::toString(output.state), " speed=", app::fixed(inputs.vehicle_speed_kph, 2),
                           "km/h target=", app::fixed(output.target_speed_kph, 0), "km/h throttle=",
                           app::fixed(output.throttle_pct, 2), "% active_faults=0x", std::hex, faults_.activeMask(),
                           std::dec, " safe_state=", safe_state_.load(), " rx_0x100=", rx_vehicle_dynamics_.load(),
                           " rxq_overruns=", platform_.rxQueue().overruns()));
  }
}

void CruiseEcu::logFaultEvents() {
  std::array<faults::FaultEvent, faults::FaultManager::kEventMemory> events{};
  const std::size_t n = faults_.eventsSince(last_event_sequence_, events.data(), events.size());
  for (std::size_t i = 0U; i < n; ++i) {
    const faults::FaultEvent& e = events[i];
    last_event_sequence_ = e.sequence;
    // Only qualification and recovery are logged; the debouncing states are visible via UDS.
    if ((e.to == faults::FaultState::Failed) || (e.to == faults::FaultState::Passed &&
                                                  e.from != faults::FaultState::NotTested &&
                                                  e.from != faults::FaultState::PreFailed)) {
      log_.warning(core::strCat("FAULT ", faults_.name(e.id), " ", faults::toString(e.from), " -> ",
                                faults::toString(e.to), " t=", app::fixed(platform_.secondsSinceEpoch(e.time), 3), "s"));
    }
  }
}

// -------------------------------------------------------------------------- firmware ----

void CruiseEcu::bootFirmware(core::TimePoint now) {
  const firmware::BootResult result = firmware_.boot();
  bootloader_mode_ = !result.application_valid;
  if (!result.application_valid) {
    log_.error("BOOTLOADER_MODE no valid application image: cruise function disabled, flashing possible");
  }
  if ((result.event == firmware::BootEvent::Rollback) || (result.event == firmware::BootEvent::FallbackCorrupt) ||
      (result.event == firmware::BootEvent::NoValidImage)) {
    (void)faults_.report(kFaultFirmware, true, now);
  }
  boot_generation_.fetch_add(1U);
  reset_control_ = true;  // the Control task picks up the image's properties
}

void CruiseEcu::superviseTrialImage(core::TimePoint now) {
  // A trial image is confirmed after a continuous healthy period: no watchdog safe state.
  // (Any reset before that, including a watchdog exit, counts as a failed boot attempt.)
  const std::uint32_t generation = boot_generation_.load();
  if ((generation != seen_boot_generation_) || safe_state_.load()) {
    seen_boot_generation_ = generation;
    healthy_since_ = now;
    return;
  }
  const auto healthy_for = std::chrono::duration<double>(now - healthy_since_).count();
  if ((healthy_for >= platform_.options().confirm_after_s) && firmware_.trialActive()) {
    (void)firmware_.confirmTrial();
  }
}

std::optional<diag::Nrc> CruiseEcu::checkSessionChange(diag::Session requested) const {
  if (requested != diag::Session::Programming) {
    return std::nullopt;
  }
  const double speed = inputs_.read().value.vehicle_speed_kph;
  const bool cruise_active = output_.read().value.state == control::CruiseState::Active;
  if ((speed > kMaxProgrammingSpeedKph) || cruise_active) {
    log_.warning(core::strCat("UDS programming session refused: speed_kph=", app::fixed(speed, 1),
                              " cruise_active=", cruise_active));
    return diag::Nrc::ConditionsNotCorrect;
  }
  return std::nullopt;
}

// ------------------------------------------------------------------------------- NVM ----

void CruiseEcu::loadNvm() {
  const std::string& path = platform_.options().nvm_path;
  if (path.empty()) {
    log_.info("NVM not configured: fault memory is volatile (use --nvm FILE)");
    return;
  }
  faults::NvmData data;
  nvm_load_status_ = faults::loadNvm(path, data);
  switch (nvm_load_status_) {
    case faults::NvmStatus::Ok:
      (void)dtcs_.importRecords(data.records.data(), data.record_count);
      reset_count_ = data.reset_count;
      operation_cycles_ = data.operation_cycles;
      last_reset_reason_ = data.last_reset_reason;
      log_.info(core::strCat("NVM_LOADED file=", path, " records=", data.record_count,
                             " last_reset=", faults::toString(data.last_reset_reason), " resets=", data.reset_count,
                             " operation_cycles=", data.operation_cycles));
      break;
    case faults::NvmStatus::NotFound:
      log_.info(core::strCat("NVM_EMPTY file=", path, " (first start)"));
      break;
    case faults::NvmStatus::Corrupted:
    case faults::NvmStatus::IoError:
      log_.error(core::strCat("NVM_CORRUPTED file=", path, " status=", faults::toString(nvm_load_status_),
                              ": starting with empty fault memory"));
      break;
  }
}

void CruiseEcu::saveNvm(faults::ResetReason reason_for_next_start) {
  const std::string& path = platform_.options().nvm_path;
  if (path.empty()) {
    return;
  }
  const std::lock_guard<std::mutex> lock(nvm_mutex_);
  faults::NvmData data;
  data.record_count = dtcs_.snapshot(data.records.data(), data.records.size(), 0U);
  data.operation_cycles = operation_cycles_.load();
  data.reset_count = reset_count_.load();
  data.last_reset_reason = reason_for_next_start;
  const faults::NvmStatus status = faults::saveNvm(path, data);
  if (status != faults::NvmStatus::Ok) {
    log_.error(core::strCat("NVM_WRITE_FAILED file=", path, " status=", faults::toString(status)));
    return;
  }
  std::copy_n(data.records.begin(), data.record_count, nvm_saved_.begin());
  nvm_saved_count_ = data.record_count;
  nvm_saved_resets_ = data.reset_count;
  nvm_writes_.fetch_add(1U);
}

void CruiseEcu::saveNvmIfChanged(core::TimePoint now) {
  if (platform_.options().nvm_path.empty() || ((now - nvm_last_save_) < kNvmMinWriteInterval)) {
    return;  // rate-limited: limits flash wear on a real ECU, file I/O here
  }
  std::array<faults::DtcRecord, faults::DtcStore::kCapacity> current{};
  const std::size_t count = dtcs_.snapshot(current.data(), current.size(), 0U);
  bool changed = false;
  {
    const std::lock_guard<std::mutex> lock(nvm_mutex_);
    changed = (count != nvm_saved_count_) || !sameRecords(current.data(), nvm_saved_.data(), count) ||
              (reset_count_.load() != nvm_saved_resets_);
  }
  if (changed) {
    nvm_last_save_ = now;
    saveNvm(last_reset_reason_.load());
  }
}

// --------------------------------------------------------------------- watchdog recovery ----

void CruiseEcu::onWatchdogExpired(const watchdog::CheckResult& result, const std::string& entity) {
  // 1) Safe state first: COM now transmits Fault / 0 % throttle, Control is inhibited.
  safe_state_ = true;
  (void)faults_.report(kFaultWatchdog, true, core::now());
  log_.error(core::strCat("SAFE_STATE entered: watchdog expired (", entity, ", ", watchdog::toString(result.violation),
                          ")"));
  // 2) Optional real restart: persist the reason and terminate (external supervisor restarts).
  if (platform_.options().watchdog_action == app::WatchdogAction::Exit) {
    reset_count_.fetch_add(1U);
    last_reset_reason_ = faults::ResetReason::Watchdog;
    platform_.emergencyExit(kWatchdogExitCode);
  }
}

void CruiseEcu::onWatchdogResumed() {
  if (platform_.options().watchdog_action != app::WatchdogAction::Reset) {
    return;  // action "none": no automatic recovery (latched fault until a reset)
  }
  if (watchdog_resets_.load() >= kMaxWatchdogResets) {
    permanent_safe_state_ = true;
    log_.error(core::strCat("WATCHDOG_RESET_LIMIT reached (", kMaxWatchdogResets, "): staying in the safe state"));
    return;
  }
  watchdog_resets_.fetch_add(1U);
  executeReset(faults::ResetReason::Watchdog);
}

void CruiseEcu::onWatchdogHealthy() {
  if (!permanent_safe_state_.load()) {
    safe_state_ = false;
    log_.warning("SAFE_STATE left: supervision healthy after recovery (driver must re-engage)");
  }
}

}  // namespace vecu::cruise
