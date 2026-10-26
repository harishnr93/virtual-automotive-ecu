#pragma once

// Virtual Cruise Control ECU application.
//
// Task structure (rate-monotonic priorities):
//   Watchdog thread  (prio 95) software watchdog of all tasks (EcuPlatform)
//   CanRx thread     (prio 90) event-driven reception, routes diagnostics to the Dcm queue
//   Control_10ms     (prio 80) CruiseController: state machine + PID speed control
//   Dcm_10ms         (prio 75) diagnostics: ISO-TP + UDS server (requests on 0x7E0/0x7DF)
//   Com_20ms         (prio 70) RX processing, signal plausibility, supervision, transmits 0x200
//   Diag_100ms       (prio 50) deadline-miss monitor, fault event log, NVM write, status,
//                              confirmation of a trial firmware image
//
// Firmware (Phase 6): FirmwareService runs the bootloader at start-up and on every hard /
// watchdog reset and handles the UDS programming services.
//
// The control algorithm (src/control) knows nothing about CAN, threads or faults; this class
// is the glue: bus signals -> CruiseInputs -> CruiseController -> CruiseOutputs -> bus, plus
// fault management, diagnostics, NVM persistence and watchdog recovery.

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <optional>

#include "can/alive_counter.hpp"
#include "can/can_receiver.hpp"
#include "can/can_signal.hpp"
#include "can/message_timeout_monitor.hpp"
#include "common/ecu_runtime.hpp"
#include "control/cruise_controller.hpp"
#include "core/periodic_scheduler.hpp"
#include "core/shared_signal.hpp"
#include "cruise_ecu/firmware_service.hpp"
#include "diagnostics/isotp.hpp"
#include "diagnostics/uds_server.hpp"
#include "faults/dtc_store.hpp"
#include "faults/fault_manager.hpp"
#include "faults/nvm_storage.hpp"

namespace vecu::cruise {

// Single instance per process; members are grouped by owning task for readability, not by size.
class CruiseEcu {  // NOLINT(clang-analyzer-optin.performance.Padding)
 public:
  explicit CruiseEcu(app::EcuPlatform& platform);

  /// Registers faults, loads the NVM, starts the operation cycle, registers DIDs, tasks,
  /// watchdog handlers and the shutdown hook. Call after platform.init(), before run().
  [[nodiscard]] bool initialize();

  void logSummary() const;

 private:
  // tasks
  void taskControl(const sched::ActivationInfo& activation);
  void taskDcm(const sched::ActivationInfo& activation);
  void taskCom(const sched::ActivationInfo& activation);
  void taskDiag(const sched::ActivationInfo& activation);

  // COM helpers
  void handleVehicleDynamics(const can::RxFrame& entry);
  void handleDriverInputs(const can::RxFrame& entry);
  void reportCommunicationFault(std::uint32_t id, faults::FaultId fault, core::TimePoint now);
  can::AliveCheckResult checkAlive(const can::CanFrame& frame, const can::SignalDef& signal,
                                   can::AliveCounterChecker& checker);
  void onTimeoutEvent(const can::RxTimeoutEvent& event);
  void sendStatus(const control::CruiseOutputs& output, bool output_stale);

  // diagnostics / reset
  void handleFunctionalRequest(const can::CanFrame& frame, core::TimePoint now);
  void respond(const std::uint8_t* request, std::size_t length, bool functional, core::TimePoint now);
  void executeReset(faults::ResetReason reason);
  void registerDids();

  // firmware
  void bootFirmware(core::TimePoint now);
  void superviseTrialImage(core::TimePoint now);
  [[nodiscard]] std::optional<diag::Nrc> checkSessionChange(diag::Session requested) const;

  // fault memory persistence
  void loadNvm();
  void saveNvm(faults::ResetReason reason_for_next_start);
  void saveNvmIfChanged(core::TimePoint now);
  void logFaultEvents();

  // watchdog recovery
  void onWatchdogExpired(const watchdog::CheckResult& result, const std::string& entity);
  void onWatchdogResumed();
  void onWatchdogHealthy();

  app::EcuPlatform& platform_;
  const core::Logger& log_;

  // Fault memory (thread-safe).
  faults::DtcStore dtcs_{3U};
  faults::FaultManager faults_{dtcs_};
  std::atomic<std::uint32_t> reset_count_{0U};
  std::atomic<std::uint32_t> operation_cycles_{0U};
  std::atomic<faults::ResetReason> last_reset_reason_{faults::ResetReason::PowerOn};
  faults::NvmStatus nvm_load_status_{faults::NvmStatus::NotFound};
  std::mutex nvm_mutex_;  // serialises NVM writes (Diag task, shutdown hook, watchdog exit)
  std::array<faults::DtcRecord, faults::DtcStore::kCapacity> nvm_saved_{};
  std::size_t nvm_saved_count_{0U};
  std::uint32_t nvm_saved_resets_{0U};
  core::TimePoint nvm_last_save_{};
  std::atomic<std::uint32_t> nvm_writes_{0U};

  // Shared between tasks.
  core::SharedSignal<control::CruiseInputs> inputs_{};
  core::SharedSignal<control::CruiseOutputs> output_{};
  std::atomic<bool> safe_state_{false};  ///< watchdog: transmit Fault / 0 % regardless of Control
  std::atomic<bool> reset_control_{false};
  std::atomic<bool> reset_com_{false};
  std::atomic<std::uint32_t> watchdog_resets_{0U};
  std::atomic<bool> permanent_safe_state_{false};
  std::atomic<bool> programming_session_{false};  ///< cruise function inhibited while flashing
  std::atomic<bool> bootloader_mode_{false};      ///< no valid image: diagnostics only

  // Firmware (thread-safe, see firmware_service.hpp).
  FirmwareService firmware_;

  // Owned by Dcm_10ms.
  diag::UdsServer uds_;
  diag::IsoTpEndpoint isotp_;
  std::array<std::uint8_t, diag::kIsoTpMaxPayload> response_{};
  std::atomic<std::uint32_t> dropped_responses_{0U};

  // Owned by Com_20ms.
  can::MessageTimeoutMonitor<2U> monitor_{};
  can::AliveCounterChecker alive_vehicle_dynamics_{};
  can::AliveCounterChecker alive_driver_inputs_{};
  can::AliveCounterGenerator alive_tx_{};
  control::CruiseInputs com_inputs_{};
  bool has_accepted_speed_{false};
  bool sender_invalid_{false};
  double accepted_speed_kph_{0.0};
  core::TimePoint accepted_speed_time_{};
  bool output_was_stale_{false};

  // Owned by Control_10ms.
  control::CruiseController controller_{};
  control::CruiseState last_state_{control::CruiseState::Off};
  std::optional<core::TimePoint> defect_hang_at_{};  ///< simulated defect of the running image

  // Owned by Diag_100ms.
  std::uint64_t last_total_misses_{0U};
  std::uint32_t last_event_sequence_{0U};
  core::TimePoint unhealthy_at_{};  ///< last Diag cycle in the watchdog safe state

  // Counters: written by one task, read by others / at the end.
  std::atomic<std::uint64_t> rx_vehicle_dynamics_{0U};
  std::atomic<std::uint64_t> rx_driver_inputs_{0U};
  std::atomic<std::uint64_t> invalid_frames_{0U};
  std::atomic<std::uint64_t> rejected_speed_samples_{0U};
  std::atomic<std::uint64_t> alive_errors_{0U};
  std::atomic<std::uint32_t> timeout_events_{0U};
  std::atomic<std::uint32_t> recovery_events_{0U};
  std::atomic<std::uint32_t> engagements_{0U};
  std::atomic<std::uint32_t> fault_entries_{0U};
  std::atomic<std::uint64_t> stale_output_frames_{0U};
};

}  // namespace vecu::cruise
