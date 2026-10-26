#pragma once

// Fault configuration of the cruise control ECU (comparable to a DEM/FiM configuration).
//
// | Fault               | Monitor (task)                    | Debounce -> Failed      | Recovery (-> Passed)   | Latched | Inhibits |
// |---------------------|-----------------------------------|-------------------------|------------------------|---------|----------|
// | SpeedMessageTimeout | 0x100 timeout monitor (Com_20ms)  | immediate (timeout 100) | 500 ms received        | no      | yes      |
// | DriverInputsTimeout | 0x110 timeout monitor (Com_20ms)  | immediate (timeout 250) | 500 ms received        | no      | yes      |
// | SpeedSignalInvalid  | validity bit, range, gradient     | 40 ms continuously      | 500 ms plausible       | no      | yes      |
// | SpeedSignalStale    | alive counter repeated (0x100)    | 3 stale frames          | 5 fresh frames         | no      | yes      |
// | TaskDeadlineMiss    | scheduler counters (Diag_100ms)   | misses in 2 windows     | 10 clean windows (1 s) | no      | yes      |
// | ControllerFault     | controller output (Control_10ms)  | immediate               | next operation cycle   | yes     | yes      |
// | WatchdogExpired     | software watchdog thread          | immediate               | next operation cycle   | yes     | yes      |
// | NvmCorrupted        | NVM load at start-up              | immediate               | next operation cycle   | yes     | no       |
// | FirmwareRollback    | bootloader result (boot / reset)  | immediate               | next operation cycle   | yes     | no       |
//
// Signal-level reactions (discard a bad value, sender-invalid speed) happen immediately in the
// COM task; the table above governs DTCs, function inhibition and recovery.

#include <array>
#include <chrono>

#include "faults/fault_manager.hpp"

namespace vecu::cruise {

using faults::DebounceConfig;
using faults::FaultId;
using std::chrono::milliseconds;

enum : FaultId {
  kFaultSpeedTimeout = 0U,
  kFaultDriverTimeout = 1U,
  kFaultSpeedInvalid = 2U,
  kFaultSpeedStale = 3U,
  kFaultDeadlineMiss = 4U,
  kFaultController = 5U,
  kFaultWatchdog = 6U,
  kFaultNvm = 7U,
  kFaultFirmware = 8U,
  kFaultCount = 9U
};

// DTCs: SAE J2012 code + failure type byte (FTB).
inline constexpr faults::DtcCode kDtcSpeedMessageMissing = 0xC12187U;  // U0121-87 missing message
inline constexpr faults::DtcCode kDtcDriverInputsMissing = 0xC14087U;  // U0140-87 missing message
inline constexpr faults::DtcCode kDtcSpeedImplausible = 0x403164U;     // C0031-64 signal plausibility failure
inline constexpr faults::DtcCode kDtcSpeedStale = 0xC12182U;           // U0121-82 alive counter not updated
inline constexpr faults::DtcCode kDtcDeadlineMiss = 0xF00048U;         // U3000-48 supervision software failure
inline constexpr faults::DtcCode kDtcControllerInternal = 0xF00049U;   // U3000-49 internal electronic failure
inline constexpr faults::DtcCode kDtcWatchdog = 0xF00047U;             // U3000-47 watchdog / safety failure
inline constexpr faults::DtcCode kDtcNvm = 0xF00044U;                  // U3000-44 data memory failure
inline constexpr faults::DtcCode kDtcFirmware = 0xF00096U;             // U3000-96 component internal failure

struct FaultDefinition {
  FaultId id{0U};
  faults::FaultConfig config{};
};

inline constexpr std::array<FaultDefinition, kFaultCount> kFaultTable{{
    {kFaultSpeedTimeout, {"SpeedMessageTimeout", kDtcSpeedMessageMissing,
                          DebounceConfig::time(milliseconds{0}, milliseconds{500}), false, true}},
    {kFaultDriverTimeout, {"DriverInputsTimeout", kDtcDriverInputsMissing,
                           DebounceConfig::time(milliseconds{0}, milliseconds{500}), false, true}},
    {kFaultSpeedInvalid, {"SpeedSignalInvalid", kDtcSpeedImplausible,
                          DebounceConfig::time(milliseconds{40}, milliseconds{500}), false, true}},
    {kFaultSpeedStale, {"SpeedSignalStale", kDtcSpeedStale, DebounceConfig::counter(34, 20, 100, 0), false, true}},
    {kFaultDeadlineMiss, {"TaskDeadlineMiss", kDtcDeadlineMiss, DebounceConfig::counter(50, 10, 100, 0), false, true}},
    {kFaultController, {"ControllerFault", kDtcControllerInternal,
                        DebounceConfig::time(milliseconds{0}, milliseconds{0}), true, true}},
    {kFaultWatchdog, {"WatchdogExpired", kDtcWatchdog, DebounceConfig::time(milliseconds{0}, milliseconds{0}), true,
                      true}},
    {kFaultNvm, {"NvmCorrupted", kDtcNvm, DebounceConfig::time(milliseconds{0}, milliseconds{0}), true, false}},
    // Reported when the bootloader rejected an image (trial never confirmed -> rollback, or
    // corrupt image -> fallback). The ECU runs the previous image normally: no inhibition.
    {kFaultFirmware, {"FirmwareRollback", kDtcFirmware, DebounceConfig::time(milliseconds{0}, milliseconds{0}), true,
                      false}},
}};

/// Physical plausibility limit for the speed signal: no car changes speed by more than
/// 100 km/h per second (~2.8 g); plus a tolerance for signal resolution and jitter.
inline constexpr double kMaxSpeedRateKphPerS = 100.0;
inline constexpr double kSpeedRateToleranceKph = 1.0;

/// Software watchdog: maximum number of automatic ECU resets per operation cycle before the
/// ECU stays in the safe state.
inline constexpr unsigned int kMaxWatchdogResets = 3U;

/// Exit code used with --watchdog-action exit (EX_TEMPFAIL: "try again", i.e. restart me).
inline constexpr int kWatchdogExitCode = 75;

}  // namespace vecu::cruise
