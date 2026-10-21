#pragma once

// Vehicle CAN matrix of the Virtual Automotive ECU Platform.
//
// Single source of truth for the C++ side. It must match config/vehicle_can.dbc exactly;
// the unit test "VehicleSignals.MatchesDbcFile" parses the DBC file and fails on any
// difference. Tables are constexpr, so they live in read-only memory (as they would in
// flash on a microcontroller) and cost nothing at start-up.

#include <array>
#include <cstdint>

#include "can/can_signal.hpp"

namespace vecu::can::vehicle {

// ---------------------------------------------------------------------------
// 0x100 VehicleDynamics  (VehicleSim -> CruiseECU), 20 ms
// ---------------------------------------------------------------------------
inline constexpr std::array<SignalDef, 4U> kVehicleDynamicsSignals{{
    {"VehicleSpeed", 0U, 16U, ByteOrder::LittleEndian, ValueType::Unsigned, 0.01, 0.0, 0.0, 300.0, "km/h"},
    {"LongitudinalAccel", 16U, 16U, ByteOrder::LittleEndian, ValueType::Signed, 0.001, 0.0, -20.0, 20.0, "m/s2"},
    {"VehicleSpeedValid", 32U, 1U, ByteOrder::LittleEndian, ValueType::Unsigned, 1.0, 0.0, 0.0, 1.0, ""},
    {"AliveCounter", 56U, 4U, ByteOrder::LittleEndian, ValueType::Unsigned, 1.0, 0.0, 0.0, 15.0, ""},
}};
inline constexpr MessageDef kVehicleDynamics{
    "VehicleDynamics", 0x100U, 8U, 20U, 100U, "VehicleSim",
    kVehicleDynamicsSignals.data(), kVehicleDynamicsSignals.size()};
inline constexpr const SignalDef& kVehicleSpeed = kVehicleDynamicsSignals[0U];
inline constexpr const SignalDef& kLongitudinalAccel = kVehicleDynamicsSignals[1U];
inline constexpr const SignalDef& kVehicleSpeedValid = kVehicleDynamicsSignals[2U];
inline constexpr const SignalDef& kVehicleDynamicsAlive = kVehicleDynamicsSignals[3U];

// ---------------------------------------------------------------------------
// 0x110 DriverInputs  (VehicleSim -> CruiseECU), 50 ms
// ---------------------------------------------------------------------------
inline constexpr std::array<SignalDef, 7U> kDriverInputsSignals{{
    {"AcceleratorPedal", 0U, 8U, ByteOrder::LittleEndian, ValueType::Unsigned, 0.5, 0.0, 0.0, 100.0, "%"},
    {"BrakePedalPressed", 8U, 1U, ByteOrder::LittleEndian, ValueType::Unsigned, 1.0, 0.0, 0.0, 1.0, ""},
    {"CruiseMainSwitch", 9U, 1U, ByteOrder::LittleEndian, ValueType::Unsigned, 1.0, 0.0, 0.0, 1.0, ""},
    {"CruiseSetRequest", 10U, 1U, ByteOrder::LittleEndian, ValueType::Unsigned, 1.0, 0.0, 0.0, 1.0, ""},
    {"CruiseCancelRequest", 11U, 1U, ByteOrder::LittleEndian, ValueType::Unsigned, 1.0, 0.0, 0.0, 1.0, ""},
    {"DriverSetSpeed", 16U, 8U, ByteOrder::LittleEndian, ValueType::Unsigned, 1.0, 0.0, 0.0, 200.0, "km/h"},
    {"AliveCounter", 56U, 4U, ByteOrder::LittleEndian, ValueType::Unsigned, 1.0, 0.0, 0.0, 15.0, ""},
}};
inline constexpr MessageDef kDriverInputs{
    "DriverInputs", 0x110U, 8U, 50U, 250U, "VehicleSim",
    kDriverInputsSignals.data(), kDriverInputsSignals.size()};
inline constexpr const SignalDef& kAcceleratorPedal = kDriverInputsSignals[0U];
inline constexpr const SignalDef& kBrakePedalPressed = kDriverInputsSignals[1U];
inline constexpr const SignalDef& kCruiseMainSwitch = kDriverInputsSignals[2U];
inline constexpr const SignalDef& kCruiseSetRequest = kDriverInputsSignals[3U];
inline constexpr const SignalDef& kCruiseCancelRequest = kDriverInputsSignals[4U];
inline constexpr const SignalDef& kDriverSetSpeed = kDriverInputsSignals[5U];
inline constexpr const SignalDef& kDriverInputsAlive = kDriverInputsSignals[6U];

// ---------------------------------------------------------------------------
// 0x200 CruiseControlStatus  (CruiseECU -> VehicleSim), 20 ms
// ---------------------------------------------------------------------------
inline constexpr std::array<SignalDef, 5U> kCruiseControlStatusSignals{{
    {"ThrottleCommand", 0U, 16U, ByteOrder::LittleEndian, ValueType::Unsigned, 0.01, 0.0, 0.0, 100.0, "%"},
    {"CruiseState", 16U, 3U, ByteOrder::LittleEndian, ValueType::Unsigned, 1.0, 0.0, 0.0, 4.0, ""},
    {"CruiseTargetSpeed", 24U, 8U, ByteOrder::LittleEndian, ValueType::Unsigned, 1.0, 0.0, 0.0, 200.0, "km/h"},
    {"CruiseFaultReason", 32U, 4U, ByteOrder::LittleEndian, ValueType::Unsigned, 1.0, 0.0, 0.0, 15.0, ""},
    {"AliveCounter", 56U, 4U, ByteOrder::LittleEndian, ValueType::Unsigned, 1.0, 0.0, 0.0, 15.0, ""},
}};
inline constexpr MessageDef kCruiseControlStatus{
    "CruiseControlStatus", 0x200U, 8U, 20U, 100U, "CruiseECU",
    kCruiseControlStatusSignals.data(), kCruiseControlStatusSignals.size()};
inline constexpr const SignalDef& kThrottleCommand = kCruiseControlStatusSignals[0U];
inline constexpr const SignalDef& kCruiseState = kCruiseControlStatusSignals[1U];
inline constexpr const SignalDef& kCruiseTargetSpeed = kCruiseControlStatusSignals[2U];
inline constexpr const SignalDef& kCruiseFaultReason = kCruiseControlStatusSignals[3U];
inline constexpr const SignalDef& kCruiseStatusAlive = kCruiseControlStatusSignals[4U];

/// Values of the CruiseState signal (DBC VAL_ table).
enum class CruiseState : std::uint8_t { Off = 0U, Standby = 1U, Active = 2U, Override = 3U, Fault = 4U };

[[nodiscard]] constexpr const char* toString(CruiseState state) noexcept {
  switch (state) {
    case CruiseState::Off:
      return "Off";
    case CruiseState::Standby:
      return "Standby";
    case CruiseState::Active:
      return "Active";
    case CruiseState::Override:
      return "Override";
    case CruiseState::Fault:
      return "Fault";
  }
  return "Unknown";
}

/// Every message of the matrix (used for lookups and the DBC consistency test).
inline constexpr std::array<const MessageDef*, 3U> kAllMessages{
    &kVehicleDynamics, &kDriverInputs, &kCruiseControlStatus};

[[nodiscard]] constexpr const MessageDef* findMessage(std::uint32_t id) noexcept {
  // Raw loop on purpose: std::find_if is only constexpr from C++20.
  for (const MessageDef* message : kAllMessages) {
    // cppcheck-suppress useStlAlgorithm
    if (message->id == id) {
      return message;
    }
  }
  return nullptr;
}

}  // namespace vecu::can::vehicle
