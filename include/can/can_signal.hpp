#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

#include "can/can_frame.hpp"

namespace vecu::can {

/// Bit layout of a signal, using DBC conventions.
enum class ByteOrder : std::uint8_t {
  LittleEndian,  ///< "Intel", DBC "@1": start_bit is the LSB, bits grow upwards
  BigEndian      ///< "Motorola", DBC "@0": start_bit is the MSB (DBC sawtooth numbering)
};

enum class ValueType : std::uint8_t { Unsigned, Signed };  ///< DBC "+" / "-"

/// One signal inside a CAN message, equivalent to a DBC "SG_" line:
///   SG_ <name> : <start_bit>|<length>@<order><sign> (<factor>,<offset>) [<min>|<max>] "<unit>"
/// physical_value = raw_value * factor + offset
struct SignalDef {
  std::string_view name;
  std::uint16_t start_bit;
  std::uint8_t length;
  ByteOrder byte_order;
  ValueType value_type;
  double factor;
  double offset;
  double minimum;
  double maximum;
  std::string_view unit;
};

/// One CAN message, equivalent to a DBC "BO_" block plus the cycle-time/timeout attributes.
/// Signals are referenced (not owned) so definitions can live in constexpr tables in ROM.
struct MessageDef {
  std::string_view name;
  std::uint32_t id;
  std::uint8_t dlc;
  std::uint32_t cycle_time_ms;  ///< transmit period (DBC GenMsgCycleTime)
  std::uint32_t timeout_ms;     ///< receiver deadline monitoring (DBC VECU_RxTimeoutMs)
  std::string_view transmitter;
  const SignalDef* signals;
  std::size_t signal_count;
};

enum class SignalStatus : std::uint8_t {
  Ok,
  Clamped,            ///< encode: value outside [min, max] or raw range; clamped value was written
  OutOfRange,         ///< decode: decoded value outside [min, max] (value still returned)
  InvalidDefinition,  ///< signal does not fit the payload / unsupported length / zero factor
  InvalidValue        ///< encode: NaN or infinity; nothing written
};

[[nodiscard]] const char* toString(SignalStatus status) noexcept;

/// Physical encode/decode supports signals up to 32 bits (raw encode/decode up to 64).
inline constexpr std::uint8_t kMaxPhysicalSignalBits = 32U;

/// True if the signal's bits lie entirely within `payload_bytes` bytes.
[[nodiscard]] bool isValidDefinition(const SignalDef& signal, std::size_t payload_bytes) noexcept;

/// Writes the lowest `signal.length` bits of `raw` into the payload; other bits are untouched.
[[nodiscard]] SignalStatus encodeRaw(const SignalDef& signal, std::uint64_t raw,
                                     std::uint8_t* payload, std::size_t payload_bytes) noexcept;

/// Extracts the raw (unscaled, not sign-extended) bits of a signal.
[[nodiscard]] SignalStatus decodeRaw(const SignalDef& signal, const std::uint8_t* payload,
                                     std::size_t payload_bytes, std::uint64_t& raw) noexcept;

/// Scales, range-limits and packs a physical value into `frame` (uses frame.dlc bytes).
[[nodiscard]] SignalStatus encodePhysical(const SignalDef& signal, double value,
                                          CanFrame& frame) noexcept;

/// Unpacks, sign-extends and scales a signal from `frame`.
[[nodiscard]] SignalStatus decodePhysical(const SignalDef& signal, const CanFrame& frame,
                                          double& value) noexcept;

/// Creates an all-zero frame with the message's identifier and DLC.
[[nodiscard]] CanFrame makeFrame(const MessageDef& message) noexcept;

/// Looks up a signal by name; nullptr if not present.
[[nodiscard]] const SignalDef* findSignal(const MessageDef& message, std::string_view name) noexcept;

}  // namespace vecu::can
