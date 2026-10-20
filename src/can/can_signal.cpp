#include "can/can_signal.hpp"

#include <cmath>
#include <cstdint>

namespace vecu::can {

namespace {

constexpr std::uint32_t kBitsPerByte = 8U;

/// Next frame bit when walking a Motorola signal from MSB towards LSB.
/// DBC numbers bits 7..0 inside each byte; after bit 0 of byte n comes bit 7 of byte n+1.
constexpr std::uint32_t nextMotorolaBit(std::uint32_t position) noexcept {
  return ((position % kBitsPerByte) == 0U) ? (position + 15U) : (position - 1U);
}

bool getBit(const std::uint8_t* payload, std::uint32_t position) noexcept {
  return ((static_cast<std::uint32_t>(payload[position / kBitsPerByte]) >> (position % kBitsPerByte)) & 1U) != 0U;
}

void setBit(std::uint8_t* payload, std::uint32_t position, bool value) noexcept {
  const std::uint32_t index = position / kBitsPerByte;
  const auto mask = static_cast<std::uint8_t>(1U << (position % kBitsPerByte));
  if (value) {
    payload[index] = static_cast<std::uint8_t>(payload[index] | mask);
  } else {
    payload[index] = static_cast<std::uint8_t>(payload[index] & static_cast<std::uint8_t>(~mask));
  }
}

constexpr std::uint64_t bitMask(std::uint8_t length) noexcept {
  return (length >= 64U) ? ~0ULL : ((1ULL << length) - 1ULL);
}

struct RawLimits {
  std::int64_t min;
  std::int64_t max;
};

constexpr RawLimits rawLimits(const SignalDef& signal) noexcept {
  if (signal.value_type == ValueType::Signed) {
    const auto half = static_cast<std::int64_t>(1ULL << (signal.length - 1U));
    return {-half, half - 1};
  }
  return {0, static_cast<std::int64_t>(bitMask(signal.length))};
}

bool isPhysicalCapable(const SignalDef& signal, std::size_t payload_bytes) noexcept {
  return (signal.length <= kMaxPhysicalSignalBits) && (signal.factor != 0.0) &&
         isValidDefinition(signal, payload_bytes);
}

}  // namespace

const char* toString(SignalStatus status) noexcept {
  switch (status) {
    case SignalStatus::Ok:
      return "Ok";
    case SignalStatus::Clamped:
      return "Clamped";
    case SignalStatus::OutOfRange:
      return "OutOfRange";
    case SignalStatus::InvalidDefinition:
      return "InvalidDefinition";
    case SignalStatus::InvalidValue:
      return "InvalidValue";
  }
  return "Unknown";
}

bool isValidDefinition(const SignalDef& signal, std::size_t payload_bytes) noexcept {
  if ((signal.length == 0U) || (signal.length > 64U) || (payload_bytes > kMaxClassicDlc)) {
    return false;
  }
  const std::uint32_t total_bits = static_cast<std::uint32_t>(payload_bytes) * kBitsPerByte;
  if (signal.byte_order == ByteOrder::LittleEndian) {
    return (static_cast<std::uint32_t>(signal.start_bit) + signal.length) <= total_bits;
  }
  std::uint32_t position = signal.start_bit;
  for (std::uint32_t i = 0U; i < signal.length; ++i) {
    if (position >= total_bits) {
      return false;
    }
    position = nextMotorolaBit(position);
  }
  return true;
}

SignalStatus encodeRaw(const SignalDef& signal, std::uint64_t raw, std::uint8_t* payload,
                       std::size_t payload_bytes) noexcept {
  if ((payload == nullptr) || !isValidDefinition(signal, payload_bytes)) {
    return SignalStatus::InvalidDefinition;
  }
  if (signal.byte_order == ByteOrder::LittleEndian) {
    // Bit i of the raw value goes to frame bit (start_bit + i).
    for (std::uint32_t i = 0U; i < signal.length; ++i) {
      setBit(payload, signal.start_bit + i, ((raw >> i) & 1ULL) != 0ULL);
    }
  } else {
    // Walk from the MSB (at start_bit) down to the LSB.
    std::uint32_t position = signal.start_bit;
    for (std::uint32_t i = 0U; i < signal.length; ++i) {
      const std::uint32_t raw_bit = signal.length - 1U - i;
      setBit(payload, position, ((raw >> raw_bit) & 1ULL) != 0ULL);
      position = nextMotorolaBit(position);
    }
  }
  return SignalStatus::Ok;
}

SignalStatus decodeRaw(const SignalDef& signal, const std::uint8_t* payload,
                       std::size_t payload_bytes, std::uint64_t& raw) noexcept {
  if ((payload == nullptr) || !isValidDefinition(signal, payload_bytes)) {
    return SignalStatus::InvalidDefinition;
  }
  std::uint64_t value = 0ULL;
  if (signal.byte_order == ByteOrder::LittleEndian) {
    for (std::uint32_t i = 0U; i < signal.length; ++i) {
      if (getBit(payload, signal.start_bit + i)) {
        value |= (1ULL << i);
      }
    }
  } else {
    std::uint32_t position = signal.start_bit;
    for (std::uint32_t i = 0U; i < signal.length; ++i) {
      value = (value << 1U) | (getBit(payload, position) ? 1ULL : 0ULL);
      position = nextMotorolaBit(position);
    }
  }
  raw = value;
  return SignalStatus::Ok;
}

SignalStatus encodePhysical(const SignalDef& signal, double value, CanFrame& frame) noexcept {
  if (!isPhysicalCapable(signal, frame.dlc)) {
    return SignalStatus::InvalidDefinition;
  }
  if (!std::isfinite(value)) {
    return SignalStatus::InvalidValue;
  }

  SignalStatus status = SignalStatus::Ok;
  double limited = value;
  if (limited < signal.minimum) {
    limited = signal.minimum;
    status = SignalStatus::Clamped;
  } else if (limited > signal.maximum) {
    limited = signal.maximum;
    status = SignalStatus::Clamped;
  } else {
    // value already within the physical range
  }

  const double scaled = std::round((limited - signal.offset) / signal.factor);
  const RawLimits limits = rawLimits(signal);
  std::int64_t raw_signed = 0;
  if (scaled < static_cast<double>(limits.min)) {
    raw_signed = limits.min;
    status = SignalStatus::Clamped;
  } else if (scaled > static_cast<double>(limits.max)) {
    raw_signed = limits.max;
    status = SignalStatus::Clamped;
  } else {
    raw_signed = static_cast<std::int64_t>(scaled);
  }

  // Two's complement representation truncated to `length` bits.
  const std::uint64_t raw = static_cast<std::uint64_t>(raw_signed) & bitMask(signal.length);
  (void)encodeRaw(signal, raw, frame.data.data(), frame.dlc);
  return status;
}

SignalStatus decodePhysical(const SignalDef& signal, const CanFrame& frame, double& value) noexcept {
  if (!isPhysicalCapable(signal, frame.dlc)) {
    return SignalStatus::InvalidDefinition;
  }
  std::uint64_t raw = 0ULL;
  (void)decodeRaw(signal, frame.data.data(), frame.dlc, raw);

  double raw_value = 0.0;
  if (signal.value_type == ValueType::Signed) {
    const std::uint64_t sign_bit = 1ULL << (signal.length - 1U);
    const std::int64_t extended = ((raw & sign_bit) != 0ULL)
                                      ? static_cast<std::int64_t>(raw) -
                                            static_cast<std::int64_t>(1ULL << signal.length)
                                      : static_cast<std::int64_t>(raw);
    raw_value = static_cast<double>(extended);
  } else {
    raw_value = static_cast<double>(raw);
  }
  value = (raw_value * signal.factor) + signal.offset;

  // Half a resolution step of tolerance absorbs floating-point rounding at the limits.
  const double tolerance = std::fabs(signal.factor) * 0.5;
  if ((value < (signal.minimum - tolerance)) || (value > (signal.maximum + tolerance))) {
    return SignalStatus::OutOfRange;
  }
  return SignalStatus::Ok;
}

CanFrame makeFrame(const MessageDef& message) noexcept {
  CanFrame frame;
  frame.id = message.id;
  frame.extended = message.id > kMaxStandardId;
  frame.dlc = message.dlc;
  return frame;
}

const SignalDef* findSignal(const MessageDef& message, std::string_view name) noexcept {
  for (std::size_t i = 0U; i < message.signal_count; ++i) {
    if (message.signals[i].name == name) {
      return &message.signals[i];
    }
  }
  return nullptr;
}

}  // namespace vecu::can
