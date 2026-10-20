#pragma once

#include <cstdint>

namespace vecu::can {

/// Result of every CAN channel operation. The ECU runtime does not use exceptions:
/// every failure is an explicit return value that the caller must handle.
enum class CanStatus : std::uint8_t {
  Ok,
  Timeout,          ///< no frame within the requested time (not an error)
  NotOpen,          ///< channel not opened, or closed
  NotAvailable,     ///< interface / socket family does not exist on this system
  InvalidArgument,  ///< bad configuration (interface name, address, port)
  InvalidFrame,     ///< frame violates Classic CAN limits, or unsupported frame type received
  TxBufferFull,     ///< transmit queue full; frame was not sent
  IoError           ///< unexpected operating-system error
};

[[nodiscard]] constexpr const char* toString(CanStatus status) noexcept {
  switch (status) {
    case CanStatus::Ok:
      return "Ok";
    case CanStatus::Timeout:
      return "Timeout";
    case CanStatus::NotOpen:
      return "NotOpen";
    case CanStatus::NotAvailable:
      return "NotAvailable";
    case CanStatus::InvalidArgument:
      return "InvalidArgument";
    case CanStatus::InvalidFrame:
      return "InvalidFrame";
    case CanStatus::TxBufferFull:
      return "TxBufferFull";
    case CanStatus::IoError:
      return "IoError";
  }
  return "Unknown";
}

}  // namespace vecu::can
