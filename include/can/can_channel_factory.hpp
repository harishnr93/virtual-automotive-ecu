#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

#include "can/can_status.hpp"
#include "can/i_can_channel.hpp"
#include "can/udp_can_channel.hpp"

namespace vecu::can {

enum class CanBackend : std::uint8_t {
  Auto,       ///< try SocketCAN first, fall back to the UDP software bus
  SocketCan,  ///< Linux SocketCAN only (fails if the interface does not exist)
  Udp         ///< UDP software bus only
};

struct CanChannelConfig {
  CanBackend backend{CanBackend::Auto};
  std::string socketcan_interface{"vcan0"};
  UdpBusConfig udp{};
};

[[nodiscard]] const char* toString(CanBackend backend) noexcept;

/// Parses "auto", "socketcan" or "udp".
[[nodiscard]] bool parseBackend(std::string_view text, CanBackend& backend) noexcept;

/// Creates and opens a channel according to `config`. Returns nullptr on failure and sets
/// `status` to the reason. The selected backend and any fallback are logged.
[[nodiscard]] std::unique_ptr<ICanChannel> createCanChannel(const CanChannelConfig& config,
                                                            CanStatus& status);

}  // namespace vecu::can
