#pragma once

#include <cstddef>
#include <memory>
#include <string>

#include "can/i_can_channel.hpp"

namespace vecu::can {

namespace detail {
struct VirtualBusCore;
}  // namespace detail

/// In-process CAN bus for deterministic tests (no sockets, no kernel, no timing noise).
///
/// Channels created from one bus see each other's frames (but not their own), exactly
/// like SocketCAN. Each channel has a fixed-depth receive queue; when full, new frames are
/// dropped and counted as overruns (CAN controller behaviour).
///
/// Ownership: the bus state is shared between the bus object and its channels, so the
/// destruction order of bus and channels does not matter.
class VirtualCanBus {
 public:
  static constexpr std::size_t kRxQueueDepth = 64U;

  VirtualCanBus();
  ~VirtualCanBus();

  VirtualCanBus(const VirtualCanBus&) = delete;
  VirtualCanBus& operator=(const VirtualCanBus&) = delete;
  VirtualCanBus(VirtualCanBus&&) = delete;
  VirtualCanBus& operator=(VirtualCanBus&&) = delete;

  /// Creates a channel attached to this bus; call open() on it before use.
  [[nodiscard]] std::unique_ptr<ICanChannel> createChannel(std::string name);

  /// Number of currently open channels.
  [[nodiscard]] std::size_t openChannelCount() const;

 private:
  std::shared_ptr<detail::VirtualBusCore> core_;
};

}  // namespace vecu::can
