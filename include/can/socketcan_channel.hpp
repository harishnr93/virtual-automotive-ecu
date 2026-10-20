#pragma once

#include <atomic>
#include <string>

#include "can/i_can_channel.hpp"
#include "core/unique_fd.hpp"

namespace vecu::can {

/// CAN channel backed by Linux SocketCAN (CAN_RAW socket), e.g. on "vcan0" or a real "can0".
///
/// This is the same kernel API that production Linux ECUs/gateways use with real CAN
/// controllers. On a kernel without the vcan driver (stock WSL2), open() returns
/// NotAvailable and the factory can fall back to UdpCanChannel.
class SocketCanChannel final : public ICanChannel {
 public:
  explicit SocketCanChannel(std::string interface_name);
  ~SocketCanChannel() override = default;  // UniqueFd closes the socket

  SocketCanChannel(const SocketCanChannel&) = delete;
  SocketCanChannel& operator=(const SocketCanChannel&) = delete;
  SocketCanChannel(SocketCanChannel&&) = delete;
  SocketCanChannel& operator=(SocketCanChannel&&) = delete;

  [[nodiscard]] CanStatus open() override;
  void close() noexcept override;
  [[nodiscard]] bool isOpen() const noexcept override;
  [[nodiscard]] CanStatus send(const CanFrame& frame) override;
  [[nodiscard]] CanStatus receive(CanFrame& frame, std::chrono::microseconds timeout) override;
  [[nodiscard]] std::string_view name() const noexcept override;
  [[nodiscard]] CanChannelStats stats() const noexcept override;

  /// errno of the most recent failed operation (0 if none).
  [[nodiscard]] int lastErrno() const noexcept;

 private:
  std::string interface_name_;
  core::UniqueFd socket_;
  std::atomic<int> last_errno_{0};
  ChannelCounters counters_;
};

}  // namespace vecu::can
