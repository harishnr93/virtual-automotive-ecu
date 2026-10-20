#pragma once

#include <atomic>
#include <cstdint>
#include <string>

#include "can/i_can_channel.hpp"
#include "core/unique_fd.hpp"

namespace vecu::can {

/// Configuration of the UDP software CAN bus.
struct UdpBusConfig {
  std::string group_address{"239.255.0.1"};  ///< IPv4 multicast group = "the bus"
  std::uint16_t port{47000U};                 ///< different ports = independent buses
};

/// Software-only CAN bus for systems without the vcan kernel driver (e.g. stock WSL2).
///
/// Every channel joins the same IPv4 multicast group on the loopback interface, so a frame
/// sent by one process is delivered to all other processes on the bus — the broadcast
/// behaviour of a real CAN bus. Datagrams use TTL 0 and never leave the host.
///
/// What this does NOT model (documented limitation): bit timing, arbitration by identifier
/// priority, ACK/error frames, bus-off. Delivery order between different senders is not
/// guaranteed to follow CAN priority.
class UdpCanChannel final : public ICanChannel {
 public:
  explicit UdpCanChannel(UdpBusConfig config);
  ~UdpCanChannel() override = default;

  UdpCanChannel(const UdpCanChannel&) = delete;
  UdpCanChannel& operator=(const UdpCanChannel&) = delete;
  UdpCanChannel(UdpCanChannel&&) = delete;
  UdpCanChannel& operator=(UdpCanChannel&&) = delete;

  [[nodiscard]] CanStatus open() override;
  void close() noexcept override;
  [[nodiscard]] bool isOpen() const noexcept override;
  [[nodiscard]] CanStatus send(const CanFrame& frame) override;
  [[nodiscard]] CanStatus receive(CanFrame& frame, std::chrono::microseconds timeout) override;
  [[nodiscard]] std::string_view name() const noexcept override;
  [[nodiscard]] CanChannelStats stats() const noexcept override;

  [[nodiscard]] const UdpBusConfig& config() const noexcept { return config_; }
  [[nodiscard]] std::uint32_t senderId() const noexcept { return sender_id_; }
  [[nodiscard]] int lastErrno() const noexcept;

 private:
  UdpBusConfig config_;
  std::uint32_t sender_id_;
  core::UniqueFd rx_socket_;
  core::UniqueFd tx_socket_;
  std::uint32_t group_be_{0U};  ///< multicast group, network byte order
  std::atomic<int> last_errno_{0};
  ChannelCounters counters_;
};

}  // namespace vecu::can
