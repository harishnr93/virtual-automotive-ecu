#include "can/udp_can_channel.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <array>
#include <cerrno>
#include <random>
#include <utility>

#include "can/udp_wire_format.hpp"
#include "core/clock.hpp"
#include "poll_util.hpp"

namespace vecu::can {

namespace {

constexpr std::string_view kBusLabel{"vbus0"};

std::uint32_t makeSenderId() {
  std::random_device entropy;
  std::uint32_t id = 0U;
  while (id == 0U) {
    id = static_cast<std::uint32_t>(entropy());
  }
  return id;
}

bool isMulticast(std::uint32_t address_host_order) noexcept {
  return (address_host_order & 0xF0000000U) == 0xE0000000U;  // 224.0.0.0/4
}

template <typename T>
bool setOption(int fd, int level, int option, const T& value) noexcept {
  return ::setsockopt(fd, level, option, &value, sizeof(value)) == 0;
}

in_addr loopbackAddress() noexcept {
  in_addr address{};
  address.s_addr = htonl(INADDR_LOOPBACK);
  return address;
}

}  // namespace

UdpCanChannel::UdpCanChannel(UdpBusConfig config)
    : config_(std::move(config)), sender_id_(makeSenderId()) {}

CanStatus UdpCanChannel::open() {
  if (rx_socket_.valid()) {
    return CanStatus::Ok;
  }
  in_addr group{};
  if ((::inet_pton(AF_INET, config_.group_address.c_str(), &group) != 1) ||
      !isMulticast(ntohl(group.s_addr)) || (config_.port == 0U)) {
    return CanStatus::InvalidArgument;
  }

  // --- receive socket: bound to the group address, joined on the loopback interface ---
  core::UniqueFd rx{::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0)};
  if (!rx.valid()) {
    last_errno_ = errno;
    return CanStatus::IoError;
  }
  const int enable = 1;
  sockaddr_in bind_address{};
  bind_address.sin_family = AF_INET;
  bind_address.sin_port = htons(config_.port);
  bind_address.sin_addr = group;
  ip_mreq membership{};
  membership.imr_multiaddr = group;
  membership.imr_interface = loopbackAddress();
  const int receive_buffer = 1 << 20;  // ask for 1 MiB; the kernel caps it at 2 x rmem_max
  (void)setOption(rx.get(), SOL_SOCKET, SO_RCVBUF, receive_buffer);
  (void)detail::enableDropCounter(rx.get());  // kernel queue overflows become visible
  if (!setOption(rx.get(), SOL_SOCKET, SO_REUSEADDR, enable) ||  // several nodes share the port
      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast): BSD socket API
      (::bind(rx.get(), reinterpret_cast<const sockaddr*>(&bind_address), sizeof(bind_address)) != 0) ||
      !setOption(rx.get(), IPPROTO_IP, IP_ADD_MEMBERSHIP, membership)) {
    last_errno_ = errno;
    return CanStatus::IoError;
  }

  // --- transmit socket: multicast via loopback, looped back to local receivers, TTL 0 ---
  core::UniqueFd tx{::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0)};
  if (!tx.valid()) {
    last_errno_ = errno;
    return CanStatus::IoError;
  }
  const in_addr interface_address = loopbackAddress();
  const unsigned char loop = 1U;
  const unsigned char ttl = 0U;
  if (!setOption(tx.get(), IPPROTO_IP, IP_MULTICAST_IF, interface_address) ||
      !setOption(tx.get(), IPPROTO_IP, IP_MULTICAST_LOOP, loop) ||
      !setOption(tx.get(), IPPROTO_IP, IP_MULTICAST_TTL, ttl)) {
    last_errno_ = errno;
    return CanStatus::IoError;
  }

  group_be_ = group.s_addr;
  rx_socket_ = std::move(rx);
  tx_socket_ = std::move(tx);
  return CanStatus::Ok;
}

void UdpCanChannel::close() noexcept {
  rx_socket_.reset();
  tx_socket_.reset();
}

bool UdpCanChannel::isOpen() const noexcept { return rx_socket_.valid() && tx_socket_.valid(); }

CanStatus UdpCanChannel::send(const CanFrame& frame) {
  if (!isOpen()) {
    return CanStatus::NotOpen;
  }
  if (!isValid(frame)) {
    counters_.tx_errors.fetch_add(1U, std::memory_order_relaxed);
    return CanStatus::InvalidFrame;
  }
  udp_wire::Datagram datagram{};
  udp_wire::encode(frame, sender_id_, datagram);

  sockaddr_in destination{};
  destination.sin_family = AF_INET;
  destination.sin_port = htons(config_.port);
  destination.sin_addr.s_addr = group_be_;
  const ssize_t sent = ::sendto(tx_socket_.get(), datagram.data(), datagram.size(), MSG_DONTWAIT,
                                // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
                                reinterpret_cast<const sockaddr*>(&destination), sizeof(destination));
  if (sent == static_cast<ssize_t>(datagram.size())) {
    counters_.tx_frames.fetch_add(1U, std::memory_order_relaxed);
    return CanStatus::Ok;
  }
  const int error = errno;
  last_errno_ = error;
  counters_.tx_errors.fetch_add(1U, std::memory_order_relaxed);
  return ((error == ENOBUFS) || (error == EAGAIN)) ? CanStatus::TxBufferFull : CanStatus::IoError;
}

CanStatus UdpCanChannel::receive(CanFrame& frame, std::chrono::microseconds timeout) {
  if (!isOpen()) {
    return CanStatus::NotOpen;
  }
  const core::TimePoint deadline = core::now() + timeout;
  std::array<std::uint8_t, 64U> buffer{};  // larger than a datagram to detect oversize packets

  // Loop because a datagram may be our own echo or garbage; keep waiting for a valid frame
  // until the deadline.
  for (;;) {
    const auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(deadline - core::now());
    const detail::WaitResult wait = detail::waitReadable(rx_socket_.get(), remaining);
    if (wait == detail::WaitResult::Timeout) {
      return CanStatus::Timeout;
    }
    if (wait == detail::WaitResult::Error) {
      last_errno_ = errno;
      return CanStatus::IoError;
    }

    auto drops = static_cast<std::uint32_t>(counters_.rx_overruns.load(std::memory_order_relaxed));
    const ssize_t received = detail::receiveWithDrops(rx_socket_.get(), buffer.data(), buffer.size(), drops);
    counters_.rx_overruns.store(drops, std::memory_order_relaxed);  // cumulative kernel drop count
    if (received < 0) {
      const int error = errno;
      if ((error == EAGAIN) || (error == EINTR)) {
        continue;
      }
      last_errno_ = error;
      return CanStatus::IoError;
    }

    CanFrame decoded;
    std::uint32_t sender = 0U;
    if (udp_wire::decode(buffer.data(), static_cast<std::size_t>(received), decoded, sender) !=
        udp_wire::DecodeResult::Ok) {
      counters_.rx_rejected.fetch_add(1U, std::memory_order_relaxed);
      continue;
    }
    if (sender == sender_id_) {
      continue;  // own transmission looped back: SocketCAN does not deliver these either
    }
    frame = decoded;
    counters_.rx_frames.fetch_add(1U, std::memory_order_relaxed);
    return CanStatus::Ok;
  }
}

std::string_view UdpCanChannel::name() const noexcept { return kBusLabel; }

CanChannelStats UdpCanChannel::stats() const noexcept { return counters_.snapshot(); }

int UdpCanChannel::lastErrno() const noexcept { return last_errno_.load(); }

}  // namespace vecu::can
