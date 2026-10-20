#include "can/socketcan_channel.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>

#include <algorithm>
#include <cerrno>
#include <utility>

#include "poll_util.hpp"

namespace vecu::can {

SocketCanChannel::SocketCanChannel(std::string interface_name)
    : interface_name_(std::move(interface_name)) {}

CanStatus SocketCanChannel::open() {
  if (socket_.valid()) {
    return CanStatus::Ok;
  }
  if (interface_name_.empty() || (interface_name_.size() >= IFNAMSIZ)) {
    return CanStatus::InvalidArgument;
  }

  const unsigned int if_index = ::if_nametoindex(interface_name_.c_str());
  if (if_index == 0U) {
    last_errno_ = errno;  // typically ENODEV: interface does not exist
    return CanStatus::NotAvailable;
  }

  core::UniqueFd fd{::socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW)};
  if (!fd.valid()) {
    last_errno_ = errno;  // EAFNOSUPPORT / EPROTONOSUPPORT: no SocketCAN in kernel
    return CanStatus::NotAvailable;
  }

  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = static_cast<int>(if_index);
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast): required by the BSD socket API
  if (::bind(fd.get(), reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0) {
    last_errno_ = errno;  // ENODEV if the interface is not a CAN device
    return CanStatus::NotAvailable;
  }

  socket_ = std::move(fd);
  return CanStatus::Ok;
}

void SocketCanChannel::close() noexcept { socket_.reset(); }

bool SocketCanChannel::isOpen() const noexcept { return socket_.valid(); }

CanStatus SocketCanChannel::send(const CanFrame& frame) {
  if (!socket_.valid()) {
    return CanStatus::NotOpen;
  }
  if (!isValid(frame)) {
    counters_.tx_errors.fetch_add(1U, std::memory_order_relaxed);
    return CanStatus::InvalidFrame;
  }

  ::can_frame raw{};
  raw.can_id = frame.extended ? (frame.id | CAN_EFF_FLAG) : frame.id;
  raw.len = frame.dlc;
  std::copy_n(frame.data.begin(), frame.dlc, std::begin(raw.data));

  // MSG_DONTWAIT: an ECU task must never block on a full transmit queue.
  const ssize_t written = ::send(socket_.get(), &raw, sizeof(raw), MSG_DONTWAIT);
  if (written == static_cast<ssize_t>(sizeof(raw))) {
    counters_.tx_frames.fetch_add(1U, std::memory_order_relaxed);
    return CanStatus::Ok;
  }
  const int error = errno;
  last_errno_ = error;
  counters_.tx_errors.fetch_add(1U, std::memory_order_relaxed);
  return ((error == ENOBUFS) || (error == EAGAIN)) ? CanStatus::TxBufferFull : CanStatus::IoError;
}

CanStatus SocketCanChannel::receive(CanFrame& frame, std::chrono::microseconds timeout) {
  if (!socket_.valid()) {
    return CanStatus::NotOpen;
  }
  const detail::WaitResult wait = detail::waitReadable(socket_.get(), timeout);
  if (wait == detail::WaitResult::Timeout) {
    return CanStatus::Timeout;
  }
  if (wait == detail::WaitResult::Error) {
    last_errno_ = errno;
    return CanStatus::IoError;
  }

  ::can_frame raw{};
  const ssize_t received = ::recv(socket_.get(), &raw, sizeof(raw), MSG_DONTWAIT);
  if (received < 0) {
    const int error = errno;
    if ((error == EAGAIN) || (error == EINTR)) {
      return CanStatus::Timeout;
    }
    last_errno_ = error;
    return CanStatus::IoError;
  }
  if (received != static_cast<ssize_t>(sizeof(raw))) {
    counters_.rx_rejected.fetch_add(1U, std::memory_order_relaxed);
    return CanStatus::InvalidFrame;
  }
  // Error frames and remote frames are outside the scope of this platform.
  if (((raw.can_id & CAN_ERR_FLAG) != 0U) || ((raw.can_id & CAN_RTR_FLAG) != 0U) ||
      (raw.len > kMaxClassicDlc)) {
    counters_.rx_rejected.fetch_add(1U, std::memory_order_relaxed);
    return CanStatus::InvalidFrame;
  }

  frame.extended = (raw.can_id & CAN_EFF_FLAG) != 0U;
  frame.id = frame.extended ? (raw.can_id & CAN_EFF_MASK) : (raw.can_id & CAN_SFF_MASK);
  frame.dlc = raw.len;
  frame.data.fill(0U);
  std::copy_n(std::begin(raw.data), raw.len, frame.data.begin());
  counters_.rx_frames.fetch_add(1U, std::memory_order_relaxed);
  return CanStatus::Ok;
}

std::string_view SocketCanChannel::name() const noexcept { return interface_name_; }

CanChannelStats SocketCanChannel::stats() const noexcept { return counters_.snapshot(); }

int SocketCanChannel::lastErrno() const noexcept { return last_errno_.load(); }

}  // namespace vecu::can
