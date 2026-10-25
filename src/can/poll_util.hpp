#pragma once
// Private helper shared by the socket-based CAN backends (not part of the public API).

#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <ctime>

namespace vecu::can::detail {

enum class WaitResult { Readable, Timeout, Error };

/// Waits until `fd` is readable or `timeout` expires (microsecond resolution via ppoll).
/// A signal interrupting the wait (EINTR) is reported as Timeout so the caller's loop
/// can check its shutdown flag.
inline WaitResult waitReadable(int fd, std::chrono::microseconds timeout) noexcept {
  if (timeout.count() < 0) {
    timeout = std::chrono::microseconds{0};
  }
  const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(timeout);
  timespec ts{};
  ts.tv_sec = static_cast<std::time_t>(seconds.count());
  ts.tv_nsec = static_cast<long>(std::chrono::duration_cast<std::chrono::nanoseconds>(timeout - seconds).count());

  pollfd descriptor{};
  descriptor.fd = fd;
  descriptor.events = POLLIN;
  const int rc = ::ppoll(&descriptor, 1U, &ts, nullptr);
  if (rc < 0) {
    return (errno == EINTR) ? WaitResult::Timeout : WaitResult::Error;
  }
  if (rc == 0) {
    return WaitResult::Timeout;
  }
  if ((static_cast<unsigned int>(descriptor.revents) & static_cast<unsigned int>(POLLIN)) != 0U) {
    return WaitResult::Readable;
  }
  return WaitResult::Error;  // POLLERR / POLLHUP / POLLNVAL without data
}

/// Enables kernel drop reporting (SO_RXQ_OVFL) on a receive socket.
inline bool enableDropCounter(int fd) noexcept {
  const int enable = 1;
  return ::setsockopt(fd, SOL_SOCKET, SO_RXQ_OVFL, &enable, sizeof(enable)) == 0;
}

/// recv() that also returns the kernel's cumulative count of datagrams/frames dropped on this
/// socket because its receive queue was full (from the SO_RXQ_OVFL control message).
inline ssize_t receiveWithDrops(int fd, void* buffer, std::size_t length, std::uint32_t& drops) noexcept {
  iovec io{};
  io.iov_base = buffer;
  io.iov_len = length;
  alignas(cmsghdr) std::array<std::uint8_t, CMSG_SPACE(sizeof(std::uint32_t))> control{};
  msghdr message{};
  message.msg_iov = &io;
  message.msg_iovlen = 1U;
  message.msg_control = control.data();
  message.msg_controllen = control.size();
  const ssize_t received = ::recvmsg(fd, &message, MSG_DONTWAIT);
  if (received >= 0) {
    for (cmsghdr* c = CMSG_FIRSTHDR(&message); c != nullptr; c = CMSG_NXTHDR(&message, c)) {
      if ((c->cmsg_level == SOL_SOCKET) && (c->cmsg_type == SO_RXQ_OVFL)) {
        std::memcpy(&drops, CMSG_DATA(c), sizeof(drops));
      }
    }
  }
  return received;
}

}  // namespace vecu::can::detail
