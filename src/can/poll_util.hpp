#pragma once
// Private helper shared by the socket-based CAN backends (not part of the public API).

#include <poll.h>

#include <cerrno>
#include <chrono>
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

}  // namespace vecu::can::detail
