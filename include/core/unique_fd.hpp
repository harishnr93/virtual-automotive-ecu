#pragma once

#include <unistd.h>

namespace vecu::core {

/// RAII owner of a POSIX file descriptor (socket, file, ...).
///
/// Exactly one UniqueFd owns a descriptor; it is closed automatically when the owner is
/// destroyed or reset. Copying is forbidden (two owners would double-close), moving
/// transfers ownership. This removes a whole class of resource-leak bugs.
class UniqueFd {
 public:
  UniqueFd() noexcept = default;
  explicit UniqueFd(int fd) noexcept : fd_(fd) {}
  ~UniqueFd() { reset(); }

  UniqueFd(const UniqueFd&) = delete;
  UniqueFd& operator=(const UniqueFd&) = delete;

  UniqueFd(UniqueFd&& other) noexcept : fd_(other.release()) {}
  UniqueFd& operator=(UniqueFd&& other) noexcept {
    if (this != &other) {
      reset(other.release());
    }
    return *this;
  }

  [[nodiscard]] int get() const noexcept { return fd_; }
  [[nodiscard]] bool valid() const noexcept { return fd_ >= 0; }

  /// Gives up ownership without closing.
  [[nodiscard]] int release() noexcept {
    const int fd = fd_;
    fd_ = -1;
    return fd;
  }

  /// Closes the current descriptor (if any) and takes ownership of `fd`.
  void reset(int fd = -1) noexcept {
    if (fd_ >= 0) {
      (void)::close(fd_);
    }
    fd_ = fd;
  }

 private:
  int fd_{-1};
};

}  // namespace vecu::core
