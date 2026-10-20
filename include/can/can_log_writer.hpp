#pragma once

#include <cstdint>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>

#include "can/can_frame.hpp"
#include "can/can_status.hpp"

namespace vecu::can {

/// Formats one line in the can-utils "candump -l" log format:
///   (1697812345.123456) vcan0 100#1122334455667788
/// Such files can be replayed with canplayer or converted with log2asc.
[[nodiscard]] std::string formatCandumpLogLine(const CanFrame& frame, std::int64_t wall_time_us,
                                               std::string_view interface_label);

/// Thread-safe CAN trace writer (candump-compatible). Owns its FILE via RAII.
///
/// Note: stdio buffering keeps writes cheap, but file I/O is still not real-time safe.
/// A later phase can move logging to a background thread fed by a lock-free queue.
class CanLogWriter {
 public:
  CanLogWriter() = default;

  [[nodiscard]] CanStatus open(const std::string& path, std::string interface_label);
  void close() noexcept;
  [[nodiscard]] bool isOpen() const;
  void write(const CanFrame& frame, std::int64_t wall_time_us);

 private:
  struct FileCloser {
    void operator()(std::FILE* file) const noexcept {
      if (file != nullptr) {
        (void)std::fclose(file);  // NOLINT(cppcoreguidelines-owning-memory): this deleter is the owner
      }
    }
  };

  mutable std::mutex mutex_;
  std::unique_ptr<std::FILE, FileCloser> file_;
  std::string interface_label_;
};

}  // namespace vecu::can
