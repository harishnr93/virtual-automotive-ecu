#include "can/can_log_writer.hpp"

#include <array>
#include <utility>

namespace vecu::can {

std::string formatCandumpLogLine(const CanFrame& frame, std::int64_t wall_time_us,
                                 std::string_view interface_label) {
  std::array<char, 32U> timestamp{};
  const auto seconds = static_cast<long long>(wall_time_us / 1000000);
  const auto micros = static_cast<long long>(wall_time_us % 1000000);
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
  (void)std::snprintf(timestamp.data(), timestamp.size(), "(%010lld.%06lld) ", seconds, micros);

  std::string line{timestamp.data()};
  line.append(interface_label);
  line.push_back(' ');
  line.append(toCandumpString(frame));
  return line;
}

CanStatus CanLogWriter::open(const std::string& path, std::string interface_label) {
  const std::lock_guard<std::mutex> lock(mutex_);
  std::unique_ptr<std::FILE, FileCloser> file{std::fopen(path.c_str(), "w")};
  if (!file) {
    return CanStatus::IoError;
  }
  file_ = std::move(file);
  interface_label_ = std::move(interface_label);
  return CanStatus::Ok;
}

void CanLogWriter::close() noexcept {
  const std::lock_guard<std::mutex> lock(mutex_);
  file_.reset();
}

bool CanLogWriter::isOpen() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return static_cast<bool>(file_);
}

void CanLogWriter::write(const CanFrame& frame, std::int64_t wall_time_us) {
  const std::string line = formatCandumpLogLine(frame, wall_time_us, interface_label_);
  const std::lock_guard<std::mutex> lock(mutex_);
  if (file_) {
    (void)std::fputs(line.c_str(), file_.get());
    (void)std::fputc('\n', file_.get());
  }
}

}  // namespace vecu::can
