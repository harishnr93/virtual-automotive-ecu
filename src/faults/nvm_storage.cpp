#include "faults/nvm_storage.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <vector>

#include "core/crc32.hpp"
#include "core/unique_fd.hpp"

namespace vecu::faults {

namespace {

constexpr std::array<std::uint8_t, 4U> kMagic{'V', 'N', 'V', 'M'};
constexpr std::uint16_t kVersion = 1U;
constexpr std::size_t kHeaderSize = 20U;
constexpr std::size_t kRecordSize = 8U;
constexpr std::size_t kCrcSize = 4U;

void putLe(std::vector<std::uint8_t>& out, std::uint32_t value, std::size_t bytes) {
  for (std::size_t i = 0U; i < bytes; ++i) {
    out.push_back(static_cast<std::uint8_t>((value >> (8U * i)) & 0xFFU));
  }
}

std::uint32_t getLe(const std::vector<std::uint8_t>& in, std::size_t offset, std::size_t bytes) {
  std::uint32_t value = 0U;
  for (std::size_t i = 0U; i < bytes; ++i) {
    value |= static_cast<std::uint32_t>(in[offset + i]) << (8U * i);
  }
  return value;
}

bool writeAll(int fd, const std::vector<std::uint8_t>& bytes) {
  std::size_t written = 0U;
  while (written < bytes.size()) {
    const ssize_t rc = ::write(fd, bytes.data() + written, bytes.size() - written);
    if (rc < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    written += static_cast<std::size_t>(rc);
  }
  return true;
}

}  // namespace

const char* toString(ResetReason reason) noexcept {
  switch (reason) {
    case ResetReason::PowerOn:
      return "PowerOn";
    case ResetReason::UdsHardReset:
      return "UdsHardReset";
    case ResetReason::UdsSoftReset:
      return "UdsSoftReset";
    case ResetReason::Watchdog:
      return "Watchdog";
  }
  return "Unknown";
}

const char* toString(NvmStatus status) noexcept {
  switch (status) {
    case NvmStatus::Ok:
      return "Ok";
    case NvmStatus::NotFound:
      return "NotFound";
    case NvmStatus::Corrupted:
      return "Corrupted";
    case NvmStatus::IoError:
      return "IoError";
  }
  return "Unknown";
}

NvmStatus saveNvm(const std::string& path, const NvmData& data) {
  const std::size_t count = std::min(data.record_count, data.records.size());
  std::vector<std::uint8_t> bytes;
  bytes.reserve(kHeaderSize + (count * kRecordSize) + kCrcSize);
  bytes.insert(bytes.end(), kMagic.begin(), kMagic.end());
  putLe(bytes, kVersion, 2U);
  putLe(bytes, static_cast<std::uint32_t>(count), 2U);
  putLe(bytes, data.operation_cycles, 4U);
  putLe(bytes, data.reset_count, 4U);
  putLe(bytes, static_cast<std::uint32_t>(data.last_reset_reason), 1U);
  putLe(bytes, 0U, 3U);
  for (std::size_t i = 0U; i < count; ++i) {
    const DtcRecord& r = data.records[i];
    putLe(bytes, r.code, 4U);
    putLe(bytes, r.status, 1U);
    putLe(bytes, r.occurrence_count, 2U);
    putLe(bytes, r.aging_counter, 1U);
  }
  putLe(bytes, core::crc32(bytes.data(), bytes.size()), 4U);

  // Atomic replacement: temp file -> fsync -> rename.
  const std::string temp = path + ".tmp";
  {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg): POSIX open() is variadic by definition
    const core::UniqueFd fd{::open(temp.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644)};
    if (!fd.valid() || !writeAll(fd.get(), bytes) || (::fsync(fd.get()) != 0)) {
      (void)std::remove(temp.c_str());
      return NvmStatus::IoError;
    }
  }
  if (std::rename(temp.c_str(), path.c_str()) != 0) {
    (void)std::remove(temp.c_str());
    return NvmStatus::IoError;
  }
  return NvmStatus::Ok;
}

NvmStatus loadNvm(const std::string& path, NvmData& data) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    return NvmStatus::NotFound;
  }
  const std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  if (bytes.size() < (kHeaderSize + kCrcSize) || !std::equal(kMagic.begin(), kMagic.end(), bytes.begin()) ||
      (getLe(bytes, 4U, 2U) != kVersion)) {
    return NvmStatus::Corrupted;
  }
  const std::size_t count = getLe(bytes, 6U, 2U);
  if ((count > data.records.size()) || (bytes.size() != kHeaderSize + (count * kRecordSize) + kCrcSize)) {
    return NvmStatus::Corrupted;
  }
  const std::size_t crc_offset = bytes.size() - kCrcSize;
  if (core::crc32(bytes.data(), crc_offset) != getLe(bytes, crc_offset, 4U)) {
    return NvmStatus::Corrupted;
  }
  const std::uint32_t reason = getLe(bytes, 16U, 1U);
  if (reason > static_cast<std::uint32_t>(ResetReason::Watchdog)) {
    return NvmStatus::Corrupted;
  }

  NvmData loaded;
  loaded.operation_cycles = getLe(bytes, 8U, 4U);
  loaded.reset_count = getLe(bytes, 12U, 4U);
  loaded.last_reset_reason = static_cast<ResetReason>(reason);
  loaded.record_count = count;
  for (std::size_t i = 0U; i < count; ++i) {
    const std::size_t offset = kHeaderSize + (i * kRecordSize);
    loaded.records[i] = DtcRecord{getLe(bytes, offset, 4U), static_cast<std::uint8_t>(getLe(bytes, offset + 4U, 1U)),
                                  static_cast<std::uint16_t>(getLe(bytes, offset + 5U, 2U)),
                                  static_cast<std::uint8_t>(getLe(bytes, offset + 7U, 1U))};
  }
  data = loaded;
  return NvmStatus::Ok;
}

}  // namespace vecu::faults
