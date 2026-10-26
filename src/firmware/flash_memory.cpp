#include "firmware/flash_memory.hpp"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <utility>

namespace vecu::firmware {

namespace {

bool inRange(std::uint32_t address, std::size_t length, std::uint32_t size) noexcept {
  return (length <= size) && (address <= size - length);
}

/// NOR rule: programming may only clear bits.
bool programmable(std::uint8_t old_value, std::uint8_t new_value) noexcept {
  return (old_value & new_value) == new_value;
}

}  // namespace

const char* toString(FlashStatus status) noexcept {
  switch (status) {
    case FlashStatus::Ok:
      return "Ok";
    case FlashStatus::OutOfRange:
      return "OutOfRange";
    case FlashStatus::NotAligned:
      return "NotAligned";
    case FlashStatus::NotErased:
      return "NotErased";
    case FlashStatus::IoError:
      return "IoError";
    case FlashStatus::PowerLoss:
      return "PowerLoss";
  }
  return "Unknown";
}

FlashStatus IFlash::eraseRange(std::uint32_t address, std::uint32_t length) {
  const std::uint32_t sector = sectorSize();
  if ((address % sector) != 0U) {
    return FlashStatus::NotAligned;
  }
  if (!inRange(address, length, size())) {
    return FlashStatus::OutOfRange;
  }
  for (std::uint32_t a = address; a < address + length; a += sector) {
    const FlashStatus status = eraseSector(a);
    if (status != FlashStatus::Ok) {
      return status;
    }
  }
  return FlashStatus::Ok;
}

// ------------------------------------------------------------------------------ RamFlash ----

RamFlash::RamFlash(std::uint32_t size, std::uint32_t sector_size)
    : memory_(size, 0xFFU), erase_counts_((sector_size == 0U) ? 0U : size / sector_size, 0U), sector_size_(sector_size) {}

FlashStatus RamFlash::read(std::uint32_t address, std::uint8_t* out, std::size_t length) const {
  if (!powered_) {
    return FlashStatus::PowerLoss;
  }
  if ((out == nullptr) || !inRange(address, length, size())) {
    return FlashStatus::OutOfRange;
  }
  std::copy_n(memory_.begin() + address, length, out);
  return FlashStatus::Ok;
}

FlashStatus RamFlash::program(std::uint32_t address, const std::uint8_t* data, std::size_t length) {
  if (!powered_) {
    return FlashStatus::PowerLoss;
  }
  if ((data == nullptr) || !inRange(address, length, size())) {
    return FlashStatus::OutOfRange;
  }
  for (std::size_t i = 0U; i < length; ++i) {
    if (!programmable(memory_[address + i], data[i])) {
      return FlashStatus::NotErased;
    }
  }
  for (std::size_t i = 0U; i < length; ++i) {
    if (power_loss_after_ == 0) {
      powered_ = false;  // the write stops here, the rest of the data never arrives
      power_loss_after_ = -1;
      return FlashStatus::PowerLoss;
    }
    if (power_loss_after_ > 0) {
      --power_loss_after_;
    }
    memory_[address + i] &= data[i];
  }
  return FlashStatus::Ok;
}

FlashStatus RamFlash::eraseSector(std::uint32_t address) {
  if (!powered_) {
    return FlashStatus::PowerLoss;
  }
  if ((address % sector_size_) != 0U) {
    return FlashStatus::NotAligned;
  }
  if (!inRange(address, sector_size_, size())) {
    return FlashStatus::OutOfRange;
  }
  std::fill_n(memory_.begin() + address, sector_size_, static_cast<std::uint8_t>(0xFFU));
  ++erase_counts_[address / sector_size_];
  return FlashStatus::Ok;
}

void RamFlash::powerCycle() noexcept {
  powered_ = true;
  power_loss_after_ = -1;
}

std::uint32_t RamFlash::eraseCount(std::uint32_t sector_address) const {
  return erase_counts_.at(sector_address / sector_size_);
}

void RamFlash::flipBit(std::uint32_t address, unsigned int bit) {
  memory_.at(address) = static_cast<std::uint8_t>(memory_.at(address) ^ (1U << (bit & 7U)));
}

// ----------------------------------------------------------------------------- FileFlash ----

FlashStatus FileFlash::open(const std::string& path, std::uint32_t size, std::uint32_t sector_size, FileFlash& out) {
  if ((sector_size == 0U) || ((size % sector_size) != 0U)) {
    return FlashStatus::NotAligned;
  }
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg): POSIX open() is variadic by definition
  core::UniqueFd fd{::open(path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0644)};
  if (!fd.valid()) {
    return FlashStatus::IoError;
  }
  struct stat info {};
  if (::fstat(fd.get(), &info) != 0) {
    return FlashStatus::IoError;
  }
  if (static_cast<std::uint64_t>(info.st_size) != size) {
    // New (or wrong-sized) device: a factory-fresh flash is fully erased.
    const std::vector<std::uint8_t> erased(size, 0xFFU);
    if ((::ftruncate(fd.get(), 0) != 0) || (::pwrite(fd.get(), erased.data(), erased.size(), 0) !=
                                            static_cast<ssize_t>(erased.size())) ||
        (::fsync(fd.get()) != 0)) {
      return FlashStatus::IoError;
    }
  }
  out.fd_ = std::move(fd);
  out.size_ = size;
  out.sector_size_ = sector_size;
  return FlashStatus::Ok;
}

FlashStatus FileFlash::read(std::uint32_t address, std::uint8_t* out, std::size_t length) const {
  if ((out == nullptr) || !inRange(address, length, size_)) {
    return FlashStatus::OutOfRange;
  }
  return (::pread(fd_.get(), out, length, address) == static_cast<ssize_t>(length)) ? FlashStatus::Ok
                                                                                     : FlashStatus::IoError;
}

FlashStatus FileFlash::program(std::uint32_t address, const std::uint8_t* data, std::size_t length) {
  if ((data == nullptr) || !inRange(address, length, size_)) {
    return FlashStatus::OutOfRange;
  }
  std::vector<std::uint8_t> current(length);
  if (::pread(fd_.get(), current.data(), length, address) != static_cast<ssize_t>(length)) {
    return FlashStatus::IoError;
  }
  for (std::size_t i = 0U; i < length; ++i) {
    if (!programmable(current[i], data[i])) {
      return FlashStatus::NotErased;
    }
    current[i] &= data[i];
  }
  return (::pwrite(fd_.get(), current.data(), length, address) == static_cast<ssize_t>(length)) ? FlashStatus::Ok
                                                                                                : FlashStatus::IoError;
}

FlashStatus FileFlash::eraseSector(std::uint32_t address) {
  if ((address % sector_size_) != 0U) {
    return FlashStatus::NotAligned;
  }
  if (!inRange(address, sector_size_, size_)) {
    return FlashStatus::OutOfRange;
  }
  const std::vector<std::uint8_t> erased(sector_size_, 0xFFU);
  return (::pwrite(fd_.get(), erased.data(), erased.size(), address) == static_cast<ssize_t>(erased.size()))
             ? FlashStatus::Ok
             : FlashStatus::IoError;
}

FlashStatus FileFlash::sync() { return (::fsync(fd_.get()) == 0) ? FlashStatus::Ok : FlashStatus::IoError; }

}  // namespace vecu::firmware
