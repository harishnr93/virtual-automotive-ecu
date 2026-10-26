#pragma once

// Simulated NOR flash memory.
//
// Behaves like a typical microcontroller/serial NOR flash:
//   - erasing a sector sets all its bytes to 0xFF,
//   - programming can only change bits from 1 to 0 (writing to non-erased bytes fails),
//   - a power loss can interrupt programming at any byte (RamFlash fault injection).
// FileFlash keeps the content in a file, so it survives process restarts ("ECU power cycles").

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "core/unique_fd.hpp"

namespace vecu::firmware {

enum class FlashStatus : std::uint8_t { Ok, OutOfRange, NotAligned, NotErased, IoError, PowerLoss };
[[nodiscard]] const char* toString(FlashStatus status) noexcept;

class IFlash {
 public:
  virtual ~IFlash() = default;
  [[nodiscard]] virtual std::uint32_t size() const noexcept = 0;
  [[nodiscard]] virtual std::uint32_t sectorSize() const noexcept = 0;
  [[nodiscard]] virtual FlashStatus read(std::uint32_t address, std::uint8_t* out, std::size_t length) const = 0;
  /// NOR programming: only 1 -> 0 bit transitions are possible.
  [[nodiscard]] virtual FlashStatus program(std::uint32_t address, const std::uint8_t* data, std::size_t length) = 0;
  /// `address` must be sector aligned.
  [[nodiscard]] virtual FlashStatus eraseSector(std::uint32_t address) = 0;
  /// Makes previous writes durable (commit point).
  [[nodiscard]] virtual FlashStatus sync() = 0;

  /// Erases all sectors of [address, address + length).
  [[nodiscard]] FlashStatus eraseRange(std::uint32_t address, std::uint32_t length);

 protected:
  IFlash() = default;
  IFlash(const IFlash&) = default;
  IFlash& operator=(const IFlash&) = default;
  IFlash(IFlash&&) = default;
  IFlash& operator=(IFlash&&) = default;
};

/// In-memory flash for unit tests, with deterministic power-loss injection.
class RamFlash final : public IFlash {
 public:
  RamFlash(std::uint32_t size, std::uint32_t sector_size);

  [[nodiscard]] std::uint32_t size() const noexcept override { return static_cast<std::uint32_t>(memory_.size()); }
  [[nodiscard]] std::uint32_t sectorSize() const noexcept override { return sector_size_; }
  [[nodiscard]] FlashStatus read(std::uint32_t address, std::uint8_t* out, std::size_t length) const override;
  [[nodiscard]] FlashStatus program(std::uint32_t address, const std::uint8_t* data, std::size_t length) override;
  [[nodiscard]] FlashStatus eraseSector(std::uint32_t address) override;
  [[nodiscard]] FlashStatus sync() override { return powered_ ? FlashStatus::Ok : FlashStatus::PowerLoss; }

  /// Simulated power loss after `bytes` more programmed bytes: the write stops in the middle,
  /// and every operation fails until powerCycle().
  void injectPowerLossAfter(std::size_t bytes) noexcept { power_loss_after_ = static_cast<std::ptrdiff_t>(bytes); }
  void powerCycle() noexcept;
  [[nodiscard]] std::uint32_t eraseCount(std::uint32_t sector_address) const;
  /// Test helper: flips one bit (simulated corruption).
  void flipBit(std::uint32_t address, unsigned int bit);

 private:
  std::vector<std::uint8_t> memory_;
  std::vector<std::uint32_t> erase_counts_;
  std::uint32_t sector_size_;
  std::ptrdiff_t power_loss_after_{-1};
  bool powered_{true};
};

/// File-backed flash: content persists across ECU (process) restarts.
class FileFlash final : public IFlash {
 public:
  /// Opens or creates (filled with 0xFF) a flash file of exactly `size` bytes.
  [[nodiscard]] static FlashStatus open(const std::string& path, std::uint32_t size, std::uint32_t sector_size,
                                        FileFlash& out);

  FileFlash() = default;
  [[nodiscard]] std::uint32_t size() const noexcept override { return size_; }
  [[nodiscard]] std::uint32_t sectorSize() const noexcept override { return sector_size_; }
  [[nodiscard]] FlashStatus read(std::uint32_t address, std::uint8_t* out, std::size_t length) const override;
  [[nodiscard]] FlashStatus program(std::uint32_t address, const std::uint8_t* data, std::size_t length) override;
  [[nodiscard]] FlashStatus eraseSector(std::uint32_t address) override;
  [[nodiscard]] FlashStatus sync() override;
  [[nodiscard]] bool isOpen() const noexcept { return fd_.valid(); }

 private:
  core::UniqueFd fd_;
  std::uint32_t size_{0U};
  std::uint32_t sector_size_{0U};
};

}  // namespace vecu::firmware
