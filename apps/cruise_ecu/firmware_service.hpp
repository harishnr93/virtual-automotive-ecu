#pragma once

// Firmware update component of the cruise ECU: glue between the UDS server (programming
// session, 0x31/0x34/0x36/0x37) and the firmware library (flash, boot record, bootloader,
// update state machine).
//
// What "running an image" means in this simulation: the ECU executable is always the same
// binary. The image in flash is data whose header decides what the ECU reports and how it
// behaves — its version (DID 0xF189), its security version (anti-rollback) and a test-only
// defect flag that makes the Control task hang, standing in for faulty new firmware. The
// boot decisions (trial, confirmation, rollback, fallback) are real and persisted in flash.
//
// UDS interface (see docs/phase6_firmware_update.md):
//   DID 0x0400  boot status   [running slot, state A, state B, boot attempts, max attempts,
//                              last boot event, rejected image error]                 (7 bytes)
//   DID 0x0401  update status [update state, last image error, bytes received (u32)]  (6 bytes)
//   DID 0x0402  versions      [version A (3), version B (3), running security version (u32)]
//   RID 0xFF00  eraseMemory (inactive slot)                        programming session
//   RID 0xFF01  check image: status [0x00 ok | 0x01 failed, image error]  programming session
//   RID 0x0201  rollback to the previous confirmed image          extended + programming
//
// Thread safety: called from Dcm_10ms (UDS), Diag_100ms (confirmation), the watchdog thread
// (reset -> boot) and initialisation; one mutex serialises all flash access.

#include <atomic>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "core/logger.hpp"
#include "diagnostics/uds_server.hpp"
#include "firmware/boot_control.hpp"
#include "firmware/bootloader.hpp"
#include "firmware/firmware_image.hpp"
#include "firmware/flash_memory.hpp"
#include "firmware/update_manager.hpp"

namespace vecu::cruise {

/// Measured durations of flash operations (for the timing analysis of the Dcm task).
struct FirmwareTimings {
  double max_erase_ms{0.0};
  double max_block_ms{0.0};
  double max_check_ms{0.0};
  double last_boot_ms{0.0};
  double last_download_s{0.0};
  std::uint32_t last_download_bytes{0U};
};

class FirmwareService final : public diag::IDownloadHandler {
 public:
  static constexpr std::uint32_t kHardwareId = 0x0000CC01U;
  static constexpr firmware::Version kFactoryVersion{0U, 6U, 0U};
  static constexpr std::uint32_t kFactorySecurityVersion = 1U;
  static constexpr std::size_t kFactoryPayloadSize = std::size_t{64U} * 1024U;

  explicit FirmwareService(const core::Logger& log);

  /// Opens the program flash: a file (persistent, survives restarts) or RAM if `path` is empty.
  [[nodiscard]] bool open(const std::string& path);

  /// Runs the bootloader: at power-on and on every hard/watchdog reset.
  firmware::BootResult boot();

  /// Confirms a running trial image (called after a healthy period). True if confirmed now.
  bool confirmTrial();

  /// Registers DIDs and routines and installs itself as download handler.
  [[nodiscard]] bool registerUds(diag::UdsServer& uds);

  [[nodiscard]] firmware::Version runningVersion() const;
  [[nodiscard]] bool applicationValid() const;
  [[nodiscard]] bool trialActive() const;
  [[nodiscard]] bool defectFlag() const;
  [[nodiscard]] FirmwareTimings timings() const;

  // diag::IDownloadHandler (Dcm_10ms)
  std::optional<diag::Nrc> requestDownload(std::uint32_t address, std::uint32_t size) override;
  std::optional<diag::Nrc> transferData(std::uint8_t counter, const std::uint8_t* data, std::size_t length) override;
  std::optional<diag::Nrc> requestTransferExit() override;
  void abortDownload() override;

 private:
  diag::RoutineResult eraseRoutine(std::uint8_t* status);
  diag::RoutineResult checkRoutine(std::uint8_t* status);
  diag::RoutineResult rollbackRoutine(std::uint8_t* status);
  void readBootStatus(std::uint8_t* out) const;
  void readUpdateStatus(std::uint8_t* out) const;
  void readVersions(std::uint8_t* out) const;

  const core::Logger& log_;
  std::vector<std::uint8_t> factory_image_;
  mutable std::mutex mutex_;
  std::optional<firmware::RamFlash> ram_flash_;
  firmware::FileFlash file_flash_;
  firmware::IFlash* flash_{nullptr};
  firmware::Crc32Verifier verifier_;
  std::optional<firmware::BootControl> control_;
  std::optional<firmware::Bootloader> bootloader_;
  std::optional<firmware::UpdateManager> updater_;
  firmware::BootResult running_{};
  bool trial_{false};
  FirmwareTimings timings_{};
  core::TimePoint download_start_{};
};

}  // namespace vecu::cruise
