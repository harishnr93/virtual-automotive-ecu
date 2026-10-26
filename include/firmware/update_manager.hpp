#pragma once

// Firmware update state machine (download side), driven by UDS:
//
//   Idle/Failed/Verified --eraseInactiveSlot()--> Erased --requestDownload()--> Downloading
//   Downloading --transferData()*--> Downloading --transferExit()--> Downloaded
//   Downloaded --checkImage()--> Verified (slot Pending + activated for the next reset)
//                             \-> Failed   (slot Invalid; the running image is untouched)
//   any state --abort()--> Failed (e.g. tester left the programming session)
//
// The download always goes to the INACTIVE slot. The running image and the boot record entry
// of the active slot are never modified, so an interrupted or corrupted update cannot make the
// ECU unbootable. The target slot is marked Receiving (persisted) BEFORE it is erased, so a
// power loss at any point leaves it non-bootable.

#include <cstddef>
#include <cstdint>
#include <optional>

#include "firmware/boot_control.hpp"
#include "firmware/firmware_image.hpp"
#include "firmware/flash_memory.hpp"

namespace vecu::firmware {

enum class UpdateState : std::uint8_t { Idle = 0U, Erased = 1U, Downloading = 2U, Downloaded = 3U, Verified = 4U, Failed = 5U };

enum class UpdateStatus : std::uint8_t {
  Ok,
  SequenceError,       ///< step not allowed in the current state
  InvalidRequest,      ///< bad address / size
  WrongBlockCounter,   ///< TransferData out of sequence
  TooMuchData,         ///< more data than announced
  ProgrammingFailure,  ///< flash write/erase failed
  VerificationFailed,  ///< image rejected (see lastImageError())
  NotPossible          ///< e.g. rollback without a previous image
};

[[nodiscard]] const char* toString(UpdateState state) noexcept;
[[nodiscard]] const char* toString(UpdateStatus status) noexcept;

class UpdateManager {
 public:
  UpdateManager(IFlash& flash, BootControl& control, const IImageVerifier& verifier) noexcept
      : flash_(flash), control_(control), verifier_(verifier) {}

  void setPolicy(const UpdatePolicy& policy) noexcept { policy_ = policy; }
  /// Slot the application was booted from (nullopt: no valid application, bootloader mode).
  /// The update always targets the *other* slot, so the running image is never erased.
  void setRunningSlot(std::optional<std::uint8_t> slot) noexcept { running_slot_ = slot; }

  /// Marks the target slot "Receiving" and erases it. Refused (NotPossible) while the running
  /// image is still on trial: its slot partner is the only confirmed fallback.
  [[nodiscard]] UpdateStatus eraseInactiveSlot();
  /// `address` is relative to the target slot (must be 0); `size` = image size in bytes.
  [[nodiscard]] UpdateStatus requestDownload(std::uint32_t address, std::uint32_t size);
  /// Block counter starts at 1 and wraps 0xFF -> 0x00. A repeated block (same counter as the
  /// last accepted one, e.g. after a lost response) is acknowledged without rewriting.
  [[nodiscard]] UpdateStatus transferData(std::uint8_t counter, const std::uint8_t* data, std::size_t length);
  [[nodiscard]] UpdateStatus transferExit();
  /// Verifies integrity and compatibility; on success activates the image for the next boot.
  [[nodiscard]] UpdateStatus checkImage();
  /// Explicit rollback: make the other Confirmed image active for the next boot.
  [[nodiscard]] UpdateStatus rollbackToPrevious();
  void abort() noexcept;

  [[nodiscard]] UpdateState state() const noexcept { return state_; }
  [[nodiscard]] std::uint32_t bytesReceived() const noexcept { return received_; }
  [[nodiscard]] std::uint32_t expectedSize() const noexcept { return expected_size_; }
  [[nodiscard]] ImageError lastImageError() const noexcept { return last_image_error_; }
  [[nodiscard]] std::uint8_t targetSlot() const noexcept { return target_slot_; }
  [[nodiscard]] const ImageHeader& verifiedHeader() const noexcept { return verified_header_; }

 private:
  [[nodiscard]] UpdateStatus fail(UpdateStatus status) noexcept;

  IFlash& flash_;
  BootControl& control_;
  const IImageVerifier& verifier_;
  UpdatePolicy policy_{};
  std::optional<std::uint8_t> running_slot_{};
  UpdateState state_{UpdateState::Idle};
  std::uint8_t target_slot_{1U};
  std::uint32_t expected_size_{0U};
  std::uint32_t received_{0U};
  std::uint8_t last_counter_{0U};
  bool has_last_block_{false};
  ImageError last_image_error_{ImageError::None};
  ImageHeader verified_header_{};
};

}  // namespace vecu::firmware
