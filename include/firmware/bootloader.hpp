#pragma once

// Boot selection ("bootloader" logic), run at every ECU start and simulated hard reset.
//
//   1. Load the boot record. If none exists (blank flash), install the factory image in slot A.
//   2. A Pending slot becomes Trial (attempts = 1). A Trial slot counts one more attempt; after
//      more than max_boot_attempts the trial image is marked Invalid and the previous
//      Confirmed image is restored (ROLLBACK).
//   3. Verify the selected image (header + CRC). If corrupt, mark it Invalid and fall back to
//      the other Confirmed image. If nothing is bootable: bootloader-only mode.
//   4. Store the updated record BEFORE starting the application (a crash of the new image then
//      already counted as one attempt).
// The application confirms a trial image once it has run healthy for a while (confirm()).

#include <cstdint>
#include <vector>

#include "firmware/boot_control.hpp"
#include "firmware/firmware_image.hpp"
#include "firmware/flash_memory.hpp"

namespace vecu::firmware {

struct BootResult {
  bool application_valid{false};
  std::uint8_t slot{0U};
  ImageHeader header{};
  BootEvent event{BootEvent::Normal};
  ImageError rejected_image_error{ImageError::None};  ///< why an image was rejected (if any)
  BootRecord record{};
  FlashStatus record_status{FlashStatus::Ok};
};

class Bootloader {
 public:
  Bootloader(IFlash& flash, BootControl& control, const IImageVerifier& verifier) noexcept
      : flash_(flash), control_(control), verifier_(verifier) {}

  /// `factory_image` is installed into slot A on a blank device (may be nullptr).
  [[nodiscard]] BootResult boot(const std::vector<std::uint8_t>* factory_image);

  /// Marks the active Trial image as Confirmed (resets the boot attempts).
  [[nodiscard]] FlashStatus confirmActive();

 private:
  [[nodiscard]] ImageError verifySlot(std::uint8_t slot, ImageHeader& header) const;
  [[nodiscard]] bool installFactoryImage(const std::vector<std::uint8_t>& image);

  IFlash& flash_;
  BootControl& control_;
  const IImageVerifier& verifier_;
};

}  // namespace vecu::firmware
