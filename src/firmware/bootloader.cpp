#include "firmware/bootloader.hpp"

namespace vecu::firmware {

ImageError Bootloader::verifySlot(std::uint8_t slot, ImageHeader& header) const {
  return verifyImageInFlash(flash_, FlashLayout::kSlotAddress[slot], FlashLayout::kSlotSize, verifier_, header);
}

bool Bootloader::installFactoryImage(const std::vector<std::uint8_t>& image) {
  if (image.size() > FlashLayout::kSlotSize) {
    return false;
  }
  return (flash_.eraseRange(FlashLayout::kSlotAddress[0], FlashLayout::kSlotSize) == FlashStatus::Ok) &&
         (flash_.program(FlashLayout::kSlotAddress[0], image.data(), image.size()) == FlashStatus::Ok) &&
         (flash_.sync() == FlashStatus::Ok);
}

BootResult Bootloader::boot(const std::vector<std::uint8_t>* factory_image) {
  BootResult result;
  BootRecord record;
  BootEvent event = BootEvent::Normal;
  bool changed = false;
  ImageHeader header;

  // 1. Record (or first provisioning of a blank device).
  if (!control_.load(record)) {
    record = BootRecord{};
    if ((verifySlot(0U, header) != ImageError::None) && (factory_image != nullptr) &&
        installFactoryImage(*factory_image)) {
      (void)verifySlot(0U, header);
    }
    if (verifySlot(0U, header) == ImageError::None) {
      record.state[0] = SlotState::Confirmed;
      record.version[0] = header.version;
    }
    event = BootEvent::FirstProvisioning;
    changed = true;
  }

  // 2. Trial handling of a newly activated image.
  std::uint8_t active = record.active_slot;
  if (record.state[active] == SlotState::Pending) {
    record.state[active] = SlotState::Trial;
    record.boot_attempts = 1U;
    event = BootEvent::TrialBoot;
    changed = true;
  } else if (record.state[active] == SlotState::Trial) {
    ++record.boot_attempts;
    event = BootEvent::TrialBoot;
    changed = true;
    if (record.boot_attempts > record.max_boot_attempts) {
      record.state[active] = SlotState::Invalid;  // the new image never became healthy
      event = BootEvent::Rollback;
    }
  } else {
    // Confirmed / not bootable: handled below
  }

  // 3. Verify the selected image; otherwise fall back to the other Confirmed image.
  const bool bootable = (record.state[active] == SlotState::Trial) || (record.state[active] == SlotState::Confirmed);
  ImageError error = bootable ? verifySlot(active, header) : ImageError::None;
  if (!bootable || (error != ImageError::None)) {
    if (bootable) {
      record.state[active] = SlotState::Invalid;
      result.rejected_image_error = error;
      if (event != BootEvent::Rollback) {
        event = BootEvent::FallbackCorrupt;
      }
    }
    const auto other = static_cast<std::uint8_t>(1U - active);
    if ((record.state[other] == SlotState::Confirmed) && (verifySlot(other, header) == ImageError::None)) {
      record.active_slot = other;
      record.boot_attempts = 0U;
      active = other;
      changed = true;
    } else {
      // Nothing can run (rejected_image_error keeps the reason): diagnostics / flashing only.
      event = BootEvent::NoValidImage;
      result.application_valid = false;
      record.last_event = event;
      result.record_status = control_.store(record);
      result.event = event;
      result.record = record;
      result.slot = active;
      return result;
    }
  }

  // 4. Persist before the application runs.
  record.last_event = event;
  if (changed) {
    result.record_status = control_.store(record);
  }
  result.application_valid = true;
  result.slot = active;
  result.header = header;
  result.event = event;
  result.record = record;
  return result;
}

FlashStatus Bootloader::confirmActive() {
  BootRecord record;
  if (!control_.load(record)) {
    return FlashStatus::IoError;
  }
  if (record.state[record.active_slot] != SlotState::Trial) {
    return FlashStatus::Ok;  // nothing to confirm
  }
  record.state[record.active_slot] = SlotState::Confirmed;
  record.boot_attempts = 0U;
  record.last_event = BootEvent::Normal;
  return control_.store(record);
}

}  // namespace vecu::firmware
