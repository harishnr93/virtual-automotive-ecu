#include "firmware/update_manager.hpp"

namespace vecu::firmware {

const char* toString(UpdateState state) noexcept {
  switch (state) {
    case UpdateState::Idle:
      return "Idle";
    case UpdateState::Erased:
      return "Erased";
    case UpdateState::Downloading:
      return "Downloading";
    case UpdateState::Downloaded:
      return "Downloaded";
    case UpdateState::Verified:
      return "Verified";
    case UpdateState::Failed:
      return "Failed";
  }
  return "Unknown";
}

const char* toString(UpdateStatus status) noexcept {
  switch (status) {
    case UpdateStatus::Ok:
      return "Ok";
    case UpdateStatus::SequenceError:
      return "SequenceError";
    case UpdateStatus::InvalidRequest:
      return "InvalidRequest";
    case UpdateStatus::WrongBlockCounter:
      return "WrongBlockCounter";
    case UpdateStatus::TooMuchData:
      return "TooMuchData";
    case UpdateStatus::ProgrammingFailure:
      return "ProgrammingFailure";
    case UpdateStatus::VerificationFailed:
      return "VerificationFailed";
    case UpdateStatus::NotPossible:
      return "NotPossible";
  }
  return "Unknown";
}

UpdateStatus UpdateManager::fail(UpdateStatus status) noexcept {
  state_ = UpdateState::Failed;
  return status;
}

UpdateStatus UpdateManager::eraseInactiveSlot() {
  if (state_ == UpdateState::Downloading) {
    return UpdateStatus::SequenceError;
  }
  BootRecord record;
  if (!control_.load(record)) {
    return fail(UpdateStatus::ProgrammingFailure);
  }
  if (running_slot_) {
    if (record.state[*running_slot_] != SlotState::Confirmed) {
      return UpdateStatus::NotPossible;  // confirm (or roll back) the trial image first
    }
    target_slot_ = static_cast<std::uint8_t>(1U - *running_slot_);
    record.active_slot = *running_slot_;  // drops an earlier, not yet booted Pending image
    record.boot_attempts = 0U;
  } else {
    target_slot_ = record.inactiveSlot();
  }
  // Persist "Receiving" first: from now on the target slot is never bootable, whatever happens.
  record.state[target_slot_] = SlotState::Receiving;
  record.version[target_slot_] = Version{};
  if (control_.store(record) != FlashStatus::Ok) {
    return fail(UpdateStatus::ProgrammingFailure);
  }
  if (flash_.eraseRange(FlashLayout::kSlotAddress[target_slot_], FlashLayout::kSlotSize) != FlashStatus::Ok) {
    return fail(UpdateStatus::ProgrammingFailure);
  }
  state_ = UpdateState::Erased;
  received_ = 0U;
  expected_size_ = 0U;
  has_last_block_ = false;
  last_image_error_ = ImageError::None;
  return UpdateStatus::Ok;
}

UpdateStatus UpdateManager::requestDownload(std::uint32_t address, std::uint32_t size) {
  if (state_ != UpdateState::Erased) {
    return UpdateStatus::SequenceError;
  }
  if ((address != 0U) || (size < kImageHeaderSize) || (size > FlashLayout::kSlotSize)) {
    return UpdateStatus::InvalidRequest;
  }
  expected_size_ = size;
  received_ = 0U;
  has_last_block_ = false;
  state_ = UpdateState::Downloading;
  return UpdateStatus::Ok;
}

UpdateStatus UpdateManager::transferData(std::uint8_t counter, const std::uint8_t* data, std::size_t length) {
  if (state_ != UpdateState::Downloading) {
    return UpdateStatus::SequenceError;
  }
  if (has_last_block_ && (counter == last_counter_)) {
    return UpdateStatus::Ok;  // retransmission of the last block: already written
  }
  const auto expected = static_cast<std::uint8_t>(has_last_block_ ? (last_counter_ + 1U) : 1U);
  if (counter != expected) {
    return UpdateStatus::WrongBlockCounter;
  }
  if ((data == nullptr) || (length == 0U) || (length > expected_size_ - received_)) {
    return UpdateStatus::TooMuchData;
  }
  if (flash_.program(FlashLayout::kSlotAddress[target_slot_] + received_, data, length) != FlashStatus::Ok) {
    return fail(UpdateStatus::ProgrammingFailure);
  }
  received_ += static_cast<std::uint32_t>(length);
  last_counter_ = counter;
  has_last_block_ = true;
  return UpdateStatus::Ok;
}

UpdateStatus UpdateManager::transferExit() {
  if ((state_ != UpdateState::Downloading) || (received_ != expected_size_)) {
    return UpdateStatus::SequenceError;
  }
  if (flash_.sync() != FlashStatus::Ok) {
    return fail(UpdateStatus::ProgrammingFailure);
  }
  state_ = UpdateState::Downloaded;
  return UpdateStatus::Ok;
}

UpdateStatus UpdateManager::checkImage() {
  if (state_ != UpdateState::Downloaded) {
    return UpdateStatus::SequenceError;
  }
  BootRecord record;
  if (!control_.load(record)) {
    return fail(UpdateStatus::ProgrammingFailure);
  }
  ImageHeader header;
  ImageError error = verifyImageInFlash(flash_, FlashLayout::kSlotAddress[target_slot_], FlashLayout::kSlotSize,
                                        verifier_, header);
  if (error == ImageError::None) {
    error = checkCompatibility(header, policy_);
  }
  last_image_error_ = error;
  if (error != ImageError::None) {
    record.state[target_slot_] = SlotState::Invalid;
    (void)control_.store(record);
    return fail(UpdateStatus::VerificationFailed);
  }
  // Activate for the next boot: the bootloader will run it as a Trial.
  record.state[target_slot_] = SlotState::Pending;
  record.version[target_slot_] = header.version;
  record.active_slot = target_slot_;
  record.boot_attempts = 0U;
  if (control_.store(record) != FlashStatus::Ok) {
    return fail(UpdateStatus::ProgrammingFailure);
  }
  verified_header_ = header;
  state_ = UpdateState::Verified;
  return UpdateStatus::Ok;
}

UpdateStatus UpdateManager::rollbackToPrevious() {
  if (state_ == UpdateState::Downloading) {
    return UpdateStatus::SequenceError;
  }
  BootRecord record;
  if (!control_.load(record)) {
    return UpdateStatus::ProgrammingFailure;
  }
  const std::uint8_t other = record.inactiveSlot();
  if (record.state[other] != SlotState::Confirmed) {
    return UpdateStatus::NotPossible;
  }
  if ((record.state[record.active_slot] == SlotState::Trial) ||
      (record.state[record.active_slot] == SlotState::Pending)) {
    record.state[record.active_slot] = SlotState::Invalid;  // reject an unconfirmed image
  }
  record.active_slot = other;
  record.boot_attempts = 0U;
  return (control_.store(record) == FlashStatus::Ok) ? UpdateStatus::Ok : UpdateStatus::ProgrammingFailure;
}

void UpdateManager::abort() noexcept {
  if ((state_ == UpdateState::Erased) || (state_ == UpdateState::Downloading) || (state_ == UpdateState::Downloaded)) {
    state_ = UpdateState::Failed;  // target slot stays "Receiving": never bootable
  }
}

}  // namespace vecu::firmware
