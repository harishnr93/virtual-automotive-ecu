#include "firmware/boot_control.hpp"

#include <algorithm>

#include "core/crc32.hpp"

namespace vecu::firmware {

namespace {

constexpr std::array<std::uint8_t, 4U> kRecordMagic{'V', 'B', 'C', 'B'};
constexpr std::uint8_t kRecordVersion = 1U;
constexpr std::size_t kCrcOffset = 28U;

bool validState(std::uint8_t value) noexcept { return value <= static_cast<std::uint8_t>(SlotState::Invalid); }

}  // namespace

const char* toString(SlotState state) noexcept {
  switch (state) {
    case SlotState::Empty:
      return "Empty";
    case SlotState::Receiving:
      return "Receiving";
    case SlotState::Pending:
      return "Pending";
    case SlotState::Trial:
      return "Trial";
    case SlotState::Confirmed:
      return "Confirmed";
    case SlotState::Invalid:
      return "Invalid";
  }
  return "Unknown";
}

const char* toString(BootEvent event) noexcept {
  switch (event) {
    case BootEvent::Normal:
      return "Normal";
    case BootEvent::FirstProvisioning:
      return "FirstProvisioning";
    case BootEvent::TrialBoot:
      return "TrialBoot";
    case BootEvent::Rollback:
      return "Rollback";
    case BootEvent::FallbackCorrupt:
      return "FallbackCorrupt";
    case BootEvent::NoValidImage:
      return "NoValidImage";
  }
  return "Unknown";
}

bool BootControl::readCopy(std::size_t copy, BootRecord& record) const {
  std::array<std::uint8_t, kRecordSize> b{};
  if (flash_.read(FlashLayout::kRecordAddress[copy], b.data(), b.size()) != FlashStatus::Ok) {
    return false;
  }
  const std::uint32_t stored_crc = static_cast<std::uint32_t>(b[kCrcOffset]) |
                                   (static_cast<std::uint32_t>(b[kCrcOffset + 1U]) << 8U) |
                                   (static_cast<std::uint32_t>(b[kCrcOffset + 2U]) << 16U) |
                                   (static_cast<std::uint32_t>(b[kCrcOffset + 3U]) << 24U);
  if (!std::equal(kRecordMagic.begin(), kRecordMagic.end(), b.begin()) || (b[4] != kRecordVersion) ||
      (core::crc32(b.data(), kCrcOffset) != stored_crc) || (b[12] > 1U) || !validState(b[13]) || !validState(b[14])) {
    return false;
  }
  BootRecord r;
  r.sequence = static_cast<std::uint32_t>(b[8]) | (static_cast<std::uint32_t>(b[9]) << 8U) |
               (static_cast<std::uint32_t>(b[10]) << 16U) | (static_cast<std::uint32_t>(b[11]) << 24U);
  r.active_slot = b[12];
  r.state = {static_cast<SlotState>(b[13]), static_cast<SlotState>(b[14])};
  r.boot_attempts = b[15];
  r.max_boot_attempts = b[16];
  r.last_event = static_cast<BootEvent>(b[17]);
  r.version[0] = Version{b[18], b[19], b[20]};
  r.version[1] = Version{b[21], b[22], b[23]};
  record = r;
  return true;
}

bool BootControl::load(BootRecord& record) const {
  BootRecord c0;
  BootRecord c1;
  const bool valid0 = readCopy(0U, c0);
  const bool valid1 = readCopy(1U, c1);
  if (!valid0 && !valid1) {
    return false;
  }
  if (valid0 && valid1) {
    record = (c1.sequence > c0.sequence) ? c1 : c0;
  } else {
    record = valid0 ? c0 : c1;
  }
  return true;
}

FlashStatus BootControl::store(BootRecord& record) {
  // Target = the copy that does NOT hold the newest valid record.
  BootRecord c0;
  BootRecord c1;
  const bool valid0 = readCopy(0U, c0);
  const bool valid1 = readCopy(1U, c1);
  std::size_t target = 0U;
  std::uint32_t newest = 0U;
  if (valid0 && valid1) {
    target = (c0.sequence > c1.sequence) ? 1U : 0U;
    newest = std::max(c0.sequence, c1.sequence);
  } else if (valid0) {
    target = 1U;
    newest = c0.sequence;
  } else if (valid1) {
    target = 0U;
    newest = c1.sequence;
  } else {
    target = 0U;
  }
  record.sequence = newest + 1U;

  std::array<std::uint8_t, kRecordSize> b{};
  std::copy(kRecordMagic.begin(), kRecordMagic.end(), b.begin());
  b[4] = kRecordVersion;
  for (std::size_t i = 0U; i < 4U; ++i) {
    b[8U + i] = static_cast<std::uint8_t>((record.sequence >> (8U * i)) & 0xFFU);
  }
  b[12] = record.active_slot;
  b[13] = static_cast<std::uint8_t>(record.state[0]);
  b[14] = static_cast<std::uint8_t>(record.state[1]);
  b[15] = record.boot_attempts;
  b[16] = record.max_boot_attempts;
  b[17] = static_cast<std::uint8_t>(record.last_event);
  b[18] = record.version[0].major;
  b[19] = record.version[0].minor;
  b[20] = record.version[0].patch;
  b[21] = record.version[1].major;
  b[22] = record.version[1].minor;
  b[23] = record.version[1].patch;
  const std::uint32_t crc = core::crc32(b.data(), kCrcOffset);
  for (std::size_t i = 0U; i < 4U; ++i) {
    b[kCrcOffset + i] = static_cast<std::uint8_t>((crc >> (8U * i)) & 0xFFU);
  }

  const std::uint32_t address = FlashLayout::kRecordAddress[target];
  FlashStatus status = flash_.eraseSector(address);
  if (status == FlashStatus::Ok) {
    status = flash_.program(address, b.data(), b.size());
  }
  if (status == FlashStatus::Ok) {
    status = flash_.sync();
  }
  return status;
}

}  // namespace vecu::firmware
