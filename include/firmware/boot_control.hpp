#pragma once

// Flash layout and the redundant boot control record.
//
//   0x00000  slot A   (256 KiB)  firmware image
//   0x40000  slot B   (256 KiB)  firmware image
//   0x80000  boot record copy 0  (1 sector)
//   0x81000  boot record copy 1  (1 sector)
//
// The boot record says which slot is active and the state of each slot. It is written
// alternately to two sectors, each copy with a sequence number and a CRC. Updating the record
// never touches the newest valid copy, so a power loss during the update leaves the previous
// record intact (A/B redundancy, as in AUTOSAR NvM "redundant blocks").

#include <array>
#include <cstdint>

#include "firmware/firmware_image.hpp"
#include "firmware/flash_memory.hpp"

namespace vecu::firmware {

struct FlashLayout {
  static constexpr std::uint32_t kSectorSize = 4096U;
  static constexpr std::uint32_t kSlotSize = 256U * 1024U;
  static constexpr std::array<std::uint32_t, 2U> kSlotAddress{0U, kSlotSize};
  static constexpr std::array<std::uint32_t, 2U> kRecordAddress{2U * kSlotSize, (2U * kSlotSize) + kSectorSize};
  static constexpr std::uint32_t kTotalSize = (2U * kSlotSize) + (2U * kSectorSize);
};

enum class SlotState : std::uint8_t {
  Empty = 0U,      ///< never programmed
  Receiving = 1U,  ///< erased / download in progress: never bootable
  Pending = 2U,    ///< downloaded and verified: boot it as a trial at the next reset
  Trial = 3U,      ///< booted, not yet confirmed healthy (boot attempts counted)
  Confirmed = 4U,  ///< known good
  Invalid = 5U     ///< failed verification or trial: never bootable
};

enum class BootEvent : std::uint8_t {
  Normal = 0U,
  FirstProvisioning = 1U,  ///< empty flash: factory image installed
  TrialBoot = 2U,          ///< booting a new, not yet confirmed image
  Rollback = 3U,           ///< trial image failed too often: previous image restored
  FallbackCorrupt = 4U,    ///< active image corrupt: other image used
  NoValidImage = 5U        ///< nothing bootable: bootloader (diagnostics) only
};

[[nodiscard]] const char* toString(SlotState state) noexcept;
[[nodiscard]] const char* toString(BootEvent event) noexcept;

struct BootRecord {
  std::uint32_t sequence{0U};
  std::uint8_t active_slot{0U};
  std::array<SlotState, 2U> state{SlotState::Empty, SlotState::Empty};
  std::array<Version, 2U> version{};
  std::uint8_t boot_attempts{0U};       ///< boots of the active slot while in Trial
  std::uint8_t max_boot_attempts{3U};
  BootEvent last_event{BootEvent::Normal};

  [[nodiscard]] std::uint8_t inactiveSlot() const noexcept { return static_cast<std::uint8_t>(1U - active_slot); }
};

class BootControl {
 public:
  explicit BootControl(IFlash& flash) noexcept : flash_(flash) {}

  /// Loads the newest valid copy. False if neither copy is valid (blank or corrupt).
  [[nodiscard]] bool load(BootRecord& record) const;

  /// Writes `record` (sequence = newest + 1) into the older copy: erase, program, sync.
  [[nodiscard]] FlashStatus store(BootRecord& record);

  static constexpr std::size_t kRecordSize = 32U;

 private:
  [[nodiscard]] bool readCopy(std::size_t copy, BootRecord& record) const;

  IFlash& flash_;
};

}  // namespace vecu::firmware
