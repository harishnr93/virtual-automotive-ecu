// Firmware image parser and bootloader on arbitrary flash content (a corrupted or hostile
// flash must never make the bootloader crash or boot an image that does not verify).
//
// Input: [selector][bytes...]. Selector bit 0: image into slot A (else slot B); bit 1: also
// write the remaining bytes over both boot-record sectors.
// Invariants: parseHeader / verify never read out of bounds; if the bootloader reports a
// valid application, the image in the reported slot verifies.

#include <algorithm>
#include <cstddef>
#include <cstdint>

#include "firmware/boot_control.hpp"
#include "firmware/bootloader.hpp"
#include "firmware/firmware_image.hpp"
#include "firmware/flash_memory.hpp"
#include "fuzz_check.hpp"

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size) {
  using namespace vecu::firmware;
  if (size < 1U) {
    return 0;
  }
  const std::uint8_t selector = data[0];
  ++data;
  --size;
  ImageHeader header;
  (void)parseHeader(data, size, header);

  RamFlash flash(FlashLayout::kTotalSize, FlashLayout::kSectorSize);
  const std::size_t image_len = std::min<std::size_t>(size, FlashLayout::kSlotSize);
  const std::uint32_t slot = (selector & 0x01U) ? 0U : 1U;
  (void)flash.program(FlashLayout::kSlotAddress[slot], data, image_len);
  if ((selector & 0x02U) != 0U) {
    for (const std::uint32_t address : FlashLayout::kRecordAddress) {
      (void)flash.program(address, data, std::min<std::size_t>(size, FlashLayout::kSectorSize));
    }
  }
  Crc32Verifier crc;
  ImageHeader verified;
  (void)verifyImageInFlash(flash, FlashLayout::kSlotAddress[slot], FlashLayout::kSlotSize, crc, verified);

  BootControl control(flash);
  Bootloader bootloader(flash, control, crc);
  const BootResult result = bootloader.boot(nullptr);
  if (result.application_valid) {
    FUZZ_CHECK(result.slot < 2U);
    ImageHeader check;
    FUZZ_CHECK(verifyImageInFlash(flash, FlashLayout::kSlotAddress[result.slot], FlashLayout::kSlotSize, crc, check) ==
               ImageError::None);
  }
  return 0;
}
