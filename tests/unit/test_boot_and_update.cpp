// Boot record redundancy, bootloader decisions and the update state machine — including
// interrupted updates (power loss) and rollback, all with deterministic in-memory flash.

#include <gtest/gtest.h>

#include <vector>

#include "firmware/boot_control.hpp"
#include "firmware/bootloader.hpp"
#include "firmware/firmware_image.hpp"
#include "firmware/flash_memory.hpp"
#include "firmware/update_manager.hpp"

namespace vecu::firmware {
namespace {

constexpr std::uint32_t kHw = 0x0000CC01U;
constexpr std::size_t kBlock = 4093U;  // UDS TransferData payload per block

std::vector<std::uint8_t> image(Version v, std::uint32_t security = 1U, std::uint32_t hw = kHw,
                                std::size_t size = 50000U, std::uint32_t flags = 0U) {
  ImageHeader h;
  h.hardware_id = hw;
  h.version = v;
  h.security_version = security;
  h.flags = flags;
  return buildImage(h, makePayload(size, v.value()));
}

class FirmwareTest : public ::testing::Test {
 protected:
  RamFlash flash{FlashLayout::kTotalSize, FlashLayout::kSectorSize};
  BootControl control{flash};
  Crc32Verifier crc;
  Bootloader bootloader{flash, control, crc};
  UpdateManager updater{flash, control, crc};
  std::vector<std::uint8_t> factory = image({1, 0, 0});

  BootResult bootAndArm() {
    const BootResult r = bootloader.boot(&factory);
    if (r.application_valid) {  // like the ECU after a reset: policy follows the running image
      updater.setPolicy(UpdatePolicy{kHw, r.header.version, r.header.security_version, false});
      updater.setRunningSlot(r.slot);
    }
    return r;
  }

  /// Full UDS-style download of `img` into the inactive slot; returns the check result.
  UpdateStatus download(const std::vector<std::uint8_t>& img) {
    EXPECT_EQ(updater.eraseInactiveSlot(), UpdateStatus::Ok);
    EXPECT_EQ(updater.requestDownload(0U, static_cast<std::uint32_t>(img.size())), UpdateStatus::Ok);
    std::uint8_t counter = 1U;
    for (std::size_t off = 0U; off < img.size(); off += kBlock) {
      const std::size_t n = std::min(kBlock, img.size() - off);
      EXPECT_EQ(updater.transferData(counter, img.data() + off, n), UpdateStatus::Ok);
      counter = static_cast<std::uint8_t>(counter + 1U);
    }
    EXPECT_EQ(updater.transferExit(), UpdateStatus::Ok);
    return updater.checkImage();
  }
};

// ---------------------------------------------------------------- boot record ----

TEST_F(FirmwareTest, BootRecordAlternatesCopiesWithSequence) {
  BootRecord r;
  EXPECT_FALSE(control.load(r));  // blank
  r.active_slot = 1U;
  ASSERT_EQ(control.store(r), FlashStatus::Ok);
  EXPECT_EQ(r.sequence, 1U);
  r.active_slot = 0U;
  ASSERT_EQ(control.store(r), FlashStatus::Ok);
  EXPECT_EQ(r.sequence, 2U);
  BootRecord loaded;
  ASSERT_TRUE(control.load(loaded));
  EXPECT_EQ(loaded.sequence, 2U);
  EXPECT_EQ(loaded.active_slot, 0U);
  EXPECT_EQ(flash.eraseCount(FlashLayout::kRecordAddress[0]), 1U);  // wear spread over both copies
  EXPECT_EQ(flash.eraseCount(FlashLayout::kRecordAddress[1]), 1U);
}

TEST_F(FirmwareTest, PowerLossDuringRecordWriteKeepsPreviousRecord) {
  BootRecord r;
  r.active_slot = 1U;
  ASSERT_EQ(control.store(r), FlashStatus::Ok);  // valid record #1 (active B)
  r.active_slot = 0U;
  flash.injectPowerLossAfter(10U);               // torn write of record #2
  EXPECT_EQ(control.store(r), FlashStatus::PowerLoss);
  flash.powerCycle();
  BootRecord loaded;
  ASSERT_TRUE(control.load(loaded));
  EXPECT_EQ(loaded.sequence, 1U);
  EXPECT_EQ(loaded.active_slot, 1U);  // the old, consistent record wins
}

TEST_F(FirmwareTest, CorruptNewestCopyFallsBackToOlder) {
  BootRecord r;
  ASSERT_EQ(control.store(r), FlashStatus::Ok);  // seq 1 -> copy 0
  r.active_slot = 1U;
  ASSERT_EQ(control.store(r), FlashStatus::Ok);  // seq 2 -> copy 1
  flash.flipBit(FlashLayout::kRecordAddress[1] + 12U, 0U);
  BootRecord loaded;
  ASSERT_TRUE(control.load(loaded));
  EXPECT_EQ(loaded.sequence, 1U);
}

// ------------------------------------------------------------------ bootloader ----

TEST_F(FirmwareTest, BlankDeviceIsProvisionedWithFactoryImage) {
  const BootResult r = bootloader.boot(&factory);
  ASSERT_TRUE(r.application_valid);
  EXPECT_EQ(r.event, BootEvent::FirstProvisioning);
  EXPECT_EQ(r.slot, 0U);
  EXPECT_EQ(r.header.version, (Version{1, 0, 0}));
  EXPECT_EQ(r.record.state[0], SlotState::Confirmed);
  EXPECT_EQ(bootloader.boot(&factory).event, BootEvent::Normal);  // second boot: nothing special
}

TEST_F(FirmwareTest, BlankDeviceWithoutFactoryImageHasNoApplication) {
  const BootResult r = bootloader.boot(nullptr);
  EXPECT_FALSE(r.application_valid);
  EXPECT_EQ(r.event, BootEvent::NoValidImage);
}

TEST_F(FirmwareTest, SuccessfulUpdateBootsAsTrialThenConfirms) {
  bootAndArm();
  ASSERT_EQ(download(image({1, 1, 0})), UpdateStatus::Ok);
  EXPECT_EQ(updater.state(), UpdateState::Verified);
  BootResult r = bootAndArm();
  EXPECT_EQ(r.event, BootEvent::TrialBoot);
  EXPECT_EQ(r.slot, 1U);
  EXPECT_EQ(r.header.version, (Version{1, 1, 0}));
  EXPECT_EQ(r.record.boot_attempts, 1U);
  EXPECT_EQ(r.record.state[0], SlotState::Confirmed);  // previous image kept as backup
  ASSERT_EQ(bootloader.confirmActive(), FlashStatus::Ok);
  r = bootAndArm();
  EXPECT_EQ(r.event, BootEvent::Normal);
  EXPECT_EQ(r.record.state[1], SlotState::Confirmed);
  EXPECT_EQ(r.record.boot_attempts, 0U);
}

TEST_F(FirmwareTest, TrialImageThatNeverConfirmsIsRolledBack) {
  bootAndArm();
  ASSERT_EQ(download(image({2, 0, 0})), UpdateStatus::Ok);
  for (std::uint8_t attempt = 1U; attempt <= 3U; ++attempt) {  // crashes before confirming
    const BootResult r = bootloader.boot(&factory);
    ASSERT_EQ(r.event, BootEvent::TrialBoot) << int{attempt};
    EXPECT_EQ(r.record.boot_attempts, attempt);
  }
  const BootResult r = bootloader.boot(&factory);  // 4th boot: attempts exceeded
  EXPECT_EQ(r.event, BootEvent::Rollback);
  EXPECT_TRUE(r.application_valid);
  EXPECT_EQ(r.slot, 0U);
  EXPECT_EQ(r.header.version, (Version{1, 0, 0}));
  EXPECT_EQ(r.record.state[1], SlotState::Invalid);
  EXPECT_EQ(bootloader.boot(&factory).event, BootEvent::Normal);  // stays on the good image
}

TEST_F(FirmwareTest, CorruptActiveImageFallsBackToBackup) {
  bootAndArm();
  ASSERT_EQ(download(image({1, 1, 0})), UpdateStatus::Ok);
  bootAndArm();
  ASSERT_EQ(bootloader.confirmActive(), FlashStatus::Ok);  // now B active, A backup
  flash.flipBit(FlashLayout::kSlotAddress[1] + 1000U, 0U);  // bit rot in B
  const BootResult r = bootloader.boot(&factory);
  EXPECT_EQ(r.event, BootEvent::FallbackCorrupt);
  EXPECT_EQ(r.rejected_image_error, ImageError::PayloadCrcMismatch);
  EXPECT_EQ(r.slot, 0U);
  EXPECT_TRUE(r.application_valid);
}

TEST_F(FirmwareTest, OnlyImageCorruptMeansNoValidImageButFlashingStillWorks) {
  bootAndArm();
  flash.flipBit(FlashLayout::kSlotAddress[0] + 2000U, 5U);  // the one and only image rots
  const BootResult r = bootloader.boot(&factory);  // factory image is not re-installed: record exists
  EXPECT_FALSE(r.application_valid);
  EXPECT_EQ(r.event, BootEvent::NoValidImage);
  EXPECT_EQ(r.rejected_image_error, ImageError::PayloadCrcMismatch);
  // Bootloader mode: no running image, any valid image for this hardware is accepted.
  updater.setRunningSlot(std::nullopt);
  updater.setPolicy(UpdatePolicy{kHw, Version{}, 0U, true});
  ASSERT_EQ(download(image({1, 0, 1})), UpdateStatus::Ok);
  const BootResult recovered = bootloader.boot(&factory);
  EXPECT_TRUE(recovered.application_valid);
  EXPECT_EQ(recovered.header.version, (Version{1, 0, 1}));
  EXPECT_EQ(recovered.event, BootEvent::TrialBoot);
}

// ---------------------------------------------------------------- update manager ----

TEST_F(FirmwareTest, CorruptedDownloadIsRejectedAndOldImageStays) {
  bootAndArm();
  auto bad = image({1, 1, 0});
  bad[30000] ^= 0x01U;  // transmission error
  EXPECT_EQ(download(bad), UpdateStatus::VerificationFailed);
  EXPECT_EQ(updater.lastImageError(), ImageError::PayloadCrcMismatch);
  EXPECT_EQ(updater.state(), UpdateState::Failed);
  const BootResult r = bootAndArm();
  EXPECT_EQ(r.event, BootEvent::Normal);
  EXPECT_EQ(r.header.version, (Version{1, 0, 0}));
  EXPECT_EQ(r.record.state[1], SlotState::Invalid);
}

TEST_F(FirmwareTest, PolicyRejectsDowngradeWrongHardwareAndSecurityRollback) {
  bootAndArm();  // running 1.0.0, security 1
  EXPECT_EQ(download(image({0, 9, 0})), UpdateStatus::VerificationFailed);
  EXPECT_EQ(updater.lastImageError(), ImageError::VersionNotNewer);
  EXPECT_EQ(download(image({1, 1, 0}, 1U, 0xBADU)), UpdateStatus::VerificationFailed);
  EXPECT_EQ(updater.lastImageError(), ImageError::WrongHardware);
  EXPECT_EQ(download(image({1, 1, 0}, 0U)), UpdateStatus::VerificationFailed);
  EXPECT_EQ(updater.lastImageError(), ImageError::SecurityVersionRollback);
  EXPECT_EQ(bootAndArm().header.version, (Version{1, 0, 0}));
}

TEST_F(FirmwareTest, TransferProtocolRules) {
  bootAndArm();
  const auto img = image({1, 1, 0});
  EXPECT_EQ(updater.requestDownload(0U, 1000U), UpdateStatus::SequenceError);  // erase first
  ASSERT_EQ(updater.eraseInactiveSlot(), UpdateStatus::Ok);
  EXPECT_EQ(updater.requestDownload(4U, 1000U), UpdateStatus::InvalidRequest);
  EXPECT_EQ(updater.requestDownload(0U, FlashLayout::kSlotSize + 1U), UpdateStatus::InvalidRequest);
  ASSERT_EQ(updater.requestDownload(0U, static_cast<std::uint32_t>(img.size())), UpdateStatus::Ok);
  EXPECT_EQ(updater.transferData(2U, img.data(), kBlock), UpdateStatus::WrongBlockCounter);
  ASSERT_EQ(updater.transferData(1U, img.data(), kBlock), UpdateStatus::Ok);
  EXPECT_EQ(updater.transferData(1U, img.data(), kBlock), UpdateStatus::Ok);  // retransmission
  EXPECT_EQ(updater.bytesReceived(), kBlock);                                 // not written twice
  EXPECT_EQ(updater.transferExit(), UpdateStatus::SequenceError);             // incomplete
  EXPECT_EQ(updater.transferData(2U, img.data(), img.size()), UpdateStatus::TooMuchData);
}

TEST_F(FirmwareTest, BlockCounterWrapsAfter255) {
  bootAndArm();
  const auto img = image({1, 1, 0}, 1U, kHw, 300U * 256U);  // 300 blocks of 256 bytes
  ASSERT_EQ(updater.eraseInactiveSlot(), UpdateStatus::Ok);
  ASSERT_EQ(updater.requestDownload(0U, static_cast<std::uint32_t>(img.size())), UpdateStatus::Ok);
  std::uint8_t counter = 1U;
  for (std::size_t off = 0U; off < img.size(); off += 256U) {
    ASSERT_EQ(updater.transferData(counter, img.data() + off, std::min<std::size_t>(256U, img.size() - off)),
              UpdateStatus::Ok);
    counter = static_cast<std::uint8_t>(counter + 1U);  // 255 -> 0
  }
  EXPECT_EQ(updater.transferExit(), UpdateStatus::Ok);
  EXPECT_EQ(updater.checkImage(), UpdateStatus::Ok);
}

TEST_F(FirmwareTest, PowerLossDuringDownloadNeverBricksTheEcu) {
  bootAndArm();
  const auto img = image({1, 1, 0});
  ASSERT_EQ(updater.eraseInactiveSlot(), UpdateStatus::Ok);
  ASSERT_EQ(updater.requestDownload(0U, static_cast<std::uint32_t>(img.size())), UpdateStatus::Ok);
  ASSERT_EQ(updater.transferData(1U, img.data(), kBlock), UpdateStatus::Ok);
  flash.injectPowerLossAfter(1000U);
  EXPECT_EQ(updater.transferData(2U, img.data() + kBlock, kBlock), UpdateStatus::ProgrammingFailure);
  flash.powerCycle();  // ECU restarts
  BootResult r = bootAndArm();
  ASSERT_TRUE(r.application_valid);
  EXPECT_EQ(r.event, BootEvent::Normal);
  EXPECT_EQ(r.header.version, (Version{1, 0, 0}));
  EXPECT_EQ(r.record.state[1], SlotState::Receiving);  // half-written slot is never bootable
  EXPECT_EQ(download(img), UpdateStatus::Ok);         // a retry succeeds
  EXPECT_EQ(bootAndArm().header.version, (Version{1, 1, 0}));
}

TEST_F(FirmwareTest, PowerLossAtEveryPointOfTheUpdateIsSafe) {
  // Exhaustive in the number of programmed bytes before the loss (step 997 bytes): whatever
  // happens, the next boot must find a valid image (old or new, never none).
  const auto img = image({1, 1, 0}, 1U, kHw, 12000U);
  for (std::size_t loss_after = 0U; loss_after < 20000U; loss_after += 997U) {
    RamFlash f{FlashLayout::kTotalSize, FlashLayout::kSectorSize};
    BootControl c{f};
    Bootloader b{f, c, crc};
    ASSERT_TRUE(b.boot(&factory).application_valid);
    UpdateManager u{f, c, crc};
    u.setPolicy(UpdatePolicy{kHw, Version{1, 0, 0}, 1U, false});
    u.setRunningSlot(0U);
    f.injectPowerLossAfter(loss_after);
    bool ok = (u.eraseInactiveSlot() == UpdateStatus::Ok) &&
              (u.requestDownload(0U, static_cast<std::uint32_t>(img.size())) == UpdateStatus::Ok);
    std::uint8_t counter = 1U;
    for (std::size_t off = 0U; ok && off < img.size(); off += kBlock) {
      ok = u.transferData(counter++, img.data() + off, std::min(kBlock, img.size() - off)) == UpdateStatus::Ok;
    }
    ok = ok && (u.transferExit() == UpdateStatus::Ok) && (u.checkImage() == UpdateStatus::Ok);
    f.powerCycle();
    const BootResult r = b.boot(&factory);
    ASSERT_TRUE(r.application_valid) << "loss after " << loss_after << " bytes";
    const bool expected_new = ok;  // only a fully verified update may activate the new image
    EXPECT_EQ(r.header.version, expected_new ? (Version{1, 1, 0}) : (Version{1, 0, 0})) << loss_after;
  }
}

TEST_F(FirmwareTest, ExplicitRollbackToPreviousImage) {
  bootAndArm();
  ASSERT_EQ(updater.rollbackToPrevious(), UpdateStatus::NotPossible);  // only one image
  ASSERT_EQ(download(image({1, 1, 0})), UpdateStatus::Ok);
  bootAndArm();
  ASSERT_EQ(bootloader.confirmActive(), FlashStatus::Ok);
  ASSERT_EQ(updater.rollbackToPrevious(), UpdateStatus::Ok);
  const BootResult r = bootAndArm();
  EXPECT_EQ(r.header.version, (Version{1, 0, 0}));
  EXPECT_EQ(r.record.state[1], SlotState::Confirmed);  // still available to roll forward
}

TEST_F(FirmwareTest, SecondDownloadBeforeResetNeverTouchesTheRunningImage) {
  bootAndArm();  // running: slot A, 1.0.0
  ASSERT_EQ(download(image({1, 1, 0})), UpdateStatus::Ok);  // B pending, active = B
  ASSERT_EQ(download(image({1, 2, 0})), UpdateStatus::Ok);  // tester flashes again before the reset
  const BootResult r = bootAndArm();
  EXPECT_EQ(r.slot, 1U);  // still B, not A
  EXPECT_EQ(r.header.version, (Version{1, 2, 0}));
  EXPECT_EQ(r.record.state[0], SlotState::Confirmed);  // running image A untouched
}

TEST_F(FirmwareTest, NoUpdateWhileRunningAnUnconfirmedTrialImage) {
  bootAndArm();
  ASSERT_EQ(download(image({1, 1, 0})), UpdateStatus::Ok);
  bootAndArm();  // trial of 1.1.0 in B; A is the only confirmed fallback
  EXPECT_EQ(updater.eraseInactiveSlot(), UpdateStatus::NotPossible);
  ASSERT_EQ(bootloader.confirmActive(), FlashStatus::Ok);
  EXPECT_EQ(updater.eraseInactiveSlot(), UpdateStatus::Ok);  // after confirmation: A may be replaced
}

TEST_F(FirmwareTest, RollbackDiscardsAPendingImage) {
  bootAndArm();
  ASSERT_EQ(download(image({1, 1, 0})), UpdateStatus::Ok);
  ASSERT_EQ(download(image({1, 2, 0})), UpdateStatus::Ok);
  BootRecord record;
  ASSERT_TRUE(control.load(record));
  ASSERT_EQ(record.state[1], SlotState::Pending);
  EXPECT_EQ(updater.rollbackToPrevious(), UpdateStatus::Ok);  // tester changes their mind
  const BootResult r = bootAndArm();
  EXPECT_EQ(r.event, BootEvent::Normal);
  EXPECT_EQ(r.header.version, (Version{1, 0, 0}));
  EXPECT_EQ(r.record.state[1], SlotState::Invalid);
}

TEST_F(FirmwareTest, AbortLeavesSlotUnbootable) {
  bootAndArm();
  const auto img = image({1, 1, 0});
  ASSERT_EQ(updater.eraseInactiveSlot(), UpdateStatus::Ok);
  ASSERT_EQ(updater.requestDownload(0U, static_cast<std::uint32_t>(img.size())), UpdateStatus::Ok);
  ASSERT_EQ(updater.transferData(1U, img.data(), kBlock), UpdateStatus::Ok);
  updater.abort();  // tester left the programming session
  EXPECT_EQ(updater.state(), UpdateState::Failed);
  EXPECT_EQ(updater.transferData(2U, img.data() + kBlock, kBlock), UpdateStatus::SequenceError);
  EXPECT_EQ(bootAndArm().header.version, (Version{1, 0, 0}));
}

}  // namespace
}  // namespace vecu::firmware
