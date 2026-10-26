// Firmware image format and simulated NOR flash.

#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdio>
#include <string>
#include <vector>

#include "firmware/firmware_image.hpp"
#include "firmware/flash_memory.hpp"

namespace vecu::firmware {
namespace {

constexpr std::uint32_t kHw = 0x0000CC01U;

ImageHeader header(Version version, std::uint32_t security = 1U, std::uint32_t hw = kHw) {
  ImageHeader h;
  h.hardware_id = hw;
  h.version = version;
  h.security_version = security;
  h.build_time = 1790000000U;
  return h;
}

// ------------------------------------------------------------------------- image ----

TEST(FirmwareImage, HeaderRoundTripAndVersionText) {
  const auto image = buildImage(header({1, 2, 3}, 7U), makePayload(1000U, 42U));
  ASSERT_EQ(image.size(), kImageHeaderSize + 1000U);
  ImageHeader parsed;
  ASSERT_EQ(parseHeader(image.data(), image.size(), parsed), ImageError::None);
  EXPECT_EQ(parsed.version, (Version{1, 2, 3}));
  EXPECT_EQ(parsed.version.toString(), "01.02.03");
  EXPECT_EQ(parsed.security_version, 7U);
  EXPECT_EQ(parsed.payload_size, 1000U);
  EXPECT_EQ(parsed.hardware_id, kHw);
  EXPECT_EQ(parsed.imageSize(), image.size());
}

TEST(FirmwareImage, EveryHeaderBitFlipIsDetected) {
  const auto good = buildImage(header({1, 0, 0}), makePayload(64U, 1U));
  for (std::size_t byte = 0U; byte < kImageHeaderSize; ++byte) {
    for (unsigned int bit = 0U; bit < 8U; ++bit) {
      auto bad = good;
      bad[byte] = static_cast<std::uint8_t>(bad[byte] ^ (1U << bit));
      ImageHeader parsed;
      EXPECT_NE(parseHeader(bad.data(), bad.size(), parsed), ImageError::None) << byte << ":" << bit;
    }
  }
}

TEST(FirmwareImage, RejectsMalformedHeaders) {
  auto image = buildImage(header({1, 0, 0}), makePayload(64U, 1U));
  ImageHeader parsed;
  EXPECT_EQ(parseHeader(image.data(), 63U, parsed), ImageError::TooSmall);
  image[0] = 'X';
  EXPECT_EQ(parseHeader(image.data(), image.size(), parsed), ImageError::BadMagic);
}

TEST(FirmwareImage, PayloadIsDeterministic) {
  EXPECT_EQ(makePayload(256U, 5U), makePayload(256U, 5U));
  EXPECT_NE(makePayload(256U, 5U), makePayload(256U, 6U));
}

TEST(FirmwareImage, CompatibilityRules) {
  UpdatePolicy policy{kHw, Version{1, 0, 0}, 3U, false};
  EXPECT_EQ(checkCompatibility(header({1, 1, 0}, 3U), policy), ImageError::None);
  EXPECT_EQ(checkCompatibility(header({1, 0, 0}, 3U), policy), ImageError::VersionNotNewer);  // same version
  EXPECT_EQ(checkCompatibility(header({0, 9, 0}, 3U), policy), ImageError::VersionNotNewer);  // downgrade
  EXPECT_EQ(checkCompatibility(header({2, 0, 0}, 2U), policy), ImageError::SecurityVersionRollback);
  EXPECT_EQ(checkCompatibility(header({2, 0, 0}, 3U, 0xBADU), policy), ImageError::WrongHardware);
  policy.allow_same_or_older_version = true;  // workshop re-flash of an older version ...
  EXPECT_EQ(checkCompatibility(header({0, 9, 0}, 3U), policy), ImageError::None);
  EXPECT_EQ(checkCompatibility(header({0, 9, 0}, 2U), policy), ImageError::SecurityVersionRollback);  // ...never below
}

TEST(FirmwareImage, VerifyInFlashDetectsPayloadCorruption) {
  RamFlash flash(64U * 1024U, 4096U);
  const auto image = buildImage(header({1, 0, 0}), makePayload(20000U, 9U));
  ASSERT_EQ(flash.program(0U, image.data(), image.size()), FlashStatus::Ok);
  Crc32Verifier crc;
  ImageHeader parsed;
  ASSERT_EQ(verifyImageInFlash(flash, 0U, 64U * 1024U, crc, parsed), ImageError::None);
  EXPECT_EQ(parsed.payload_size, 20000U);
  for (const std::uint32_t offset : {64U, 5000U, 20063U}) {  // first, middle, last payload byte
    flash.flipBit(offset, 3U);
    EXPECT_EQ(verifyImageInFlash(flash, 0U, 64U * 1024U, crc, parsed), ImageError::PayloadCrcMismatch) << offset;
    flash.flipBit(offset, 3U);
  }
  EXPECT_EQ(verifyImageInFlash(flash, 0U, 10000U, crc, parsed), ImageError::PayloadSizeInvalid);  // larger than slot
}

TEST(FirmwareImage, SignatureMethodIsReservedNotSilentlyAccepted) {
  ImageHeader h = header({1, 0, 0});
  h.payload_size = 16U;
  h.verification_method = VerificationMethod::Signature;
  h.signature_size = 64U;
  const auto bytes = serializeHeader(h);
  RamFlash flash(8192U, 4096U);
  ASSERT_EQ(flash.program(0U, bytes.data(), bytes.size()), FlashStatus::Ok);
  Crc32Verifier crc;
  ImageHeader parsed;
  EXPECT_EQ(verifyImageInFlash(flash, 0U, 8192U, crc, parsed), ImageError::UnsupportedVerification);
}

// ------------------------------------------------------------------------- flash ----

TEST(RamFlash, NorSemantics) {
  RamFlash flash(8192U, 4096U);
  std::uint8_t value = 0U;
  ASSERT_EQ(flash.read(100U, &value, 1U), FlashStatus::Ok);
  EXPECT_EQ(value, 0xFFU);  // erased state
  const std::uint8_t a = 0xA5U;
  ASSERT_EQ(flash.program(100U, &a, 1U), FlashStatus::Ok);
  const std::uint8_t b = 0x5AU;  // would need 0 -> 1 transitions
  EXPECT_EQ(flash.program(100U, &b, 1U), FlashStatus::NotErased);
  const std::uint8_t c = 0x21U;  // only clears bits: allowed
  EXPECT_EQ(flash.program(100U, &c, 1U), FlashStatus::Ok);
  ASSERT_EQ(flash.eraseSector(0U), FlashStatus::Ok);
  ASSERT_EQ(flash.read(100U, &value, 1U), FlashStatus::Ok);
  EXPECT_EQ(value, 0xFFU);
  EXPECT_EQ(flash.eraseCount(0U), 1U);
  EXPECT_EQ(flash.eraseSector(100U), FlashStatus::NotAligned);
  EXPECT_EQ(flash.program(8191U, &a, 2U), FlashStatus::OutOfRange);
  EXPECT_EQ(flash.eraseRange(0U, 8192U), FlashStatus::Ok);
}

TEST(RamFlash, PowerLossInterruptsProgrammingMidway) {
  RamFlash flash(4096U, 4096U);
  const std::vector<std::uint8_t> data(100U, 0x00U);
  flash.injectPowerLossAfter(40U);
  EXPECT_EQ(flash.program(0U, data.data(), data.size()), FlashStatus::PowerLoss);
  std::uint8_t v = 0U;
  EXPECT_EQ(flash.read(0U, &v, 1U), FlashStatus::PowerLoss);  // device off
  flash.powerCycle();
  ASSERT_EQ(flash.read(39U, &v, 1U), FlashStatus::Ok);
  EXPECT_EQ(v, 0x00U);  // written before the loss
  ASSERT_EQ(flash.read(40U, &v, 1U), FlashStatus::Ok);
  EXPECT_EQ(v, 0xFFU);  // never written
}

TEST(FileFlash, PersistsAcrossReopenLikeAPowerCycle) {
  const std::string path = ::testing::TempDir() + "vecu_flash_" + std::to_string(::getpid()) + ".bin";
  (void)std::remove(path.c_str());
  {
    FileFlash flash;
    ASSERT_EQ(FileFlash::open(path, 16384U, 4096U, flash), FlashStatus::Ok);
    std::uint8_t v = 0U;
    ASSERT_EQ(flash.read(5000U, &v, 1U), FlashStatus::Ok);
    EXPECT_EQ(v, 0xFFU);  // new device is erased
    const std::uint8_t data = 0x42U;
    ASSERT_EQ(flash.program(5000U, &data, 1U), FlashStatus::Ok);
    EXPECT_EQ(flash.program(5000U, std::vector<std::uint8_t>{0xFFU}.data(), 1U), FlashStatus::NotErased);
    ASSERT_EQ(flash.sync(), FlashStatus::Ok);
  }
  FileFlash reopened;
  ASSERT_EQ(FileFlash::open(path, 16384U, 4096U, reopened), FlashStatus::Ok);
  std::uint8_t v = 0U;
  ASSERT_EQ(reopened.read(5000U, &v, 1U), FlashStatus::Ok);
  EXPECT_EQ(v, 0x42U);
  (void)std::remove(path.c_str());
}

}  // namespace
}  // namespace vecu::firmware
