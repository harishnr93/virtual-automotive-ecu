#include "firmware/firmware_image.hpp"

#include <algorithm>
#include <array>
#include <cstdio>

#include "core/crc32.hpp"
#include "firmware/flash_memory.hpp"

namespace vecu::firmware {

namespace {

constexpr std::size_t kHeaderCrcOffset = 60U;

void put(std::array<std::uint8_t, kImageHeaderSize>& out, std::size_t offset, std::uint64_t value, std::size_t bytes) {
  for (std::size_t i = 0U; i < bytes; ++i) {
    out[offset + i] = static_cast<std::uint8_t>((value >> (8U * i)) & 0xFFU);
  }
}

std::uint64_t get(const std::uint8_t* in, std::size_t offset, std::size_t bytes) {
  std::uint64_t value = 0U;
  for (std::size_t i = 0U; i < bytes; ++i) {
    value |= static_cast<std::uint64_t>(in[offset + i]) << (8U * i);
  }
  return value;
}

}  // namespace

std::string Version::toString() const {
  std::array<char, 16U> text{};
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
  (void)std::snprintf(text.data(), text.size(), "%02u.%02u.%02u", static_cast<unsigned int>(major),
                      static_cast<unsigned int>(minor), static_cast<unsigned int>(patch));
  return std::string{text.data()};
}

const char* toString(ImageError error) noexcept {
  switch (error) {
    case ImageError::None:
      return "None";
    case ImageError::TooSmall:
      return "TooSmall";
    case ImageError::BadMagic:
      return "BadMagic";
    case ImageError::UnsupportedHeaderVersion:
      return "UnsupportedHeaderVersion";
    case ImageError::HeaderCrcMismatch:
      return "HeaderCrcMismatch";
    case ImageError::PayloadSizeInvalid:
      return "PayloadSizeInvalid";
    case ImageError::PayloadCrcMismatch:
      return "PayloadCrcMismatch";
    case ImageError::WrongHardware:
      return "WrongHardware";
    case ImageError::VersionNotNewer:
      return "VersionNotNewer";
    case ImageError::SecurityVersionRollback:
      return "SecurityVersionRollback";
    case ImageError::UnsupportedVerification:
      return "UnsupportedVerification";
    case ImageError::ReadError:
      return "ReadError";
  }
  return "Unknown";
}

std::array<std::uint8_t, kImageHeaderSize> serializeHeader(const ImageHeader& header) {
  std::array<std::uint8_t, kImageHeaderSize> out{};
  std::copy(kImageMagic.begin(), kImageMagic.end(), out.begin());
  put(out, 4U, kImageHeaderVersion, 2U);
  put(out, 6U, kImageHeaderSize, 2U);
  put(out, 8U, header.hardware_id, 4U);
  out[12] = header.version.major;
  out[13] = header.version.minor;
  out[14] = header.version.patch;
  put(out, 16U, header.security_version, 4U);
  put(out, 20U, header.payload_size, 4U);
  put(out, 24U, header.payload_crc32, 4U);
  put(out, 28U, header.build_time, 8U);
  out[36] = static_cast<std::uint8_t>(header.verification_method);
  put(out, 38U, header.signature_size, 2U);
  put(out, 40U, header.flags, 4U);
  put(out, kHeaderCrcOffset, core::crc32(out.data(), kHeaderCrcOffset), 4U);
  return out;
}

ImageError parseHeader(const std::uint8_t* data, std::size_t length, ImageHeader& header) {
  if ((data == nullptr) || (length < kImageHeaderSize)) {
    return ImageError::TooSmall;
  }
  if (!std::equal(kImageMagic.begin(), kImageMagic.end(), data)) {
    return ImageError::BadMagic;
  }
  if ((get(data, 4U, 2U) != kImageHeaderVersion) || (get(data, 6U, 2U) != kImageHeaderSize)) {
    return ImageError::UnsupportedHeaderVersion;
  }
  if (core::crc32(data, kHeaderCrcOffset) != get(data, kHeaderCrcOffset, 4U)) {
    return ImageError::HeaderCrcMismatch;
  }
  ImageHeader parsed;
  parsed.hardware_id = static_cast<std::uint32_t>(get(data, 8U, 4U));
  parsed.version = Version{data[12], data[13], data[14]};
  parsed.security_version = static_cast<std::uint32_t>(get(data, 16U, 4U));
  parsed.payload_size = static_cast<std::uint32_t>(get(data, 20U, 4U));
  parsed.payload_crc32 = static_cast<std::uint32_t>(get(data, 24U, 4U));
  parsed.build_time = get(data, 28U, 8U);
  parsed.verification_method = static_cast<VerificationMethod>(data[36]);
  parsed.signature_size = static_cast<std::uint16_t>(get(data, 38U, 2U));
  parsed.flags = static_cast<std::uint32_t>(get(data, 40U, 4U));
  header = parsed;
  return ImageError::None;
}

std::vector<std::uint8_t> buildImage(ImageHeader header, const std::vector<std::uint8_t>& payload) {
  header.payload_size = static_cast<std::uint32_t>(payload.size());
  header.payload_crc32 = core::crc32(payload.data(), payload.size());
  header.signature_size = 0U;
  header.verification_method = VerificationMethod::Crc32;
  const auto bytes = serializeHeader(header);
  // Pre-sized copy (vector::insert of a range triggers a GCC 11 -Wstringop-overread false positive).
  std::vector<std::uint8_t> image(bytes.size() + payload.size());
  std::copy(bytes.begin(), bytes.end(), image.begin());
  std::copy(payload.begin(), payload.end(), image.begin() + static_cast<std::ptrdiff_t>(bytes.size()));
  return image;
}

std::vector<std::uint8_t> makePayload(std::size_t size, std::uint32_t seed) {
  std::vector<std::uint8_t> payload(size);
  std::uint32_t state = (seed == 0U) ? 0x9E3779B9U : seed;
  for (std::uint8_t& byte : payload) {  // xorshift32
    state ^= state << 13U;
    state ^= state >> 17U;
    state ^= state << 5U;
    byte = static_cast<std::uint8_t>(state & 0xFFU);
  }
  return payload;
}

ImageError checkCompatibility(const ImageHeader& header, const UpdatePolicy& policy) noexcept {
  if (header.hardware_id != policy.hardware_id) {
    return ImageError::WrongHardware;
  }
  if (header.security_version < policy.running_security_version) {
    return ImageError::SecurityVersionRollback;  // anti-rollback: never install a known-vulnerable image
  }
  if (!policy.allow_same_or_older_version && !(policy.running_version < header.version)) {
    return ImageError::VersionNotNewer;
  }
  return ImageError::None;
}

ImageError Crc32Verifier::verify(const ImageHeader& header, const IFlash& flash, std::uint32_t image_offset) const {
  if (header.verification_method != VerificationMethod::Crc32) {
    return ImageError::UnsupportedVerification;
  }
  std::array<std::uint8_t, 1024U> chunk{};
  std::uint32_t crc = core::crc32Init();
  std::uint32_t done = 0U;
  while (done < header.payload_size) {
    const std::uint32_t n = std::min<std::uint32_t>(static_cast<std::uint32_t>(chunk.size()), header.payload_size - done);
    if (flash.read(image_offset + static_cast<std::uint32_t>(kImageHeaderSize) + done, chunk.data(), n) !=
        FlashStatus::Ok) {
      return ImageError::ReadError;
    }
    crc = core::crc32Update(crc, chunk.data(), n);
    done += n;
  }
  return (core::crc32Final(crc) == header.payload_crc32) ? ImageError::None : ImageError::PayloadCrcMismatch;
}

ImageError verifyImageInFlash(const IFlash& flash, std::uint32_t offset, std::uint32_t max_size,
                              const IImageVerifier& verifier, ImageHeader& header) {
  std::array<std::uint8_t, kImageHeaderSize> bytes{};
  if (flash.read(offset, bytes.data(), bytes.size()) != FlashStatus::Ok) {
    return ImageError::ReadError;
  }
  ImageHeader parsed;
  const ImageError error = parseHeader(bytes.data(), bytes.size(), parsed);
  if (error != ImageError::None) {
    return error;
  }
  if ((parsed.payload_size == 0U) || (parsed.imageSize() > max_size)) {
    return ImageError::PayloadSizeInvalid;
  }
  if (parsed.verification_method != verifier.method()) {
    return ImageError::UnsupportedVerification;
  }
  const ImageError verified = verifier.verify(parsed, flash, offset);
  if (verified == ImageError::None) {
    header = parsed;
  }
  return verified;
}

}  // namespace vecu::firmware
