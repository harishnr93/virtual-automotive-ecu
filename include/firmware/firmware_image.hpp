#pragma once

// Firmware image format of the virtual ECU (simulated: the payload is data, not executable code).
//
//   [ header 64 bytes ][ payload (payload_size bytes) ][ signature (signature_size bytes) ]
//
// Header layout (little-endian):
//    0  "VFWI" magic                    4      28  build time (Unix s)      8
//    4  header version (1)              2      36  verification method      1
//    6  header size (64)                2      37  reserved                 1
//    8  hardware id                     4      38  signature size           2
//   12  version major/minor/patch, 0    4      40  flags                    4
//   16  security version                4      44  reserved (zero)         16
//   20  payload size                    4      60  header CRC-32 (0..59)    4
//   24  payload CRC-32                  4
//
// Integrity vs. authenticity: CRC-32 detects ACCIDENTAL corruption (transfer errors, bit flips,
// truncated or torn writes). It does NOT authenticate the image: anyone can modify the payload
// and recompute a matching CRC. Authenticity needs a cryptographic signature checked against a
// key stored in the ECU (e.g. Ed25519/ECDSA). The header reserves `verification_method` and
// `signature_size`, and verification goes through the IImageVerifier interface, so a signature
// verifier can be added without changing the format.

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace vecu::firmware {

class IFlash;

inline constexpr std::array<std::uint8_t, 4U> kImageMagic{'V', 'F', 'W', 'I'};
inline constexpr std::uint16_t kImageHeaderVersion = 1U;
inline constexpr std::size_t kImageHeaderSize = 64U;

struct Version {
  std::uint8_t major{0U};
  std::uint8_t minor{0U};
  std::uint8_t patch{0U};

  [[nodiscard]] constexpr std::uint32_t value() const noexcept {
    return (static_cast<std::uint32_t>(major) << 16U) | (static_cast<std::uint32_t>(minor) << 8U) | patch;
  }
  [[nodiscard]] std::string toString() const;  ///< "MM.mm.pp", e.g. "01.02.03"
};

[[nodiscard]] constexpr bool operator<(const Version& a, const Version& b) noexcept { return a.value() < b.value(); }
[[nodiscard]] constexpr bool operator==(const Version& a, const Version& b) noexcept { return a.value() == b.value(); }

enum class VerificationMethod : std::uint8_t {
  Crc32 = 0U,     ///< integrity only (this project)
  Signature = 1U  ///< reserved for a cryptographic signature (not implemented)
};

namespace image_flags {
/// TEST ONLY: the image simulates a software defect (the application hangs after boot), used
/// to demonstrate trial boot and automatic rollback.
inline constexpr std::uint32_t kSimulatedDefectHang = 0x00000001U;
}  // namespace image_flags

struct ImageHeader {
  std::uint32_t hardware_id{0U};
  Version version{};
  std::uint32_t security_version{0U};  ///< anti-rollback counter: never install a lower value
  std::uint32_t payload_size{0U};
  std::uint32_t payload_crc32{0U};
  std::uint64_t build_time{0U};
  VerificationMethod verification_method{VerificationMethod::Crc32};
  std::uint16_t signature_size{0U};
  std::uint32_t flags{0U};

  [[nodiscard]] std::uint32_t imageSize() const noexcept {
    return static_cast<std::uint32_t>(kImageHeaderSize) + payload_size + signature_size;
  }
};

enum class ImageError : std::uint8_t {
  None = 0U,
  TooSmall = 1U,
  BadMagic = 2U,
  UnsupportedHeaderVersion = 3U,
  HeaderCrcMismatch = 4U,
  PayloadSizeInvalid = 5U,
  PayloadCrcMismatch = 6U,
  WrongHardware = 7U,
  VersionNotNewer = 8U,
  SecurityVersionRollback = 9U,
  UnsupportedVerification = 10U,
  ReadError = 11U
};

[[nodiscard]] const char* toString(ImageError error) noexcept;

/// Serialises a header (computes the header CRC).
[[nodiscard]] std::array<std::uint8_t, kImageHeaderSize> serializeHeader(const ImageHeader& header);

/// Parses and checks magic, header version/size and header CRC.
[[nodiscard]] ImageError parseHeader(const std::uint8_t* data, std::size_t length, ImageHeader& header);

/// Builds a complete image (header with payload size/CRC filled in + payload). Used for the
/// factory image and tests; the Python tool scripts/fwtool.py implements the same format.
[[nodiscard]] std::vector<std::uint8_t> buildImage(ImageHeader header, const std::vector<std::uint8_t>& payload);

/// Deterministic pseudo-random payload ("simulated machine code").
[[nodiscard]] std::vector<std::uint8_t> makePayload(std::size_t size, std::uint32_t seed);

/// What a new image must satisfy to be installed.
struct UpdatePolicy {
  std::uint32_t hardware_id{0U};
  Version running_version{};
  std::uint32_t running_security_version{0U};
  bool allow_same_or_older_version{false};  ///< e.g. for workshop re-flash; anti-rollback still applies
};

[[nodiscard]] ImageError checkCompatibility(const ImageHeader& header, const UpdatePolicy& policy) noexcept;

/// Strategy for the integrity/authenticity check of an image stored in flash.
class IImageVerifier {
 public:
  virtual ~IImageVerifier() = default;
  [[nodiscard]] virtual VerificationMethod method() const noexcept = 0;
  /// Verifies the image whose header was already parsed; `image_offset` is its flash address.
  [[nodiscard]] virtual ImageError verify(const ImageHeader& header, const IFlash& flash,
                                          std::uint32_t image_offset) const = 0;

 protected:
  IImageVerifier() = default;
  IImageVerifier(const IImageVerifier&) = default;
  IImageVerifier& operator=(const IImageVerifier&) = default;
  IImageVerifier(IImageVerifier&&) = default;
  IImageVerifier& operator=(IImageVerifier&&) = default;
};

/// CRC-32 over the payload (integrity only — see the note at the top of this file).
class Crc32Verifier final : public IImageVerifier {
 public:
  [[nodiscard]] VerificationMethod method() const noexcept override { return VerificationMethod::Crc32; }
  [[nodiscard]] ImageError verify(const ImageHeader& header, const IFlash& flash,
                                  std::uint32_t image_offset) const override;
};

/// Reads the header at `offset`, checks it, and verifies the image with `verifier`.
/// `max_size` limits the image to its flash slot.
[[nodiscard]] ImageError verifyImageInFlash(const IFlash& flash, std::uint32_t offset, std::uint32_t max_size,
                                            const IImageVerifier& verifier, ImageHeader& header);

}  // namespace vecu::firmware
