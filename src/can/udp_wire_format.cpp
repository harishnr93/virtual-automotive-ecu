#include "can/udp_wire_format.hpp"

#include <algorithm>

namespace vecu::can::udp_wire {

namespace {

constexpr std::size_t kOffsetVersion = 4U;
constexpr std::size_t kOffsetFlags = 5U;
constexpr std::size_t kOffsetDlc = 6U;
constexpr std::size_t kOffsetSender = 8U;
constexpr std::size_t kOffsetId = 12U;
constexpr std::size_t kOffsetData = 16U;

void putLe32(Datagram& out, std::size_t offset, std::uint32_t value) noexcept {
  for (std::size_t i = 0U; i < 4U; ++i) {
    out[offset + i] = static_cast<std::uint8_t>((value >> (8U * i)) & 0xFFU);
  }
}

std::uint32_t getLe32(const std::uint8_t* buffer, std::size_t offset) noexcept {
  std::uint32_t value = 0U;
  for (std::size_t i = 0U; i < 4U; ++i) {
    value |= static_cast<std::uint32_t>(buffer[offset + i]) << (8U * i);
  }
  return value;
}

}  // namespace

void encode(const CanFrame& frame, std::uint32_t sender_id, Datagram& out) noexcept {
  out.fill(0U);
  std::copy(kMagic.begin(), kMagic.end(), out.begin());
  out[kOffsetVersion] = kVersion;
  out[kOffsetFlags] = frame.extended ? kFlagExtended : 0U;
  out[kOffsetDlc] = frame.dlc;
  putLe32(out, kOffsetSender, sender_id);
  putLe32(out, kOffsetId, frame.id);
  const std::size_t length = std::min<std::size_t>(frame.dlc, kMaxClassicDlc);
  for (std::size_t i = 0U; i < length; ++i) {
    out[kOffsetData + i] = frame.data[i];
  }
}

DecodeResult decode(const std::uint8_t* buffer, std::size_t length, CanFrame& frame,
                    std::uint32_t& sender_id) noexcept {
  if ((buffer == nullptr) || (length != kDatagramSize)) {
    return DecodeResult::BadSize;
  }
  if (!std::equal(kMagic.begin(), kMagic.end(), buffer)) {
    return DecodeResult::BadMagic;
  }
  if (buffer[kOffsetVersion] != kVersion) {
    return DecodeResult::BadVersion;
  }
  const std::uint8_t flags = buffer[kOffsetFlags];
  if ((flags & static_cast<std::uint8_t>(~kFlagExtended)) != 0U) {
    return DecodeResult::InvalidFrame;  // unknown flag bits
  }

  CanFrame decoded;
  decoded.extended = (flags & kFlagExtended) != 0U;
  decoded.dlc = buffer[kOffsetDlc];
  decoded.id = getLe32(buffer, kOffsetId);
  if (!isValid(decoded)) {
    return DecodeResult::InvalidFrame;
  }
  for (std::size_t i = 0U; i < decoded.dlc; ++i) {
    decoded.data[i] = buffer[kOffsetData + i];
  }
  frame = decoded;
  sender_id = getLe32(buffer, kOffsetSender);
  return DecodeResult::Ok;
}

}  // namespace vecu::can::udp_wire
