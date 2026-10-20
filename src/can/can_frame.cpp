#include "can/can_frame.hpp"

#include <cstddef>

namespace vecu::can {

namespace {

constexpr std::array<char, 16U> kHexDigits{'0', '1', '2', '3', '4', '5', '6', '7',
                                           '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

void appendHex(std::string& out, std::uint32_t value, unsigned int digits) {
  for (unsigned int i = digits; i > 0U; --i) {
    const unsigned int shift = (i - 1U) * 4U;
    out.push_back(kHexDigits[(value >> shift) & 0xFU]);
  }
}

}  // namespace

bool isValid(const CanFrame& frame) noexcept {
  const std::uint32_t max_id = frame.extended ? kMaxExtendedId : kMaxStandardId;
  return (frame.id <= max_id) && (frame.dlc <= kMaxClassicDlc);
}

bool operator==(const CanFrame& lhs, const CanFrame& rhs) noexcept {
  if ((lhs.id != rhs.id) || (lhs.extended != rhs.extended) || (lhs.dlc != rhs.dlc)) {
    return false;
  }
  const std::size_t length = (lhs.dlc <= kMaxClassicDlc) ? lhs.dlc : kMaxClassicDlc;
  for (std::size_t i = 0U; i < length; ++i) {
    if (lhs.data[i] != rhs.data[i]) {
      return false;
    }
  }
  return true;
}

std::string toCandumpString(const CanFrame& frame) {
  std::string out;
  out.reserve(28U);
  appendHex(out, frame.id, frame.extended ? 8U : 3U);
  out.push_back('#');
  const std::size_t length = (frame.dlc <= kMaxClassicDlc) ? frame.dlc : kMaxClassicDlc;
  for (std::size_t i = 0U; i < length; ++i) {
    appendHex(out, frame.data[i], 2U);
  }
  return out;
}

std::string formatId(std::uint32_t id) {
  std::string out{"0x"};
  appendHex(out, id, (id > kMaxStandardId) ? 8U : 3U);
  return out;
}

}  // namespace vecu::can
