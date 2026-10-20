#pragma once

#include <cstdint>

namespace vecu::can {

/// 4-bit rolling ("alive") counter, as used in automotive end-to-end protection
/// (e.g. the counter part of AUTOSAR E2E profiles). The sender increments it each
/// transmission; the receiver detects repeated (stale/frozen sender) and skipped (lost) frames.
inline constexpr std::uint8_t kAliveCounterModulo = 16U;

class AliveCounterGenerator {
 public:
  /// Returns the current value and advances (0, 1, ..., 15, 0, ...).
  [[nodiscard]] std::uint8_t next() noexcept {
    const std::uint8_t value = value_;
    value_ = static_cast<std::uint8_t>((value_ + 1U) % kAliveCounterModulo);
    return value;
  }

 private:
  std::uint8_t value_{0U};
};

enum class AliveCheckResult : std::uint8_t {
  First,     ///< first value after start/reset (nothing to compare against)
  Ok,        ///< incremented by exactly one
  Repeated,  ///< same value as before: sender stuck or frame duplicated
  Skipped,   ///< jumped forward: frames lost (count added to lostFrames())
  Invalid    ///< value outside 0..15
};

/// Receiver-side check. Limitation: a 4-bit counter cannot detect the loss of exactly a
/// multiple of 16 consecutive frames; message timeout monitoring covers that case.
class AliveCounterChecker {
 public:
  [[nodiscard]] AliveCheckResult check(std::uint8_t counter) noexcept {
    if (counter >= kAliveCounterModulo) {
      return AliveCheckResult::Invalid;
    }
    if (!has_previous_) {
      has_previous_ = true;
      previous_ = counter;
      return AliveCheckResult::First;
    }
    const auto delta = static_cast<std::uint8_t>((counter + kAliveCounterModulo - previous_) % kAliveCounterModulo);
    previous_ = counter;
    if (delta == 1U) {
      return AliveCheckResult::Ok;
    }
    if (delta == 0U) {
      return AliveCheckResult::Repeated;
    }
    lost_frames_ += static_cast<std::uint32_t>(delta) - 1U;
    return AliveCheckResult::Skipped;
  }

  /// Re-synchronise (e.g. after a message timeout): the next value is accepted as First.
  void reset() noexcept { has_previous_ = false; }

  [[nodiscard]] std::uint32_t lostFrames() const noexcept { return lost_frames_; }

 private:
  bool has_previous_{false};
  std::uint8_t previous_{0U};
  std::uint32_t lost_frames_{0U};
};

[[nodiscard]] constexpr const char* toString(AliveCheckResult result) noexcept {
  switch (result) {
    case AliveCheckResult::First:
      return "First";
    case AliveCheckResult::Ok:
      return "Ok";
    case AliveCheckResult::Repeated:
      return "Repeated";
    case AliveCheckResult::Skipped:
      return "Skipped";
    case AliveCheckResult::Invalid:
      return "Invalid";
  }
  return "Unknown";
}

}  // namespace vecu::can
