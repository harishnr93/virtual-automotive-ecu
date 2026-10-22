#include "diagnostics/isotp.hpp"

#include <algorithm>
#include <utility>

namespace vecu::diag {

namespace {

constexpr std::uint8_t kPciSingle = 0x0U;
constexpr std::uint8_t kPciFirst = 0x1U;
constexpr std::uint8_t kPciConsecutive = 0x2U;
constexpr std::uint8_t kPciFlowControl = 0x3U;

constexpr std::uint8_t kFlowContinue = 0x0U;
constexpr std::uint8_t kFlowWait = 0x1U;
constexpr std::uint8_t kFlowOverflow = 0x2U;

constexpr std::size_t kSingleFrameMax = 7U;
constexpr std::size_t kFirstFrameData = 6U;
constexpr std::size_t kConsecutiveData = 7U;

std::uint8_t pciType(const can::CanFrame& frame) noexcept {
  return static_cast<std::uint8_t>(frame.data[0] >> 4U);
}

}  // namespace

const char* toString(IsoTpError error) noexcept {
  switch (error) {
    case IsoTpError::None:
      return "None";
    case IsoTpError::TimeoutBs:
      return "TimeoutBs";
    case IsoTpError::TimeoutCr:
      return "TimeoutCr";
    case IsoTpError::WrongSequenceNumber:
      return "WrongSequenceNumber";
    case IsoTpError::UnexpectedPdu:
      return "UnexpectedPdu";
    case IsoTpError::ReceiverOverflow:
      return "ReceiverOverflow";
    case IsoTpError::WaitLimitExceeded:
      return "WaitLimitExceeded";
    case IsoTpError::MessageTooLarge:
      return "MessageTooLarge";
    case IsoTpError::TxFailed:
      return "TxFailed";
  }
  return "Unknown";
}

std::chrono::microseconds decodeStMin(std::uint8_t raw) noexcept {
  if (raw <= 0x7FU) {
    return std::chrono::milliseconds{raw};
  }
  if ((raw >= 0xF1U) && (raw <= 0xF9U)) {
    return std::chrono::microseconds{(raw - 0xF0U) * 100U};
  }
  return std::chrono::milliseconds{0x7F};  // reserved value: use the maximum
}

IsoTpEndpoint::IsoTpEndpoint(const IsoTpConfig& config, FrameSender sender)
    : config_(config), sender_(std::move(sender)) {}

IsoTpError IsoTpEndpoint::takeError() noexcept {
  const IsoTpError error = last_error_;
  last_error_ = IsoTpError::None;
  return error;
}

void IsoTpEndpoint::fail(IsoTpError error) noexcept {
  last_error_ = error;
  ++error_count_;
}

bool IsoTpEndpoint::sendFrame(const std::uint8_t* bytes, std::size_t length) {
  can::CanFrame frame;
  frame.id = config_.tx_id;
  frame.extended = config_.tx_id > can::kMaxStandardId;
  frame.dlc = can::kMaxClassicDlc;
  frame.data.fill(config_.padding_byte);
  std::copy_n(bytes, std::min(length, frame.data.size()), frame.data.begin());
  return sender_ && sender_(frame);
}

void IsoTpEndpoint::sendFlowControl(std::uint8_t flow_status) {
  const std::array<std::uint8_t, 3U> fc{static_cast<std::uint8_t>((kPciFlowControl << 4U) | flow_status),
                                        config_.block_size, config_.st_min};
  if (!sendFrame(fc.data(), fc.size())) {
    fail(IsoTpError::TxFailed);
  }
}

// ------------------------------------------------------------------------------ transmit ----

IsoTpSendResult IsoTpEndpoint::send(const std::uint8_t* data, std::size_t length, core::TimePoint now) {
  if (tx_state_ != TxState::Idle) {
    return IsoTpSendResult::Busy;
  }
  if ((data == nullptr) || (length == 0U) || (length > kIsoTpMaxPayload)) {
    return IsoTpSendResult::InvalidLength;
  }

  std::array<std::uint8_t, 8U> frame{};
  if (length <= kSingleFrameMax) {
    frame[0] = static_cast<std::uint8_t>((kPciSingle << 4U) | length);
    std::copy_n(data, length, frame.begin() + 1);
    if (!sendFrame(frame.data(), length + 1U)) {
      fail(IsoTpError::TxFailed);
      return IsoTpSendResult::TxFailed;
    }
    ++tx_messages_;
    return IsoTpSendResult::Ok;
  }

  std::copy_n(data, length, tx_buffer_.begin());
  tx_length_ = length;
  frame[0] = static_cast<std::uint8_t>((kPciFirst << 4U) | ((length >> 8U) & 0x0FU));
  frame[1] = static_cast<std::uint8_t>(length & 0xFFU);
  std::copy_n(data, kFirstFrameData, frame.begin() + 2);
  if (!sendFrame(frame.data(), frame.size())) {
    fail(IsoTpError::TxFailed);
    return IsoTpSendResult::TxFailed;
  }
  tx_offset_ = kFirstFrameData;
  tx_sequence_ = 1U;
  tx_wait_count_ = 0U;
  tx_state_ = TxState::WaitFlowControl;
  tx_deadline_ = now + config_.n_bs;
  return IsoTpSendResult::Ok;
}

void IsoTpEndpoint::handleFlowControl(const can::CanFrame& frame, core::TimePoint now) {
  if (tx_state_ != TxState::WaitFlowControl) {
    return;  // unexpected FC: ignored (ISO 15765-2)
  }
  if (frame.dlc < 3U) {
    return;
  }
  const auto flow_status = static_cast<std::uint8_t>(frame.data[0] & 0x0FU);
  if (flow_status == kFlowContinue) {
    tx_block_size_ = frame.data[1];
    tx_st_min_ = decodeStMin(frame.data[2]);
    tx_block_count_ = 0U;
    tx_state_ = TxState::SendConsecutive;
    tx_next_cf_ = now;  // first CF may follow immediately
    poll(now);
  } else if (flow_status == kFlowWait) {
    ++tx_wait_count_;
    if (tx_wait_count_ > config_.max_wait_frames) {
      tx_state_ = TxState::Idle;
      fail(IsoTpError::WaitLimitExceeded);
    } else {
      tx_deadline_ = now + config_.n_bs;  // restart N_Bs
    }
  } else if (flow_status == kFlowOverflow) {
    tx_state_ = TxState::Idle;
    fail(IsoTpError::ReceiverOverflow);
  } else {
    // reserved flow status: abort
    tx_state_ = TxState::Idle;
    fail(IsoTpError::UnexpectedPdu);
  }
}

void IsoTpEndpoint::poll(core::TimePoint now) {
  if (rx_active_ && (now >= rx_deadline_)) {
    rx_active_ = false;
    fail(IsoTpError::TimeoutCr);
  }
  if ((tx_state_ == TxState::WaitFlowControl) && (now >= tx_deadline_)) {
    tx_state_ = TxState::Idle;
    fail(IsoTpError::TimeoutBs);
  }
  // With STmin = 0 all frames of a block go out now; otherwise at most one per STmin.
  while ((tx_state_ == TxState::SendConsecutive) && (now >= tx_next_cf_)) {
    std::array<std::uint8_t, 8U> frame{};
    const std::size_t chunk = std::min(kConsecutiveData, tx_length_ - tx_offset_);
    frame[0] = static_cast<std::uint8_t>((kPciConsecutive << 4U) | tx_sequence_);
    std::copy_n(tx_buffer_.begin() + static_cast<std::ptrdiff_t>(tx_offset_), chunk, frame.begin() + 1);
    if (!sendFrame(frame.data(), chunk + 1U)) {
      tx_state_ = TxState::Idle;
      fail(IsoTpError::TxFailed);
      return;
    }
    tx_offset_ += chunk;
    tx_sequence_ = static_cast<std::uint8_t>((tx_sequence_ + 1U) & 0x0FU);
    ++tx_block_count_;
    if (tx_offset_ >= tx_length_) {
      tx_state_ = TxState::Idle;
      ++tx_messages_;
      return;
    }
    if ((tx_block_size_ != 0U) && (tx_block_count_ >= tx_block_size_)) {
      tx_state_ = TxState::WaitFlowControl;  // next block needs a new FC
      tx_deadline_ = now + config_.n_bs;
      return;
    }
    tx_next_cf_ = now + tx_st_min_;
    if (tx_st_min_.count() > 0) {
      return;  // time does not advance inside one poll()
    }
  }
}

// ------------------------------------------------------------------------------- receive ----

void IsoTpEndpoint::onFrame(const can::CanFrame& frame, core::TimePoint now) {
  if ((frame.id != config_.rx_id) || (frame.dlc == 0U)) {
    return;
  }
  switch (pciType(frame)) {
    case kPciSingle:
      handleSingleFrame(frame);
      break;
    case kPciFirst:
      handleFirstFrame(frame, now);
      break;
    case kPciConsecutive:
      handleConsecutiveFrame(frame, now);
      break;
    case kPciFlowControl:
      handleFlowControl(frame, now);
      break;
    default:
      break;  // reserved PCI: ignored
  }
}

void IsoTpEndpoint::handleSingleFrame(const can::CanFrame& frame) {
  const std::size_t length = frame.data[0] & 0x0FU;
  if ((length == 0U) || (length > kSingleFrameMax) || (length + 1U > frame.dlc)) {
    return;  // invalid SF_DL: ignored
  }
  if (rx_active_) {
    rx_active_ = false;
    fail(IsoTpError::UnexpectedPdu);  // the new message replaces the interrupted one
  }
  std::copy_n(frame.data.begin() + 1, length, rx_buffer_.begin());
  rx_length_ = length;
  completeReception();
}

void IsoTpEndpoint::handleFirstFrame(const can::CanFrame& frame, core::TimePoint now) {
  if (frame.dlc < 8U) {
    return;
  }
  const std::size_t length = (static_cast<std::size_t>(frame.data[0] & 0x0FU) << 8U) | frame.data[1];
  if (length <= kSingleFrameMax) {
    return;  // FF_DL must exceed a single frame: ignored
  }
  if (rx_active_) {
    fail(IsoTpError::UnexpectedPdu);
  }
  if (length > kIsoTpMaxPayload) {  // cannot happen with 12 bits, kept for smaller buffers
    rx_active_ = false;
    sendFlowControl(kFlowOverflow);
    fail(IsoTpError::MessageTooLarge);
    return;
  }
  std::copy_n(frame.data.begin() + 2, kFirstFrameData, rx_buffer_.begin());
  rx_length_ = length;
  rx_offset_ = kFirstFrameData;
  rx_sequence_ = 1U;
  rx_block_count_ = 0U;
  rx_active_ = true;
  rx_deadline_ = now + config_.n_cr;
  sendFlowControl(kFlowContinue);
}

void IsoTpEndpoint::handleConsecutiveFrame(const can::CanFrame& frame, core::TimePoint now) {
  if (!rx_active_) {
    return;  // unexpected CF: ignored
  }
  const auto sequence = static_cast<std::uint8_t>(frame.data[0] & 0x0FU);
  if (sequence != rx_sequence_) {
    rx_active_ = false;
    fail(IsoTpError::WrongSequenceNumber);
    return;
  }
  const std::size_t chunk = std::min(kConsecutiveData, rx_length_ - rx_offset_);
  if (frame.dlc < chunk + 1U) {
    rx_active_ = false;
    fail(IsoTpError::UnexpectedPdu);
    return;
  }
  std::copy_n(frame.data.begin() + 1, chunk, rx_buffer_.begin() + static_cast<std::ptrdiff_t>(rx_offset_));
  rx_offset_ += chunk;
  rx_sequence_ = static_cast<std::uint8_t>((rx_sequence_ + 1U) & 0x0FU);
  rx_deadline_ = now + config_.n_cr;

  if (rx_offset_ >= rx_length_) {
    rx_active_ = false;
    completeReception();
    return;
  }
  ++rx_block_count_;
  if ((config_.block_size != 0U) && (rx_block_count_ >= config_.block_size)) {
    rx_block_count_ = 0U;
    sendFlowControl(kFlowContinue);
  }
}

void IsoTpEndpoint::completeReception() {
  std::copy_n(rx_buffer_.begin(), rx_length_, message_.begin());
  message_length_ = rx_length_;
  message_ready_ = true;  // an unconsumed older message is overwritten (last-is-best)
  ++rx_messages_;
}

}  // namespace vecu::diag
