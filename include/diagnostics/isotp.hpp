#pragma once

// ISO-TP transport protocol (ISO 15765-2) for Classic CAN, normal addressing.
//
// UDS messages can be up to 4095 bytes; a CAN frame carries 8. ISO-TP segments them:
//
//   payload <= 7 bytes : Single Frame       [0x0L | data...]                      (L = length)
//   payload >  7 bytes : First Frame        [0x1L LL | 6 data bytes]              (12-bit length)
//                        Consecutive Frames [0x2N | 7 data bytes] ...             (N = 1,2..15,0,1..)
//   receiver controls the pace with Flow Control [0x3S | BS | STmin]
//        S: 0 ContinueToSend, 1 Wait, 2 Overflow;  BS: CFs per block (0 = all);
//        STmin: minimum gap between CFs (0x00-0x7F ms, 0xF1-0xF9 = 100-900 us)
//
// Design: one endpoint = one (tx_id, rx_id) pair, full duplex (one transmission and one
// reception may be in progress at the same time). Pure state machine — time is passed in and
// frames go out through a callback — so every protocol rule is unit-tested deterministically.
// Buffers are fixed-size (no heap use). Not thread-safe: use from one task.
//
// Out of scope: extended/mixed addressing, CAN FD (escape sequence > 4095 bytes), N_As/N_Ar
// (transmit confirmation) timing.

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>

#include "can/can_frame.hpp"
#include "core/clock.hpp"

namespace vecu::diag {

inline constexpr std::size_t kIsoTpMaxPayload = 4095U;

struct IsoTpConfig {
  std::uint32_t tx_id{0U};
  std::uint32_t rx_id{0U};
  std::uint8_t block_size{0U};      ///< BS advertised in our flow control (0 = no limit)
  std::uint8_t st_min{0U};          ///< STmin advertised in our flow control (raw byte)
  std::chrono::milliseconds n_bs{1000};  ///< sender: max. wait for a flow control
  std::chrono::milliseconds n_cr{1000};  ///< receiver: max. wait for the next consecutive frame
  std::uint8_t max_wait_frames{10U};     ///< FC.Wait frames accepted before giving up
  std::uint8_t padding_byte{0xCCU};      ///< frames are always padded to 8 bytes
};

enum class IsoTpSendResult : std::uint8_t { Ok, Busy, InvalidLength, TxFailed };

enum class IsoTpError : std::uint8_t {
  None,
  TimeoutBs,            ///< no flow control within N_Bs
  TimeoutCr,            ///< no consecutive frame within N_Cr
  WrongSequenceNumber,  ///< CF with unexpected SN: reception aborted
  UnexpectedPdu,        ///< new message started while another was being received
  ReceiverOverflow,     ///< peer answered FC.Overflow
  WaitLimitExceeded,    ///< too many FC.Wait frames
  MessageTooLarge,      ///< incoming FF announced more than kIsoTpMaxPayload
  TxFailed              ///< the CAN driver refused a frame
};

[[nodiscard]] const char* toString(IsoTpError error) noexcept;

/// Decodes an STmin byte to a duration. Reserved values are treated as 127 ms (ISO 15765-2).
[[nodiscard]] std::chrono::microseconds decodeStMin(std::uint8_t raw) noexcept;

class IsoTpEndpoint {
 public:
  using FrameSender = std::function<bool(const can::CanFrame&)>;

  IsoTpEndpoint(const IsoTpConfig& config, FrameSender sender);

  /// Starts transmitting a message. Single frames go out immediately; longer messages send
  /// the First Frame and then wait for flow control.
  [[nodiscard]] IsoTpSendResult send(const std::uint8_t* data, std::size_t length, core::TimePoint now);

  /// Feeds a received CAN frame (frames with another identifier are ignored).
  void onFrame(const can::CanFrame& frame, core::TimePoint now);

  /// Sends due consecutive frames and checks timeouts. Call periodically (e.g. every 10 ms).
  void poll(core::TimePoint now);

  /// A completely received message is available until consumed.
  [[nodiscard]] bool hasMessage() const noexcept { return message_ready_; }
  [[nodiscard]] const std::uint8_t* message() const noexcept { return message_.data(); }
  [[nodiscard]] std::size_t messageLength() const noexcept { return message_length_; }
  void consumeMessage() noexcept { message_ready_ = false; }

  [[nodiscard]] bool txBusy() const noexcept { return tx_state_ != TxState::Idle; }
  [[nodiscard]] bool rxBusy() const noexcept { return rx_active_; }

  /// Most recent error (cleared by takeError()), and total error count.
  [[nodiscard]] IsoTpError takeError() noexcept;
  [[nodiscard]] std::uint32_t errorCount() const noexcept { return error_count_; }
  [[nodiscard]] std::uint32_t messagesReceived() const noexcept { return rx_messages_; }
  [[nodiscard]] std::uint32_t messagesSent() const noexcept { return tx_messages_; }

 private:
  enum class TxState : std::uint8_t { Idle, WaitFlowControl, SendConsecutive };

  bool sendFrame(const std::uint8_t* bytes, std::size_t length);
  void sendFlowControl(std::uint8_t flow_status);
  void handleSingleFrame(const can::CanFrame& frame);
  void handleFirstFrame(const can::CanFrame& frame, core::TimePoint now);
  void handleConsecutiveFrame(const can::CanFrame& frame, core::TimePoint now);
  void handleFlowControl(const can::CanFrame& frame, core::TimePoint now);
  void completeReception();
  void fail(IsoTpError error) noexcept;

  IsoTpConfig config_;
  FrameSender sender_;

  // transmit side
  TxState tx_state_{TxState::Idle};
  std::array<std::uint8_t, kIsoTpMaxPayload> tx_buffer_{};
  std::size_t tx_length_{0U};
  std::size_t tx_offset_{0U};
  std::uint8_t tx_sequence_{0U};
  std::uint8_t tx_block_size_{0U};
  std::uint8_t tx_block_count_{0U};
  std::uint8_t tx_wait_count_{0U};
  std::chrono::microseconds tx_st_min_{0};
  core::TimePoint tx_deadline_{};   ///< N_Bs deadline while waiting for FC
  core::TimePoint tx_next_cf_{};    ///< earliest time for the next CF (STmin)

  // receive side
  bool rx_active_{false};
  std::array<std::uint8_t, kIsoTpMaxPayload> rx_buffer_{};
  std::size_t rx_length_{0U};
  std::size_t rx_offset_{0U};
  std::uint8_t rx_sequence_{0U};
  std::uint8_t rx_block_count_{0U};
  core::TimePoint rx_deadline_{};   ///< N_Cr deadline

  // completed message
  std::array<std::uint8_t, kIsoTpMaxPayload> message_{};
  std::size_t message_length_{0U};
  bool message_ready_{false};

  IsoTpError last_error_{IsoTpError::None};
  std::uint32_t error_count_{0U};
  std::uint32_t rx_messages_{0U};
  std::uint32_t tx_messages_{0U};
};

}  // namespace vecu::diag
