// ISO-TP endpoint: arbitrary CAN frame sequences (receive and flow control) and sends.
//
// Input: records of 10 bytes: [time step ms | flags][dlc][8 data bytes].
//   flags bit 7: start sending a message (length from the data bytes) instead of receiving
// Invariants: a reassembled message is 1..4095 bytes; every frame we transmit is valid.

#include <cstddef>
#include <cstdint>
#include <vector>

#include "can/can_frame.hpp"
#include "diagnostics/isotp.hpp"
#include "fuzz_check.hpp"

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size) {
  using namespace vecu;
  diag::IsoTpConfig config{0x7E8U, 0x7E0U};
  config.block_size = static_cast<std::uint8_t>(size % 4U);  // also exercise BS > 0
  diag::IsoTpEndpoint endpoint(config, [](const can::CanFrame& frame) {
    FUZZ_CHECK(can::isValid(frame) && frame.id == 0x7E8U);
    return true;
  });
  core::TimePoint now{};
  std::vector<std::uint8_t> payload(diag::kIsoTpMaxPayload, 0xA5U);
  for (std::size_t i = 0; i + 10U <= size; i += 10U) {
    now += std::chrono::milliseconds{data[i] & 0x7FU};
    if ((data[i] & 0x80U) != 0U) {
      const std::size_t length = (static_cast<std::size_t>(data[i + 2U]) << 8U | data[i + 3U]) % 5000U;
      (void)endpoint.send(payload.data(), length, now);  // invalid lengths must be rejected, not crash
    } else {
      can::CanFrame frame;
      frame.id = (data[i + 1U] & 0x80U) ? 0x7DFU : 0x7E0U;
      frame.dlc = static_cast<std::uint8_t>((data[i + 1U] & 0x0FU) % 9U);
      for (std::size_t b = 0; b < 8U; ++b) {
        frame.data[b] = data[i + 2U + b];
      }
      endpoint.onFrame(frame, now);
    }
    endpoint.poll(now);
    if (endpoint.hasMessage()) {
      FUZZ_CHECK(endpoint.messageLength() >= 1U && endpoint.messageLength() <= diag::kIsoTpMaxPayload);
      endpoint.consumeMessage();
    }
    (void)endpoint.takeError();
  }
  return 0;
}
