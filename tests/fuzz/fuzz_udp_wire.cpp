// UDP software-bus datagrams (what any process on the host can send to the bus) and the
// signal decoder on the resulting frames.
// Invariants: an accepted datagram is a valid CAN frame and survives encode -> decode unchanged.

#include <cstddef>
#include <cstdint>

#include "can/can_signal.hpp"
#include "can/udp_wire_format.hpp"
#include "can/vehicle_signals.hpp"
#include "fuzz_check.hpp"

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size) {
  using namespace vecu::can;
  CanFrame frame;
  std::uint32_t sender = 0U;
  if (udp_wire::decode(data, size, frame, sender) != udp_wire::DecodeResult::Ok) {
    return 0;
  }
  FUZZ_CHECK(isValid(frame));
  udp_wire::Datagram again{};
  udp_wire::encode(frame, sender, again);
  CanFrame decoded;
  std::uint32_t sender2 = 0U;
  FUZZ_CHECK(udp_wire::decode(again.data(), again.size(), decoded, sender2) == udp_wire::DecodeResult::Ok);
  FUZZ_CHECK(decoded.id == frame.id && decoded.dlc == frame.dlc && decoded.extended == frame.extended &&
             sender2 == sender);
  for (std::size_t b = 0; b < frame.dlc; ++b) {
    FUZZ_CHECK(decoded.data[b] == frame.data[b]);
  }
  double value = 0.0;
  for (const SignalDef& signal : vehicle::kVehicleDynamicsSignals) {
    (void)decodePhysical(signal, frame, value);
  }
  for (const SignalDef& signal : vehicle::kDriverInputsSignals) {
    (void)decodePhysical(signal, frame, value);
  }
  return 0;
}
