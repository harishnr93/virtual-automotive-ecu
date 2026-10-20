"""Unit tests of the Python UDP CAN bus client (wire format must match C++)."""

import os

import pytest

from vecu_sim.udp_can_bus import CanFrame, UdpCanBus, decode, encode

# Same reference vector as tests/unit/test_udp_wire_format.cpp.
REFERENCE = bytes.fromhex(
    "5643414E"          # magic "VCAN"
    "01" "00" "03" "00"  # version, flags, dlc, reserved
    "D4C3B2A1"          # sender id 0xA1B2C3D4 (little-endian)
    "23010000"          # CAN id 0x123 (little-endian)
    "0102030000000000"  # data
)


def test_encode_matches_cpp_reference_vector():
    assert encode(CanFrame(0x123, bytes([1, 2, 3])), 0xA1B2C3D4) == REFERENCE


def test_decode_matches_cpp_reference_vector():
    frame, sender = decode(REFERENCE)
    assert frame == CanFrame(0x123, bytes([1, 2, 3]))
    assert sender == 0xA1B2C3D4


def test_round_trip_extended_frame():
    frame = CanFrame(0x18DAF110, bytes(range(8)), extended=True)
    assert decode(encode(frame, 7))[0] == frame


@pytest.mark.parametrize("mutate", [
    lambda b: b[:-1],                       # wrong size
    lambda b: b"X" + b[1:],                 # bad magic
    lambda b: b[:4] + b"\x02" + b[5:],      # bad version
    lambda b: b[:6] + b"\x09" + b[7:],      # dlc > 8
])
def test_decode_rejects_malformed(mutate):
    with pytest.raises(ValueError):
        decode(mutate(REFERENCE))


def test_frame_validation():
    with pytest.raises(ValueError):
        CanFrame(0x800)
    with pytest.raises(ValueError):
        CanFrame(0x100, bytes(9))
    assert CanFrame(0x123, b"\xDE\xAD").candump() == "123#DEAD"


def test_two_python_nodes_exchange_frames():
    port = 40000 + (os.getpid() % 20000)
    with UdpCanBus(port=port) as a, UdpCanBus(port=port) as b:
        a.send(CanFrame(0x321, b"\x01\x02"))
        assert b.recv(timeout=1.0) == CanFrame(0x321, b"\x01\x02")
        assert a.recv(timeout=0.05) is None  # own frame is not received
