"""Python client for the UDP software CAN bus.

Byte-compatible with the C++ implementation (include/can/udp_wire_format.hpp).
Used by the debugging tools in scripts/ and, from Phase 3, by the vehicle simulator.
Standard library only.
"""

from __future__ import annotations

import random
import select
import socket
import struct
from dataclasses import dataclass, field
from typing import Optional, Tuple

MAGIC = b"VCAN"
VERSION = 1
FLAG_EXTENDED = 0x01
WIRE = struct.Struct("<4sBBBBII8s")  # magic, version, flags, dlc, reserved, sender, id, data
assert WIRE.size == 24

DEFAULT_GROUP = "239.255.0.1"
DEFAULT_PORT = 47000
MAX_STD_ID = 0x7FF
MAX_EXT_ID = 0x1FFFFFFF


@dataclass(frozen=True)
class CanFrame:
    can_id: int
    data: bytes = field(default=b"")
    extended: bool = False

    def __post_init__(self) -> None:
        limit = MAX_EXT_ID if self.extended else MAX_STD_ID
        if not 0 <= self.can_id <= limit:
            raise ValueError(f"CAN id 0x{self.can_id:X} out of range")
        if len(self.data) > 8:
            raise ValueError("Classic CAN payload is limited to 8 bytes")

    def candump(self) -> str:
        """can-utils notation, e.g. '123#DEADBEEF'."""
        width = 8 if self.extended else 3
        return f"{self.can_id:0{width}X}#{self.data.hex().upper()}"


def encode(frame: CanFrame, sender_id: int) -> bytes:
    flags = FLAG_EXTENDED if frame.extended else 0
    return WIRE.pack(MAGIC, VERSION, flags, len(frame.data), 0, sender_id, frame.can_id,
                     frame.data.ljust(8, b"\x00"))


def decode(datagram: bytes) -> Tuple[CanFrame, int]:
    """Returns (frame, sender_id); raises ValueError on malformed input."""
    if len(datagram) != WIRE.size:
        raise ValueError("bad datagram size")
    magic, version, flags, dlc, _reserved, sender, can_id, data = WIRE.unpack(datagram)
    if magic != MAGIC:
        raise ValueError("bad magic")
    if version != VERSION:
        raise ValueError("unsupported version")
    if flags & ~FLAG_EXTENDED or dlc > 8:
        raise ValueError("invalid frame")
    return CanFrame(can_id, data[:dlc], bool(flags & FLAG_EXTENDED)), sender


class UdpCanBus:
    """One node on the UDP software CAN bus (multicast on 127.0.0.1, TTL 0)."""

    def __init__(self, group: str = DEFAULT_GROUP, port: int = DEFAULT_PORT) -> None:
        self.group = group
        self.port = port
        self.sender_id = random.getrandbits(32) or 1

        self._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._rx.bind((group, port))
        membership = socket.inet_aton(group) + socket.inet_aton("127.0.0.1")
        self._rx.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)

        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._tx.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton("127.0.0.1"))
        self._tx.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        self._tx.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 0)

    def send(self, frame: CanFrame) -> None:
        self._tx.sendto(encode(frame, self.sender_id), (self.group, self.port))

    def recv(self, timeout: Optional[float] = None) -> Optional[CanFrame]:
        """Next frame from another node, or None on timeout."""
        while True:
            ready, _, _ = select.select([self._rx], [], [], timeout)
            if not ready:
                return None
            datagram = self._rx.recv(64)
            try:
                frame, sender = decode(datagram)
            except ValueError:
                continue
            if sender != self.sender_id:
                return frame

    def close(self) -> None:
        self._rx.close()
        self._tx.close()

    def __enter__(self) -> "UdpCanBus":
        return self

    def __exit__(self, *_exc) -> None:
        self.close()
