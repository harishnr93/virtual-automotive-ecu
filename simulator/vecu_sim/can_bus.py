"""CAN bus access for the Python side: Linux SocketCAN or the UDP software bus.

Both classes offer the same small interface (send / recv / close), mirroring the C++
ICanChannel abstraction, and open_bus() selects one like the C++ factory ("auto" tries
SocketCAN on vcan0 first, then falls back to the UDP bus).
"""

from __future__ import annotations

import pathlib
import select
import socket
import struct
from typing import Optional

from .udp_can_bus import DEFAULT_GROUP, DEFAULT_PORT, CanFrame, UdpCanBus

_CAN_FRAME = struct.Struct("=IB3x8s")  # struct can_frame: id, len, pad, data
_CAN_EFF_FLAG = 0x80000000
_CAN_RTR_FLAG = 0x40000000
_CAN_ERR_FLAG = 0x20000000
_CAN_EFF_MASK = 0x1FFFFFFF


class SocketCanBus:
    """Raw SocketCAN socket (e.g. on vcan0). Own frames are not received (kernel default)."""

    def __init__(self, interface: str = "vcan0") -> None:
        self.name = interface
        self._sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self._sock.bind((interface,))
        except OSError:
            self._sock.close()
            raise

    def send(self, frame: CanFrame) -> None:
        can_id = frame.can_id | (_CAN_EFF_FLAG if frame.extended else 0)
        self._sock.send(_CAN_FRAME.pack(can_id, len(frame.data), frame.data.ljust(8, b"\x00")))

    def recv(self, timeout: Optional[float] = None) -> Optional[CanFrame]:
        while True:
            ready, _, _ = select.select([self._sock], [], [], timeout)
            if not ready:
                return None
            can_id, length, data = _CAN_FRAME.unpack(self._sock.recv(_CAN_FRAME.size))
            if can_id & (_CAN_ERR_FLAG | _CAN_RTR_FLAG) or length > 8:
                continue  # error / remote frames are out of scope
            extended = bool(can_id & _CAN_EFF_FLAG)
            return CanFrame(can_id & (_CAN_EFF_MASK if extended else 0x7FF), data[:length], extended)

    def flush(self) -> int:
        """Discards all queued frames; returns how many."""
        discarded = 0
        while self.recv(timeout=0.0) is not None:
            discarded += 1
        return discarded

    def close(self) -> None:
        self._sock.close()

    def __enter__(self) -> "SocketCanBus":
        return self

    def __exit__(self, *_exc) -> None:
        self.close()


def open_bus(kind: str = "auto", interface: str = "vcan0", group: str = DEFAULT_GROUP,
             port: int = DEFAULT_PORT):
    """Returns (bus, description)."""
    if kind in ("auto", "socketcan"):
        if pathlib.Path(f"/sys/class/net/{interface}").exists():
            return SocketCanBus(interface), f"socketcan:{interface}"
        if kind == "socketcan":
            raise OSError(f"SocketCAN interface {interface} not available")
    if kind not in ("auto", "udp"):
        raise ValueError(f"unknown bus kind {kind!r}")
    bus = UdpCanBus(group, port)
    bus.name = "vbus0"
    return bus, f"udp:{group}:{port}"
