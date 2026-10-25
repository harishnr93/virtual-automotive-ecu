"""ISO-TP (ISO 15765-2) for the Python tester — independent of the C++ implementation.

Blocking, tester-oriented API on top of a bus with send()/recv() (UdpCanBus or SocketCanBus).
Classic CAN, normal addressing, frames padded to 8 bytes. Frames with other identifiers
(e.g. normal vehicle traffic on the same bus) are ignored.
"""

from __future__ import annotations

import time
from typing import Optional

from .udp_can_bus import CanFrame

MAX_PAYLOAD = 4095


class IsoTpError(Exception):
    """Transport-level failure (timeout, wrong sequence number, overflow ...)."""


def decode_st_min(raw: int) -> float:
    """STmin byte -> seconds (reserved values are treated as 127 ms)."""
    if raw <= 0x7F:
        return raw / 1000.0
    if 0xF1 <= raw <= 0xF9:
        return (raw - 0xF0) * 100e-6
    return 0.127


class IsoTpEndpoint:
    def __init__(self, bus, tx_id: int = 0x7E0, rx_id: int = 0x7E8, block_size: int = 0, st_min: int = 0,
                 padding: int = 0xCC, n_bs: float = 1.0, n_cr: float = 1.0, max_wait_frames: int = 10) -> None:
        self.bus = bus
        self.tx_id = tx_id
        self.rx_id = rx_id
        self.block_size = block_size
        self.st_min = st_min
        self.padding = padding
        self.n_bs = n_bs
        self.n_cr = n_cr
        self.max_wait_frames = max_wait_frames

    def flush(self) -> int:
        """Discards stale queued frames (the bus carries other traffic while the tester is idle)."""
        flush = getattr(self.bus, "flush", None)
        return flush() if flush else 0

    # ------------------------------------------------------------------------- frames ----
    def _send_frame(self, data: bytes, can_id: Optional[int] = None) -> None:
        self.bus.send(CanFrame(self.tx_id if can_id is None else can_id, bytes(data).ljust(8, bytes([self.padding]))))

    def _next_frame(self, deadline: float) -> Optional[CanFrame]:
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            frame = self.bus.recv(timeout=remaining)
            if frame is not None and frame.can_id == self.rx_id and frame.data:
                return frame

    def _send_flow_control(self, status: int = 0) -> None:
        self._send_frame(bytes([0x30 | status, self.block_size, self.st_min]))

    # ----------------------------------------------------------------------- transmit ----
    def send(self, payload: bytes, functional_id: Optional[int] = None) -> None:
        payload = bytes(payload)
        if not 0 < len(payload) <= MAX_PAYLOAD:
            raise ValueError(f"invalid ISO-TP payload length {len(payload)}")
        if len(payload) <= 7:
            self._send_frame(bytes([len(payload)]) + payload, functional_id)
            return
        if functional_id is not None:
            raise ValueError("functional requests must fit into a single frame")

        self._send_frame(bytes([0x10 | (len(payload) >> 8), len(payload) & 0xFF]) + payload[:6])
        offset, sequence = 6, 1
        while offset < len(payload):
            block_size, st_min = self._await_flow_control()
            sent_in_block = 0
            while offset < len(payload) and (block_size == 0 or sent_in_block < block_size):
                if sent_in_block or st_min:
                    time.sleep(st_min)
                self._send_frame(bytes([0x20 | sequence]) + payload[offset:offset + 7])
                offset += 7
                sequence = (sequence + 1) & 0x0F
                sent_in_block += 1

    def _await_flow_control(self):
        waits = 0
        deadline = time.monotonic() + self.n_bs
        while True:
            frame = self._next_frame(deadline)
            if frame is None:
                raise IsoTpError("N_Bs timeout: no flow control")
            pci, status = frame.data[0] >> 4, frame.data[0] & 0x0F
            if pci != 0x3:
                continue
            if status == 0:
                return frame.data[1], decode_st_min(frame.data[2])
            if status == 1:
                waits += 1
                if waits > self.max_wait_frames:
                    raise IsoTpError("too many FC.Wait frames")
                deadline = time.monotonic() + self.n_bs
                continue
            raise IsoTpError("receiver reported overflow" if status == 2 else f"invalid flow status {status}")

    # ------------------------------------------------------------------------ receive ----
    def recv(self, timeout: float) -> Optional[bytes]:
        """Returns the next complete message, or None if nothing started within `timeout`."""
        deadline = time.monotonic() + timeout
        while True:
            frame = self._next_frame(deadline)
            if frame is None:
                return None
            pci = frame.data[0] >> 4
            if pci == 0x0:
                length = frame.data[0] & 0x0F
                if 0 < length <= 7:
                    return bytes(frame.data[1:1 + length])
            elif pci == 0x1:
                return self._receive_multi_frame(frame)
            # CF / FC without context: ignored

    def _receive_multi_frame(self, first: CanFrame) -> bytes:
        length = ((first.data[0] & 0x0F) << 8) | first.data[1]
        if length <= 7:
            raise IsoTpError(f"invalid first frame length {length}")
        data = bytearray(first.data[2:8])
        self._send_flow_control(0)
        sequence, in_block = 1, 0
        while len(data) < length:
            frame = self._next_frame(time.monotonic() + self.n_cr)
            if frame is None:
                raise IsoTpError("N_Cr timeout: consecutive frame missing")
            pci = frame.data[0] >> 4
            if pci == 0x0 or pci == 0x1:
                raise IsoTpError("unexpected new message during reception")
            if pci != 0x2:
                continue
            if frame.data[0] & 0x0F != sequence:
                raise IsoTpError(f"wrong sequence number {frame.data[0] & 0x0F}, expected {sequence}")
            data += frame.data[1:1 + min(7, length - len(data))]
            sequence = (sequence + 1) & 0x0F
            in_block += 1
            if self.block_size and in_block == self.block_size and len(data) < length:
                in_block = 0
                self._send_flow_control(0)
        return bytes(data)
