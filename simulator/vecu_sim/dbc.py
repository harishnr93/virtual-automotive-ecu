"""Minimal DBC reader and signal codec (standard library only).

Supports the subset used by config/vehicle_can.dbc: BO_, SG_ (Intel and Motorola, signed and
unsigned, factor/offset/range), BA_ GenMsgCycleTime / VECU_RxTimeoutMs and VAL_ tables.
Packing follows the same rules as the C++ codec (src/can/can_signal.cpp):
values are clamped to [min, max], rounded to the nearest raw step and clamped to the raw range.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, Mapping, Optional

_BO = re.compile(r"^BO_\s+(\d+)\s+(\w+)\s*:\s*(\d+)\s+(\w+)")
_SG = re.compile(r"^\s+SG_\s+(\w+)\s*:\s*(\d+)\|(\d+)@([01])([+-])\s*\(([^,]+),([^)]+)\)\s*"
                 r"\[([^|]+)\|([^\]]+)\]\s*\"([^\"]*)\"")
_BA = re.compile(r'^BA_\s+"(\w+)"\s+BO_\s+(\d+)\s+(\d+)\s*;')
_VAL = re.compile(r"^VAL_\s+(\d+)\s+(\w+)\s+(.*);")
_VAL_PAIR = re.compile(r'(-?\d+)\s+"([^"]*)"')


def _next_motorola_bit(position: int) -> int:
    return position + 15 if position % 8 == 0 else position - 1


@dataclass(frozen=True)
class Signal:
    name: str
    start_bit: int
    length: int
    little_endian: bool
    signed: bool
    factor: float
    offset: float
    minimum: float
    maximum: float
    unit: str = ""
    choices: Mapping[int, str] = field(default_factory=dict)

    def _positions(self):
        """Frame bit positions from the raw value's MSB to LSB."""
        if self.little_endian:
            return [self.start_bit + i for i in reversed(range(self.length))]
        positions, position = [], self.start_bit
        for _ in range(self.length):
            positions.append(position)
            position = _next_motorola_bit(position)
        return positions

    def raw_limits(self):
        if self.signed:
            half = 1 << (self.length - 1)
            return -half, half - 1
        return 0, (1 << self.length) - 1

    def insert_raw(self, data: bytearray, raw: int) -> None:
        raw &= (1 << self.length) - 1
        for i, position in enumerate(self._positions()):
            bit = (raw >> (self.length - 1 - i)) & 1
            if position // 8 >= len(data):
                raise ValueError(f"signal {self.name} does not fit into {len(data)} bytes")
            if bit:
                data[position // 8] |= 1 << (position % 8)
            else:
                data[position // 8] &= ~(1 << (position % 8)) & 0xFF

    def extract_raw(self, data: bytes) -> int:
        raw = 0
        for position in self._positions():
            raw = (raw << 1) | ((data[position // 8] >> (position % 8)) & 1)
        if self.signed and raw & (1 << (self.length - 1)):
            raw -= 1 << self.length
        return raw

    def to_raw(self, value: float) -> int:
        if not math.isfinite(value):
            raise ValueError(f"non-finite value for {self.name}")
        value = min(max(value, self.minimum), self.maximum)
        raw = round((value - self.offset) / self.factor)
        low, high = self.raw_limits()
        return min(max(raw, low), high)

    def to_physical(self, raw: int) -> float:
        return raw * self.factor + self.offset


@dataclass
class Message:
    frame_id: int
    name: str
    dlc: int
    transmitter: str
    signals: Dict[str, Signal] = field(default_factory=dict)
    cycle_time_ms: Optional[int] = None
    timeout_ms: Optional[int] = None

    def encode(self, values: Mapping[str, float]) -> bytes:
        """Packs physical values; signals not given are encoded as raw 0."""
        unknown = set(values) - set(self.signals)
        if unknown:
            raise KeyError(f"unknown signals for {self.name}: {sorted(unknown)}")
        data = bytearray(self.dlc)
        for name, signal in self.signals.items():
            if name in values:
                signal.insert_raw(data, signal.to_raw(float(values[name])))
        return bytes(data)

    def decode(self, data: bytes) -> Dict[str, float]:
        if len(data) != self.dlc:
            raise ValueError(f"{self.name}: expected {self.dlc} bytes, got {len(data)}")
        return {name: signal.to_physical(signal.extract_raw(data)) for name, signal in self.signals.items()}


class Database:
    def __init__(self, messages: Iterable[Message]) -> None:
        self.messages = {m.frame_id: m for m in messages}
        self._by_name = {m.name: m for m in self.messages.values()}

    def __getitem__(self, name: str) -> Message:
        return self._by_name[name]

    def by_id(self, frame_id: int) -> Optional[Message]:
        return self.messages.get(frame_id)

    @classmethod
    def from_text(cls, text: str) -> "Database":
        messages: Dict[int, Message] = {}
        current: Optional[Message] = None
        choices: Dict[tuple, Dict[int, str]] = {}
        for line in text.splitlines():
            if m := _BO.match(line):
                current = Message(int(m[1]), m[2], int(m[3]), m[4])
                messages[current.frame_id] = current
            elif current is not None and (m := _SG.match(line)):
                current.signals[m[1]] = Signal(m[1], int(m[2]), int(m[3]), m[4] == "1", m[5] == "-",
                                               float(m[6]), float(m[7]), float(m[8]), float(m[9]), m[10])
            elif m := _BA.match(line):
                message = messages.get(int(m[2]))
                if message is not None and m[1] == "GenMsgCycleTime":
                    message.cycle_time_ms = int(m[3])
                elif message is not None and m[1] == "VECU_RxTimeoutMs":
                    message.timeout_ms = int(m[3])
            elif m := _VAL.match(line):
                choices[(int(m[1]), m[2])] = {int(k): v for k, v in _VAL_PAIR.findall(m[3])}
            elif not line.strip():
                current = None
        for (frame_id, name), table in choices.items():
            message = messages.get(frame_id)
            if message is not None and name in message.signals:
                old = message.signals[name]
                message.signals[name] = Signal(**{**old.__dict__, "choices": table})
        return cls(messages.values())

    @classmethod
    def load(cls, path: Path | str) -> "Database":
        return cls.from_text(Path(path).read_text(encoding="utf-8"))


def default_dbc_path() -> Path:
    return Path(__file__).resolve().parents[2] / "config" / "vehicle_can.dbc"
