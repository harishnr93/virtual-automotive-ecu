"""Driver / environment scenarios (JSON) for the vehicle simulator.

Example:
{
  "name": "step_response",
  "duration_s": 25,
  "initial": {"speed_kph": 60, "accelerator_pct": "hold", "main_switch": true, "set_speed_kph": 80},
  "events": [
    {"t": 1.0, "set": true},                       # SET button pulse (200 ms)
    {"t": 1.1, "accelerator_pct": 0},              # driver releases the pedal
    {"t": 10.0, "set_speed_kph": 100},             # driver requests a new speed
    {"t": 12.0, "grade_pct": 4},                   # road goes uphill
    {"t": 15.0, "brake_pct": 30}, {"t": 16.0, "brake_pct": 0},
    {"t": 18.0, "cancel": true},                   # CANCEL button pulse
    {"t": 20.0, "fault": "mute_vehicle_dynamics", "duration_s": 0.5}
  ]
}

Levels (accelerator, brake, grade, set speed, main switch) hold until changed.
Buttons (set, cancel) are pulses of BUTTON_PULSE_S. Faults are active in [t, t + duration).
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Union

BUTTON_PULSE_S = 0.2
FAULT_TYPES = ("mute_vehicle_dynamics", "mute_driver_inputs", "invalid_speed")
_LEVELS = ("accelerator_pct", "brake_pct", "grade_pct", "set_speed_kph", "main_switch")


@dataclass(frozen=True)
class DriverState:
    accelerator_pct: float = 0.0
    brake_pct: float = 0.0
    grade_pct: float = 0.0
    set_speed_kph: float = 0.0
    main_switch: bool = False
    set_request: bool = False
    cancel_request: bool = False


@dataclass(frozen=True)
class FaultWindow:
    kind: str
    start_s: float
    end_s: float

    def active(self, t: float) -> bool:
        return self.start_s <= t < self.end_s


@dataclass
class Scenario:
    name: str
    duration_s: float
    initial_speed_kph: float = 0.0
    initial_accelerator: Union[float, str] = 0.0  # number or "hold" (road-load throttle)
    initial_main_switch: bool = False
    initial_set_speed_kph: float = 0.0
    initial_grade_pct: float = 0.0
    events: List[dict] = field(default_factory=list)
    description: str = ""

    def __post_init__(self) -> None:
        self.events = sorted(self.events, key=lambda e: float(e["t"]))
        for event in self.events:
            if "fault" in event and event["fault"] not in FAULT_TYPES:
                raise ValueError(f"unknown fault type {event['fault']!r}")
            known = set(_LEVELS) | {"t", "set", "cancel", "fault", "duration_s", "note"}
            if set(event) - known:
                raise ValueError(f"unknown event keys {sorted(set(event) - known)}")

    @classmethod
    def from_dict(cls, data: dict) -> "Scenario":
        initial = data.get("initial", {})
        return cls(name=data["name"], duration_s=float(data["duration_s"]),
                   initial_speed_kph=float(initial.get("speed_kph", 0.0)),
                   initial_accelerator=initial.get("accelerator_pct", 0.0),
                   initial_main_switch=bool(initial.get("main_switch", False)),
                   initial_set_speed_kph=float(initial.get("set_speed_kph", 0.0)),
                   initial_grade_pct=float(initial.get("grade_pct", 0.0)),
                   events=list(data.get("events", [])), description=data.get("description", ""))

    @classmethod
    def load(cls, path: Union[Path, str]) -> "Scenario":
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))

    def driver_state(self, t: float, hold_throttle_pct: float = 0.0) -> DriverState:
        accelerator = hold_throttle_pct if self.initial_accelerator == "hold" else float(self.initial_accelerator)
        levels = {"accelerator_pct": accelerator, "brake_pct": 0.0, "grade_pct": self.initial_grade_pct,
                  "set_speed_kph": self.initial_set_speed_kph, "main_switch": self.initial_main_switch}
        set_request = cancel_request = False
        for event in self.events:
            start = float(event["t"])
            if start > t:
                break
            for key in _LEVELS:
                if key in event:
                    levels[key] = event[key]
            pulse = start <= t < start + BUTTON_PULSE_S
            set_request |= bool(event.get("set")) and pulse
            cancel_request |= bool(event.get("cancel")) and pulse
        return DriverState(float(levels["accelerator_pct"]), float(levels["brake_pct"]), float(levels["grade_pct"]),
                           float(levels["set_speed_kph"]), bool(levels["main_switch"]), set_request, cancel_request)

    def faults(self) -> List[FaultWindow]:
        return [FaultWindow(e["fault"], float(e["t"]), float(e["t"]) + float(e.get("duration_s", 0.0)))
                for e in self.events if "fault" in e]

    def active_fault(self, t: float, kind: str) -> bool:
        return any(w.kind == kind and w.active(t) for w in self.faults())

    def first_event_time(self, key: str, value: Optional[object] = None) -> Optional[float]:
        for event in self.events:
            if key in event and (value is None or event[key] == value):
                return float(event["t"])
        return None
