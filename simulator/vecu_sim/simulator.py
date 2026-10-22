"""Vehicle simulator node: closes the loop with the cruise control ECU over CAN.

    scenario (driver, road, faults) ──► VehicleModel ──► 0x100 VehicleDynamics (20 ms)
                                   └──────────────────► 0x110 DriverInputs   (50 ms)
    0x200 CruiseControlStatus ◄─── cruise ECU (C++)

Actuator-side arbitration (what a powertrain ECU would do):
    applied throttle = max(driver pedal, cruise command)  — cruise command only if the ECU
    reports Active, the command is fresh (< 100 ms old) and the brake is not pressed.

Run:  python3 -m vecu_sim.simulator --scenario config/scenarios/cruise_step_response.json
(with PYTHONPATH=simulator, or via scripts/run_closed_loop.sh)
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

from . import analysis, svg_plot
from .can_bus import open_bus
from .dbc import Database, default_dbc_path
from .scenario import Scenario
from .udp_can_bus import DEFAULT_GROUP, DEFAULT_PORT, CanFrame
from .vehicle_model import VehicleModel, VehicleParams

STEP_S = 0.01
COMMAND_MAX_AGE_S = 0.1
STATE_ACTIVE = 2
STATE_NAMES = {0: "Off", 1: "Standby", 2: "Active", 3: "Override", 4: "Fault"}


@dataclass
class CruiseStatus:
    throttle_pct: float = 0.0
    state: int = 0
    target_kph: float = 0.0
    fault_reason: int = 0
    rx_time_s: float = -1.0
    frames: int = 0


class VehicleSimulator:
    """Simulation logic without I/O timing: call step() every 10 ms of simulated time."""

    def __init__(self, scenario: Scenario, db: Database, params: VehicleParams = VehicleParams()) -> None:
        self.scenario = scenario
        self.db = db
        self.vehicle = VehicleModel(params, scenario.initial_speed_kph)
        self.hold_throttle_pct = self.vehicle.road_load_throttle_pct(scenario.initial_speed_kph)
        self.vehicle.settle_drive_force(self.driver(0.0).accelerator_pct)
        self.msg_dynamics = db["VehicleDynamics"]
        self.msg_driver = db["DriverInputs"]
        self.msg_status = db["CruiseControlStatus"]
        self.dynamics_every = int(round(self.msg_dynamics.cycle_time_ms / 1000.0 / STEP_S))
        self.driver_every = int(round(self.msg_driver.cycle_time_ms / 1000.0 / STEP_S))
        self.status = CruiseStatus()
        self.step_index = 0
        self.alive = {"dynamics": 0, "driver": 0}
        self.records: List[Dict[str, float]] = []

    @property
    def t(self) -> float:
        return self.step_index * STEP_S

    def driver(self, t: float):
        return self.scenario.driver_state(t, self.hold_throttle_pct)

    def on_frame(self, frame: CanFrame, t: float) -> None:
        if frame.can_id != self.msg_status.frame_id or len(frame.data) != self.msg_status.dlc:
            return
        values = self.msg_status.decode(frame.data)
        self.status = CruiseStatus(values["ThrottleCommand"], int(values["CruiseState"]),
                                   values["CruiseTargetSpeed"], int(values["CruiseFaultReason"]), t,
                                   self.status.frames + 1)

    def cruise_command_pct(self, t: float, brake_pressed: bool) -> float:
        fresh = self.status.rx_time_s >= 0 and (t - self.status.rx_time_s) <= COMMAND_MAX_AGE_S
        if fresh and self.status.state == STATE_ACTIVE and not brake_pressed:
            return self.status.throttle_pct
        return 0.0

    def step(self) -> List[CanFrame]:
        """Advances one step; returns the frames to transmit at this instant."""
        t = self.t
        drv = self.driver(t)
        brake_pressed = drv.brake_pct > 1.0
        cruise_cmd = self.cruise_command_pct(t, brake_pressed)
        applied = max(drv.accelerator_pct, cruise_cmd)

        frames: List[CanFrame] = []
        if self.step_index % self.dynamics_every == 0 and not self.scenario.active_fault(t, "mute_vehicle_dynamics"):
            valid = 0.0 if self.scenario.active_fault(t, "invalid_speed") else 1.0
            data = self.msg_dynamics.encode({"VehicleSpeed": self.vehicle.speed_kph,
                                             "LongitudinalAccel": self.vehicle.accel_mps2,
                                             "VehicleSpeedValid": valid,
                                             "AliveCounter": self.alive["dynamics"]})
            frames.append(CanFrame(self.msg_dynamics.frame_id, data))
        if self.step_index % self.dynamics_every == 0:
            self.alive["dynamics"] = (self.alive["dynamics"] + 1) % 16  # advances even if muted (lost frames)
        if self.step_index % self.driver_every == 0:
            if not self.scenario.active_fault(t, "mute_driver_inputs"):
                data = self.msg_driver.encode({"AcceleratorPedal": drv.accelerator_pct,
                                               "BrakePedalPressed": 1.0 if brake_pressed else 0.0,
                                               "CruiseMainSwitch": 1.0 if drv.main_switch else 0.0,
                                               "CruiseSetRequest": 1.0 if drv.set_request else 0.0,
                                               "CruiseCancelRequest": 1.0 if drv.cancel_request else 0.0,
                                               "DriverSetSpeed": drv.set_speed_kph,
                                               "AliveCounter": self.alive["driver"]})
                frames.append(CanFrame(self.msg_driver.frame_id, data))
            self.alive["driver"] = (self.alive["driver"] + 1) % 16

        self.records.append({
            "t": round(t, 3), "speed_kph": self.vehicle.speed_kph, "accel_mps2": self.vehicle.accel_mps2,
            "applied_throttle_pct": applied, "cruise_throttle_pct": cruise_cmd,
            "ecu_throttle_cmd_pct": self.status.throttle_pct, "cruise_state": self.status.state,
            "target_kph": self.status.target_kph, "fault_reason": self.status.fault_reason,
            "accelerator_pct": drv.accelerator_pct, "brake_pct": drv.brake_pct, "grade_pct": drv.grade_pct,
            "set_speed_kph": drv.set_speed_kph, "main_switch": int(drv.main_switch),
            "fault_injected": int(any(w.active(t) for w in self.scenario.faults())),
            "status_age_ms": (t - self.status.rx_time_s) * 1000.0 if self.status.rx_time_s >= 0 else -1.0,
        })
        self.vehicle.step(applied, drv.brake_pct, drv.grade_pct, STEP_S)
        self.step_index += 1
        return frames


def run_realtime(sim: VehicleSimulator, bus, duration_s: float) -> Dict[str, float]:
    """Paces the simulation with the wall clock (the ECU runs in real time)."""
    steps = int(round(duration_s / STEP_S))
    start = time.monotonic()
    max_lateness = 0.0
    late_steps = 0
    late_at = []  # simulated times of late steps (diagnostics of host stalls)
    for k in range(steps):
        release = start + k * STEP_S
        delay = release - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        lateness = time.monotonic() - release
        max_lateness = max(max_lateness, lateness)
        if lateness > STEP_S:
            late_steps += 1
            if len(late_at) < 10:
                late_at.append(round(k * STEP_S, 2))
        while (frame := bus.recv(timeout=0.0)) is not None:
            sim.on_frame(frame, sim.t)
        for frame in sim.step():
            bus.send(frame)
    return {"max_step_lateness_ms": max_lateness * 1000.0, "late_steps": late_steps, "late_at_s": late_at}


def write_csv(records: List[Dict[str, float]], path: Path) -> None:
    with path.open("w", newline="", encoding="ascii") as file:
        writer = csv.DictWriter(file, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Vehicle simulator for the cruise control ECU")
    parser.add_argument("--scenario", required=True, type=Path)
    parser.add_argument("--duration", type=float, help="override scenario duration [s]")
    parser.add_argument("--bus", default="auto", choices=["auto", "udp", "socketcan"])
    parser.add_argument("--iface", default="vcan0")
    parser.add_argument("--udp-group", default=DEFAULT_GROUP)
    parser.add_argument("--udp-port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--dbc", type=Path, default=default_dbc_path())
    parser.add_argument("--csv", type=Path, help="write the time series")
    parser.add_argument("--svg", type=Path, help="write a plot")
    parser.add_argument("--report", type=Path, help="write KPIs as JSON")
    args = parser.parse_args(argv)

    scenario = Scenario.load(args.scenario)
    sim = VehicleSimulator(scenario, Database.load(args.dbc))
    bus, description = open_bus(args.bus, args.iface, args.udp_group, args.udp_port)
    duration = args.duration if args.duration else scenario.duration_s
    print(f"[sim] scenario={scenario.name} duration={duration:.1f}s bus={description}", flush=True)
    try:
        timing = run_realtime(sim, bus, duration)
    except KeyboardInterrupt:
        timing = {}
    finally:
        bus.close()

    report = analysis.evaluate(sim.records, scenario)
    report["simulator"] = {**timing, "status_frames_received": sim.status.frames}
    print(analysis.format_report(report), flush=True)
    if args.csv:
        write_csv(sim.records, args.csv)
    if args.svg:
        svg_plot.write_run_plot(sim.records, args.svg, title=f"{scenario.name}: {scenario.description}")
    if args.report:
        args.report.write_text(json.dumps(report, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
