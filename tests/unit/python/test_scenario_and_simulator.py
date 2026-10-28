"""Scenario parsing, simulator logic (offline, no bus) and KPI analysis."""

import pathlib

import pytest

from vecu_sim import analysis
from vecu_sim.dbc import Database, default_dbc_path
from vecu_sim.scenario import Scenario
from vecu_sim.simulator import STEP_S, VehicleSimulator
from vecu_sim.udp_can_bus import CanFrame

DB = Database.load(default_dbc_path())
SCENARIO_DIR = pathlib.Path(__file__).resolve().parents[3] / "config" / "scenarios"


def scenario(events, **initial):
    return Scenario.from_dict({"name": "t", "duration_s": 10, "initial": initial, "events": events})


def status_frame(state, throttle=0.0, target=0.0, fault=0):
    message = DB["CruiseControlStatus"]
    return CanFrame(message.frame_id, message.encode({"CruiseState": state, "ThrottleCommand": throttle,
                                                      "CruiseTargetSpeed": target, "CruiseFaultReason": fault}))


# ------------------------------------------------------------------------- scenario ----

def test_all_shipped_scenarios_load():
    files = sorted(SCENARIO_DIR.glob("*.json"))
    assert len(files) >= 5
    for path in files:
        s = Scenario.load(path)
        assert s.duration_s > 0 and s.name == path.stem


def test_levels_hold_and_buttons_pulse():
    s = scenario([{"t": 1.0, "set": True}, {"t": 2.0, "accelerator_pct": 30}, {"t": 3.0, "accelerator_pct": 0}],
                 main_switch=True, set_speed_kph=80)
    assert not s.driver_state(0.99).set_request
    assert s.driver_state(1.0).set_request and s.driver_state(1.19).set_request
    assert not s.driver_state(1.2).set_request  # 200 ms pulse
    assert s.driver_state(2.5).accelerator_pct == 30
    assert s.driver_state(3.5).accelerator_pct == 0
    assert s.driver_state(5).main_switch and s.driver_state(5).set_speed_kph == 80


def test_hold_accelerator_uses_road_load():
    s = scenario([], speed_kph=80, accelerator_pct="hold")
    assert s.driver_state(0, hold_throttle_pct=8.7).accelerator_pct == 8.7


def test_rejects_unknown_fault_and_keys():
    with pytest.raises(ValueError):
        scenario([{"t": 1, "fault": "explode"}])
    with pytest.raises(ValueError):
        scenario([{"t": 1, "throtle": 3}])


# ------------------------------------------------------------------------ simulator ----

def test_transmission_schedule_and_alive_counters():
    sim = VehicleSimulator(scenario([], main_switch=True), DB)
    frames = [sim.step() for _ in range(10)]  # 100 ms
    ids = [f.can_id for step in frames for f in step]
    assert ids.count(0x100) == 5 and ids.count(0x110) == 2
    counters = [DB.by_id(0x100).decode(f.data)["AliveCounter"] for step in frames for f in step if f.can_id == 0x100]
    assert counters == [0, 1, 2, 3, 4]


def test_mute_fault_drops_frames_but_counter_advances():
    sim = VehicleSimulator(scenario([{"t": 0.02, "fault": "mute_vehicle_dynamics", "duration_s": 0.04}]), DB)
    dynamics = []
    for _ in range(10):
        dynamics += [f for f in sim.step() if f.can_id == 0x100]
    counters = [DB.by_id(0x100).decode(f.data)["AliveCounter"] for f in dynamics]
    assert counters == [0, 3, 4]  # frames at 20 and 40 ms muted


def test_invalid_speed_fault_clears_validity_bit():
    sim = VehicleSimulator(scenario([{"t": 0.0, "fault": "invalid_speed", "duration_s": 0.05}], speed_kph=50), DB)
    frame = next(f for f in sim.step() if f.can_id == 0x100)
    assert DB.by_id(0x100).decode(frame.data)["VehicleSpeedValid"] == 0


def test_cruise_command_applied_only_when_active_fresh_and_not_braking():
    sim = VehicleSimulator(scenario([{"t": 0.5, "brake_pct": 20}], speed_kph=80), DB)
    sim.on_frame(status_frame(1, throttle=30.0), 0.0)  # Standby: ignored
    sim.step()
    assert sim.records[-1]["applied_throttle_pct"] == 0.0
    sim.on_frame(status_frame(2, throttle=30.0), sim.t)  # Active: applied
    sim.step()
    assert sim.records[-1]["applied_throttle_pct"] == pytest.approx(30.0)
    for _ in range(12):  # no new status for > 100 ms: command is stale
        sim.step()
    assert sim.records[-1]["cruise_throttle_pct"] == 0.0
    while sim.t < 0.5:
        sim.on_frame(status_frame(2, throttle=30.0), sim.t)
        sim.step()
    sim.on_frame(status_frame(2, throttle=30.0), sim.t)
    sim.step()  # brake pressed: actuator ignores the cruise command
    assert sim.records[-1]["cruise_throttle_pct"] == 0.0


def test_driver_pedal_wins_when_higher():
    sim = VehicleSimulator(scenario([], accelerator_pct=40), DB)
    sim.on_frame(status_frame(2, throttle=10.0), 0.0)
    sim.step()
    assert sim.records[-1]["applied_throttle_pct"] == 40.0


# ------------------------------------------------------------------------- analysis ----

def synthetic(speeds, state=2, target=80.0):
    return [{"t": round(i * STEP_S, 3), "speed_kph": v, "cruise_throttle_pct": 10.0, "cruise_state": state,
             "target_kph": target, "ecu_throttle_cmd_pct": 10.0, "fault_reason": 0, "status_age_ms": 5.0}
            for i, v in enumerate(speeds)]


def test_step_metrics_on_known_response():
    speeds = [60 + min(20.0, 4.0 * i * STEP_S) for i in range(1000)]  # ramp to 80 in 5 s, then flat
    speeds[700] = 81.0  # single overshoot sample at 7 s
    m = analysis.step_metrics(synthetic(speeds), 0.0, 10.0, 60.0, 80.0)
    assert m["overshoot_kph"] == pytest.approx(1.0)
    assert m["rise_time_s"] == pytest.approx(4.0, abs=0.02)  # 62 km/h at 0.5 s -> 78 km/h at 4.5 s
    assert m["settling_time_s"] == pytest.approx(7.01, abs=0.005)  # first sample after the last excursion
    assert m["steady_state_error_kph"] == pytest.approx(0.0, abs=0.01)


def test_fault_latency_from_records():
    records = synthetic([80.0] * 300)
    for r in records[120:]:
        r["cruise_state"], r["ecu_throttle_cmd_pct"], r["fault_reason"] = 4, 0.0, 1
    for r in records[200:]:
        r["cruise_state"], r["fault_reason"] = 1, 0
    s = scenario([{"t": 1.0, "fault": "mute_vehicle_dynamics", "duration_s": 0.5}])
    fault = analysis.evaluate(records, s)["faults"][0]
    assert fault["fault_detected_after_ms"] == pytest.approx(200.0)
    assert fault["throttle_cut_after_ms"] == pytest.approx(200.0)
    assert fault["fault_reason"] == 1 and fault["state_after_fault"] == 1


# -------------------------------------------------------------------------- svg plot ----

def test_svg_plot_is_well_formed_and_inside_canvas(tmp_path):
    import xml.etree.ElementTree as ET
    from vecu_sim import svg_plot

    records = synthetic([60 + i * 0.02 for i in range(500)])
    for i, r in enumerate(records):
        r.update(applied_throttle_pct=20.0, accelerator_pct=0.0, brake_pct=0.0, grade_pct=0.0,
                 cruise_state=1 if i < 100 else 2)
    path = tmp_path / "plot.svg"
    svg_plot.write_run_plot(records, path, title="unit <test>", events=[{"t": 1.0, "set": True}])
    root = ET.parse(path).getroot()
    width, height = float(root.get("width")), float(root.get("height"))
    lines = [e for e in root.iter() if e.tag.endswith("polyline")]
    assert len(lines) == 3  # speed, target, applied throttle (pedals never pressed: not drawn)
    for line in lines:
        for point in line.get("points").split():
            x, y = map(float, point.split(","))
            assert 0 <= x <= width and 0 <= y <= height
    labels = [e.text for e in root.iter() if e.tag.endswith("text")]
    assert "Standby" in labels and "Active" in labels and "set" in labels


def test_speed_spike_adds_impossible_jump():
    sim = VehicleSimulator(scenario([{"t": 0.02, "fault": "speed_spike", "duration_s": 0.02}], speed_kph=80), DB)
    speeds = []
    for _ in range(6):
        speeds += [DB.by_id(0x100).decode(f.data)["VehicleSpeed"] for f in sim.step() if f.can_id == 0x100]
    assert speeds[0] == pytest.approx(80, abs=0.1)
    assert speeds[1] == pytest.approx(110, abs=0.1)  # +30 km/h within 20 ms
    assert speeds[2] == pytest.approx(80, abs=0.1)


def test_freeze_alive_repeats_the_counter():
    sim = VehicleSimulator(scenario([{"t": 0.02, "fault": "freeze_alive", "duration_s": 0.05}]), DB)
    counters = []
    for _ in range(10):
        counters += [DB.by_id(0x100).decode(f.data)["AliveCounter"] for f in sim.step() if f.can_id == 0x100]
    assert counters == [0, 1, 1, 1, 1]  # frozen from 20 ms to 70 ms
