"""Closed-loop system tests: real cruise ECU process + real vehicle simulator over CAN.

The four scenarios run in parallel (each pair on its own isolated UDP bus) to keep the test
time near the longest scenario (~22 s). Acceptance limits are the controller's requirements;
the measured KPIs are printed (pytest -s) and recorded in docs/phase3_cruise_control.md.
"""

import json
import os
import pathlib
import random
import re
import subprocess
import sys
import time

import pytest

REPO = pathlib.Path(__file__).resolve().parents[2]
BIN_DIR = pathlib.Path(os.environ.get("VECU_BIN_DIR", REPO / "build" / "bin"))
ECU = BIN_DIR / "vecu_cruise_ecu"
SCENARIOS = ["cruise_step_response", "cruise_grade", "cruise_driver_override", "cruise_sensor_fault"]

pytestmark = pytest.mark.skipif(not ECU.exists(), reason=f"{ECU} not built")


@pytest.fixture(scope="module")
def runs(tmp_path_factory):
    out = tmp_path_factory.mktemp("closed_loop")
    env = {**os.environ, "PYTHONPATH": str(REPO / "simulator")}
    ports = random.sample(range(40000, 60000), len(SCENARIOS))
    started = []
    for name, port in zip(SCENARIOS, ports):
        scenario = REPO / "config" / "scenarios" / f"{name}.json"
        duration = json.loads(scenario.read_text())["duration_s"]
        ecu = subprocess.Popen([str(ECU), "--bus", "udp", "--udp-port", str(port), "--duration", str(duration + 1.0)],
                               stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
        started.append((name, port, scenario, ecu))
    time.sleep(0.5)  # ECUs up before the simulators start
    sims = []
    for name, port, scenario, ecu in started:
        sim = subprocess.Popen([sys.executable, "-m", "vecu_sim.simulator", "--scenario", str(scenario), "--bus", "udp",
                                "--udp-port", str(port), "--report", str(out / f"{name}.json"),
                                "--csv", str(out / f"{name}.csv")],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
        sims.append((name, ecu, sim))
    results = {}
    for name, ecu, sim in sims:
        sim_out, _ = sim.communicate(timeout=120)
        _, ecu_log = ecu.communicate(timeout=30)
        assert sim.returncode == 0, sim_out
        assert ecu.returncode == 0, ecu_log
        print(sim_out)
        results[name] = {"kpi": json.loads((out / f"{name}.json").read_text()), "ecu_log": ecu_log,
                         "csv": out / f"{name}.csv"}
    return results


def transitions(ecu_log):
    return re.findall(r"CRUISE_STATE (\w+) -> (\w+)", ecu_log)


def ecu_summary(ecu_log):
    match = re.search(r"\] SUMMARY (ecu=.*)", ecu_log)
    assert match, ecu_log
    return dict(item.split("=", 1) for item in match.group(1).split())


@pytest.mark.parametrize("name", SCENARIOS)
def test_communication_is_healthy(runs, name):
    log = runs[name]["ecu_log"]
    summary = ecu_summary(log)
    assert summary["alive_errors"] == "0" and summary["invalid"] == "0", log
    assert "queue_overruns=0" in log
    sim = runs[name]["kpi"]["simulator"]
    assert sim["late_steps"] == 0, sim  # simulator kept its 10 ms pace
    misses = [int(m) for m in re.findall(r"SCHED_SUMMARY task=\S+ .*? deadline_misses=(\d+)", log)]
    assert len(misses) == 3 and sum(misses) <= 2, log


def test_step_response_60_to_80(runs):
    kpi = runs["cruise_step_response"]["kpi"]
    (step,) = kpi["speed_responses"]
    assert step["target_kph"] == 80
    assert step["overshoot_kph"] < 2.0
    assert step["settling_time_s"] is not None and step["settling_time_s"] < 12.0
    assert step["steady_state_error_kph"] < 0.2
    assert step["max_throttle_pct"] <= 60.0 + 0.01  # saturation limit respected on the bus


def test_grade_disturbance_rejection(runs):
    (dist,) = runs["cruise_grade"]["kpi"]["disturbances"]
    assert dist["max_deviation_kph"] < 2.5
    assert dist["recovery_time_s"] is not None and dist["recovery_time_s"] < 15.0


def test_driver_override_and_brake(runs):
    run = runs["cruise_driver_override"]
    seq = transitions(run["ecu_log"])
    assert ("Active", "Override") in seq and ("Override", "Active") in seq
    assert ("Active", "Standby") in seq  # brake
    assert seq[-1] == ("Standby", "Active") or ("Standby", "Active") in seq[-2:]  # re-engaged
    reactions = {o["kind"]: o for o in run["kpi"]["overrides"]}
    assert reactions["brake"]["disengaged_after_ms"] < 150.0
    assert reactions["accelerator"]["override_after_ms"] < 150.0


def test_sensor_faults_reach_safe_state_and_never_resume_automatically(runs):
    run = runs["cruise_sensor_fault"]
    mute, invalid = run["kpi"]["faults"]
    # 100 ms message timeout + 20 ms COM task + bus/scheduling delays.
    assert 100.0 <= mute["fault_detected_after_ms"] < 250.0
    assert mute["throttle_cut_after_ms"] <= mute["fault_detected_after_ms"]
    assert mute["fault_reason"] == 1  # SpeedSignalTimeout
    assert invalid["fault_detected_after_ms"] < 150.0
    assert invalid["fault_reason"] == 3  # SpeedInvalid
    assert mute["state_after_fault"] == invalid["state_after_fault"] == 1  # Standby, not Active
    seq = transitions(run["ecu_log"])
    assert ("Fault", "Active") not in seq
    # While the ECU reported Fault, the simulator never applied a cruise throttle.
    import csv
    with run["csv"].open() as file:
        for row in csv.DictReader(file):
            if row["cruise_state"] == "4":
                assert float(row["cruise_throttle_pct"]) == 0.0
