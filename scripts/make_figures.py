#!/usr/bin/env python3
"""Runs simulations and draws the result figures (SVG) from what was measured.

    ./scripts/make_figures.py                    # ~2.5 min, needs build/bin; writes docs/images/fig_*.svg
    ./scripts/make_figures.py --only firmware    # one group: closed_loop | faults | firmware | timing

Every figure is drawn from data recorded in this run (simulator time series, ECU logs, CAN traces,
benchmark samples); nothing is synthesised. Raw data stays in logs/figures/.

Clock alignment: ECU events carry "t=...s" on the ECU's scheduler clock, the ECU logs that clock's
wall-clock origin (SCHED_EPOCH), and the simulator reports the wall-clock time of its t = 0, so
simulator data and ECU events are drawn on one time axis (accuracy: a few ms).
"""

import argparse
import csv
import json
import os
import pathlib
import random
import re
import signal
import subprocess
import sys
import threading
import time

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "simulator"))
from vecu_sim import figures as F  # noqa: E402
from vecu_sim import firmware_image as fw  # noqa: E402
from vecu_sim.isotp import IsoTpEndpoint, IsoTpError  # noqa: E402
from vecu_sim.udp_can_bus import UdpCanBus  # noqa: E402
from vecu_sim.uds_client import UdsClient  # noqa: E402

SCENARIO_DIR = REPO / "config" / "scenarios"
SIM_ENV = {**os.environ, "PYTHONPATH": str(REPO / "simulator")}
CLOSED_LOOP = {"cruise_step_response": "Step response: engage at 60 km/h, desired speed 80 km/h",
               "cruise_grade": "Disturbance rejection: 4 % uphill grade at 90 km/h",
               "cruise_driver_override": "Driver override: accelerator, resume, brake, re-engage",
               "cruise_sensor_fault": "Sensor faults: speed message lost, speed marked invalid",
               "cruise_full_demo": "Full drive: manual acceleration, engage, new set speed, grade, brake"}


def port():
    return random.randint(40000, 59999)


def ecu(bins, p, *extra, duration=None):
    args = [str(bins / "vecu_cruise_ecu"), "--bus", "udp", "--udp-port", str(p), *map(str, extra)]
    if duration:
        args += ["--duration", str(duration)]
    return subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)


def simulator(scenario, p, out, name):
    return subprocess.Popen([sys.executable, "-m", "vecu_sim.simulator", "--scenario", str(scenario), "--bus", "udp",
                             "--udp-port", str(p), "--csv", str(out / f"{name}.csv"), "--report",
                             str(out / f"{name}.json")], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, env=SIM_ENV)


def read_csv(path):
    with open(path, newline="") as file:
        return [{k: float(v) for k, v in row.items()} for row in csv.DictReader(file)]


def wait_ready(client, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            return client.read_data_by_identifier(0xF189)[0xF189].decode()
        except (TimeoutError, IsoTpError):
            time.sleep(0.05)
    raise TimeoutError("ECU not answering")


def subtitle(extra="", base="software CAN bus, real-time processes"):
    stamp = time.strftime("%Y-%m-%d")
    return f"Measured {stamp}, WSL2, {base}{extra}"


# ----------------------------------------------------------------- closed loop ----

def run_closed_loop(bins, out, images, uds_samples):
    pairs = []
    for name in CLOSED_LOOP:
        p = port()
        scenario = SCENARIO_DIR / f"{name}.json"
        duration = json.loads(scenario.read_text())["duration_s"]
        extra = ["--log", out / f"{name}.candump"] if name == "cruise_sensor_fault" else []
        pairs.append((name, p, scenario, ecu(bins, p, *extra, duration=duration + 1.0)))
    time.sleep(0.5)
    sims = [(name, e, scenario, simulator(scenario, p, out, name)) for name, p, scenario, e in pairs]

    # UDS response times while cruising, on the step-response bus
    time.sleep(3.0)
    bus = UdpCanBus(port=pairs[0][1])
    try:
        client = UdsClient(IsoTpEndpoint(bus))
        for label, did in (("single-frame response (22 0100)", 0x0100), ("multi-frame response (22 F190)", 0xF190)):
            uds_samples[label] = []
            for _ in range(300):
                client.read_data_by_identifier(did)
                uds_samples[label].append(client.last_response_time_ms)
    finally:
        bus.close()

    for name, e, scenario, sim in sims:
        sim_out, _ = sim.communicate(timeout=150)
        e.communicate(timeout=30)
        if sim.returncode != 0:
            raise RuntimeError(f"{name}: {sim_out[-1500:]}")
        records = read_csv(out / f"{name}.csv")
        report = json.loads((out / f"{name}.json").read_text())
        late = report["simulator"]["late_steps"]
        note = f"; host stall: {late} late simulator step(s)" if late else ""
        F.closed_loop_figure(records, json.loads(scenario.read_text()), CLOSED_LOOP[name],
                             subtitle(note)).save(images / f"fig_{name}.svg")
        print(f"[figures] fig_{name}.svg")

    # CAN trace around the lost speed message (sim t = 5.0 .. 5.5 s)
    frames = F.parse_candump((out / "cruise_sensor_fault.candump").read_text())
    report = json.loads((out / "cruise_sensor_fault.json").read_text())
    t0 = report["simulator"]["wall_start_s"]
    rel = [(t - t0, fid, data) for t, fid, data in frames]
    F.bus_trace_figure(rel, (4.6, 6.2), "CAN bus trace: speed message lost for 0.5 s",
                       subtitle("; recorded by the cruise ECU (candump format)"),
                       (5.0, 5.5, "simulator stops 0x100")).save(images / "fig_can_trace_timeout.svg")
    print("[figures] fig_can_trace_timeout.svg")


# ------------------------------------------------------- fault management run ----

def run_faults(bins, out, images):
    p = port()
    scenario = SCENARIO_DIR / "fault_management.json"
    injections = [("TaskDeadlineMiss", "overrun:Com_20ms:14.5:1.0:25", 14.5, 15.5),
                  ("WatchdogExpired", "stall:Control_10ms:21.0:0.4", 21.0, 21.4),
                  ("ControllerFault", "controller-nan:27.0", 27.0, 27.0)]
    args = []
    for _, spec, _, _ in injections:
        args += ["--inject", spec]
    e = ecu(bins, p, *args, duration=35)
    started = time.monotonic()
    time.sleep(0.5)
    sim = simulator(scenario, p, out, "fault_management")
    # UDS hard reset ~2 s after the controller fault: clears the latched fault (new operation cycle)
    time.sleep(max(0.0, 29.0 - (time.monotonic() - started)))
    bus = UdpCanBus(port=p)
    try:
        client = UdsClient(IsoTpEndpoint(bus))
        client.diagnostic_session_control(0x03)
        client.ecu_reset(0x01)
    finally:
        bus.close()
    sim_out, _ = sim.communicate(timeout=120)
    _, log = e.communicate(timeout=60)
    (out / "fault_management.ecu.txt").write_text(log)
    if sim.returncode != 0:
        raise RuntimeError(sim_out[-1500:])

    lines = F.parse_ecu_log(log)
    offset = F.epoch_offset(lines)
    report = json.loads((out / "fault_management.json").read_text())
    sim_shift = report["simulator"]["wall_start_s"] - F.epoch_wall(lines)
    records = read_csv(out / "fault_management.csv")
    for r in records:
        r["t"] += sim_shift
    t_end = records[-1]["t"]

    resets = [(stamp + offset, "UDS hard reset") for stamp, msg in lines if msg.startswith("ECU_RESET type=hard")]
    transitions = [(m.group(1), m.group(3), float(m.group(4))) for _, msg in lines
                   if (m := re.match(r"FAULT (\w+) (\w+) -> (\w+) t=([\d.]+)s", msg))]
    names = []
    for name, _, _ in transitions:
        if name not in names:
            names.append(name)
    sim_faults = {"speed_spike": "SpeedSignalInvalid", "freeze_alive": "SpeedSignalStale"}
    causes = {name: [] for name in names}
    for e_ in json.loads(scenario.read_text())["events"]:
        if e_.get("fault") in sim_faults:
            causes.setdefault(sim_faults[e_["fault"]], []).append(
                (e_["t"] + sim_shift, e_["t"] + e_.get("duration_s", 0) + sim_shift))
    for name, _, a, b in injections:
        causes.setdefault(name, []).append((a, b))
    rows = []
    for name in names:
        actives, start = [], None
        for fname, to, t in transitions:
            if fname != name:
                continue
            if to == "Failed" and start is None:
                start = t
            elif to == "Passed" and start is not None:
                actives.append((start, t))
                start = None
        if start is not None:  # latched: ends with the next operation cycle (reset) or the run
            end = next((r for r, _ in resets if r > start), t_end)
            actives.append((start, end))
        actives = [(a, min(b, t_end)) for a, b in actives if a < t_end]  # only while the simulator ran
        if actives:  # (timeouts after the simulator has finished are not part of the experiment)
            rows.append((name, causes.get(name, []), actives))
    segs = F.state_segments([r["t"] for r in records], [int(r["cruise_state"]) for r in records])
    F.fault_timeline_figure(rows, segs, t_end, "Fault detection and recovery: every fault class injected in one run",
                            subtitle("; cruise ECU + vehicle simulator + test-only ECU injections"),
                            resets).save(images / "fig_fault_timeline.svg")
    print("[figures] fig_fault_timeline.svg")

    def first(prefix, after=0.0, use_t=True):
        for stamp, msg in lines:
            if msg.startswith(prefix):
                m = re.search(r" t=([\d.]+)s", msg)
                value = float(m.group(1)) if (m and use_t) else stamp + offset
                if value >= after:
                    return value
        return None

    stale_on, stale_off = first("CONTROL_OUTPUT_STALE", 20.5), first("CONTROL_OUTPUT_FRESH", 20.5)
    F.watchdog_figure((20.85, 24.4), (21.0, 21.4), (stale_on, stale_off) if stale_on and stale_off else None,
                      first("WATCHDOG_EXPIRED", 20.5), first("ECU_RESET type=watchdog", 20.5, use_t=False),
                      first("SAFE_STATE left", 20.5, use_t=False), records,
                      "Software watchdog: Control task stalled for 400 ms",
                      subtitle("; two independent mechanisms: stale-output check (COM) and watchdog")
                      ).save(images / "fig_watchdog_recovery.svg")
    print("[figures] fig_watchdog_recovery.svg")


# --------------------------------------------------------------- firmware story ----

def run_firmware(bins, out, images):
    p = port()
    flash = out / "flash.bin"
    flash.unlink(missing_ok=True)
    opts = ["--flash", flash, "--watchdog-action", "exit", "--confirm-after", "2"]
    proc = ecu(bins, p, *opts, duration=120)
    bus = UdpCanBus(port=p)
    client = UdsClient(IsoTpEndpoint(bus))
    t0 = time.monotonic()
    samples, downloads, progress, restarts = [], [], [], []
    now = lambda: time.monotonic() - t0  # noqa: E731

    def poll():
        nonlocal proc
        if proc.poll() is not None:
            restarts.append((now(), proc.returncode))
            samples.append({"t": now(), "up": False})
            proc.stderr.read()
            time.sleep(0.3)  # external supervisor restarts the ECU
            proc = ecu(bins, p, *opts, duration=120)
            return
        try:
            raw = client.read_data_by_identifier(0xF189, 0x0400)
            status = fw.decode_boot_status(raw[0x0400])
            samples.append({"t": now(), "up": True, "version": raw[0xF189].decode(), "state_a": status["state_a"],
                            "state_b": status["state_b"], "event": status["last_boot_event"]})
        except (TimeoutError, IsoTpError):
            pass

    def poll_for(seconds, stop=None):
        end = now() + seconds
        while now() < end:
            poll()
            if stop and samples and samples[-1].get("up") and stop(samples[-1]):
                return
            time.sleep(0.1)

    def flash_image(version, defect, label):
        header = fw.ImageHeader(hardware_id=0x0000CC01, version=version, build_time=0,
                                flags=fw.FLAG_SIMULATED_DEFECT_HANG if defect else 0)
        image = fw.build_image(header, fw.make_payload(48 * 1024, version[1]))
        start = now()
        points = [(now(), 0.0)]
        progress.append(points)
        client.flash_image(image, progress=lambda done, total: points.append((now(), 100.0 * done / total)))
        downloads.append((start, now(), label))

    try:
        wait_ready(client)
        poll_for(1.0)
        flash_image((0, 7, 0), False, "update 00.07.00")
        poll_for(6.0, stop=lambda s: s["state_b"] == "Confirmed")
        poll_for(1.0)
        flash_image((0, 9, 0), True, "update 00.09.00 (defective)")
        poll_for(20.0, stop=lambda s: s["event"] == "Rollback")
        poll_for(1.5)
    finally:
        bus.close()
        if proc.poll() is None:
            proc.send_signal(signal.SIGTERM)
            proc.communicate(timeout=30)
    (out / "firmware_story.json").write_text(json.dumps({"samples": samples, "restarts": restarts}, indent=1))
    F.firmware_figure(samples, downloads, progress, restarts, now(),
                      "Firmware update: trial boot, confirmation, and rollback of a defective image",
                      subtitle("; ECU restarted by the test after each watchdog exit (code 75)")
                      ).save(images / "fig_firmware_update.svg")
    print("[figures] fig_firmware_update.svg")


# ------------------------------------------------------------------- timing ----

def run_timing(bins, out, images, uds_samples):
    runs = {}
    stress = os.cpu_count() or 4
    for label, extra in (("light load", []), (f"{stress} CPU-stress threads", ["--stress", str(stress)])):
        path = out / f"sched_{len(runs)}.json"
        subprocess.run([str(bins / "vecu_sched_bench"), "--duration", "10", "--json", str(path), *extra],
                       check=True, capture_output=True, timeout=120)
        runs[label] = json.loads(path.read_text())
    F.scheduler_figure(runs, "Scheduler: task start latency against the deadline",
                       subtitle("; 10 s per condition, SCHED_OTHER", base="vecu_sched_bench")
                       ).save(images / "fig_scheduler_latency.svg")
    print("[figures] fig_scheduler_latency.svg")

    series = {}
    for fps, label in ((0, "probes only"), (2000, "+ 2000 background frames/s")):
        path = out / f"latency_{fps}.txt"
        subprocess.run([str(bins / "vecu_can_latency"), "--bus", "udp", "--udp-port", str(port()), "--count", "5000",
                        "--background-fps", str(fps), "--samples", str(path)], check=True, capture_output=True,
                       timeout=60)
        series[label] = [float(v) for v in path.read_text().split()]
    F.latency_cdf_figure(series, "CAN message latency on the software bus (one-way)",
                         subtitle("; 1 ms probe period; no wire time modelled", base="vecu_can_latency")
                         ).save(images / "fig_can_latency.svg")
    print("[figures] fig_can_latency.svg")

    if uds_samples:
        F.uds_histogram_figure(uds_samples, "UDS response time while cruising",
                               subtitle("; 300 requests each, Python tester, P2 server = 50 ms")
                               ).save(images / "fig_uds_response.svg")
        print("[figures] fig_uds_response.svg")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bin-dir", type=pathlib.Path, default=REPO / "build" / "bin")
    parser.add_argument("--out", type=pathlib.Path, default=REPO / "logs" / "figures")
    parser.add_argument("--images", type=pathlib.Path, default=REPO / "docs" / "images")
    parser.add_argument("--only", choices=["closed_loop", "faults", "firmware", "timing"])
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    args.images.mkdir(parents=True, exist_ok=True)
    bins = args.bin_dir.resolve()
    uds = {}
    if args.only in (None, "firmware"):
        run_firmware(bins, args.out, args.images)
    if args.only in (None, "closed_loop", "faults"):
        threads = []
        if args.only in (None, "faults"):
            threads.append(threading.Thread(target=run_faults, args=(bins, args.out, args.images)))
        for thread in threads:
            thread.start()
        if args.only in (None, "closed_loop"):
            run_closed_loop(bins, args.out, args.images, uds)
        for thread in threads:
            thread.join()
    if args.only in (None, "timing"):
        run_timing(bins, args.out, args.images, uds)
    return 0


if __name__ == "__main__":
    sys.exit(main())
