#!/usr/bin/env python3
"""Measures the platform's performance and writes docs/performance_report.md.

Every number in the report comes from this run; nothing is copied from elsewhere.

    ./scripts/run_performance.py                       # ~1.5 min, needs a build in build/
    ./scripts/run_performance.py --sched-duration 30   # longer scheduler runs
    ./scripts/run_performance.py --bin-dir build/bin --report /tmp/report.md

Measurements (all on isolated UDP software buses, so runs do not disturb each other's traffic):
  1. scheduler: task execution time, start latency, jitter, deadline misses (light load and
     with CPU stress threads)                                         vecu_sched_bench
  2. CAN message latency, idle bus and with background traffic        vecu_can_latency
  3. closed loop: 5 ECU + simulator pairs in parallel: controller response (step, grade,
     overrides), fault detection latency (sensor faults), ECU task timing, and the UDS
     response time while cruising                                      vecu_cruise_ecu + simulator
  4. watchdog detection of a stalled Control task (test-only injection)
  5. firmware update of a 64 KiB image over UDS / ISO-TP
Raw results: logs/performance/results.json (or --out).
"""

import argparse
import datetime
import json
import os
import pathlib
import platform
import random
import re
import signal
import statistics
import subprocess
import sys
import time

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "simulator"))
from vecu_sim import firmware_image as fw  # noqa: E402
from vecu_sim.isotp import IsoTpEndpoint  # noqa: E402
from vecu_sim.udp_can_bus import UdpCanBus  # noqa: E402
from vecu_sim.uds_client import UdsClient  # noqa: E402

SCENARIOS = ["cruise_step_response", "cruise_grade", "cruise_driver_override", "cruise_sensor_fault",
             "fault_management"]
SIM_ENV = {**os.environ, "PYTHONPATH": str(REPO / "simulator")}


def port():
    return random.randint(40000, 59999)


def run(cmd, timeout):
    result = subprocess.run([str(c) for c in cmd], capture_output=True, text=True, timeout=timeout)
    if result.returncode != 0:
        raise RuntimeError(f"{cmd[0]} failed ({result.returncode}):\n{result.stdout[-2000:]}{result.stderr[-2000:]}")
    return result


def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGTERM)
    _, err = process.communicate(timeout=30)
    return err


def log_time(log, pattern):
    match = re.search(pattern + r".*?t=([\d.]+)s", log)
    return float(match.group(1)) if match else None


def percentiles(values):
    values = sorted(values)
    return {"n": len(values), "min": values[0], "median": statistics.median(values),
            "p95": values[int(0.95 * (len(values) - 1))], "max": values[-1]}


# ------------------------------------------------------------------- measurements ----

def measure_scheduler(bins, out, duration):
    results = {}
    stress = os.cpu_count() or 4
    for label, extra in (("light load", []), (f"{stress} CPU-stress threads", ["--stress", str(stress)])):
        path = out / f"sched_{len(results)}.json"
        run([bins / "vecu_sched_bench", "--duration", duration, "--json", path, *extra], timeout=float(duration) + 60)
        results[label] = json.loads(path.read_text())
    return results


def measure_can_latency(bins, out):
    results = {}
    for fps in (0, 2000):
        path = out / f"can_latency_{fps}.json"
        run([bins / "vecu_can_latency", "--bus", "udp", "--udp-port", port(), "--count", "5000", "--period-us", "1000",
             "--background-fps", fps, "--json", path], timeout=60)
        results[fps] = json.loads(path.read_text())
    return results


def uds_benchmark(client, n=200):
    results = {}
    for label, did in (("single-frame (22 0100)", 0x0100), ("multi-frame (22 F190)", 0xF190)):
        times = []
        for _ in range(n):
            client.read_data_by_identifier(did)
            times.append(client.last_response_time_ms)
        results[label] = percentiles(times)
    return results


def measure_closed_loop(bins, out):
    """5 ECU + simulator pairs in parallel; UDS benchmark on the step-response bus; one more
    ECU with a stalled Control task for the watchdog."""
    pairs = []
    for name in SCENARIOS:
        p = port()
        scenario = REPO / "config" / "scenarios" / f"{name}.json"
        duration = json.loads(scenario.read_text())["duration_s"]
        ecu = subprocess.Popen([bins / "vecu_cruise_ecu", "--bus", "udp", "--udp-port", str(p), "--duration",
                                str(duration + 1.0), "--timing-report", out / f"ecu_{name}.json"],
                               stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
        pairs.append((name, p, scenario, ecu))
    wd_port = port()
    watchdog_ecu = subprocess.Popen([bins / "vecu_cruise_ecu", "--bus", "udp", "--udp-port", str(wd_port),
                                     "--duration", "4", "--inject", "stall:Control_10ms:2.0:0.4"],
                                    stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    time.sleep(0.5)
    sims = [(name, ecu, subprocess.Popen([sys.executable, "-m", "vecu_sim.simulator", "--scenario", str(scenario),
                                          "--bus", "udp", "--udp-port", str(p), "--report",
                                          str(out / f"kpi_{name}.json")],
                                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=SIM_ENV))
            for name, p, scenario, ecu in pairs]

    time.sleep(3.0)  # cruise engaged in the step scenario
    bus = UdpCanBus(port=pairs[0][1])
    try:
        uds = uds_benchmark(UdsClient(IsoTpEndpoint(bus)))
    finally:
        bus.close()

    results = {"uds": uds, "kpi": {}, "ecu_timing": {}}
    for name, ecu, sim in sims:
        sim_out, _ = sim.communicate(timeout=120)
        ecu.communicate(timeout=30)
        if sim.returncode != 0 or ecu.returncode != 0:
            raise RuntimeError(f"closed loop {name} failed:\n{sim_out[-2000:]}")
        results["kpi"][name] = json.loads((out / f"kpi_{name}.json").read_text())
        results["ecu_timing"][name] = json.loads((out / f"ecu_{name}.json").read_text())
    _, wd_log = watchdog_ecu.communicate(timeout=30)
    t_stall = log_time(wd_log, r"FAULT_INJECTION stall task=Control_10ms")
    t_stale = log_time(wd_log, r"CONTROL_OUTPUT_STALE")
    t_expired = log_time(wd_log, r"WATCHDOG_EXPIRED entity=Control_10ms")
    results["watchdog"] = {
        "stale_output_blocked_ms": (t_stale - t_stall) * 1000.0 if t_stale and t_stall else None,
        "watchdog_expired_ms": (t_expired - t_stall) * 1000.0 if t_expired and t_stall else None}
    return results


def measure_firmware(bins, out):
    p = port()
    flash = out / "flash.bin"
    flash.unlink(missing_ok=True)
    ecu = subprocess.Popen([bins / "vecu_cruise_ecu", "--bus", "udp", "--udp-port", str(p), "--duration", "60",
                            "--flash", str(flash)], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    bus = UdpCanBus(port=p)
    try:
        client = UdsClient(IsoTpEndpoint(bus))
        deadline = time.monotonic() + 5
        while True:
            try:
                client.read_data_by_identifier(0xF189)
                break
            except TimeoutError:
                if time.monotonic() > deadline:
                    raise
        header = fw.ImageHeader(hardware_id=0x0000CC01, version=(0, 7, 0), build_time=0)
        image = fw.build_image(header, fw.make_payload(64 * 1024, 0x000700))
        timing = client.flash_image(image)
        time.sleep(0.5)
        timing["version_after_reset"] = client.read_data_by_identifier(0xF189)[0xF189].decode()
        timing["image_bytes"] = len(image)
    finally:
        bus.close()
        log = stop(ecu)
    match = re.search(r"FW_SUMMARY (.*)", log)
    timing["ecu"] = dict(item.split("=", 1) for item in match.group(1).split()) if match else {}
    dcm = re.search(r"SCHED_SUMMARY task=Dcm_10ms .*", log)
    timing["dcm"] = dict(item.split("=", 1) for item in dcm.group(0).split()[1:]) if dcm else {}
    return timing


# ------------------------------------------------------------------------ report ----

def host_info(build_dir):
    def first_line(cmd):
        try:
            return subprocess.run(cmd, capture_output=True, text=True, timeout=10).stdout.splitlines()[0]
        except (OSError, IndexError):
            return "unknown"
    cpu = next((line.split(":", 1)[1].strip() for line in pathlib.Path("/proc/cpuinfo").read_text().splitlines()
                if line.startswith("model name")), "unknown")
    cache = (build_dir / "CMakeCache.txt").read_text() if (build_dir / "CMakeCache.txt").exists() else ""
    compiler = re.search(r"CMAKE_CXX_COMPILER:FILEPATH=(.*)", cache)
    build_type = re.search(r"CMAKE_BUILD_TYPE:STRING=(.*)", cache)
    commit = first_line(["git", "-C", str(REPO), "rev-parse", "--short", "HEAD"])
    dirty = subprocess.run(["git", "-C", str(REPO), "status", "--porcelain"], capture_output=True, text=True).stdout
    return {"date": datetime.datetime.now().strftime("%Y-%m-%d %H:%M"), "kernel": platform.release(),
            "cpu": cpu, "logical_cpus": os.cpu_count(),
            "compiler": first_line([compiler.group(1), "--version"]) if compiler else "unknown",
            "build_type": build_type.group(1) if build_type else "unknown",
            "python": platform.python_version(), "commit": commit + (" (uncommitted changes)" if dirty else "")}


def f(value, digits=1):
    return "–" if value is None else f"{value:.{digits}f}"


def sched_table(data):
    rows = ["| Task | Activations | Deadline misses | Exec mean / p99 / max [µs] | Start latency p50 / p99 / max [µs] "
            "| Period jitter max [µs] |", "|---|---|---|---|---|---|"]
    for t in data["tasks"]:
        rows.append(f"| {t['name']} | {t['activations']} | {t['deadline_misses']} | "
                    f"{f(t['execution_time']['mean_us'])} / {f(t['execution_p99_us'])} / {f(t['execution_time']['max_us'])} | "
                    f"{f(t['start_latency_p50_us'])} / {f(t['start_latency_p99_us'])} / {f(t['start_latency']['max_us'])} | "
                    f"{f(t['period_jitter']['max_us'])} |")
    return "\n".join(rows)


def write_report(path, host, r):
    kpi = r["closed_loop"]["kpi"]
    step = kpi["cruise_step_response"]["speed_responses"]
    step80 = next((s for s in step if s["target_kph"] == 80), step[0] if step else None)
    grade = kpi["cruise_grade"]["disturbances"]
    overrides = {o["kind"]: o for o in kpi["cruise_driver_override"]["overrides"]}
    faults = kpi["cruise_sensor_fault"]["faults"] + kpi["fault_management"]["faults"]
    fwr = r["firmware"]
    lines = [
        "# Performance Report",
        "",
        f"Generated by `scripts/run_performance.py` on {host['date']}. **Every value below was measured in that run**;",
        "rerun the script to reproduce. These are observations on a general-purpose, non-real-time OS",
        "(Linux under WSL2, `SCHED_OTHER`, no CPU isolation), **not guarantees**: other load on the host",
        "changes them, and a real ECU on an RTOS behaves differently.",
        "",
        "| Host | |", "|---|---|",
        f"| Kernel | {host['kernel']} |", f"| CPU | {host['cpu']} ({host['logical_cpus']} logical CPUs) |",
        f"| Compiler / build | {host['compiler']} / {host['build_type']} |", f"| Python | {host['python']} |",
        f"| Commit | {host['commit']} |",
        "",
        "## 1. Task execution time and scheduling jitter (`vecu_sched_bench`)",
        "",
        "Three periodic tasks (10/20/100 ms) with synthetic CPU work of 100/200/500 µs. *Start latency* is",
        "the delay between the planned release and the actual start; *jitter* is the deviation of the",
        "start-to-start interval from the period.",
    ]
    for label, data in r["scheduler"].items():
        lines += ["", f"**{label}**", "", sched_table(data)]
    lines += ["", "## 2. CAN message latency (`vecu_can_latency`)", "",
              "One-way latency of 5000 probe frames (1 ms period) between two channels on the UDP software bus:",
              "`send()` → kernel multicast loopback → receiver thread wake-up, timestamped with the same",
              "monotonic clock. A physical CAN bus adds the frame's transmission time (≈ 0.22–0.27 ms for 8 data",
              "bytes at 500 kbit/s), which a software bus does not model.", "",
              "| Bus traffic | Received | min | median | p90 | p99 | max [µs] |", "|---|---|---|---|---|---|---|"]
    for fps, d in r["can_latency"].items():
        label = "probes only" if int(fps) == 0 else f"+ {fps} background frames/s"
        lines.append(f"| {label} | {d['received']}/{d['sent']} | {f(d['min_us'])} | {f(d['p50_us'])} | "
                     f"{f(d['p90_us'])} | {f(d['p99_us'])} | {f(d['max_us'])} |")
    lines += ["", "## 3. Controller response (cruise ECU + vehicle simulator over CAN, real time)", "",
              "| Measurement | Result |", "|---|---|"]
    if step80:
        lines.append(f"| Step {f(step80['start_kph'], 0)} → {f(step80['target_kph'], 0)} km/h | rise time "
                     f"{f(step80['rise_time_s'], 2)} s, overshoot {f(step80['overshoot_kph'], 2)} km/h, settling (±0.5 km/h) "
                     f"{f(step80['settling_time_s'], 2)} s, steady-state error {f(step80['steady_state_error_kph'], 2)} km/h |")
    for g in grade:
        lines.append(f"| {f(g['grade_pct'], 0)} % uphill grade | max speed drop {f(g['max_deviation_kph'], 2)} km/h, "
                     f"recovered after {f(g['recovery_time_s'], 2)} s |")
    if "brake" in overrides:
        lines.append(f"| Brake pedal → cruise disengaged | {f(overrides['brake']['disengaged_after_ms'], 0)} ms |")
    if "accelerator" in overrides:
        lines.append(f"| Accelerator pedal → Override | {f(overrides['accelerator']['override_after_ms'], 0)} ms |")
    pace = {name: k["simulator"] for name, k in kpi.items()}
    late = {name: s for name, s in pace.items() if s["late_steps"]}
    lines += ["", "Simulator pace (validity of these measurements): " +
              ("all simulators kept their 10 ms step in every scenario." if not late else
               "host stall during the run, late steps: " +
               ", ".join(f"{n}: {s['late_steps']} (max {s['max_step_lateness_ms']:.1f} ms at {s['late_at_s']} s)"
                         for n, s in late.items()) + ". The values above include that disturbance.")]
    lines += ["", "## 4. Fault detection latency", "",
              "Time from the start of the fault on the bus (simulator) or in the ECU (test-only injection) to the",
              "reaction. Includes CAN cycle times (0x100: 20 ms, 0x110: 50 ms) and task periods.", "",
              "| Fault | Detected (cruise Fault on the bus) | Throttle cut |", "|---|---|---|"]
    names = {"mute_vehicle_dynamics": "VehicleDynamics (0x100) not transmitted (timeout 100 ms)",
             "invalid_speed": "speed signal marked invalid by the sender",
             "speed_spike": "speed jumps +30 km/h (gradient plausibility, 40 ms debounce)",
             "freeze_alive": "sender alive counter frozen (3 stale frames)"}
    for fault in faults:
        lines.append(f"| {names.get(fault['kind'], fault['kind'])} | {f(fault['fault_detected_after_ms'], 0)} ms | "
                     f"{f(fault['throttle_cut_after_ms'], 0)} ms |")
    wd = r["closed_loop"]["watchdog"]
    lines += [f"| Control_10ms stalled 400 ms | frozen output blocked after {f(wd['stale_output_blocked_ms'], 0)} ms; "
              f"watchdog expired after {f(wd['watchdog_expired_ms'], 0)} ms | (safe state) |",
              "", "## 5. ECU task timing during closed-loop operation", "",
              "`vecu_cruise_ecu --timing-report` of the step-response run (application tasks with real work):", "",
              sched_table(r["closed_loop"]["ecu_timing"]["cruise_step_response"]),
              "", "## 6. Diagnostic response time (UDS over ISO-TP, while cruising)", "",
              "Request sent → complete response received by the Python tester (P2 server = 50 ms).", "",
              "| Request | n | min | median | p95 | max [ms] |", "|---|---|---|---|---|---|"]
    for label, s in r["closed_loop"]["uds"].items():
        lines.append(f"| {label} | {s['n']} | {f(s['min'], 2)} | {f(s['median'], 2)} | {f(s['p95'], 2)} | {f(s['max'], 2)} |")
    lines += ["", "The Dcm task runs every 10 ms, so the response time is quantised in 10 ms steps; a",
              "multi-frame response needs one more Dcm cycle for the flow control.",
              "", "## 7. Firmware update (64 KiB image over UDS, ISO-TP block size 32)", "",
              "| Measurement | Result |", "|---|---|",
              f"| Image | {fwr['image_bytes']} B, version after reset {fwr['version_after_reset']} |",
              f"| Tester view | erase {f(fwr['erase_s'] * 1000)} ms, download {f(fwr['download_s'], 2)} s "
              f"(**{f(fwr['throughput_kib_s'])} KiB/s**), check {f(fwr['check_s'] * 1000)} ms, total {f(fwr['total_s'], 2)} s |",
              f"| ECU view (file flash) | erase {fwr['ecu'].get('max_erase_ms', '–')} ms, longest block write "
              f"{fwr['ecu'].get('max_block_ms', '–')} ms, check {fwr['ecu'].get('max_check_ms', '–')} ms, "
              f"trial boot {fwr['ecu'].get('boot_ms', '–')} ms |",
              f"| Dcm_10ms over the whole run | deadline misses {fwr['dcm'].get('deadline_misses', '–')}, budget overruns "
              f"{fwr['dcm'].get('budget_overruns', '–')} (budget 3 ms), max execution {fwr['dcm'].get('exec_max_us', '–')} µs |",
              "", "Raw data: `logs/performance/results.json` (not committed; regenerate with the script).", ""]
    path.write_text("\n".join(lines))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bin-dir", type=pathlib.Path, default=REPO / "build" / "bin")
    parser.add_argument("--out", type=pathlib.Path, default=REPO / "logs" / "performance")
    parser.add_argument("--report", type=pathlib.Path, default=REPO / "docs" / "performance_report.md")
    parser.add_argument("--sched-duration", default="10")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    bins = args.bin_dir.resolve()

    results = {}
    steps = [("scheduler", lambda: measure_scheduler(bins, args.out, args.sched_duration)),
             ("can_latency", lambda: measure_can_latency(bins, args.out)),
             ("closed_loop", lambda: measure_closed_loop(bins, args.out)),
             ("firmware", lambda: measure_firmware(bins, args.out))]
    for name, step in steps:
        start = time.monotonic()
        print(f"[performance] {name} ...", flush=True)
        results[name] = step()
        print(f"[performance] {name} done in {time.monotonic() - start:.1f} s", flush=True)
    host = host_info(bins.parent)
    (args.out / "results.json").write_text(json.dumps({"host": host, "results": results}, indent=1, default=str))
    write_report(args.report, host, results)
    print(f"[performance] report: {args.report}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
