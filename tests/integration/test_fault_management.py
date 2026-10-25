"""System tests of fault management, NVM persistence and the software watchdog.

Faults are injected deterministically at defined times:
  - by the vehicle simulator (scenario fault_management.json): speed spike, frozen alive counter
  - inside the ECU (--inject, test-only): task overrun, task stall, task hang, controller NaN
and observed through UDS (DTCs, DIDs), the ECU log and the simulator's CAN recording.
Measured latencies are printed (pytest -s) and recorded in docs/phase5_fault_management.md.
"""

import csv
import os
import pathlib
import random
import re
import subprocess
import sys
import time
import zlib

import pytest

from vecu_sim.isotp import IsoTpEndpoint
from vecu_sim.udp_can_bus import UdpCanBus
from vecu_sim.uds_client import UdsClient, decode_did

REPO = pathlib.Path(__file__).resolve().parents[2]
BIN_DIR = pathlib.Path(os.environ.get("VECU_BIN_DIR", REPO / "build" / "bin"))
ECU = BIN_DIR / "vecu_cruise_ecu"
SCENARIO = REPO / "config" / "scenarios" / "fault_management.json"

pytestmark = pytest.mark.skipif(not ECU.exists(), reason=f"{ECU} not built")

CONFIRMED, TEST_FAILED = 0x08, 0x01
DTC = {"speed_missing": 0xC12187, "driver_missing": 0xC14087, "implausible": 0x403164, "stale": 0xC12182,
       "deadline": 0xF00048, "controller": 0xF00049, "watchdog": 0xF00047, "nvm": 0xF00044}


# --------------------------------------------------------------------------- helpers ----

def start_ecu(port, nvm, duration, *extra):
    args = [str(ECU), "--bus", "udp", "--udp-port", str(port), "--duration", str(duration), "--nvm", str(nvm), *extra]
    return subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)


def open_tester(port):
    bus = UdpCanBus(port=port)
    return bus, UdsClient(IsoTpEndpoint(bus))


def did(client, number):
    return decode_did(number, client.read_data_by_identifier(number)[number])


def dtcs(client, mask=0x00):
    return dict(client.read_supported_dtcs()) if mask == 0 else dict(client.read_dtc_by_status_mask(mask))


def wait_for(predicate, timeout, interval=0.05):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            value = predicate()
        except TimeoutError:
            value = None
        if value:
            return value
        time.sleep(interval)
    return None


def confirmed_and_healed(client, code):
    status = dtcs(client).get(code, 0)
    return (status & CONFIRMED) and not (status & TEST_FAILED)


def log_time(log, pattern):
    """Epoch-relative time 't=...s' of the first log line matching `pattern`."""
    match = re.search(pattern + r".*?t=([\d.]+)s", log)
    return float(match.group(1)) if match else None


# -------------------------------------------------------- main fault-injection scenario ----

@pytest.fixture(scope="module")
def fault_run(tmp_path_factory):
    out = tmp_path_factory.mktemp("faults")
    nvm = out / "nvm.bin"
    port = random.randint(40000, 59999)
    ecu = start_ecu(port, nvm, 35, "--inject", "overrun:Com_20ms:14.5:1.0:25",
                    "--inject", "stall:Control_10ms:21.0:0.4", "--inject", "controller-nan:27.0")
    time.sleep(0.5)
    sim = subprocess.Popen([sys.executable, "-m", "vecu_sim.simulator", "--scenario", str(SCENARIO), "--bus", "udp",
                            "--udp-port", str(port), "--csv", str(out / "run.csv"), "--report", str(out / "run.json")],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                           env={**os.environ, "PYTHONPATH": str(REPO / "simulator")})
    bus, client = open_tester(port)
    obs = {}
    try:
        obs["engaged"] = wait_for(lambda: did(client, 0x0101) == 2, 5.0)
        obs["implausible_healed"] = wait_for(lambda: confirmed_and_healed(client, DTC["implausible"]), 10.0)
        obs["stale_healed"] = wait_for(lambda: confirmed_and_healed(client, DTC["stale"]), 10.0)
        obs["deadline_active_mask"] = wait_for(lambda: did(client, 0x0300) & (1 << 4), 10.0)
        obs["deadline_healed"] = wait_for(lambda: confirmed_and_healed(client, DTC["deadline"]), 6.0)
        obs["watchdog_confirmed"] = wait_for(lambda: dtcs(client).get(DTC["watchdog"], 0) & CONFIRMED, 8.0)
        obs["watchdog_status_after"] = wait_for(
            lambda: (lambda s: s if s[0] == 0 else None)(did(client, 0x0301)), 3.0)
        obs["reset_reason_after_watchdog"] = did(client, 0x0302)
        obs["controller_mask"] = wait_for(lambda: did(client, 0x0300) & (1 << 5), 8.0)
        obs["controller_reason"] = did(client, 0x0104)
        obs["controller_dtc"] = dtcs(client).get(DTC["controller"], 0)
        client.ecu_reset(0x01)  # UDS hard reset: new operation cycle clears the latched fault
        obs["mask_after_reset"] = wait_for(lambda: (lambda m: str(m) if not m & (1 << 5) else None)(did(client, 0x0300)), 2.0)
        obs["reengaged_after_reset"] = wait_for(lambda: did(client, 0x0101) == 2, 6.0)
        obs["reset_count"] = did(client, 0x0202)
        obs["all_dtcs"] = dtcs(client)
    finally:
        bus.close()
        sim_out, _ = sim.communicate(timeout=90)
        _, log = ecu.communicate(timeout=90)
    obs["log"], obs["sim_out"], obs["nvm"], obs["csv"] = log, sim_out, nvm, out / "run.csv"
    print(sim_out)
    for line in log.splitlines():
        if re.search(r"FAULT |WATCHDOG|SAFE_STATE|CONTROL_OUTPUT|ECU_RESET|FAULT_INJECTION", line):
            print(line)
    return obs


def test_signal_faults_are_qualified_reported_and_recover(fault_run):
    assert fault_run["engaged"]
    assert fault_run["implausible_healed"], "C0031-64 (speed spike, gradient check) not confirmed/healed"
    assert fault_run["stale_healed"], "U0121-82 (frozen alive counter) not confirmed/healed"
    log = fault_run["log"]
    assert "FAULT SpeedSignalInvalid PreFailed -> Failed" in log
    assert "FAULT SpeedSignalStale PreFailed -> Failed" in log
    summary = re.search(r"rejected_speed=(\d+)", log)
    assert summary and int(summary.group(1)) > 0  # spike samples were discarded, not used
    # simulator side: throttle cut on the bus, recovery to Standby (never directly Active)
    sim_faults = {f["kind"]: f for f in __import__("json").loads((fault_run["csv"].parent / "run.json").read_text())["faults"]}
    for kind in ("speed_spike", "freeze_alive"):
        f = sim_faults[kind]
        print(f"[latency] {kind}: Fault on the bus after {f['fault_detected_after_ms']:.0f} ms")
        assert f["fault_detected_after_ms"] is not None and f["fault_detected_after_ms"] < 200
        assert f["throttle_cut_after_ms"] <= f["fault_detected_after_ms"]
        assert f["state_after_fault"] == 1


def test_deadline_misses_inhibit_function_and_recover(fault_run):
    assert fault_run["deadline_active_mask"], "TaskDeadlineMiss never active (DID 0x0300 bit 4)"
    assert fault_run["deadline_healed"]
    log = fault_run["log"]
    t_inject = log_time(log, r"FAULT_INJECTION overrun")
    t_failed = log_time(log, r"FAULT TaskDeadlineMiss PreFailed -> Failed")
    t_healed = log_time(log, r"FAULT TaskDeadlineMiss PrePassed -> Passed")
    print(f"[latency] deadline-miss fault qualified {1000 * (t_failed - t_inject):.0f} ms after the overrun started; "
          f"healed {t_healed - (t_inject + 1.0):.2f} s after it ended")
    assert t_failed - t_inject < 0.5
    assert 0.9 <= t_healed - (t_inject + 1.0) < 1.5  # recovery condition: 10 clean 100 ms windows
    assert "reason=FunctionInhibited" in log


def test_watchdog_detects_stall_enters_safe_state_and_recovers(fault_run):
    log = fault_run["log"]
    t_stall = log_time(log, r"FAULT_INJECTION stall task=Control_10ms")
    t_stale = log_time(log, r"CONTROL_OUTPUT_STALE")
    t_expired = log_time(log, r"WATCHDOG_EXPIRED entity=Control_10ms violation=ExecutionTimeout")
    print(f"[latency] stall -> stale control output blocked on the bus: {1000 * (t_stale - t_stall):.0f} ms; "
          f"-> watchdog expired: {1000 * (t_expired - t_stall):.0f} ms")
    assert t_stale - t_stall < 0.07
    assert t_expired - t_stall < 0.1
    for marker in ("SAFE_STATE entered", "WATCHDOG_RESUMED", "ECU_RESET type=watchdog", "SAFE_STATE left"):
        assert marker in log, marker
    assert fault_run["watchdog_confirmed"]
    status = fault_run["watchdog_status_after"]
    assert status is not None and status[0] == 0 and int.from_bytes(status[1:3], "big") == 1  # Ok, 1 expiration
    assert fault_run["reset_reason_after_watchdog"] == 3

    # On the bus: Active -> Fault with reason 8 (stale output) / 7 (watchdog) and never throttle.
    with fault_run["csv"].open() as file:
        rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(file)]
    safe = [r for r in rows if r["fault_reason"] in (7, 8)]
    assert safe and max(r["cruise_throttle_pct"] for r in safe) == 0.0
    assert {int(r["fault_reason"]) for r in safe} == {7, 8}


def test_latched_controller_fault_needs_reset(fault_run):
    assert fault_run["controller_mask"], "ControllerFault never active"
    assert fault_run["controller_reason"] in (5, 6)  # ControllerFault / FunctionInhibited
    assert fault_run["controller_dtc"] & CONFIRMED
    assert fault_run["mask_after_reset"] is not None, "UDS hard reset did not release the latched fault"
    assert fault_run["reengaged_after_reset"]
    assert fault_run["reset_count"] == 2  # watchdog reset + UDS hard reset


def test_all_fault_classes_left_confirmed_dtcs(fault_run):
    confirmed = {code for code, status in fault_run["all_dtcs"].items() if status & CONFIRMED}
    for name in ("implausible", "stale", "deadline", "watchdog", "controller"):
        assert DTC[name] in confirmed, name
    assert DTC["nvm"] not in confirmed


# ------------------------------------------------- NVM persistence and aging across restarts ----

def run_and_read(nvm, *reads, duration=2.5, extra=()):
    port = random.randint(40000, 59999)
    ecu = start_ecu(port, nvm, duration, *extra)
    time.sleep(0.6)
    bus, client = open_tester(port)
    try:
        values = [read(client) for read in reads]
    finally:
        bus.close()
        _, log = ecu.communicate(timeout=30)
    return values, log, ecu.returncode


def test_fault_memory_survives_restart_and_ages(fault_run):
    nvm = fault_run["nvm"]
    (statuses, counters), log, _ = run_and_read(
        nvm, lambda c: dtcs(c), lambda c: (did(c, 0x0202), did(c, 0x0302), did(c, 0x0303), did(c, 0x0304)))
    assert "NVM_LOADED" in log and "last_reset=PowerOn" in log  # clean shutdown before
    resets, reason, cycles, nvm_status = counters
    assert (resets, reason, cycles, nvm_status) == (2, 0, 4, 0)
    for name in ("implausible", "stale", "deadline", "watchdog", "controller"):
        assert statuses[DTC[name]] & CONFIRMED, name  # still confirmed after a restart

    # Aging (threshold 3 clean operation cycles): the spike fault last failed in cycle 1;
    # cycles 2, 3 and 4 were clean -> it ages out at the start of cycle 5. The controller
    # fault failed in cycle 2 -> still confirmed in cycle 5.
    (statuses,), _, _ = run_and_read(nvm, lambda c: dtcs(c))
    print(f"[aging] cycle 5 statuses: implausible=0x{statuses[DTC['implausible']]:02X} "
          f"controller=0x{statuses[DTC['controller']]:02X}")
    assert not statuses[DTC["implausible"]] & CONFIRMED
    assert statuses[DTC["implausible"]] & 0x20  # testFailedSinceLastClear: history kept
    assert statuses[DTC["controller"]] & CONFIRMED


def test_corrupted_nvm_is_detected_and_rewritten(tmp_path):
    nvm = tmp_path / "nvm.bin"
    nvm.write_bytes(b"VNVM\x01\x00garbage-after-a-torn-write")
    (values,), log, rc = run_and_read(nvm, lambda c: (did(c, 0x0304), dtcs(c).get(DTC["nvm"], 0)))
    assert rc == 0
    assert "NVM_CORRUPTED" in log
    assert values[0] == 2 and values[1] & CONFIRMED  # status Corrupted, DTC U3000-44
    # The ECU rewrote a well-formed block: check it independently with Python's zlib CRC-32
    # (same IEEE polynomial), which also cross-checks the C++ CRC implementation.
    data = nvm.read_bytes()
    count = int.from_bytes(data[6:8], "little")
    assert data[:4] == b"VNVM" and int.from_bytes(data[4:6], "little") == 1
    assert len(data) == 20 + 8 * count + 4
    assert zlib.crc32(data[:-4]) == int.from_bytes(data[-4:], "little")


# ------------------------------------------------------------- watchdog: restart and hang ----

def test_watchdog_exit_action_restarts_process_and_records_reason(tmp_path):
    nvm = tmp_path / "nvm.bin"
    port = random.randint(40000, 59999)
    start = time.monotonic()
    ecu = start_ecu(port, nvm, 10, "--inject", "hang:Control_10ms:1.0", "--watchdog-action", "exit")
    _, log = ecu.communicate(timeout=30)
    elapsed = time.monotonic() - start
    t_hang = log_time(log, r"FAULT_INJECTION hang")
    t_expired = log_time(log, r"WATCHDOG_EXPIRED")
    print(f"[latency] hang -> watchdog expired {1000 * (t_expired - t_hang):.0f} ms; process exited after {elapsed:.2f} s")
    assert ecu.returncode == 75, log
    assert "EMERGENCY_EXIT code=75" in log
    assert elapsed < 4.0

    # "External supervisor" restarts the ECU: the reset reason survives in NVM.
    (values,), log2, _ = run_and_read(nvm, lambda c: (did(c, 0x0302), did(c, 0x0202),
                                                      dtcs(c).get(DTC["watchdog"], 0)))
    assert "last_reset=Watchdog" in log2
    reason, resets, wd_status = values
    assert reason == 3 and resets == 1 and wd_status & CONFIRMED


def test_permanent_hang_keeps_safe_state_and_shuts_down_cleanly(tmp_path):
    (values,), log, rc = run_and_read(tmp_path / "nvm.bin",
                                      lambda c: wait_for(lambda: (lambda s, r: (s, r) if r == 7 else None)(
                                          did(c, 0x0101), did(c, 0x0104)), 3.0),
                                      duration=4.0, extra=("--inject", "hang:Control_10ms:1.0"))
    assert rc == 0, "shutdown must not deadlock on the hung task"
    assert values == (4, 7)  # Fault, WatchdogSafeState
    assert "SAFE_STATE entered" in log and "WATCHDOG_RESUMED" not in log
