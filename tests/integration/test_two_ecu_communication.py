"""Multi-process integration tests: two virtual ECUs communicating over the CAN bus.

Each test starts the real ECU executables as separate processes (like separate ECUs
on a vehicle network) on an isolated bus and evaluates their SUMMARY / event logs.

Environment:
  VECU_BIN_DIR  directory with vecu_sensor_ecu / vecu_cruise_ecu (default: build/bin)
"""

import os
import pathlib
import random
import re
import subprocess
import time

import pytest

from vecu_sim.udp_can_bus import UdpCanBus

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
BIN_DIR = pathlib.Path(os.environ.get("VECU_BIN_DIR", REPO_ROOT / "build" / "bin"))
SENSOR = BIN_DIR / "vecu_sensor_ecu"
CRUISE = BIN_DIR / "vecu_cruise_ecu"

pytestmark = pytest.mark.skipif(not SENSOR.exists() or not CRUISE.exists(),
                                reason=f"ECU binaries not built in {BIN_DIR}")

BACKENDS = ["udp"]
if pathlib.Path("/sys/class/net/vcan0").exists():
    BACKENDS.append("socketcan")


def bus_args(backend, port):
    if backend == "udp":
        return ["--bus", "udp", "--udp-port", str(port)]
    return ["--bus", "socketcan", "--iface", "vcan0"]


def start(binary, *args):
    return subprocess.Popen([str(binary), *args], stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, text=True)


def finish(process, timeout=15):
    _, stderr = process.communicate(timeout=timeout)
    assert process.returncode == 0, stderr
    return stderr


def summary(log):
    match = re.search(r"SUMMARY (.*)", log)
    assert match, f"no SUMMARY line in:\n{log}"
    fields = dict(item.split("=", 1) for item in match.group(1).split())
    return {k: (float(v) if re.fullmatch(r"-?[\d.]+", v) else v) for k, v in fields.items()}


@pytest.fixture
def port():
    return random.randint(40000, 59999)  # isolated bus per test


@pytest.mark.parametrize("backend", BACKENDS)
def test_two_ecus_exchange_periodic_messages(backend, port):
    # Sensor runs longer than cruise, so the sensor must detect the cruise ECU going silent.
    sensor = start(SENSOR, *bus_args(backend, port), "--duration", "3.5")
    cruise = start(CRUISE, *bus_args(backend, port), "--duration", "3.0")
    cruise_log, sensor_log = finish(cruise), finish(sensor)
    c, s = summary(cruise_log), summary(sensor_log)

    # ~3 s of overlap: 0x100 @ 20 ms -> ~150 frames, 0x110 @ 50 ms -> ~60, 0x200 @ 20 ms -> ~150.
    assert c["rx_0x100"] >= 120
    assert c["rx_0x110"] >= 48
    assert s["rx_0x200"] >= 120
    assert c["tx_failed"] == 0 and s["tx_failed"] == 0
    assert c["alive_errors"] == 0 and s["alive_errors"] == 0
    assert c["invalid"] == 0 and s["invalid"] == 0
    assert c["timeouts"] == 0, cruise_log
    assert s["timeouts"] == 1, sensor_log      # cruise ECU stopped before the sensor ECU
    assert "CRUISE_STATE Off -> Standby" in cruise_log  # main switch on after 1 s


@pytest.mark.parametrize("backend", BACKENDS)
def test_timeout_detection_and_recovery(backend, port):
    # Fault injection: the sensor ECU stops sending 0x100 between t=1.0 s and t=1.5 s.
    sensor = start(SENSOR, *bus_args(backend, port), "--duration", "3.0", "--mute-tx", "1.0:1.5")
    cruise = start(CRUISE, *bus_args(backend, port), "--duration", "2.7")
    cruise_log, _ = finish(cruise), finish(sensor)
    c = summary(cruise_log)

    timeout = re.search(r"RX_TIMEOUT id=0x100 silence_ms=([\d.]+)", cruise_log)
    assert timeout, cruise_log
    assert "RX_RECOVERED id=0x100" in cruise_log
    assert "RX_TIMEOUT id=0x110" not in cruise_log  # only the muted message is affected
    assert "CRUISE_STATE Standby -> Fault" in cruise_log
    assert "CRUISE_STATE Fault -> Standby" in cruise_log
    assert c["timeouts"] == 1 and c["recoveries"] == 1
    assert c["alive_errors"] == 0  # counter resynchronised after the timeout

    # Detection latency = configured 100 ms timeout + at most one 10 ms supervision tick,
    # plus OS scheduling noise. The measured value is printed for the record.
    silence_ms = float(timeout.group(1))
    print(f"measured timeout detection: silence_ms={silence_ms}")
    assert 100.0 <= silence_ms < 200.0


def test_python_node_decodes_frames_from_cpp_ecu(port):
    """Cross-language check: C++ encoder -> UDP bus -> Python decoder."""
    with UdpCanBus(port=port) as bus:
        sensor = start(SENSOR, "--bus", "udp", "--udp-port", str(port), "--duration", "1.5")
        frames = []
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            frame = bus.recv(timeout=0.2)
            if frame is not None:
                frames.append(frame)
        finish(sensor)

    dynamics = [f for f in frames if f.can_id == 0x100]
    assert len(dynamics) >= 40
    assert any(f.can_id == 0x110 for f in frames)
    for frame in dynamics:
        assert len(frame.data) == 8
        speed_kph = int.from_bytes(frame.data[0:2], "little") * 0.01
        assert 0.0 <= speed_kph <= 300.0
    counters = [f.data[7] & 0x0F for f in dynamics]
    assert all((b - a) % 16 == 1 for a, b in zip(counters, counters[1:])), counters
