"""CAN message latency benchmark (vecu_can_latency): every probe frame must arrive, and the
summary line must be complete. The measured values themselves are reported, not asserted
(see docs/performance_report.md): they depend on the host."""

import json
import os
import pathlib
import random
import re
import subprocess

import pytest

REPO = pathlib.Path(__file__).resolve().parents[2]
BIN_DIR = pathlib.Path(os.environ.get("VECU_BIN_DIR", REPO / "build" / "bin"))
TOOL = BIN_DIR / "vecu_can_latency"

pytestmark = pytest.mark.skipif(not TOOL.exists(), reason=f"{TOOL} not built")


@pytest.mark.parametrize("background_fps", [0, 2000])
def test_every_probe_arrives_and_latency_is_reported(tmp_path, background_fps):
    report = tmp_path / "latency.json"
    result = subprocess.run([str(TOOL), "--bus", "udp", "--udp-port", str(random.randint(40000, 59999)),
                             "--count", "1000", "--period-us", "1000", "--background-fps", str(background_fps),
                             "--json", str(report)], capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stdout + result.stderr
    summary = re.search(r"CAN_LATENCY_SUMMARY (.*)", result.stdout).group(1)
    data = json.loads(report.read_text())
    assert data["sent"] == data["received"] == 1000 and data["out_of_order"] == 0
    assert 0 < data["min_us"] <= data["p50_us"] <= data["p99_us"] <= data["max_us"]
    print(f"\n[can latency] {summary}")
