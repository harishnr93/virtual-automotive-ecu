"""Scheduler timing tests using the vecu_sched_bench executable.

The benchmark prints measured values; these tests check *behaviour* (activation counts,
deadline-miss detection, report format) and record the measurements in the test output.
They deliberately do not assert tight latency bounds: WSL2 is not a real-time OS.
"""

import json
import os
import pathlib
import re
import subprocess

import pytest

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
BIN_DIR = pathlib.Path(os.environ.get("VECU_BIN_DIR", REPO_ROOT / "build" / "bin"))
BENCH = BIN_DIR / "vecu_sched_bench"

pytestmark = pytest.mark.skipif(not BENCH.exists(), reason=f"{BENCH} not built")


def run_bench(*args, timeout=60):
    result = subprocess.run([str(BENCH), *args], capture_output=True, text=True, timeout=timeout)
    assert result.returncode == 0, result.stderr
    print(result.stdout)  # visible with pytest -s: the measured table
    tasks = {}
    for line in result.stdout.splitlines():
        if line.startswith("SCHED_SUMMARY "):
            fields = dict(item.split("=", 1) for item in line.split()[1:])
            tasks[fields["task"]] = {k: (float(v) if re.fullmatch(r"-?[\d.]+", v) else v)
                                     for k, v in fields.items()}
    return tasks


def test_light_load_runs_all_tasks_at_their_rates(tmp_path):
    report = tmp_path / "timing.json"
    tasks = run_bench("--duration", "3", "--json", str(report), "--label", "pytest-light")

    assert set(tasks) == {"Task_10ms", "Task_20ms", "Task_100ms"}
    # 3 s run -> ~300 / 150 / 30 activations (small tolerance for start/stop edges).
    assert abs(tasks["Task_10ms"]["activations"] - 300) <= 4
    assert abs(tasks["Task_20ms"]["activations"] - 150) <= 3
    assert abs(tasks["Task_100ms"]["activations"] - 30) <= 2
    for name, task in tasks.items():
        # Execution time is dominated by the configured busy-wait.
        assert task["exec_mean_us"] > 0
        # Misses are possible on a non-RT OS but must stay rare under light load.
        assert task["deadline_misses"] <= 0.01 * task["activations"], name

    data = json.loads(report.read_text())
    assert data["label"] == "pytest-light"
    assert [t["name"] for t in data["tasks"]] == ["Task_10ms", "Task_20ms", "Task_100ms"]
    assert data["tasks"][0]["period_us"] == 10000
    assert 0.0 < data["estimated_utilization"] < 1.0


def test_overloaded_task_is_detected_and_does_not_affect_others():
    # 15 ms of work in a 10 ms task: every activation misses its deadline.
    tasks = run_bench("--duration", "2", "--load-10ms", "15000")
    fast = tasks["Task_10ms"]
    assert fast["activations"] > 50
    assert fast["deadline_misses"] == fast["activations"]
    assert fast["exec_mean_us"] >= 15000
    # Each task has its own thread; on a multi-core host the others keep their deadlines.
    assert tasks["Task_100ms"]["deadline_misses"] == 0


def test_realtime_request_falls_back_or_succeeds():
    tasks = run_bench("--duration", "1", "--realtime")
    policies = {task["policy"] for task in tasks.values()}
    assert policies <= {"SCHED_FIFO", "SCHED_OTHER"} and len(policies) == 1
