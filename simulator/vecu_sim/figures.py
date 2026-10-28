"""Result figures built from measured data (simulator records, ECU logs, CAN traces, benchmark
results). scripts/make_figures.py runs the simulations and calls these functions; the functions
themselves only draw, so they can be tested with synthetic data.

Colour roles (fixed for all figures):
  cruise state   Off = neutral, Standby = slot 1, Active = slot 3, Override = slot 4,
                 Fault = status "critical" (always with its text label)
  firmware       Confirmed = status "good", Invalid = status "critical", others categorical
"""

from __future__ import annotations

import re
from typing import Dict, List, Optional, Sequence, Tuple

from .svg_charts import Figure, Lanes, Panel, cdf, color, ink, nice_ticks, percentile, status

W = 960
LEFT = 150       # room for lane labels
RIGHT = 30
PLOT_W = W - LEFT - RIGHT

CRUISE_STATES = {0: ("Off", ink("neutral"), "ink"), 1: ("Standby", color(1), False), 2: ("Active", color(3), False),
                 3: ("Override", color(4), True), 4: ("Fault", status("critical"), False)}
SLOT_STATES = {"Empty": (ink("neutral"), "ink"), "Receiving": (color(4), True), "Pending": (color(7), False),
               "Trial": (color(2), False), "Confirmed": (status("good"), False), "Invalid": (status("critical"), False)}
FAULT_LABELS = {"mute_vehicle_dynamics": "0x100 lost", "invalid_speed": "speed invalid",
                "speed_spike": "speed spike", "freeze_alive": "alive frozen"}


# ------------------------------------------------------------------------- helpers ----

def state_segments(times: Sequence[float], states: Sequence[int]) -> List[Tuple[float, float, int]]:
    """Run-length segments (start, end, state) of a sampled state signal."""
    segments: List[Tuple[float, float, int]] = []
    if not times:
        return segments
    start = 0
    for i in range(1, len(times) + 1):
        if i == len(times) or states[i] != states[start]:
            end = times[i] if i < len(times) else times[-1]
            segments.append((times[start], end, int(states[start])))
            start = i
    return segments


def scenario_marks(scenario: dict) -> Tuple[List[Tuple[float, str]], List[Tuple[float, float, str]]]:
    """Driver events (time, label) and fault windows (start, end, label) of a scenario."""
    events, faults = [], []
    for e in scenario.get("events", []):
        t = float(e["t"])
        if "fault" in e:
            faults.append((t, t + float(e.get("duration_s", 0.0)), FAULT_LABELS.get(e["fault"], e["fault"])))
        elif e.get("set"):
            events.append((t, "set"))
        elif e.get("brake_pct", 0) > 0:
            events.append((t, "brake"))
        elif e.get("accelerator_pct", 0) > 0:
            events.append((t, f"pedal {e['accelerator_pct']:g} %"))
        elif "set_speed_kph" in e:
            events.append((t, f"set speed {e['set_speed_kph']:g}"))
        elif e.get("grade_pct", 0) > 0:
            events.append((t, f"grade {e['grade_pct']:g} %"))
        elif e.get("main_switch"):
            events.append((t, "main switch"))
    return events, faults


def cruise_state_legend(fig: Figure, x: float, y: float, used: Sequence[int]) -> None:
    fig.legend([("box", CRUISE_STATES[s][1], CRUISE_STATES[s][0]) for s in sorted(set(used))], x, y, gap=14)


def time_ticks(t_max: float) -> List[float]:
    return nice_ticks(0.0, t_max, 8 if t_max > 30 else 10)


# ---------------------------------------------------------------- closed loop run ----

def closed_loop_figure(records: List[Dict[str, float]], scenario: dict, title: str, subtitle: str = "") -> Figure:
    t = [r["t"] for r in records]
    t_max = t[-1] if t else 1.0
    has_grade = any(r["grade_pct"] for r in records)
    events, faults = scenario_marks(scenario)
    lane_labels = ["Cruise state (0x200)"] + (["Injected fault"] if faults else []) + ["Driver / road"]

    top = 112
    speed_h, thr_h, grade_h = 210, 110, 60
    speed = Panel(LEFT, top, PLOT_W, speed_h, (0, t_max), (0, 1))
    speeds = [r["speed_kph"] for r in records] + [r["target_kph"] for r in records if r["target_kph"] > 0]
    s_ticks = nice_ticks(max(0.0, min(speeds) - 2), max(speeds) + 2, 5, cover=True)
    speed.ylim = (s_ticks[0], s_ticks[-1])
    thr = Panel(LEFT, top + speed_h + 40, PLOT_W, thr_h, (0, t_max), (0, 100))
    y_next = thr.y + thr_h + 40
    grade: Optional[Panel] = None
    if has_grade:
        grade = Panel(LEFT, y_next, PLOT_W, grade_h, (0, t_max), (0, max(5.0, max(r["grade_pct"] for r in records))))
        y_next += grade_h + 40
    lanes = Lanes(LEFT, y_next, PLOT_W, 24, (0, t_max), lane_labels)
    height = lanes.y + lanes.h + 80

    fig = Figure(W, height, title, subtitle)
    ticks = time_ticks(t_max)
    for panel in [speed, thr] + ([grade] if grade else []):
        panel.x_grid(ticks)
    speed.y_grid(s_ticks, "Speed [km/h]")
    speed.x_axis(ticks, show_labels=False)
    target_t = [r["t"] for r in records]
    target_v = [r["target_kph"] if r["target_kph"] > 0 else None for r in records]
    speed.line(target_t, target_v, color(2), step=True)
    speed.line(t, [r["speed_kph"] for r in records], color(1))
    thr.y_grid([0, 25, 50, 75, 100], "Throttle and pedals [%]")
    thr.x_axis(ticks, show_labels=False)
    pedals = [(key, slot, label) for key, slot, label in (("accelerator_pct", 5, "accelerator pedal"),
                                                          ("brake_pct", 7, "brake pedal"))
              if any(r[key] > 0 for r in records)]  # a pedal that is never pressed is not drawn
    thr.line(t, [r["applied_throttle_pct"] for r in records], color(3))
    for key, slot, _ in pedals:  # on top: during manual driving the throttle equals the pedal
        thr.line(t, [r[key] for r in records], color(slot))
    if grade:
        grade.y_grid(nice_ticks(0, grade.ylim[1], 2), "Road grade [%]")
        grade.x_axis(ticks, show_labels=False)
        grade.line(t, [r["grade_pct"] for r in records], color(6), step=True)

    lanes.frame()
    for start, end, state in state_segments(t, [int(r["cruise_state"]) for r in records]):
        name, fill, dark_text = CRUISE_STATES.get(state, ("?", ink("neutral"), "ink"))
        lanes.segment(0, start, end, fill, name, dark_text)
    row = 1
    if faults:
        for start, end, label in faults:
            lanes.segment(row, start, max(end, start + t_max / 200), status("critical"))
            lanes.parts.append(f'<text class="t2" x="{lanes.sx(end) + 6:.1f}" y="{lanes.y + row * 24 + 16:.1f}" '
                               f'font-size="10.5">{label}</text>')
        row += 1
    # Events closer than their label's width share one label (no overlapping text).
    merged: List[Tuple[float, str]] = []
    for when, label in sorted(events):
        # label starts 10 px right of its marker; ~6 px per character at 10.5 px; 8 px margin
        if merged and lanes.sx(when) - 6 < lanes.sx(merged[-1][0]) + 10 + 6.0 * len(merged[-1][1]) + 8:
            lanes.event(row, when, ink("text2"))
            merged[-1] = (merged[-1][0], f"{merged[-1][1]}, {label}")
        else:
            merged.append((when, label))
    for when, label in merged:
        lanes.event(row, when, ink("text2"), label)
    # time axis under the lanes
    for tick in ticks:
        fig.note(lanes.sx(tick), lanes.y + lanes.h + 15, f"{tick:g}", anchor="middle")
    fig.note(LEFT + PLOT_W, lanes.y + lanes.h + 30, "time [s]", anchor="end")

    fig.legend([("line", color(1), "vehicle speed"), ("line", color(2), "cruise target"),
                ("line", color(3), "applied throttle")] + [("line", color(s), label) for _, s, label in pedals] +
               ([("line", color(6), "road grade")] if grade else []), LEFT, 74, gap=16)
    cruise_state_legend(fig, LEFT, height - 22, [int(r["cruise_state"]) for r in records])
    for panel in [speed, thr] + ([grade] if grade else []):
        fig.add(panel)
    fig.add(lanes)
    return fig


# --------------------------------------------------------------------- CAN trace ----

def decode_cruise_state(data_hex: str) -> int:
    """CruiseState: bits 16..18 (Intel) of 0x200."""
    data = bytes.fromhex(data_hex)
    return (data[2] & 0x07) if len(data) > 2 else 0


def bus_trace_figure(frames: List[Tuple[float, int, str]], window: Tuple[float, float], title: str,
                     subtitle: str = "", fault_window: Optional[Tuple[float, float, str]] = None) -> Figure:
    """frames: (time s, id, data hex). Lanes: one per identifier; 0x200 ticks coloured by state."""
    ids = [(0x100, "0x100 VehicleDynamics"), (0x110, "0x110 DriverInputs"), (0x200, "0x200 CruiseStatus")]
    lanes = Lanes(LEFT, 90, PLOT_W, 34, window, [label for _, label in ids])
    fig = Figure(W, lanes.y + lanes.h + 100, title, subtitle)
    lanes.frame()
    if fault_window:  # (the figure height leaves room for the detection note under the axis)
        a, b, label = fault_window
        fig.raw(f'<rect x="{lanes.sx(a):.1f}" y="{lanes.y:.1f}" width="{lanes.sx(b) - lanes.sx(a):.1f}" '
                f'height="{lanes.h:.1f}" fill="{status("critical")}" opacity="0.10"/>')
        fig.note(lanes.sx(a) + 4, lanes.y - 6, label, cls="t2")
    used = []
    for t, frame_id, data in frames:
        if not window[0] <= t <= window[1]:
            continue
        for row, (fid, _) in enumerate(ids):
            if frame_id == fid:
                if fid == 0x200:
                    state = decode_cruise_state(data)
                    used.append(state)
                    lanes.tick(row, t, CRUISE_STATES.get(state, ("", ink("text2"), True))[1], 0.7)
                else:
                    lanes.tick(row, t, color(1), 0.7)
    fig.add(lanes)
    for tick in nice_ticks(window[0], window[1], 8):
        fig.note(lanes.sx(tick), lanes.y + lanes.h + 15, f"{tick:g}", anchor="middle")
    fig.note(LEFT + PLOT_W, lanes.y + lanes.h + 30, "time [s]", anchor="end")
    if fault_window:
        first_fault = next((t for t, fid, data in frames if fid == 0x200 and t >= fault_window[0]
                            and decode_cruise_state(data) == 4), None)
        if first_fault is not None:
            xx = lanes.sx(first_fault)
            yy = lanes.y + 2 * lanes.row_h
            below = lanes.y + lanes.h + 36  # under the time-axis labels
            fig.raw(f'<line x1="{xx:.1f}" x2="{xx:.1f}" y1="{yy - 8:.1f}" y2="{below - 10:.1f}" '
                    f'stroke="var(--text1)" stroke-width="1"/>')
            fig.note(xx + 4, below, f"first Fault frame: +{(first_fault - fault_window[0]) * 1000:.0f} ms "
                     "after the last 0x100", cls="t1", size=11)
    x = fig.legend([("box", color(1), "frame received / sent")], LEFT, lanes.y + lanes.h + 70)
    fig.note(x, lanes.y + lanes.h + 74, "0x200 frames coloured by the cruise state they carry:", cls="t2", size=11)
    cruise_state_legend(fig, x + 345, lanes.y + lanes.h + 70, used or [2])
    return fig


# --------------------------------------------------------------- fault timeline ----

def fault_timeline_figure(rows: List[Tuple[str, List[Tuple[float, float]], List[Tuple[float, float]]]],
                          state_segs: List[Tuple[float, float, int]], t_max: float, title: str,
                          subtitle: str = "", resets: Sequence[Tuple[float, str]] = ()) -> Figure:
    """rows: (fault name, cause windows, active intervals). Times on the ECU clock [s]."""
    labels = [name for name, _, _ in rows] + ["Cruise state (0x200)"]
    lanes = Lanes(LEFT + 40, 100, PLOT_W - 40, 30, (0, t_max), labels)
    fig = Figure(W, lanes.y + lanes.h + 110, title, subtitle)
    lanes.frame()
    for tick in time_ticks(t_max):
        xx = lanes.sx(tick)
        fig.raw(f'<line class="grid" x1="{xx:.1f}" x2="{xx:.1f}" y1="{lanes.y:.1f}" y2="{lanes.y + lanes.h:.1f}"/>')
        fig.note(xx, lanes.y + lanes.h + 15, f"{tick:g}", anchor="middle")
    for i, (_, causes, actives) in enumerate(rows):
        for a, b in actives:
            lanes.segment(i, a, b, status("critical"), thickness=0.45, center=0.62)
        for a, b in causes:  # own thin track at the top of the row, never hidden by the fault bar
            lanes.segment(i, a, max(b, a + t_max / 300), ink("text2"), thickness=0.16, center=0.2)
    for start, end, state in state_segs:
        name, fill, dark = CRUISE_STATES.get(state, ("?", ink("neutral"), "ink"))
        lanes.segment(len(rows), start, end, fill, name, dark)
    for when, label in resets:
        xx = lanes.sx(when)
        fig.raw(f'<line x1="{xx:.1f}" x2="{xx:.1f}" y1="{lanes.y - 12:.1f}" y2="{lanes.y + lanes.h:.1f}" '
                f'stroke="var(--text2)" stroke-width="1"/>')
        fig.note(xx + 4, lanes.y - 14, label, cls="t2")
    fig.add(lanes)
    fig.note(lanes.x + lanes.w, lanes.y + lanes.h + 30, "ECU time [s]", anchor="end")
    fig.legend([("box", ink("text2"), "fault cause (injected)"),
                ("box", status("critical"), "fault active (qualified until recovered)")],
               LEFT, lanes.y + lanes.h + 52)
    cruise_state_legend(fig, LEFT, lanes.y + lanes.h + 76, [s for _, _, s in state_segs])
    return fig


# --------------------------------------------------------------------- watchdog ----

def watchdog_figure(window: Tuple[float, float], stall: Tuple[float, float], stale: Optional[Tuple[float, float]],
                    expired: Optional[float], reset: Optional[float], healthy: Optional[float],
                    records: List[Dict[str, float]], title: str, subtitle: str = "") -> Figure:
    """ECU-clock times; records already shifted to the ECU clock."""
    thr = Panel(LEFT, 100, PLOT_W, 120, window, (0, 100))
    rec_thr = [r["ecu_throttle_cmd_pct"] for r in records if window[0] <= r["t"] <= window[1]]
    thr_ticks = nice_ticks(0, max(rec_thr + [5.0]) * 1.2, 4, cover=True)
    thr.ylim = (0, thr_ticks[-1])
    lanes = Lanes(LEFT, thr.y + thr.h + 40, PLOT_W, 28, window,
                  ["Control_10ms", "Output (COM check)", "Watchdog expired", "Reset", "Safe state left",
                   "Cruise state (0x200)"])
    fig = Figure(W, lanes.y + lanes.h + 90, title, subtitle)
    rec = [r for r in records if window[0] <= r["t"] <= window[1]]
    ticks = nice_ticks(window[0], window[1], 10)
    thr.x_grid(ticks)
    thr.y_grid(thr_ticks, "Throttle command on the bus [%]")
    thr.x_axis(ticks, show_labels=False)
    thr.vband(stall[0], stall[1], status("critical"), 0.08)
    thr.line([r["t"] for r in rec], [r["ecu_throttle_cmd_pct"] for r in rec], color(3))
    lanes.frame()
    lanes.segment(0, window[0], stall[0], ink("neutral"), "running", "ink")
    lanes.segment(0, stall[1], window[1], ink("neutral"), "running", "ink")  # drawn first: the stall label goes on top
    lanes.segment(0, stall[0], stall[1], status("critical"), "stalled 400 ms (injected)", outside=True)
    if stale:
        lanes.segment(1, stale[0], stale[1], status("critical"),
                      f"stale after {(stale[0] - stall[0]) * 1000:.0f} ms: safe values sent", outside=True)
    if expired is not None:
        lanes.event(2, expired, status("critical"), f"+{(expired - stall[0]) * 1000:.0f} ms after the stall began")
    if reset is not None:
        lanes.event(3, reset, ink("text2"), "task responds again: simulated ECU reset")
    if healthy is not None:
        lanes.event(4, healthy, status("good"), "after 200 ms healthy probation (driver must re-engage)")
    for start, end, state in state_segments([r["t"] for r in rec], [int(r["cruise_state"]) for r in rec]):
        name, fill, dark = CRUISE_STATES.get(state, ("?", ink("neutral"), "ink"))
        lanes.segment(5, start, end, fill, name, dark)
    fig.add(thr)
    fig.add(lanes)
    for tick in ticks:
        fig.note(lanes.sx(tick), lanes.y + lanes.h + 15, f"{tick:g}", anchor="middle")
    fig.note(LEFT + PLOT_W, lanes.y + lanes.h + 30, "ECU time [s]", anchor="end")
    cruise_state_legend(fig, LEFT, lanes.y + lanes.h + 52,
                        [int(r["cruise_state"]) for r in rec])
    return fig


# --------------------------------------------------------------------- firmware ----

def firmware_figure(samples: List[dict], downloads: List[Tuple[float, float, str]],
                    progress: List[List[Tuple[float, float]]], restarts: List[Tuple[float, int]],
                    t_max: float, title: str, subtitle: str = "") -> Figure:
    """samples: tester polls {t, up, version, state_a, state_b}; downloads: (start, end, label);
    progress: one (t, percent) list per download; restarts: (t, exit code) of ECU process exits."""
    prog = Panel(LEFT, 100, PLOT_W, 90, (0, t_max), (0, 100))
    lanes = Lanes(LEFT, prog.y + prog.h + 40, PLOT_W, 28, (0, t_max),
                  ["ECU process", "Running image", "Slot A", "Slot B"])
    fig = Figure(W, lanes.y + lanes.h + 100, title, subtitle)
    ticks = time_ticks(t_max)
    prog.x_grid(ticks)
    prog.y_grid([0, 50, 100], "Download progress [%]")
    prog.x_axis(ticks, show_labels=False)
    for a, b, label in downloads:
        prog.vband(a, b, color(1), 0.08)
        prog.text(a, 100, label, dx=4, dy=14)
    for points in progress:  # one line per download: never connect two transfers
        prog.line([p[0] for p in points], [p[1] for p in points], color(1))
    lanes.frame()
    versions = sorted({s["version"] for s in samples if s.get("up")})
    version_colors = {v: color(slot) for v, slot in zip(versions, (1, 3, 5, 6))}

    def runs(key: str):
        out, start = [], None
        for i, s in enumerate(samples):
            value = s.get(key) if s.get("up") else None
            if start is None or value != samples[start].get(key) or (samples[start].get("up") != s.get("up")):
                if start is not None:
                    out.append((samples[start]["t"], s["t"], samples[start].get(key) if samples[start].get("up") else None))
                start = i
        if start is not None:
            out.append((samples[start]["t"], samples[-1]["t"], samples[start].get(key) if samples[start].get("up") else None))
        return out

    for a, b, up in runs("up"):
        lanes.segment(0, a, b, ink("neutral") if up else status("critical"), "running" if up else "down", "ink" if up else False)
    for t_exit, _ in restarts:
        lanes.event(0, t_exit, status("critical"))
    for a, b, value in runs("version"):
        if value:
            lanes.segment(1, a, b, version_colors[value], value)
    for row, key in ((2, "state_a"), (3, "state_b")):
        for a, b, value in runs(key):
            if value:
                fill, dark = SLOT_STATES.get(value, (ink("neutral"), "ink"))
                lanes.segment(row, a, b, fill, value, dark)
    fig.add(prog)
    fig.add(lanes)
    for tick in ticks:
        fig.note(lanes.sx(tick), lanes.y + lanes.h + 15, f"{tick:g}", anchor="middle")
    fig.note(LEFT + PLOT_W, lanes.y + lanes.h + 30, "tester time [s]", anchor="end")
    fig.legend([("box", fill, name) for name, (fill, _) in SLOT_STATES.items()
                if any(s.get("state_a") == name or s.get("state_b") == name for s in samples)],
               LEFT, lanes.y + lanes.h + 52, gap=12)
    x = fig.legend([("box", c, f"image {v}") for v, c in version_colors.items()], LEFT, lanes.y + lanes.h + 74, gap=12)
    if restarts:
        codes = sorted({code for _, code in restarts})
        fig.legend([("diamond", status("critical"), f"ECU exit (code {', '.join(map(str, codes))}), restarted"),
                    ("box", status("critical"), "ECU down")], x + 6, lanes.y + lanes.h + 74, gap=12)
    return fig


# ------------------------------------------------------------------- scheduler ----

def scheduler_figure(runs: Dict[str, dict], title: str, subtitle: str = "") -> Figure:
    """runs: {condition label: timing JSON}. One row per task and condition: start latency
    p50 / p99 / max on a log axis, with the task's deadline."""
    conditions = list(runs)
    tasks = [t["name"] for t in runs[conditions[0]]["tasks"]]
    row_h = 26
    rows = [(task, cond) for task in tasks for cond in conditions]
    panel = Panel(LEFT + 60, 100, PLOT_W - 60, row_h * len(rows), (10.0, 200000.0), (0, len(rows)), xlog=True)
    fig = Figure(W, panel.y + panel.h + 100, title, subtitle)
    ticks = [10, 100, 1000, 10000, 100000]
    panel.x_grid(ticks)
    panel.x_axis(ticks, "start latency (log scale)",
                 fmt=lambda v: {10: "10 µs", 100: "100 µs", 1000: "1 ms", 10000: "10 ms", 100000: "100 ms"}[int(v)])
    for i, (task, cond) in enumerate(rows):
        data = next(t for t in runs[cond]["tasks"] if t["name"] == task)
        yy = len(rows) - i - 0.5
        slot = 1 + conditions.index(cond)
        p50, p99, mx = data["start_latency_p50_us"], data["start_latency_p99_us"], data["start_latency"]["max_us"]
        panel.parts.append(f'<line x1="{panel.sx(p50):.1f}" x2="{panel.sx(mx):.1f}" y1="{panel.sy(yy):.1f}" '
                           f'y2="{panel.sy(yy):.1f}" stroke="{color(slot)}" stroke-width="2"/>')
        panel.dot(p50, yy, color(slot))
        panel.dot(p99, yy, color(slot), hollow=True)
        xx = panel.sx(mx)
        panel.parts.append(f'<path d="M{xx:.1f},{panel.sy(yy) - 6:.1f} L{xx + 6:.1f},{panel.sy(yy):.1f} '
                           f'L{xx:.1f},{panel.sy(yy) + 6:.1f} L{xx - 6:.1f},{panel.sy(yy):.1f} Z" fill="{color(slot)}"/>')
        misses = data["deadline_misses"]
        miss_text = f", {misses} deadline miss{'es' if misses != 1 else ''}" if misses else ""
        panel.text(mx, yy, (f"max {mx / 1000:.1f} ms" if mx >= 1000 else f"max {mx:.0f} µs") + miss_text,
                   cls="t1" if misses else "t2", dx=10, dy=4)
        deadline = data["deadline_us"] or data["period_us"]
        dx = panel.sx(deadline)
        panel.parts.append(f'<line x1="{dx:.1f}" x2="{dx:.1f}" y1="{panel.sy(yy) - 9:.1f}" y2="{panel.sy(yy) + 9:.1f}" '
                           f'stroke="{status("critical")}" stroke-width="2"/>')
        fig.note(panel.x - 8, panel.sy(yy) + 4, f"{task} · {cond}", cls="t2", anchor="end")
        if i % len(conditions) == 0 and i:
            yb = panel.sy(len(rows) - i)
            fig.raw(f'<line class="grid" x1="{LEFT - 120:.1f}" x2="{panel.x + panel.w:.1f}" y1="{yb:.1f}" y2="{yb:.1f}"/>')
    fig.add(panel)
    y = panel.y + panel.h + 52
    x = fig.legend([("box", color(1 + i), c) for i, c in enumerate(conditions)], LEFT, y)
    fig.legend([("dot", ink("text2"), "median"), ("hollow", ink("text2"), "p99"), ("diamond", ink("text2"), "max"),
                ("box", status("critical"), "deadline")], x + 10, y, gap=14)
    return fig


# ------------------------------------------------------------------ CAN latency ----

def latency_cdf_figure(series: Dict[str, List[float]], title: str, subtitle: str = "") -> Figure:
    """series: label -> latencies [µs]. Empirical CDFs; x clipped at the overall p99.9."""
    every = [v for values in series.values() for v in values]
    x_hi = nice_ticks(0, percentile(every, 99.9) or 1, 6)[-1]
    panel = Panel(LEFT - 70, 110, PLOT_W + 70, 260, (0, x_hi), (0, 100))
    fig = Figure(W, panel.y + panel.h + 80, title, subtitle)
    ticks = nice_ticks(0, x_hi, 8)
    panel.x_grid(ticks)
    panel.y_grid([0, 25, 50, 75, 90, 99], "Frames delivered within the latency [%]", fmt=lambda v: f"{v:g} %")
    panel.x_axis(ticks, "one-way latency [µs]")
    entries = []
    for slot, (label, values) in enumerate(series.items(), start=1):
        xs, ys = cdf(values)
        keep = [(x, y) for x, y in zip(xs, ys) if x <= x_hi]
        panel.line([p[0] for p in keep], [p[1] for p in keep], color(slot), step=True)
        p50, p99 = percentile(values, 50), percentile(values, 99)
        panel.dot(p50, 50, color(slot))
        panel.text(p50, 50, f"median {p50:.0f} µs", dx=8, dy=14 + 14 * (slot - 1))
        if p99 <= x_hi:
            panel.dot(p99, 99, color(slot))
            panel.text(p99, 99, f"p99 {p99:.0f} µs", dx=8, dy=16 + 14 * (slot - 1))
        entries.append(("line", color(slot), f"{label} (n = {len(values)}, max {max(values):.0f} µs)"))
    fig.add(panel)
    fig.legend(entries, LEFT - 70, 72)
    return fig


# ------------------------------------------------------------------ UDS response ----

def uds_histogram_figure(series: Dict[str, List[float]], title: str, subtitle: str = "",
                         bin_ms: float = 0.5, x_max: float = 25.0) -> Figure:
    bins = int(x_max / bin_ms)
    counts = {label: [0] * bins for label in series}
    for label, values in series.items():
        for v in values:
            counts[label][min(bins - 1, int(v / bin_ms))] += 1
    y_ticks = nice_ticks(0, max(max(c) for c in counts.values()), 4, cover=True)
    y_hi = y_ticks[-1]
    panel = Panel(LEFT - 70, 112, PLOT_W + 70, 220, (0, x_max), (0, y_hi))
    fig = Figure(W, panel.y + panel.h + 80, title, subtitle)
    ticks = nice_ticks(0, x_max, 5)
    panel.y_grid(y_ticks, "Requests")
    panel.x_axis(ticks, "response time [ms] (request sent → complete response received)")
    for tick in (10, 20):
        panel.parts.append(f'<line x1="{panel.sx(tick):.1f}" x2="{panel.sx(tick):.1f}" y1="{panel.y:.1f}" '
                           f'y2="{panel.y + panel.h:.1f}" stroke="var(--muted)" stroke-width="1"/>')
        panel.text(tick, y_hi, f"{tick} ms Dcm cycle", cls="tm", dx=4, dy=12)
    width = panel.w / bins - 2
    entries = []
    for slot, (label, c) in enumerate(counts.items(), start=1):
        for i, count in enumerate(c):
            if count:
                panel.column((i + 0.5) * bin_ms, min(width, 24), count, color(slot))
        med = percentile(series[label], 50)
        entries.append(("box", color(slot), f"{label}: median {med:.1f} ms, n = {len(series[label])}"))
    fig.add(panel)
    fig.legend(entries, LEFT - 70, 72)
    return fig


# ------------------------------------------------------------------ log parsing ----

LOG_LINE = re.compile(r"^\[\s*([\d.]+)\] \[\w+\s*\] \[[\w ]+\] (.*)$")


def parse_ecu_log(text: str) -> List[Tuple[float, str]]:
    """(process time [s], message) for every log line."""
    out = []
    for line in text.splitlines():
        match = LOG_LINE.match(line)
        if match:
            out.append((float(match.group(1)), match.group(2)))
    return out


def epoch_offset(lines: List[Tuple[float, str]]) -> float:
    """Offset that converts log-prefix time into the ECU's epoch time (from lines with t=...s)."""
    diffs = sorted(float(m.group(1)) - stamp for stamp, msg in lines if (m := re.search(r" t=([\d.]+)s", msg)))
    return diffs[len(diffs) // 2] if diffs else 0.0


def epoch_wall(lines: List[Tuple[float, str]]) -> Optional[float]:
    for _, msg in lines:
        if msg.startswith("SCHED_EPOCH wall_us="):
            return int(msg.split("=", 1)[1]) / 1e6
    return None


def parse_candump(text: str) -> List[Tuple[float, int, str]]:
    frames = []
    for line in text.splitlines():
        match = re.match(r"\((\d+\.\d+)\) \S+ ([0-9A-Fa-f]+)#([0-9A-Fa-f]*)", line)
        if match:
            frames.append((float(match.group(1)), int(match.group(2), 16), match.group(3)))
    return frames
