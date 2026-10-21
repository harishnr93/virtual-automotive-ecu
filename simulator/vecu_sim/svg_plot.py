"""Dependency-free SVG plot of a closed-loop run (speed/target, throttle, cruise state)."""

from __future__ import annotations

from html import escape
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

STATE_COLORS = {0: "#d9d9d9", 1: "#9ecae1", 2: "#74c476", 3: "#fdae6b", 4: "#e6550d"}
STATE_NAMES = {0: "Off", 1: "Standby", 2: "Active", 3: "Override", 4: "Fault"}

WIDTH, LEFT, RIGHT = 960, 70, 20
PANEL_H, GAP, TOP = 220, 50, 40


def _scale(value: float, lo: float, hi: float, out_lo: float, out_hi: float) -> float:
    return out_lo + (value - lo) / (hi - lo) * (out_hi - out_lo) if hi > lo else out_lo


def _polyline(points: Sequence[Tuple[float, float]], color: str, dash: str = "") -> str:
    coords = " ".join(f"{x:.1f},{y:.1f}" for x, y in points)
    extra = f' stroke-dasharray="{dash}"' if dash else ""
    return f'<polyline fill="none" stroke="{color}" stroke-width="1.6"{extra} points="{coords}"/>'


def _axis(y0: float, lo: float, hi: float, t_max: float, label: str, ticks: int = 5) -> List[str]:
    out = [f'<rect x="{LEFT}" y="{y0}" width="{WIDTH - LEFT - RIGHT}" height="{PANEL_H}" fill="none" stroke="#888"/>',
           f'<text x="15" y="{y0 + PANEL_H / 2}" transform="rotate(-90 15 {y0 + PANEL_H / 2})" '
           f'font-size="12" text-anchor="middle">{escape(label)}</text>']
    for i in range(ticks + 1):
        value = lo + (hi - lo) * i / ticks
        y = _scale(value, lo, hi, y0 + PANEL_H, y0)
        out.append(f'<line x1="{LEFT}" x2="{WIDTH - RIGHT}" y1="{y:.1f}" y2="{y:.1f}" stroke="#eee"/>')
        out.append(f'<text x="{LEFT - 6}" y="{y + 4:.1f}" font-size="11" text-anchor="end">{value:.0f}</text>')
    step = 5 if t_max <= 60 else 10
    t = 0.0
    while t <= t_max + 1e-9:
        x = _scale(t, 0, t_max, LEFT, WIDTH - RIGHT)
        out.append(f'<text x="{x:.1f}" y="{y0 + PANEL_H + 14}" font-size="11" text-anchor="middle">{t:.0f}</text>')
        t += step
    return out


def write_run_plot(records: List[Dict[str, float]], path: Path, title: str = "") -> None:
    t_max = records[-1]["t"] or 1.0
    x = lambda t: _scale(t, 0, t_max, LEFT, WIDTH - RIGHT)  # noqa: E731
    speeds = [r["speed_kph"] for r in records] + [r["target_kph"] for r in records if r["target_kph"] > 0]
    s_lo = max(0.0, (min(speeds) // 10) * 10 - 10)
    s_hi = (max(speeds) // 10) * 10 + 10
    y1 = TOP
    y2 = TOP + PANEL_H + GAP
    height = y2 + PANEL_H + 70

    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{WIDTH}" height="{height}" '
             f'font-family="sans-serif"><rect width="100%" height="100%" fill="white"/>',
             f'<text x="{LEFT}" y="22" font-size="14" font-weight="bold">{escape(title)}</text>']

    # Cruise state as a coloured band behind the speed panel.
    start = 0
    for i in range(1, len(records) + 1):
        if i == len(records) or records[i]["cruise_state"] != records[start]["cruise_state"]:
            state = int(records[start]["cruise_state"])
            x0, x1 = x(records[start]["t"]), x(records[i - 1]["t"] + 0.01)
            parts.append(f'<rect x="{x0:.1f}" y="{y1}" width="{max(x1 - x0, 0.5):.1f}" height="{PANEL_H}" '
                         f'fill="{STATE_COLORS.get(state, "#fff")}" opacity="0.25"/>')
            start = i

    parts += _axis(y1, s_lo, s_hi, t_max, "speed [km/h]")
    parts.append(_polyline([(x(r["t"]), _scale(r["target_kph"], s_lo, s_hi, y1 + PANEL_H, y1))
                            for r in records if r["target_kph"] > 0], "#d62728", "6,4"))
    parts.append(_polyline([(x(r["t"]), _scale(r["speed_kph"], s_lo, s_hi, y1 + PANEL_H, y1)) for r in records],
                           "#1f77b4"))

    parts += _axis(y2, 0, 100, t_max, "throttle [%]")
    parts.append(_polyline([(x(r["t"]), _scale(r["applied_throttle_pct"], 0, 100, y2 + PANEL_H, y2))
                            for r in records], "#2ca02c"))
    parts.append(_polyline([(x(r["t"]), _scale(r["accelerator_pct"], 0, 100, y2 + PANEL_H, y2))
                            for r in records], "#9467bd", "3,3"))
    parts.append(_polyline([(x(r["t"]), _scale(r["grade_pct"] * 10, 0, 100, y2 + PANEL_H, y2))
                            for r in records], "#8c564b", "1,3"))

    legend = [("#1f77b4", "vehicle speed"), ("#d62728", "cruise target"), ("#2ca02c", "applied throttle"),
              ("#9467bd", "driver pedal"), ("#8c564b", "grade x10 [%]")]
    lx = LEFT
    ly = height - 30
    for color, name in legend:
        parts.append(f'<line x1="{lx}" x2="{lx + 22}" y1="{ly}" y2="{ly}" stroke="{color}" stroke-width="2"/>')
        parts.append(f'<text x="{lx + 26}" y="{ly + 4}" font-size="11">{escape(name)}</text>')
        lx += 135
    lx = LEFT
    for state, name in STATE_NAMES.items():
        parts.append(f'<rect x="{lx}" y="{ly + 10}" width="14" height="10" fill="{STATE_COLORS[state]}" opacity="0.5"/>')
        parts.append(f'<text x="{lx + 18}" y="{ly + 19}" font-size="11">{name}</text>')
        lx += 90
    parts.append(f'<text x="{WIDTH - RIGHT}" y="{height - 4}" font-size="11" text-anchor="end">time [s]</text>')
    parts.append("</svg>")
    path.write_text("\n".join(parts), encoding="utf-8")
