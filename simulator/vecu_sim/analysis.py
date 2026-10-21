"""KPIs of a closed-loop run, computed from the simulator's time series.

All values are derived from recorded data; nothing is estimated or assumed.
"""

from __future__ import annotations

from typing import Callable, Dict, List, Optional

from .scenario import Scenario

STATE_ACTIVE = 2
STATE_FAULT = 4
SETTLING_BAND_KPH = 0.5

Record = Dict[str, float]


def first_time(records: List[Record], predicate: Callable[[Record], bool], after: float = 0.0) -> Optional[float]:
    for r in records:
        if r["t"] >= after and predicate(r):
            return r["t"]
    return None


def step_metrics(records: List[Record], t_start: float, t_end: float, start_kph: float,
                 target_kph: float, band: float = SETTLING_BAND_KPH) -> Dict[str, Optional[float]]:
    """Response to a speed-target change between t_start and t_end."""
    window = [r for r in records if t_start <= r["t"] < t_end]
    if not window:
        return {}
    direction = 1.0 if target_kph >= start_kph else -1.0
    overshoot = max(0.0, max(direction * (r["speed_kph"] - target_kph) for r in window))
    rise = None
    if abs(target_kph - start_kph) > 1e-6:
        low = start_kph + 0.1 * (target_kph - start_kph)
        high = start_kph + 0.9 * (target_kph - start_kph)
        t10 = first_time(window, lambda r: direction * (r["speed_kph"] - low) >= 0)
        t90 = first_time(window, lambda r: direction * (r["speed_kph"] - high) >= 0)
        rise = (t90 - t10) if (t10 is not None and t90 is not None) else None
    settling = None
    for i in range(len(window) - 1, -1, -1):
        if abs(window[i]["speed_kph"] - target_kph) > band:
            settling = window[i + 1]["t"] - t_start if i + 1 < len(window) else None
            break
    else:
        settling = 0.0
    tail = [r for r in window if r["t"] >= t_end - 2.0]
    steady_error = sum(abs(r["speed_kph"] - target_kph) for r in tail) / len(tail) if tail else None
    return {"start_kph": start_kph, "target_kph": target_kph, "rise_time_s": rise,
            "overshoot_kph": overshoot, "settling_time_s": settling, "steady_state_error_kph": steady_error,
            "max_throttle_pct": max(r["cruise_throttle_pct"] for r in window)}


def evaluate(records: List[Record], scenario: Scenario) -> dict:
    report: dict = {"scenario": scenario.name, "samples": len(records)}
    if not records:
        return report
    end = records[-1]["t"]
    report["max_speed_kph"] = max(r["speed_kph"] for r in records)
    report["final_speed_kph"] = records[-1]["speed_kph"]

    engaged = first_time(records, lambda r: r["cruise_state"] == STATE_ACTIVE)
    report["engaged_at_s"] = engaged

    # Responses to each target the ECU reported while Active.
    responses = []
    if engaged is not None:
        segments = []
        current = None
        for r in records:
            if r["cruise_state"] == STATE_ACTIVE and r["target_kph"] > 0:
                if current is None or r["target_kph"] != current["target"]:
                    current = {"target": r["target_kph"], "start": r["t"], "start_speed": r["speed_kph"]}
                    segments.append(current)
                current["end"] = r["t"] + 0.01
            else:
                current = None
        event_times = sorted(float(e["t"]) for e in scenario.events)
        for seg in segments:
            # A response window ends at the next scenario event (e.g. a grade change), so that
            # disturbances are not attributed to the step response.
            next_event = next((t for t in event_times if t > seg["start"] + 0.5), None)
            if next_event is not None:
                seg["end"] = min(seg["end"], next_event)
            # Only real steps (>= 1 km/h) observed for at least 2 s are step responses.
            if seg["end"] - seg["start"] >= 2.0 and abs(seg["target"] - seg["start_speed"]) >= 1.0:
                responses.append(step_metrics(records, seg["start"], seg["end"], seg["start_speed"], seg["target"]))
    report["speed_responses"] = responses

    # Grade disturbances while Active.
    disturbances = []
    all_event_times = sorted(float(e["t"]) for e in scenario.events)
    for event in scenario.events:
        if "grade_pct" in event:
            t0 = float(event["t"])
            t1 = min([t0 + 20.0, end] + [t for t in all_event_times if t > t0])
            window = [r for r in records if t0 <= r["t"] < t1 and r["cruise_state"] == STATE_ACTIVE]
            if window:
                target = window[0]["target_kph"]
                worst = max(window, key=lambda r: abs(r["speed_kph"] - target))
                back = None
                worst_index = window.index(worst)
                for i in range(worst_index, len(window)):
                    if all(abs(x["speed_kph"] - target) <= SETTLING_BAND_KPH for x in window[i:]):
                        back = window[i]["t"] - t0
                        break
                disturbances.append({"t_s": t0, "grade_pct": event["grade_pct"],
                                     "max_deviation_kph": abs(worst["speed_kph"] - target),
                                     "recovery_time_s": back})
    report["disturbances"] = disturbances

    # Fault reactions as observed on the bus (simulator side).
    faults = []
    for window in scenario.faults():
        t_fault = first_time(records, lambda r: r["cruise_state"] == STATE_FAULT, window.start_s)
        t_cut = first_time(records, lambda r: r["ecu_throttle_cmd_pct"] == 0.0 and r["cruise_state"] != STATE_ACTIVE,
                           window.start_s)
        t_recover = first_time(records, lambda r: r["cruise_state"] not in (STATE_FAULT,), t_fault) if t_fault else None
        faults.append({"kind": window.kind, "start_s": window.start_s, "duration_s": window.end_s - window.start_s,
                       "fault_detected_after_ms": (t_fault - window.start_s) * 1000.0 if t_fault else None,
                       "throttle_cut_after_ms": (t_cut - window.start_s) * 1000.0 if t_cut else None,
                       "fault_reason": next((r["fault_reason"] for r in records
                                             if t_fault and r["t"] >= t_fault and r["fault_reason"]), None),
                       "left_fault_at_s": t_recover,
                       "state_after_fault": next((r["cruise_state"] for r in records
                                                  if t_recover and r["t"] >= t_recover), None)})
    report["faults"] = faults

    # Driver overrides.
    overrides = []
    for event in scenario.events:
        t0 = float(event["t"])
        if event.get("brake_pct", 0) > 1.0:
            t_off = first_time(records, lambda r: r["cruise_state"] != STATE_ACTIVE, t0)
            overrides.append({"kind": "brake", "t_s": t0,
                              "disengaged_after_ms": (t_off - t0) * 1000.0 if t_off is not None else None})
        if event.get("accelerator_pct", 0) > 5.0 and engaged is not None and t0 > engaged:
            t_ovr = first_time(records, lambda r: r["cruise_state"] == 3, t0)
            overrides.append({"kind": "accelerator", "t_s": t0,
                              "override_after_ms": (t_ovr - t0) * 1000.0 if t_ovr is not None else None})
    report["overrides"] = overrides
    report["max_status_age_ms"] = max(r["status_age_ms"] for r in records if r["t"] > 1.0) if end > 1.0 else None
    return report


def _fmt(value, unit="", digits=2):
    return "n/a" if value is None else f"{value:.{digits}f}{unit}"


def format_report(report: dict) -> str:
    lines = [f"[kpi] scenario={report.get('scenario')} engaged_at={_fmt(report.get('engaged_at_s'), 's')} "
             f"final_speed={_fmt(report.get('final_speed_kph'), 'km/h')}"]
    for r in report.get("speed_responses", []):
        lines.append(f"[kpi] response {r['start_kph']:.1f}->{r['target_kph']:.0f} km/h: "
                     f"rise={_fmt(r['rise_time_s'], 's')} overshoot={_fmt(r['overshoot_kph'], 'km/h')} "
                     f"settling={_fmt(r['settling_time_s'], 's')} "
                     f"steady_error={_fmt(r['steady_state_error_kph'], 'km/h', 3)} "
                     f"max_throttle={_fmt(r['max_throttle_pct'], '%', 1)}")
    for d in report.get("disturbances", []):
        lines.append(f"[kpi] grade {d['grade_pct']}% at {d['t_s']}s: max_deviation={_fmt(d['max_deviation_kph'], 'km/h')} "
                     f"recovery={_fmt(d['recovery_time_s'], 's')}")
    for f in report.get("faults", []):
        lines.append(f"[kpi] fault {f['kind']} at {f['start_s']}s: detected_after={_fmt(f['fault_detected_after_ms'], 'ms', 1)} "
                     f"throttle_cut_after={_fmt(f['throttle_cut_after_ms'], 'ms', 1)} reason={f['fault_reason']} "
                     f"state_after={f['state_after_fault']}")
    for o in report.get("overrides", []):
        value = o.get("disengaged_after_ms", o.get("override_after_ms"))
        lines.append(f"[kpi] {o['kind']} override at {o['t_s']}s: reaction={_fmt(value, 'ms', 1)}")
    sim = report.get("simulator", {})
    if sim:
        lines.append(f"[kpi] simulator: status_frames={sim.get('status_frames_received')} "
                     f"max_step_lateness={_fmt(sim.get('max_step_lateness_ms'), 'ms')} late_steps={sim.get('late_steps')}")
    return "\n".join(lines)
