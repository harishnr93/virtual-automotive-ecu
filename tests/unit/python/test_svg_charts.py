"""SVG chart toolkit and result figures (synthetic data; scripts/make_figures.py uses real runs)."""

import xml.dom.minidom

import pytest

from vecu_sim import figures as F
from vecu_sim import svg_charts as C


def parse(svg: str):
    return xml.dom.minidom.parseString(svg.encode())


def records(seconds=5.0, brake=False):
    out = []
    for k in range(int(seconds / 0.01)):
        t = k * 0.01
        out.append({"t": t, "speed_kph": 60 + t, "target_kph": 80.0 if t > 1 else 0.0,
                    "applied_throttle_pct": 20.0, "accelerator_pct": 0.0, "brake_pct": 30.0 if brake and t > 4 else 0.0,
                    "grade_pct": 0.0, "cruise_state": 2 if t > 1 else 1, "ecu_throttle_cmd_pct": 20.0})
    return out


def test_nice_ticks_inside_and_covering():
    assert C.nice_ticks(0, 100, 5) == [0, 20, 40, 60, 80, 100]
    assert C.nice_ticks(0, 178, 4)[-1] <= 178
    covering = C.nice_ticks(0, 178, 4, cover=True)
    assert covering[0] == 0 and covering[-1] >= 178  # an axis built from these encloses every mark
    assert C.nice_ticks(3.7, 3.7) == [3.7]


def test_simplify_keeps_shape_and_ends():
    flat = [(x * 0.1, 50.0) for x in range(1000)]  # 1000 points over 100 px, constant
    kept = C.simplify(flat)
    assert kept[0] == flat[0] and kept[-1] == flat[-1] and len(kept) <= 102
    step = [(0.0, 10.0), (0.2, 10.0), (0.3, 90.0), (0.4, 90.0)]  # a vertical jump must survive
    assert (0.3, 90.0) in C.simplify(step)


def test_only_eight_categorical_slots():
    assert C.color(8) == "var(--s8)"
    with pytest.raises(ValueError):
        C.color(9)


def test_figure_is_valid_svg_with_resolved_colours_and_dark_mode():
    fig = C.Figure(400, 200, "Title & <check>", "subtitle")
    panel = C.Panel(50, 60, 300, 100, (0, 10), (0, 1))
    panel.y_grid([0, 1], "label")
    panel.line([0, 5, 10], [0, 1, 0.5], C.color(1))
    panel.dot(5, 1, C.color(2))
    fig.add(panel)
    svg = fig.svg()
    doc = parse(svg)  # well-formed, title escaped
    assert doc.getElementsByTagName("title")[0].firstChild.data == "Title & <check>"
    assert "var(--" not in svg.split("</style>")[1]  # every token resolved to a plain colour
    assert 'stroke="#2a78d6"' in svg and "s-s1" in svg  # light value + dark-override class
    assert "@media (prefers-color-scheme: dark)" in svg and ".s-s1{stroke:#3987e5}" in svg


def test_lane_label_inside_only_when_it_fits():
    lanes = C.Lanes(100, 50, 400, 24, (0, 10), ["row"])
    lanes.segment(0, 0, 9, C.color(1), "fits easily")
    lanes.segment(0, 9, 9.2, C.color(2), "much too long to fit")
    lanes.segment(0, 9.5, 9.6, C.color(3), "outside", outside=True)
    text = "".join(lanes.parts)
    assert "fits easily" in text and "much too long to fit" not in text and ">outside<" in text


def test_state_segments():
    assert F.state_segments([0, 1, 2, 3], [1, 1, 2, 2]) == [(0, 2, 1), (2, 3, 2)]
    assert F.state_segments([], []) == []


def test_closed_loop_figure_omits_unused_pedals():
    scenario = {"events": [{"t": 1.0, "set": True}, {"t": 2.0, "fault": "invalid_speed", "duration_s": 0.3}]}
    svg = F.closed_loop_figure(records(), scenario, "Run").svg()
    parse(svg)
    assert "brake pedal" not in svg and "accelerator pedal" not in svg  # never pressed: not drawn
    assert "speed invalid" in svg and ">set<" in svg and ">Active<" in svg
    svg = F.closed_loop_figure(records(brake=True), scenario, "Run").svg()
    assert "brake pedal" in svg


def test_close_events_share_one_label():
    scenario = {"events": [{"t": 1.0, "main_switch": True}, {"t": 1.05, "set": True}]}
    svg = F.closed_loop_figure(records(), scenario, "Run").svg()
    assert "main switch, set" in svg


def test_log_and_trace_parsing():
    log = ("[   0.010000] [INFO ] [cruise_ecu] SCHED_EPOCH wall_us=1000000000\n"
           "[   1.000000] [WARN ] [cruise_ecu] FAULT SpeedSignalInvalid PreFailed -> Failed t=0.995s\n"
           "[   2.000000] [ERROR] [cruise_ecu] WATCHDOG_EXPIRED entity=Control_10ms t=1.995s\n"
           "not a log line\n")
    lines = F.parse_ecu_log(log)
    assert len(lines) == 3 and lines[1][1].startswith("FAULT")
    assert F.epoch_offset(lines) == pytest.approx(-0.005)
    assert F.epoch_wall(lines) == pytest.approx(1000.0)
    frames = F.parse_candump("(1793182378.347213) vbus0 200#0000020000000000\n(1.5) vbus0 100#\n")
    assert frames[0] == (1793182378.347213, 0x200, "0000020000000000") and frames[1][1] == 0x100
    assert F.decode_cruise_state("0000020000000000") == 2 and F.decode_cruise_state("") == 0


def test_measurement_figures_render():
    timing = {"tasks": [{"name": "Task_10ms", "deadline_us": 10000, "period_us": 10000, "deadline_misses": 1,
                         "start_latency_p50_us": 100.0, "start_latency_p99_us": 900.0,
                         "start_latency": {"max_us": 12000.0}}]}
    svg = F.scheduler_figure({"light": timing, "stress": timing}, "Scheduler").svg()
    parse(svg)
    assert "1 deadline miss" in svg
    parse(F.latency_cdf_figure({"a": [10.0, 20.0, 30.0], "b": [5.0, 6.0]}, "Latency").svg())
    svg = F.uds_histogram_figure({"single": [9.9, 10.1, 10.2]}, "UDS").svg()
    parse(svg)
    assert "median 10.1 ms" in svg
