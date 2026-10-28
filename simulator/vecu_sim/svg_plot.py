"""SVG plot of a closed-loop run, written by `vecu_sim.simulator --svg`.

Uses the same figure as scripts/make_figures.py (vecu_sim.figures.closed_loop_figure): speed and
target, throttle and pedals, road grade, cruise state and scenario events on one time axis.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Optional

from .figures import closed_loop_figure


def write_run_plot(records: List[Dict[str, float]], path: Path, title: str = "",
                   events: Optional[List[dict]] = None, subtitle: str = "") -> None:
    closed_loop_figure(records, {"events": events or []}, title, subtitle).save(path)
