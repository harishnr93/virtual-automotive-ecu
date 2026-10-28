"""Small dependency-free SVG chart toolkit for the result figures.

Design rules (applied everywhere, see docs/results_gallery.md):
  - colours come from one reference palette, defined once as CSS variables, with a dark-mode
    variant selected by `prefers-color-scheme` (GitHub shows SVGs as images: no scripts, but
    the media query works)
  - categorical colours follow the entity, in a fixed slot order; status colours (critical,
    warning, good) only where the colour *means* a state, always next to a text label
  - one y-axis per panel; different units go into separate panels sharing the x-axis
  - thin marks: 2 px lines, solid hairline grid and axes, >= 8 px markers
  - text uses ink tokens, never the series colour; >= 2 series always get a legend
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from html import escape
from pathlib import Path
from typing import Callable, List, Optional, Sequence, Tuple

# Reference palette (light / dark). Categorical slots in their validated order.
SERIES = [("#2a78d6", "#3987e5"), ("#eb6834", "#d95926"), ("#1baf7a", "#199e70"), ("#eda100", "#c98500"),
          ("#e87ba4", "#d55181"), ("#008300", "#008300"), ("#6250d6", "#9085e9"), ("#e34948", "#e66767")]
STATUS = {"good": "#0ca30c", "warning": "#fab219", "serious": "#ec835a", "critical": "#d03b3b"}
INK = {"surface": ("#fcfcfb", "#1a1a19"), "text1": ("#0b0b0b", "#f0efec"), "text2": ("#52514e", "#c3c2b7"),
       "muted": ("#898781", "#898781"), "grid": ("#e1e0d9", "#2c2c2a"), "axis": ("#c3c2b7", "#383835"),
       "neutral": ("#d6d5cf", "#4a4a46")}
FONT = 'system-ui, -apple-system, "Segoe UI", sans-serif'


# Colour tokens: name -> (light, dark). Drawing code refers to them as "var(--name)"; Figure.svg()
# resolves every token into a plain presentation attribute (the light value) plus a class that a
# prefers-color-scheme block overrides. Renderers without CSS support still show the light figure.
TOKENS = {f"s{i + 1}": pair for i, pair in enumerate(SERIES)}
TOKENS.update(INK)
TOKENS.update({name: (hexv, hexv) for name, hexv in STATUS.items()})
# Ink classes used directly in the drawing code: class -> (attribute, token)
INK_CLASSES = {"bg": ("fill", "surface"), "t1": ("fill", "text1"), "t2": ("fill", "text2"), "tm": ("fill", "muted"),
               "grid": ("stroke", "grid"), "axis": ("stroke", "axis"), "ring": ("stroke", "surface")}


def _style() -> str:
    dark = [f".{cls}{{{attr}:{TOKENS[token][1]}}}" for cls, (attr, token) in INK_CLASSES.items()]
    dark += [f".f-{name}{{fill:{pair[1]}}}.s-{name}{{stroke:{pair[1]}}}" for name, pair in TOKENS.items()
             if pair[0] != pair[1]]
    return ("<style>.grid,.axis{stroke-width:1}.ring{stroke-width:2}text{font-family:" + FONT.replace('"', "'") +
            "}@media (prefers-color-scheme: dark){" + "".join(dark) + "}</style>")


def _resolve_tokens(svg: str) -> str:
    """var(--token) in fill/stroke -> light hex + dark-override class; ink classes -> attributes."""
    def fix(tag_match: "re.Match[str]") -> str:
        tag = tag_match.group(0)
        classes: List[str] = []

        def token(attr_match: "re.Match[str]") -> str:
            attr, name = attr_match.group(1), attr_match.group(2)
            classes.append(("f-" if attr == "fill" else "s-") + name)
            return f'{attr}="{TOKENS[name][0]}"'

        tag = re.sub(r'(fill|stroke)="var\(--([\w-]+)\)"', token, tag)
        existing = re.search(r' class="([^"]*)"', tag)
        for cls in (existing.group(1).split() if existing else []):
            attr, name = INK_CLASSES.get(cls, (None, None))
            if attr and f' {attr}="' not in tag:
                tag = tag.replace(f' class="{existing.group(1)}"', f' class="{existing.group(1)}" {attr}="{TOKENS[name][0]}"', 1)
        if classes:
            if existing:
                tag = tag.replace(f' class="{existing.group(1)}"', f' class="{existing.group(1)} {" ".join(classes)}"', 1)
            else:
                tag = re.sub(r"^<(\w+)", lambda m: f'<{m.group(1)} class="{" ".join(classes)}"', tag)
        return tag

    return re.sub(r"<[a-zA-Z][^<>]*>", fix, svg)


def color(slot: int) -> str:
    """CSS colour of categorical slot 1..8 (fixed order, never cycled)."""
    if not 1 <= slot <= len(SERIES):
        raise ValueError("only 8 categorical slots: fold the rest into 'Other' or facet")
    return f"var(--s{slot})"


def status(name: str) -> str:
    return f"var(--{name})"


def ink(name: str) -> str:
    return f"var(--{name})"


def fmt_number(value: float) -> str:
    if value == 0:
        return "0"
    if abs(value) >= 1000:
        return f"{value / 1000:g}K" if abs(value) < 1e6 else f"{value / 1e6:g}M"
    return f"{value:g}"


def nice_ticks(lo: float, hi: float, count: int = 5, cover: bool = False) -> List[float]:
    """Round tick values inside [lo, hi]; with cover=True the ticks extend to enclose [lo, hi]
    (use that for axis limits, so no mark can exceed the axis)."""
    if hi <= lo:
        return [lo]
    raw = (hi - lo) / count
    magnitude = 10 ** math.floor(math.log10(raw))
    step = next(m * magnitude for m in (1, 2, 2.5, 5, 10) if m * magnitude >= raw)
    first = (math.floor(lo / step + 1e-9) if cover else math.ceil(lo / step - 1e-9)) * step
    ticks = []
    value = first
    while value <= hi + step * 1e-6:
        ticks.append(round(value, 10))
        value += step
    if cover and ticks[-1] < hi - step * 1e-6:
        ticks.append(round(ticks[-1] + step, 10))
    return ticks


@dataclass
class Panel:
    """One plotting area with its own y-scale; panels of a figure share the x-scale."""
    x: float
    y: float
    w: float
    h: float
    xlim: Tuple[float, float]
    ylim: Tuple[float, float]
    xlog: bool = False
    ylog: bool = False
    parts: List[str] = field(default_factory=list)

    def sx(self, value: float) -> float:
        lo, hi = self.xlim
        if self.xlog:
            value, lo, hi = math.log10(max(value, 1e-12)), math.log10(lo), math.log10(hi)
        return self.x + (value - lo) / (hi - lo) * self.w if hi > lo else self.x

    def sy(self, value: float) -> float:
        lo, hi = self.ylim
        if self.ylog:
            value, lo, hi = math.log10(max(value, 1e-12)), math.log10(lo), math.log10(hi)
        return self.y + self.h - (value - lo) / (hi - lo) * self.h if hi > lo else self.y + self.h

    # ---- chrome ----
    def y_grid(self, ticks: Sequence[float], label: str = "", fmt: Callable[[float], str] = fmt_number) -> None:
        for tick in ticks:
            yy = self.sy(tick)
            self.parts.append(f'<line class="grid" x1="{self.x:.1f}" x2="{self.x + self.w:.1f}" y1="{yy:.1f}" '
                              f'y2="{yy:.1f}"/>')
            self.parts.append(f'<text class="tm" x="{self.x - 6:.1f}" y="{yy + 4:.1f}" font-size="11" '
                              f'text-anchor="end">{escape(fmt(tick))}</text>')
        if label:
            self.parts.append(f'<text class="t2" x="{self.x:.1f}" y="{self.y - 8:.1f}" font-size="12">'
                              f'{escape(label)}</text>')

    def x_axis(self, ticks: Sequence[float], label: str = "", fmt: Callable[[float], str] = fmt_number,
               show_labels: bool = True) -> None:
        base = self.y + self.h
        self.parts.append(f'<line class="axis" x1="{self.x:.1f}" x2="{self.x + self.w:.1f}" y1="{base:.1f}" '
                          f'y2="{base:.1f}"/>')
        if not show_labels:
            return
        for tick in ticks:
            xx = self.sx(tick)
            self.parts.append(f'<text class="tm" x="{xx:.1f}" y="{base + 15:.1f}" font-size="11" '
                              f'text-anchor="middle">{escape(fmt(tick))}</text>')
        if label:
            self.parts.append(f'<text class="t2" x="{self.x + self.w:.1f}" y="{base + 30:.1f}" font-size="11" '
                              f'text-anchor="end">{escape(label)}</text>')

    def x_grid(self, ticks: Sequence[float]) -> None:
        for tick in ticks:
            xx = self.sx(tick)
            self.parts.append(f'<line class="grid" x1="{xx:.1f}" x2="{xx:.1f}" y1="{self.y:.1f}" '
                              f'y2="{self.y + self.h:.1f}"/>')

    # ---- marks ----
    def line(self, xs: Sequence[float], ys: Sequence[float], stroke: str, width: float = 2.0,
             step: bool = False) -> None:
        points: List[Tuple[float, float]] = []
        for i, (xv, yv) in enumerate(zip(xs, ys)):
            if yv is None or (isinstance(yv, float) and math.isnan(yv)):
                if len(points) > 1:
                    self._polyline(points, stroke, width)
                points = []
                continue
            if step and points:
                points.append((self.sx(xv), points[-1][1]))
            points.append((self.sx(xv), self.sy(yv)))
        if len(points) > 1:
            self._polyline(points, stroke, width)

    def _polyline(self, points: Sequence[Tuple[float, float]], stroke: str, width: float) -> None:
        coords = " ".join(f"{px:.1f},{py:.1f}" for px, py in simplify(points))
        self.parts.append(f'<polyline fill="none" stroke="{stroke}" stroke-width="{width}" stroke-linejoin="round" '
                          f'stroke-linecap="round" points="{coords}"/>')

    def dot(self, xv: float, yv: float, fill: str, r: float = 4.0, hollow: bool = False) -> None:
        if hollow:
            self.parts.append(f'<circle cx="{self.sx(xv):.1f}" cy="{self.sy(yv):.1f}" r="{r}" fill="var(--surface)" '
                              f'stroke="{fill}" stroke-width="2"/>')
        else:
            self.parts.append(f'<circle class="ring" cx="{self.sx(xv):.1f}" cy="{self.sy(yv):.1f}" r="{r}" '
                              f'fill="{fill}"/>')

    def vband(self, x0: float, x1: float, fill: str, opacity: float = 0.12) -> None:
        a, b = self.sx(x0), self.sx(x1)
        self.parts.append(f'<rect x="{a:.1f}" y="{self.y:.1f}" width="{max(b - a, 1):.1f}" height="{self.h:.1f}" '
                          f'fill="{fill}" opacity="{opacity}"/>')

    def column(self, xv: float, width_px: float, yv: float, fill: str) -> None:
        """Column from the baseline; 4 px rounded top, square base (only when tall enough)."""
        x0 = self.sx(xv) - width_px / 2
        top, base = self.sy(yv), self.sy(self.ylim[0])
        height = base - top
        if height <= 0:
            return
        r = min(4.0, width_px / 2, height)
        self.parts.append(f'<path d="M{x0:.1f},{base:.1f} V{top + r:.1f} Q{x0:.1f},{top:.1f} {x0 + r:.1f},{top:.1f} '
                          f'H{x0 + width_px - r:.1f} Q{x0 + width_px:.1f},{top:.1f} {x0 + width_px:.1f},{top + r:.1f} '
                          f'V{base:.1f} Z" fill="{fill}"/>')

    def text(self, xv: float, yv: float, label: str, cls: str = "t2", anchor: str = "start", size: int = 11,
             dx: float = 0, dy: float = 0) -> None:
        self.parts.append(f'<text class="{cls}" x="{self.sx(xv) + dx:.1f}" y="{self.sy(yv) + dy:.1f}" '
                          f'font-size="{size}" text-anchor="{anchor}">{escape(label)}</text>')


@dataclass
class Lanes:
    """Timeline rows ("Gantt" lanes) sharing an x-scale with the panels."""
    x: float
    y: float
    w: float
    row_h: float
    xlim: Tuple[float, float]
    labels: List[str]
    parts: List[str] = field(default_factory=list)

    def sx(self, value: float) -> float:
        lo, hi = self.xlim
        return self.x + (min(max(value, lo), hi) - lo) / (hi - lo) * self.w

    @property
    def h(self) -> float:
        return self.row_h * len(self.labels)

    def frame(self) -> None:
        for i, label in enumerate(self.labels):
            yy = self.y + i * self.row_h
            self.parts.append(f'<text class="t2" x="{self.x - 8:.1f}" y="{yy + self.row_h / 2 + 4:.1f}" '
                              f'font-size="11" text-anchor="end">{escape(label)}</text>')
            self.parts.append(f'<line class="grid" x1="{self.x:.1f}" x2="{self.x + self.w:.1f}" '
                              f'y1="{yy + self.row_h:.1f}" y2="{yy + self.row_h:.1f}"/>')

    def segment(self, row: int, x0: float, x1: float, fill: str, label: str = "", label_dark: object = False,
                thickness: float = 0.62, center: float = 0.5, outside: bool = False) -> None:
        """Bar from x0 to x1 in a row. The label goes inside only if it fits; with outside=True a
        label that does not fit is drawn after the bar end instead (never clipped)."""
        a, b = self.sx(x0), self.sx(x1)
        bar_h = self.row_h * thickness
        yy = self.y + row * self.row_h + self.row_h * center - bar_h / 2
        width = max(b - a - 2, 1.5)  # 2 px surface gap to the next segment
        self.parts.append(f'<rect x="{a:.1f}" y="{yy:.1f}" width="{width:.1f}" height="{bar_h:.1f}" rx="2" '
                          f'fill="{fill}"/>')
        if label and width >= 7 * len(label) + 8:
            # label_dark: False = white (on saturated fills), True = near-black in both modes (on
            # yellow), "ink" = primary ink token (on the neutral fill, which flips with the theme)
            label_fill = "var(--text1)" if label_dark == "ink" else ("#0b0b0b" if label_dark else "#ffffff")
            self.parts.append(f'<text x="{a + width / 2:.1f}" y="{yy + bar_h / 2 + 4:.1f}" font-size="10.5" '
                              f'text-anchor="middle" fill="{label_fill}">{escape(label)}</text>')
        elif label and outside:
            self.parts.append(f'<text class="t2" x="{a + width + 6:.1f}" y="{yy + bar_h / 2 + 4:.1f}" '
                              f'font-size="10.5">{escape(label)}</text>')

    def tick(self, row: int, xv: float, stroke: str, height: float = 0.6) -> None:
        xx = self.sx(xv)
        mid = self.y + row * self.row_h + self.row_h / 2
        half = self.row_h * height / 2
        self.parts.append(f'<line x1="{xx:.1f}" x2="{xx:.1f}" y1="{mid - half:.1f}" y2="{mid + half:.1f}" '
                          f'stroke="{stroke}" stroke-width="1.2"/>')

    def event(self, row: int, xv: float, fill: str, label: str = "", label_side: str = "right") -> None:
        xx = self.sx(xv)
        mid = self.y + row * self.row_h + self.row_h / 2
        self.parts.append(f'<path class="ring" d="M{xx:.1f},{mid - 6:.1f} L{xx + 6:.1f},{mid:.1f} L{xx:.1f},{mid + 6:.1f} '
                          f'L{xx - 6:.1f},{mid:.1f} Z" fill="{fill}"/>')
        if label:
            anchor, dx = ("start", 10) if label_side == "right" else ("end", -10)
            self.parts.append(f'<text class="t2" x="{xx + dx:.1f}" y="{mid + 4:.1f}" font-size="10.5" '
                              f'text-anchor="{anchor}">{escape(label)}</text>')


class Figure:
    def __init__(self, width: float, height: float, title: str, subtitle: str = "") -> None:
        self.width = width
        self.height = height
        self.parts: List[str] = [
            f'<rect class="bg" width="{width}" height="{height}" rx="6"/>',
            f'<text class="t1" x="20" y="28" font-size="15" font-weight="600">{escape(title)}</text>']
        if subtitle:
            self.parts.append(f'<text class="t2" x="20" y="47" font-size="12">{escape(subtitle)}</text>')
        self.title = title
        self.items: List[object] = []

    def add(self, item) -> None:
        self.items.append(item)

    def raw(self, svg: str) -> None:
        self.parts.append(svg)

    def legend(self, entries: Sequence[Tuple[str, str, str]], x: float, y: float, gap: float = 22) -> float:
        """entries: (kind, colour, label) with kind 'line' | 'box' | 'dot' | 'hollow' | 'diamond'.
        Returns the x after the last entry."""
        for kind, fill, label in entries:
            if kind == "line":
                self.parts.append(f'<line x1="{x:.1f}" x2="{x + 20:.1f}" y1="{y:.1f}" y2="{y:.1f}" stroke="{fill}" '
                                  f'stroke-width="2" stroke-linecap="round"/>')
                x += 26
            elif kind == "box":
                self.parts.append(f'<rect x="{x:.1f}" y="{y - 6:.1f}" width="14" height="12" rx="2" fill="{fill}"/>')
                x += 20
            elif kind == "hollow":
                self.parts.append(f'<circle cx="{x + 5:.1f}" cy="{y:.1f}" r="4" fill="var(--surface)" stroke="{fill}" '
                                  f'stroke-width="2"/>')
                x += 16
            elif kind == "diamond":
                self.parts.append(f'<path d="M{x + 5:.1f},{y - 6:.1f} L{x + 11:.1f},{y:.1f} L{x + 5:.1f},{y + 6:.1f} '
                                  f'L{x - 1:.1f},{y:.1f} Z" fill="{fill}"/>')
                x += 18
            else:
                self.parts.append(f'<circle cx="{x + 5:.1f}" cy="{y:.1f}" r="4" fill="{fill}"/>')
                x += 16
            self.parts.append(f'<text class="t2" x="{x:.1f}" y="{y + 4:.1f}" font-size="11">{escape(label)}</text>')
            x += 7.0 * len(label) + gap
        return x

    def note(self, x: float, y: float, text: str, cls: str = "tm", size: int = 10.5, anchor: str = "start") -> None:
        self.parts.append(f'<text class="{cls}" x="{x:.1f}" y="{y:.1f}" font-size="{size}" '
                          f'text-anchor="{anchor}">{escape(text)}</text>')

    def svg(self) -> str:
        body = list(self.parts)
        for item in self.items:
            body += item.parts  # type: ignore[attr-defined]
        return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.width}" height="{self.height}" '
                f'viewBox="0 0 {self.width} {self.height}" role="img" aria-label="{escape(self.title)}" '
                f'font-family="{FONT.replace(chr(34), chr(39))}">'
                f'<title>{escape(self.title)}</title>{_style()}' + _resolve_tokens("\n".join(body)) + "</svg>\n")

    def save(self, path: Path) -> None:
        path.write_text(self.svg(), encoding="utf-8")


def simplify(points: Sequence[Tuple[float, float]], min_dx: float = 1.0, min_dy: float = 0.5) -> List[Tuple[float, float]]:
    """Drops points (in pixels) that would not change the drawing: a point is kept when it moves
    >= min_dx horizontally or >= min_dy vertically from the last kept point. First and last are
    always kept. Visually lossless at the drawn size; the raw data is kept elsewhere."""
    if len(points) <= 2:
        return list(points)
    kept = [points[0]]
    for i in range(1, len(points) - 1):
        px, py = points[i]
        lx, ly = kept[-1]
        if abs(px - lx) >= min_dx or abs(py - ly) >= min_dy:
            kept.append(points[i])
    kept.append(points[-1])
    return kept


def cdf(values: Sequence[float]) -> Tuple[List[float], List[float]]:
    """Empirical CDF points (x sorted, y in percent)."""
    xs = sorted(values)
    n = len(xs)
    return xs, [100.0 * (i + 1) / n for i in range(n)]


def percentile(values: Sequence[float], p: float) -> Optional[float]:
    if not values:
        return None
    xs = sorted(values)
    return xs[min(len(xs) - 1, max(0, int(round(p / 100.0 * (len(xs) - 1)))))]
