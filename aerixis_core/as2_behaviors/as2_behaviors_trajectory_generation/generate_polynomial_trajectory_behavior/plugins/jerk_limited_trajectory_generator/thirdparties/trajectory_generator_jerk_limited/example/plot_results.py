#!/usr/bin/env python3

# Copyright 2025 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


"""Plot a trajectory logged by the ``run_example`` driver.

Reads two artefacts:
    1. A sample CSV with columns
       ``t, px, py, pz, vx, vy, vz, ax, ay, az`` (produced by
       ``run_example``).
    2. An optional sibling spline CSV ``<base>.spline.csv`` (produced by
       ``Spline::saveToCsv``) with the per-segment polynomial coefficients.

Produces two figures:
    1. 3D trajectory: each polynomial piece is evaluated independently from
       its own coefficients and rendered with a distinct color. The CSV
       sampled trace is overlaid as a thin dashed gray line so the user can
       visually verify that the analytical reconstruction matches the
       sampled output.
    2. 4x3 grid of time histories. Columns are position, velocity and
       acceleration; the first three rows are the x/y/z components and the
       fourth row is the Euclidean norm. The component subplots overlay the
       per-segment polynomial evaluation (and its derivatives) on top of
       the CSV samples; vertical dashed lines mark the segment breakpoints
       on every subplot. The norm subplots are not piecewise-polynomial
       (norm of three polynomials is not a polynomial), so they only show
       the sampled trace and the breakpoints.

If the spline CSV is not found, the per-segment overlay and breakpoints
are skipped and the legacy plots are produced (with a warning).

Usage:
    python3 plot_results.py -f example/trajectory_logs/trajectory.csv \\
        [-c example/config_example.yaml] [--vmax 4.0] \\
        [--spline example/trajectory_logs/trajectory.spline.csv] \\
        [--no-segments]

Author: Rafael Perez-Segui <r.psegui@upm.es>
"""

from __future__ import annotations

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import argparse
import os
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from numpy.polynomial import polynomial as np_poly

CSV_COLUMNS = ["t", "px", "py", "pz", "vx", "vy", "vz", "ax", "ay", "az"]

AXIS_NAMES = ("x", "y", "z")
AXIS_COLORS = ("tab:blue", "tab:orange", "tab:green")
NORM_COLOR = "tab:purple"

BREAKPOINT_KW = dict(color="gray", linestyle="--", alpha=0.35, linewidth=0.8)
SEGMENT_SAMPLES_PER_PIECE = 80
SEGMENT_EXTENSION_SAMPLES = 40
SEGMENT_EXT_LINE_KW = dict(linestyle="--", linewidth=1.1, alpha=0.75, zorder=3)
SEGMENT_IN_LINE_KW = dict(linewidth=1.8, alpha=1.0, zorder=4)
SPLINE_FORMAT_VERSION_SUPPORTED = 1


@dataclass
class SegmentPoly:
    """Single polynomial piece on local time tau in [0, duration].

    coeffs has shape (3, N) with rows = axes (x, y, z) and columns =
    polynomial coefficients in increasing power of tau (c0, c1, ...).
    """
    duration: float
    coeffs: np.ndarray  # shape (3, N)


@dataclass
class Spline:
    """Piecewise polynomial trajectory loaded from a `.spline.csv`."""
    start_time: float
    segments: List[SegmentPoly] = field(default_factory=list)

    def breakpoints(self) -> np.ndarray:
        if not self.segments:
            return np.array([self.start_time])
        durations = np.asarray([s.duration for s in self.segments], dtype=float)
        return self.start_time + np.concatenate([[0.0], np.cumsum(durations)])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot a trajectory CSV produced by run_example.",
    )
    parser.add_argument(
        "-f",
        "--file",
        default="example/trajectory_logs/trajectory.csv",
        help="Path to the trajectory CSV.",
    )
    parser.add_argument(
        "-c",
        "--sim-config",
        default=None,
        help="Optional sim config YAML used to overlay the configured "
        "waypoints on the 3D plot and as references on the position panels.",
    )
    parser.add_argument(
        "--vmax",
        type=float,
        default=None,
        help="Reference max velocity [m/s] drawn as a horizontal line on the "
        "|v| panel.",
    )
    parser.add_argument(
        "--spline",
        default=None,
        help="Path to the spline CSV produced by Spline::saveToCsv. "
        "Defaults to the sibling '<base>.spline.csv' next to --file.",
    )
    parser.add_argument(
        "--no-segments",
        action="store_true",
        help="Disable per-segment polynomial overlay (legacy single-line plot).",
    )
    return parser.parse_args()


def load_csv(path: str) -> pd.DataFrame:
    if not os.path.isfile(path):
        sys.exit(f"CSV not found: {path}")
    df = pd.read_csv(path)
    missing = [c for c in CSV_COLUMNS if c not in df.columns]
    if missing:
        sys.exit(f"CSV {path} is missing columns: {missing}")
    return df


def load_waypoints(path: Optional[str]) -> Optional[np.ndarray]:
    if path is None:
        return None
    if not os.path.isfile(path):
        print(f"Warning: sim config not found at {path}, skipping waypoints.",
              file=sys.stderr)
        return None
    try:
        import yaml  # Local import — pyyaml is optional.
    except ImportError:
        print("Warning: pyyaml not installed, skipping waypoints overlay.",
              file=sys.stderr)
        return None
    with open(path, "r", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh) or {}
    wps = cfg.get("sim_config", {}).get("waypoints", [])
    if not wps:
        return None
    return np.asarray(wps, dtype=float)


def _default_spline_path(csv_path: str) -> str:
    """Mirror of ``splineCsvPathFor()`` in run_example.cpp."""
    p = Path(csv_path)
    if p.suffix == ".csv":
        return str(p.with_suffix(".spline.csv"))
    return str(p.with_name(p.name + ".spline.csv"))


def load_spline(csv_path: str, override_path: Optional[str]) -> Optional[Spline]:
    """Load a spline CSV.

    Resolution order: `override_path` > sibling auto-detect.
    Returns None on missing file or unsupported format, after a stderr
    warning.
    """
    spline_path = override_path or _default_spline_path(csv_path)
    if not os.path.isfile(spline_path):
        if override_path is not None:
            print(f"Warning: spline CSV not found at {spline_path}.",
                  file=sys.stderr)
        return None

    meta: dict = {}
    data_lines: List[str] = []
    header_seen = False
    with open(spline_path, "r", encoding="utf-8") as fh:
        for raw in fh:
            line = raw.strip()
            if not line:
                continue
            if line.startswith("#"):
                _, _, body = line.partition("#")
                key, sep, value = body.strip().partition(":")
                if sep:
                    meta[key.strip()] = value.strip()
                continue
            if not header_seen:
                header_seen = True  # first non-comment row is the column header
                continue
            data_lines.append(line)

    try:
        version = int(meta.get("spline_format_version", "-1"))
        start_time = float(meta.get("start_time", "0.0"))
        dim = int(meta.get("dimension", "0"))
        n_poly = int(meta.get("polynomial_order", "0"))
    except ValueError:
        print(f"Warning: malformed metadata in {spline_path}, skipping spline overlay.",
              file=sys.stderr)
        return None

    if version != SPLINE_FORMAT_VERSION_SUPPORTED:
        print(f"Warning: unsupported spline format version {version} in "
              f"{spline_path} (expected {SPLINE_FORMAT_VERSION_SUPPORTED}); "
              "skipping spline overlay.", file=sys.stderr)
        return None
    if dim != 3 or n_poly <= 0:
        print(f"Warning: unsupported spline shape (dim={dim}, n_poly={n_poly}) "
              f"in {spline_path}; skipping overlay.", file=sys.stderr)
        return None
    if not data_lines:
        print(f"Warning: spline CSV {spline_path} has no segments; "
              "skipping overlay.", file=sys.stderr)
        return None

    expected = 1 + dim * n_poly
    segments: List[SegmentPoly] = []
    for ln in data_lines:
        cells = [c.strip() for c in ln.split(",")]
        if len(cells) != expected:
            print(f"Warning: malformed row in {spline_path} "
                  f"(got {len(cells)} cells, expected {expected}); skipping.",
                  file=sys.stderr)
            return None
        try:
            values = [float(c) for c in cells]
        except ValueError:
            print(f"Warning: non-numeric row in {spline_path}; skipping.",
                  file=sys.stderr)
            return None
        duration = values[0]
        coeffs = np.asarray(values[1:], dtype=float).reshape(dim, n_poly)
        segments.append(SegmentPoly(duration=duration, coeffs=coeffs))

    return Spline(start_time=start_time, segments=segments)


def _eval_axis(coeffs_axis: np.ndarray, tau: np.ndarray, derivative: int) -> np.ndarray:
    """Evaluate ``p^{(derivative)}(tau)`` for a 1D coefficient row.

    coeffs_axis shape: (N,), ascending powers (c0, c1, ..., c_{N-1}).
    Returns array with shape ``tau.shape``.
    """
    c = coeffs_axis
    for _ in range(derivative):
        c = np_poly.polyder(c)
    if c.size == 0:
        return np.zeros_like(tau)
    return np_poly.polyval(tau, c)


def _segment_curves(seg: SegmentPoly,
                    derivative: int,
                    n_samples: int = SEGMENT_SAMPLES_PER_PIECE
                    ) -> Tuple[np.ndarray, np.ndarray]:
    """Sample one segment at `n_samples` points along tau in [0, duration].

    Returns (tau_grid, values[3, n_samples]).
    """
    tau = np.linspace(0.0, seg.duration, n_samples)
    vals = np.vstack([_eval_axis(seg.coeffs[d], tau, derivative) for d in range(3)])
    return tau, vals


def _spline_extension_amount(spline: Spline) -> float:
    """Return the time amount each polynomial is extended on each side.

    Uses the mean segment duration so every polynomial is shown over a
    similar time span regardless of how short its immediate neighbours
    are (otherwise a long segment surrounded by tiny anchor pieces
    would have a barely-visible dashed extension).
    """
    n = len(spline.segments)
    if n == 0:
        return 0.0
    return sum(s.duration for s in spline.segments) / n


def _segment_extension_taus(spline: Spline, i: int, extension: float) -> Tuple[float, float]:
    """Return (tau_left, tau_right) bounds for the dashed extension of
    segment i, expressed in the segment's local time (tau in [0, T_i]).

    Each polynomial is extended by ``extension`` seconds on each side so
    the user can see how it diverges away from the spline outside its
    active window.
    """
    seg_dur = spline.segments[i].duration
    return -extension, seg_dur + extension


def _segment_color(idx: int, n_segments: int):
    """Return a unique color for segment ``idx`` out of ``n_segments``.

    Avoids the wrap-around that happens when more segments than the
    qualitative cmap can hold are plotted (e.g. 13 segments on tab10
    would reuse segment-0's color for segment-10).
    """
    if n_segments <= 10:
        return plt.get_cmap("tab10")(idx)
    if n_segments <= 20:
        return plt.get_cmap("tab20")(idx)
    # Fall back to a continuous map sampled uniformly. `turbo` is
    # perceptually distinct across its full range.
    return plt.get_cmap("turbo")(idx / max(n_segments - 1, 1))


def plot_trajectory_3d(df: pd.DataFrame,
                       waypoints: Optional[np.ndarray],
                       spline: Optional[Spline]) -> plt.Figure:
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")
    px = df["px"].to_numpy()
    py = df["py"].to_numpy()
    pz = df["pz"].to_numpy()

    if spline is None:
        ax.plot(px, py, pz, color="tab:blue", label="trajectory")
    else:
        n_segs = len(spline.segments)
        extension = _spline_extension_amount(spline)
        # First pass: dashed extensions for every segment.
        for i, seg in enumerate(spline.segments):
            color = _segment_color(i, n_segs)
            tau_left, tau_right = _segment_extension_taus(spline, i, extension)
            if tau_left < 0.0:
                tau_a = np.linspace(tau_left, 0.0, SEGMENT_EXTENSION_SAMPLES)
                ext_a = np.vstack(
                    [_eval_axis(seg.coeffs[d], tau_a, derivative=0) for d in range(3)]
                )
                ax.plot(ext_a[0], ext_a[1], ext_a[2], color=color,
                        **SEGMENT_EXT_LINE_KW)
            if tau_right > seg.duration:
                tau_b = np.linspace(seg.duration, tau_right, SEGMENT_EXTENSION_SAMPLES)
                ext_b = np.vstack(
                    [_eval_axis(seg.coeffs[d], tau_b, derivative=0) for d in range(3)]
                )
                ax.plot(ext_b[0], ext_b[1], ext_b[2], color=color,
                        **SEGMENT_EXT_LINE_KW)
        # Second pass: solid in-window slices on top of all dashed extensions.
        for i, seg in enumerate(spline.segments):
            color = _segment_color(i, n_segs)
            _, p = _segment_curves(seg, derivative=0)
            ax.plot(p[0], p[1], p[2], color=color,
                    label=f"seg {i}",
                    **SEGMENT_IN_LINE_KW)

    ax.scatter(px[0], py[0], pz[0],
               color="tab:green", s=60, label="start")
    ax.scatter(px[-1], py[-1], pz[-1],
               color="tab:red", s=60, label="end")
    if waypoints is not None and waypoints.size > 0:
        ax.scatter(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2],
                   color="black", marker="x", s=80, label="waypoints")
        for idx, wp in enumerate(waypoints):
            ax.text(wp[0], wp[1], wp[2], f"  w{idx}",
                    color="black", fontsize=9)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    title = "3D trajectory"
    if spline is not None:
        title += f" — {len(spline.segments)} polynomial pieces"
    ax.set_title(title)
    ax.legend(loc="best", fontsize=8, ncol=2)

    if spline is not None:
        # Clamp the 3D bbox to the CSV trace so the dashed extensions
        # cannot blow up the axes.
        margin = 0.1
        for axis_lim, axis_data in zip(
                (ax.set_xlim, ax.set_ylim, ax.set_zlim), (px, py, pz)):
            lo, hi = float(np.nanmin(axis_data)), float(np.nanmax(axis_data))
            span = max(hi - lo, 1e-6)
            axis_lim(lo - margin * span, hi + margin * span)

    fig.tight_layout()
    return fig


def _waypoint_closest_times(df: pd.DataFrame,
                            waypoints: np.ndarray) -> np.ndarray:
    pos = df[["px", "py", "pz"]].to_numpy()
    t = df["t"].to_numpy()
    idx = np.argmin(
        np.linalg.norm(pos[:, None, :] - waypoints[None, :, :], axis=2),
        axis=0,
    )
    return t[idx]


def _overlay_segments_on_axes(ax: plt.Axes,
                              spline: Spline,
                              axis_index: int,
                              derivative: int) -> None:
    """Plot each polynomial both inside its active window (solid) and
    extended into its immediate neighbours' windows (dashed).

    The dashed extension makes the spline construction explicit: each
    polynomial exists everywhere, but the spline only takes the slice
    inside ``[t_break_i, t_break_{i+1}]``. Outside that slice the
    polynomial typically diverges from the spline.

    Dashed extensions of all segments are drawn first, then every solid
    in-window slice on top, so that adjacent segments' solids never hide
    a dashed extension of a different polynomial that overlaps the same
    time window.
    """
    knots = spline.breakpoints()
    n_segs = len(spline.segments)
    extension = _spline_extension_amount(spline)

    # First pass: dashed extensions for every segment.
    for i, seg in enumerate(spline.segments):
        color = _segment_color(i, n_segs)
        tau_left, tau_right = _segment_extension_taus(spline, i, extension)
        if tau_left < 0.0:
            tau_a = np.linspace(tau_left, 0.0, SEGMENT_EXTENSION_SAMPLES)
            v_a = _eval_axis(seg.coeffs[axis_index], tau_a, derivative)
            ax.plot(knots[i] + tau_a, v_a, color=color, **SEGMENT_EXT_LINE_KW)
        if tau_right > seg.duration:
            tau_b = np.linspace(seg.duration, tau_right, SEGMENT_EXTENSION_SAMPLES)
            v_b = _eval_axis(seg.coeffs[axis_index], tau_b, derivative)
            ax.plot(knots[i] + tau_b, v_b, color=color, **SEGMENT_EXT_LINE_KW)

    # Second pass: solid in-window slices on top of all dashed extensions.
    is_legend_panel = axis_index == 0 and derivative == 0
    for i, seg in enumerate(spline.segments):
        color = _segment_color(i, n_segs)
        tau, vals = _segment_curves(seg, derivative=derivative)
        ax.plot(knots[i] + tau, vals[axis_index], color=color,
                label=f"seg {i}" if is_legend_panel else None,
                **SEGMENT_IN_LINE_KW)


def _draw_breakpoints(ax: plt.Axes, knots: np.ndarray) -> None:
    for k in knots:
        ax.axvline(k, **BREAKPOINT_KW)


def _clamp_ylim_to_csv(ax: plt.Axes, csv_values: np.ndarray, margin_frac: float = 0.1) -> None:
    """Force ax y-limits to fit the CSV trace plus a small margin.

    Without this, the dashed polynomial extensions (degree 5, evaluated
    well outside their valid window) would auto-scale the axis to absurd
    ranges and squash the actual trajectory into a flat line.
    """
    lo = float(np.nanmin(csv_values))
    hi = float(np.nanmax(csv_values))
    span = max(hi - lo, 1e-6)
    ax.set_ylim(lo - margin_frac * span, hi + margin_frac * span)


def plot_time_history(df: pd.DataFrame,
                      waypoints: Optional[np.ndarray],
                      vmax: Optional[float],
                      spline: Optional[Spline]) -> plt.Figure:
    fig, axes = plt.subplots(4, 3, sharex=True, figsize=(15, 11))
    t = df["t"].to_numpy()

    pos_cols = ("px", "py", "pz")
    vel_cols = ("vx", "vy", "vz")
    acc_cols = ("ax", "ay", "az")

    wp_times = None
    if waypoints is not None and waypoints.size > 0:
        wp_times = _waypoint_closest_times(df, waypoints)

    knots = spline.breakpoints() if spline is not None else None

    for row in range(3):
        color = AXIS_COLORS[row]
        if spline is None:
            axes[row, 0].plot(t, df[pos_cols[row]].to_numpy(), color=color)
            axes[row, 1].plot(t, df[vel_cols[row]].to_numpy(), color=color)
            axes[row, 2].plot(t, df[acc_cols[row]].to_numpy(), color=color)
        else:
            _overlay_segments_on_axes(axes[row, 0], spline, row, derivative=0)
            _overlay_segments_on_axes(axes[row, 1], spline, row, derivative=1)
            _overlay_segments_on_axes(axes[row, 2], spline, row, derivative=2)
            # Clamp y-limits to the in-window values (the dashed extensions
            # would otherwise let high-degree polynomials blow up the axes).
            _clamp_ylim_to_csv(axes[row, 0], df[pos_cols[row]].to_numpy())
            _clamp_ylim_to_csv(axes[row, 1], df[vel_cols[row]].to_numpy())
            _clamp_ylim_to_csv(axes[row, 2], df[acc_cols[row]].to_numpy())

        axes[row, 0].set_ylabel(f"{pos_cols[row]} [m]")
        axes[row, 0].grid(True, alpha=0.3)
        if wp_times is not None:
            axes[row, 0].scatter(
                wp_times, waypoints[:, row],
                color="black", marker="x", s=60, zorder=5,
                label="waypoints" if row == 0 else None,
            )

        axes[row, 1].set_ylabel(f"{vel_cols[row]} [m/s]")
        axes[row, 1].grid(True, alpha=0.3)

        axes[row, 2].set_ylabel(f"{acc_cols[row]} [m/s^2]")
        axes[row, 2].grid(True, alpha=0.3)

    norm_p = np.linalg.norm(df[list(pos_cols)].to_numpy(), axis=1)
    norm_v = np.linalg.norm(df[list(vel_cols)].to_numpy(), axis=1)
    norm_a = np.linalg.norm(df[list(acc_cols)].to_numpy(), axis=1)

    axes[3, 0].plot(t, norm_p, color=NORM_COLOR)
    axes[3, 0].set_ylabel("|p| [m]")
    axes[3, 0].grid(True, alpha=0.3)

    axes[3, 1].plot(t, norm_v, color=NORM_COLOR, label="|v|")
    axes[3, 1].set_ylabel("|v| [m/s]")
    axes[3, 1].grid(True, alpha=0.3)
    if vmax is not None:
        axes[3, 1].axhline(vmax, color="tab:red", linestyle="--",
                           label=f"v_max = {vmax} m/s")
        axes[3, 1].legend(loc="best", fontsize=8)

    axes[3, 2].plot(t, norm_a, color=NORM_COLOR)
    axes[3, 2].set_ylabel("|a| [m/s^2]")
    axes[3, 2].grid(True, alpha=0.3)

    if knots is not None:
        for ax in axes.ravel():
            _draw_breakpoints(ax, knots)

    for ax in axes[-1, :]:
        ax.set_xlabel("t [s]")

    axes[0, 0].set_title("Position")
    axes[0, 1].set_title("Velocity")
    axes[0, 2].set_title("Acceleration")
    if wp_times is not None or spline is not None:
        axes[0, 0].legend(loc="best", fontsize=7, ncol=2)

    suptitle = "Trajectory time history"
    if spline is not None:
        suptitle += f" — {len(spline.segments)} polynomial pieces"
    fig.suptitle(suptitle)
    fig.tight_layout()
    return fig


def main() -> None:
    args = parse_args()
    df = load_csv(args.file)
    waypoints = load_waypoints(args.sim_config)
    spline = None
    if not args.no_segments:
        spline = load_spline(args.file, args.spline)
        if spline is None:
            print("Note: per-segment polynomial overlay disabled "
                  "(spline CSV unavailable).", file=sys.stderr)

    figs = [
        plot_trajectory_3d(df, waypoints, spline),
        plot_time_history(df, waypoints, args.vmax, spline),
    ]

    for fig in figs:
        fig.show()
    plt.pause(0.001)
    if sys.stdin.isatty():
        print("\nPress [Enter] to close.")
        try:
            input()
        except KeyboardInterrupt:
            pass
    plt.close("all")


if __name__ == "__main__":
    main()
