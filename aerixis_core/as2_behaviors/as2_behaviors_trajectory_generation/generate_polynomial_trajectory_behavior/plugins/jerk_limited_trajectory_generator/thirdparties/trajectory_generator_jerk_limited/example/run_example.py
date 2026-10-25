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

"""Python equivalent of example/run_example.cpp using the one-shot API.

Loads ``config_example.yaml`` (sim_config: step_dt + waypoints) and
``config_trajectory.yaml`` (trajectory_parameters: kinematic bounds),
generates a multi-waypoint trajectory in one shot via
:class:`Trajectory`, samples it at fixed ``step_dt`` and writes a
10-column CSV plus a sibling ``<base>.spline.csv``.

Usage (from the repo root, after cmake --build build with BUILD_PYBIND=ON):
    ./example/run_example_py.sh
    python3 example/run_example.py \
        -c example/config_example.yaml \
        -d example/config_trajectory.yaml \
        -f example/trajectory_logs/trajectory_py.csv
"""

from __future__ import annotations

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import argparse
import sys
from pathlib import Path
from typing import List, Tuple

import numpy as np
import yaml

from trajectory_generator_jerk_limited import (
    EndWaypoint,
    GeneratorConfig,
    Trajectory,
    Waypoint,
)


def _parse_cli() -> argparse.Namespace:
    """Parse command-line arguments mirroring the C++ example flags."""
    parser = argparse.ArgumentParser(
        description='One-shot multi-waypoint trajectory_generator_jerk_limited example.')
    parser.add_argument('-c', '--sim-config', required=True, dest='sim_config',
                        help='Path to the simulation YAML.')
    parser.add_argument('-d', '--trajectory-config', required=True, dest='trajectory_config',
                        help='Path to the trajectory parameters YAML.')
    parser.add_argument('-f', '--output', required=True, dest='output',
                        help='Path of the output CSV file.')
    return parser.parse_args()


def _load_sim_config(path: str) -> Tuple[float, List[np.ndarray]]:
    """Load ``sim_config`` block: step_dt + waypoints."""
    with open(path, encoding='utf-8') as handle:
        root = yaml.safe_load(handle)
    if root is None or 'sim_config' not in root:
        raise ValueError(f"{path}: missing 'sim_config' section.")
    sim = root['sim_config']
    step_dt = float(sim.get('step_dt', 0.02))
    raw_waypoints = sim.get('waypoints', [])
    if not raw_waypoints:
        raise ValueError(f"{path}: 'sim_config.waypoints' must be a non-empty sequence.")
    waypoints: List[np.ndarray] = []
    for i, wp in enumerate(raw_waypoints):
        arr = np.array(wp, dtype=np.float64)
        if arr.shape != (3,):
            raise ValueError(f"{path}: 'sim_config.waypoints[{i}]' must be a 3-element sequence.")
        waypoints.append(arr)
    if len(waypoints) < 2:
        raise ValueError(
            f"{path}: 'sim_config.waypoints' must contain at least 2 waypoints.")
    return step_dt, waypoints


def _load_trajectory_config(path: str) -> dict:
    """Load ``trajectory_parameters`` and the optional ``simulator`` block as a flat dict."""
    with open(path, encoding='utf-8') as handle:
        root = yaml.safe_load(handle)
    if root is None or 'trajectory_parameters' not in root:
        raise ValueError(f"{path}: missing 'trajectory_parameters' section.")
    block = root['trajectory_parameters']
    sim = root.get('simulator', {}) or {}
    return {
        'max_speed': float(block.get('max_speed', 0.0)),
        'max_acceleration': float(block.get('max_acceleration', 0.0)),
        'max_jerk': float(block.get('max_jerk', 0.0)),
        'max_tracking_error': float(block.get('max_tracking_error', 0.0)),
        'acceptance_radius': float(block.get('acceptance_radius', 0.0)),
        'trajectory_gain': float(block.get('trajectory_gain', 1.0)),
        'max_simulation_time': float(sim.get('max_simulation_time', 600.0)),
        'settle_velocity': float(sim.get('settle_velocity', 0.05)),
        'polynomial_order': int(sim.get('polynomial_order', 16)),
    }


def _build_waypoints(positions: List[np.ndarray]) -> List[Waypoint]:
    """Compose the waypoint list. positions[0] is the start (pinned to rest);
    the last is also pinned to rest; intermediates only constrain position."""
    if not positions:
        return []
    wps: List[Waypoint] = [EndWaypoint(positions[0])]
    for p in positions[1:-1]:
        wps.append(Waypoint(p))
    if len(positions) >= 2:
        wps.append(EndWaypoint(positions[-1]))
    return wps


def _print_progress(fraction: float) -> None:
    """Render a 40-char progress bar in place on stdout."""
    width = 40
    clamped = max(0.0, min(1.0, fraction))
    filled = int(width * clamped)
    if filled < width:
        bar = '=' * filled + '>' + ' ' * (width - filled - 1)
    else:
        bar = '=' * width
    print(f'\r[{bar}] {int(clamped * 100):3d}%', end='', flush=True)


def _print_spline(spline) -> None:
    """Pretty-print the planned spline (segment timing + per-axis coefficients)."""
    segments = list(spline.segments)
    if not segments:
        print('(empty spline)')
        return
    knots = spline.breakpoints()
    print(f'\nSpline ({len(segments)} segments, total duration {spline.duration():.4f} s):')
    print('  seg   t_start     t_end   duration')
    print('  ---  --------  --------  --------')
    for i, seg in enumerate(segments):
        print(f'  {i:>3}  {knots[i]:>8.4f}  {knots[i + 1]:>8.4f}  {seg.duration:>8.4f}')

    print('\nPolynomials  p_d(tau) = c0 + c1*tau + c2*tau^2 + ... + c_{N-1}*tau^{N-1},  '
          'tau in [0, T_seg]:')
    axis_name = ('px', 'py', 'pz')
    for i, seg in enumerate(segments):
        coeffs = np.asarray(seg.coefficients)
        print(f'  Segment {i}  (T={seg.duration:.4f} s):')
        for d in range(3):
            row = '  '.join(f'{coeffs[d, k]:>+11.4f}' for k in range(coeffs.shape[1]))
            print(f'    {axis_name[d]}:  {row}')


def _spline_csv_path(output_path: Path) -> Path:
    """Mirror of ``splineCsvPathFor()`` in run_example.cpp."""
    if output_path.suffix == '.csv':
        return output_path.with_suffix('.spline.csv')
    return output_path.with_name(output_path.name + '.spline.csv')


def main() -> int:
    """Run the Python one-shot example: load configs, generate, sample, log."""
    args = _parse_cli()
    try:
        step_dt, waypoints = _load_sim_config(args.sim_config)
        params = _load_trajectory_config(args.trajectory_config)
    except (FileNotFoundError, ValueError, yaml.YAMLError) as e:
        print(f'Configuration error: {e}', file=sys.stderr)
        return 1

    print('Configuration')
    print(f'  sim:                {args.sim_config}')
    print(f'  trajectory:         {args.trajectory_config}')
    print(f'  csv:                {args.output}')
    print(f'  step_dt:            {step_dt} s')
    print(f'  waypoints:          {len(waypoints)}')
    print(f'  max_speed:          {params["max_speed"]} m/s')
    print(f'  max_acceleration:   {params["max_acceleration"]} m/s^2')
    print(f'  max_jerk:           {params["max_jerk"]} m/s^3')
    print(f'  acceptance_radius:  {params["acceptance_radius"]} m')
    print(f'  trajectory_gain:    {params["trajectory_gain"]}')
    print()

    cfg = GeneratorConfig(
        max_speed=params['max_speed'],
        max_acceleration=params['max_acceleration'],
        max_jerk=params['max_jerk'],
        max_tracking_error=params['max_tracking_error'],
        acceptance_radius=params['acceptance_radius'],
        trajectory_gain=params['trajectory_gain'],
        step_dt=step_dt,
        max_simulation_time=params['max_simulation_time'],
        settle_velocity=params['settle_velocity'],
        polynomial_order=params['polynomial_order'],
    )

    output_path = Path(args.output)
    if output_path.parent and not output_path.parent.exists():
        output_path.parent.mkdir(parents=True, exist_ok=True)

    traj = Trajectory(cfg)
    wps = _build_waypoints(waypoints)
    if not traj.generate(wps, params['max_speed']):
        print('Trajectory generation failed (check waypoints, limits, and step_dt).',
              file=sys.stderr)
        return 1

    print(f'Trajectory generated:')
    print(f'  duration:         {traj.duration:.3f} s')
    print(f'  max velocity:     {traj.max_velocity:.3f} m/s')
    print(f'  max acceleration: {traj.max_acceleration:.3f} m/s^2')
    print()

    _print_spline(traj.spline())

    spline_path = _spline_csv_path(output_path)
    if traj.spline().save_to_csv(str(spline_path)):
        print(f'Spline written to:    {spline_path}')
    else:
        print(f'WARNING: failed to write spline CSV to {spline_path}', file=sys.stderr)

    duration = traj.duration
    n_steps = int(duration // step_dt) + 1
    progress_every = max(1, n_steps // 100)

    with output_path.open('w', encoding='utf-8') as csv:
        csv.write('t,px,py,pz,vx,vy,vz,ax,ay,az\n')
        for i in range(n_steps):
            t = min(i * step_dt, duration)
            pt = traj.evaluate(t)
            csv.write(
                f'{t:.10g},'
                f'{pt.position[0]:.10g},{pt.position[1]:.10g},{pt.position[2]:.10g},'
                f'{pt.velocity[0]:.10g},{pt.velocity[1]:.10g},{pt.velocity[2]:.10g},'
                f'{pt.acceleration[0]:.10g},{pt.acceleration[1]:.10g},{pt.acceleration[2]:.10g}\n'
            )
            if i % progress_every == 0:
                _print_progress(t / duration)
        # Final exact sample at t = duration.
        pt = traj.evaluate(duration)
        csv.write(
            f'{duration:.10g},'
            f'{pt.position[0]:.10g},{pt.position[1]:.10g},{pt.position[2]:.10g},'
            f'{pt.velocity[0]:.10g},{pt.velocity[1]:.10g},{pt.velocity[2]:.10g},'
            f'{pt.acceleration[0]:.10g},{pt.acceleration[1]:.10g},{pt.acceleration[2]:.10g}\n'
        )
    _print_progress(1.0)
    print()
    print(f'Done. CSV written to: {output_path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
