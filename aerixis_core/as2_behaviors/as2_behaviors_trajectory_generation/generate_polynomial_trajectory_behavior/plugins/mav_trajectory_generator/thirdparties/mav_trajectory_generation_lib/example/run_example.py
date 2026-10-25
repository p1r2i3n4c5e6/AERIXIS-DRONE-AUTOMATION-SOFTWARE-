#!/usr/bin/env python3

# Copyright 2026 Universidad Politécnica de Madrid
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

r"""
Python equivalent of example/run_example.cpp.

Loads ``config_example.yaml`` (sim_config: step_dt + waypoints) and
``config_trajectory.yaml`` (optimization: solver knobs), generates a single
multi-waypoint polynomial trajectory, samples it at fixed ``step_dt`` and
writes a 10-column CSV plus a sibling ``<base>.spline.csv``.

Usage (from the repo root, after cmake --build build with BUILD_PYBIND=ON):
    ./example/run_example_py.sh
    python3 example/run_example.py \\
        -c example/config_example.yaml \\
        -d example/config_trajectory.yaml \\
        -f example/trajectory_logs/trajectory_py.csv
"""

from __future__ import annotations

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2026 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import argparse
import sys
from pathlib import Path
from typing import List, Tuple

import numpy as np
import yaml

from mav_trajectory_generation_py import (
    EndWaypoint,
    GeneratorConfig,
    OptimizationConfig,
    Solver,
    TrajectoryGenerator,
    Waypoint,
)


def _parse_cli() -> argparse.Namespace:
    """Parse command-line arguments mirroring the C++ example flags."""
    parser = argparse.ArgumentParser(
        description='One-shot multi-waypoint mav_trajectory_generation_cpp example.')
    parser.add_argument('-c', '--sim-config', required=True, dest='sim_config',
                        help='Path to the simulation YAML.')
    parser.add_argument('-d', '--trajectory-config', required=True, dest='trajectory_config',
                        help='Path to the optimisation YAML.')
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
    step_dt = float(sim.get('step_dt', 0.01))
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


def _solver_from_string(s: str) -> Solver:
    """Convert a YAML solver string to the native Solver enum."""
    s = s.lower()
    if s == 'linear':
        return Solver.linear
    if s == 'nonlinear':
        return Solver.nonlinear
    raise ValueError(f"Unknown solver '{s}'. Expected 'linear' or 'nonlinear'.")


def _load_trajectory_config(path: str) -> GeneratorConfig:
    """Load ``optimization`` block as a GeneratorConfig."""
    with open(path, encoding='utf-8') as handle:
        root = yaml.safe_load(handle)
    if root is None or 'optimization' not in root:
        raise ValueError(f"{path}: missing 'optimization' section.")
    block = root['optimization']
    opt = OptimizationConfig()
    if 'derivative_to_optimize' in block:
        opt.derivative_to_optimize = int(block['derivative_to_optimize'])
    if 'solver' in block:
        opt.solver = _solver_from_string(str(block['solver']))
    if 'a_max' in block:
        opt.a_max = float(block['a_max'])
    if 'nl_max_iterations' in block:
        opt.nl_max_iterations = int(block['nl_max_iterations'])
    if 'nl_f_rel' in block:
        opt.nl_f_rel = float(block['nl_f_rel'])
    if 'nl_x_rel' in block:
        opt.nl_x_rel = float(block['nl_x_rel'])
    if 'nl_time_penalty' in block:
        opt.nl_time_penalty = float(block['nl_time_penalty'])
    if 'nl_initial_stepsize_rel' in block:
        opt.nl_initial_stepsize_rel = float(block['nl_initial_stepsize_rel'])
    if 'nl_inequality_constraint_tolerance' in block:
        opt.nl_inequality_constraint_tolerance = float(
            block['nl_inequality_constraint_tolerance'])
    cfg = GeneratorConfig()
    cfg.optimization = opt
    return cfg


def _build_waypoints(positions: List[np.ndarray]) -> List[Waypoint]:
    """
    Compose the waypoint list.

    positions[0] is the start pinned to rest; the last is also pinned to
    rest; intermediates only constrain position.
    """
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
        opt_cfg = _load_trajectory_config(args.trajectory_config)
    except (FileNotFoundError, ValueError, yaml.YAMLError) as e:
        print(f'Configuration error: {e}', file=sys.stderr)
        return 1

    # Default cruise speed: pick a conservative value tied to the configured
    # a_max (the only kinematic bound exposed by mav_trajectory_generation_cpp).
    max_speed = max(0.1, opt_cfg.optimization.a_max)

    print('Configuration')
    print(f'  sim:                {args.sim_config}')
    print(f'  trajectory:         {args.trajectory_config}')
    print(f'  csv:                {args.output}')
    print(f'  step_dt:            {step_dt} s')
    print(f'  waypoints:          {len(waypoints)}')
    print(f'  max_speed:          {max_speed} m/s')
    print(f'  a_max:              {opt_cfg.optimization.a_max} m/s^2')
    print(f'  solver:             {"linear" if opt_cfg.optimization.solver == Solver.linear else "nonlinear"}')
    print()

    output_path = Path(args.output)
    if output_path.parent and not output_path.parent.exists():
        output_path.parent.mkdir(parents=True, exist_ok=True)

    generator = TrajectoryGenerator(opt_cfg)
    wps = _build_waypoints(waypoints)
    if not generator.generate(wps, max_speed):
        print('Trajectory generation failed (check waypoints and solver settings).',
              file=sys.stderr)
        return 1

    duration = generator.duration()
    print(f'Trajectory generated:')
    print(f'  duration:         {duration:.3f} s')
    print(f'  max velocity:     {generator.max_velocity():.3f} m/s')
    print(f'  max acceleration: {generator.max_acceleration():.3f} m/s^2')
    print()

    _print_spline(generator.spline())

    spline_path = _spline_csv_path(output_path)
    if generator.spline().save_to_csv(str(spline_path)):
        print(f'Spline written to:    {spline_path}')
    else:
        print(f'WARNING: failed to write spline CSV to {spline_path}', file=sys.stderr)

    n_steps = int(duration // step_dt) + 1
    progress_every = max(1, n_steps // 100)

    with output_path.open('w', encoding='utf-8') as csv:
        csv.write('t,px,py,pz,vx,vy,vz,ax,ay,az\n')
        for i in range(n_steps):
            t = min(i * step_dt, duration)
            s = generator.evaluate(t)
            csv.write(
                f'{t:.10g},'
                f'{s.position[0]:.10g},{s.position[1]:.10g},{s.position[2]:.10g},'
                f'{s.velocity[0]:.10g},{s.velocity[1]:.10g},{s.velocity[2]:.10g},'
                f'{s.acceleration[0]:.10g},{s.acceleration[1]:.10g},{s.acceleration[2]:.10g}\n'
            )
            if i % progress_every == 0:
                _print_progress(t / duration)
        # Final exact sample at t = duration.
        s = generator.evaluate(duration)
        csv.write(
            f'{duration:.10g},'
            f'{s.position[0]:.10g},{s.position[1]:.10g},{s.position[2]:.10g},'
            f'{s.velocity[0]:.10g},{s.velocity[1]:.10g},{s.velocity[2]:.10g},'
            f'{s.acceleration[0]:.10g},{s.acceleration[1]:.10g},{s.acceleration[2]:.10g}\n'
        )
    _print_progress(1.0)
    print()
    print(f'Done. CSV written to: {output_path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
