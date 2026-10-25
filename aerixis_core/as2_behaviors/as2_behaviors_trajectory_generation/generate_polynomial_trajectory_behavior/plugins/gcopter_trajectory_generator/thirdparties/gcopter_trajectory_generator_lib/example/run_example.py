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

"""
Python equivalent of example/run_example.cpp.

Reads config_example.yaml and config_trajectory.yaml, generates a polynomial
trajectory with GCOPTER via the high-level ``Trajectory`` wrapper, samples it
at a fixed step and writes a CSV.

Usage (from repo root, after cmake --build build with BUILD_PYBIND=ON):
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

from gcopterpy import (
    Trajectory,
    TrajectoryLogger,
    load_sim_config,
    load_trajectory_config,
)


def _print_spline(spline) -> None:
    """
    Pretty-print a Spline: segment timing table + per-axis polynomial coefficients.

    :param spline: Snapshot returned by ``Trajectory.spline()``.
    :type spline: gcopterpy.Spline
    :rtype: None
    """
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

    print(
        '\nPolynomials  p_d(tau) = c0 + c1*tau + c2*tau^2 + ... + c_{N-1}*tau^{N-1}, '
        'tau in [0, T_seg]:'
    )
    axis_name = ('px', 'py', 'pz')
    for i, seg in enumerate(segments):
        coeffs = seg.coefficients
        print(f'  Segment {i}  (T={seg.duration:.4f} s):')
        for d in range(3):
            row = '  '.join(f'{coeffs[d, k]:>+11.4f}' for k in range(coeffs.shape[1]))
            print(f'    {axis_name[d]}:  {row}')


def _spline_csv_path(output_path: Path) -> Path:
    """
    Derive the spline CSV path from the sample CSV path.

    Replaces a trailing ``.csv`` with ``.spline.csv``; otherwise appends
    ``.spline.csv``. Mirrors ``splineCsvPathFor()`` in ``run_example.cpp``.

    :param output_path: Path of the sample CSV (``trajectory.csv``).
    :type output_path: Path
    :return: Sibling path for the spline CSV (``trajectory.spline.csv``).
    :rtype: Path
    """
    if output_path.suffix == '.csv':
        return output_path.with_suffix('.spline.csv')
    return output_path.with_name(output_path.name + '.spline.csv')


def _print_progress(fraction: float) -> None:
    """
    Render a 40-char progress bar in place on stdout.

    :param fraction: Progress in [0, 1]; clamped if outside the range.
    :type fraction: float
    :rtype: None
    """
    width = 40
    clamped = max(0.0, min(1.0, fraction))
    filled = int(width * clamped)
    if filled < width:
        bar = '=' * filled + '>' + ' ' * (width - filled - 1)
    else:
        bar = '=' * width
    print(f'\r[{bar}] {int(clamped * 100):3d}%', end='', flush=True)


def main() -> int:
    """
    Run the Python example: load configs, generate, sample, log.

    :return: Process exit code (0 on success).
    :rtype: int
    """
    repo_dir = Path(__file__).resolve().parent.parent

    parser = argparse.ArgumentParser(
        description='Generate a gcopter_lib trajectory (Python example)'
    )
    parser.add_argument(
        '-c', '--sim-config',
        default=str(repo_dir / 'example' / 'config_example.yaml'),
    )
    parser.add_argument(
        '-d', '--trajectory-config',
        default=str(repo_dir / 'example' / 'config_trajectory.yaml'),
    )
    parser.add_argument(
        '-f', '--output',
        default=str(repo_dir / 'example' / 'trajectory_logs' / 'trajectory_py.csv'),
    )
    args = parser.parse_args()

    sim = load_sim_config(args.sim_config)
    traj_cfg = load_trajectory_config(args.trajectory_config)

    print(f'Sim config:        {args.sim_config}')
    print(f'Trajectory config: {args.trajectory_config}')
    print(f'Output:            {args.output}')
    print(f'Waypoints:         {len(sim.waypoints)}')

    print('Generating trajectory...')
    traj = Trajectory(traj_cfg)
    try:
        traj.generate(sim.waypoints)
    except RuntimeError as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1

    print(f'  Duration:         {traj.duration:.3f} s')
    print(f'  Max velocity:     {traj.max_velocity:.3f} m/s')
    print(f'  Max acceleration: {traj.max_acceleration:.3f} m/s^2')

    # Dump the planned spline (segment timing + per-axis polynomial coefficients).
    _print_spline(traj.spline())

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    spline_path = _spline_csv_path(output_path)
    if traj.spline().save_to_csv(str(spline_path)):
        print(f'Spline written to:    {spline_path}')
    else:
        print(f'WARNING: failed to write spline CSV to {spline_path}', file=sys.stderr)

    logger = TrajectoryLogger(str(output_path))
    step_dt = sim.step_dt
    n_steps = int(traj.duration // step_dt) + 1
    report_every = max(1, n_steps // 100)

    print('Sampling trajectory...')
    for i in range(n_steps):
        pt = traj.evaluate(i * step_dt)
        logger.save(pt.time, pt.position, pt.velocity, pt.acceleration)
        if i % report_every == 0:
            _print_progress(pt.time / traj.duration)

    pt = traj.evaluate(traj.duration)
    logger.save(pt.time, pt.position, pt.velocity, pt.acceleration)
    _print_progress(1.0)
    logger.close()
    print(f'\nDone. CSV at: {output_path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
