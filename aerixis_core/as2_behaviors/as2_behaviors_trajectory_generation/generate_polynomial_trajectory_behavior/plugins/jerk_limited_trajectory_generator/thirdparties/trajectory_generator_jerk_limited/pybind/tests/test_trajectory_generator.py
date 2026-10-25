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

"""pytest suite for the one-shot Trajectory wrapper and the unified types."""

from __future__ import annotations

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import os
import tempfile

import numpy as np
import pytest

from trajectory_generator_jerk_limited import (
    EndWaypoint,
    GeneratorConfig,
    Spline,
    Trajectory,
)


def _default_cfg() -> GeneratorConfig:
    return GeneratorConfig(
        max_speed=2.0,
        max_acceleration=1.0,
        max_jerk=5.0,
        max_tracking_error=1.0,
        acceptance_radius=0.5,
        trajectory_gain=1.0,
        step_dt=0.02,
        max_simulation_time=60.0,
        settle_velocity=0.05,
        polynomial_order=16,
    )


def test_invalid_until_generated() -> None:
    traj = Trajectory(_default_cfg())
    assert not traj.is_valid


def test_two_waypoint_straight_line() -> None:
    traj = Trajectory(_default_cfg())
    ok = traj.generate(
        [EndWaypoint(np.array([0.0, 0.0, 0.0])),
         EndWaypoint(np.array([1.0, 0.0, 0.0]))],
        max_speed=1.0,
    )
    assert ok
    assert traj.is_valid
    assert traj.duration > 0.0
    assert traj.max_velocity <= 1.0 + 1e-3
    assert traj.max_acceleration <= 1.0 + 1e-3
    pt = traj.evaluate(0.0)
    np.testing.assert_allclose(pt.position, [0.0, 0.0, 0.0], atol=1e-9)
    np.testing.assert_allclose(pt.velocity, [0.0, 0.0, 0.0], atol=1e-9)


def test_multi_waypoint_segments_match_pairs() -> None:
    traj = Trajectory(_default_cfg())
    wps = [
        EndWaypoint(np.array([0.0, 0.0, 0.0])),
        np.array([2.0, 0.0, 0.0]),
        np.array([2.0, 2.0, 0.0]),
        EndWaypoint(np.array([0.0, 2.0, 0.0])),
    ]
    assert traj.generate(wps, max_speed=1.0)
    sp = traj.spline()
    assert len(sp.segments) == len(wps) - 1
    knots = np.asarray(sp.breakpoints())
    assert knots.size == len(wps)
    assert np.all(np.diff(knots) >= 0.0)


def test_generate_rejects_short_input() -> None:
    traj = Trajectory(_default_cfg())
    with pytest.raises(ValueError):
        traj.generate([EndWaypoint(np.zeros(3))], max_speed=1.0)


def test_generate_rejects_non_positive_speed() -> None:
    traj = Trajectory(_default_cfg())
    with pytest.raises(ValueError):
        traj.generate([EndWaypoint(np.zeros(3)),
                       EndWaypoint(np.array([1.0, 0.0, 0.0]))], max_speed=0.0)


def test_spline_csv_roundtrip() -> None:
    traj = Trajectory(_default_cfg())
    assert traj.generate(
        [EndWaypoint(np.array([0.0, 0.0, 0.0])),
         EndWaypoint(np.array([1.0, 0.0, 0.0]))],
        max_speed=1.0,
    )
    sp = traj.spline()
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, 'traj.spline.csv')
        assert sp.save_to_csv(path)
        sp2 = Spline()
        assert sp2.load_from_csv(path)
        assert len(sp2.segments) == len(sp.segments)
        np.testing.assert_allclose(sp2.breakpoints(), sp.breakpoints(), atol=1e-12)


def test_evaluate_clamps_outside_range() -> None:
    traj = Trajectory(_default_cfg())
    assert traj.generate(
        [EndWaypoint(np.array([0.0, 0.0, 0.0])),
         EndWaypoint(np.array([0.5, 0.0, 0.0]))],
        max_speed=0.5,
    )
    p_before = traj.evaluate(-1.0).position
    p_start = traj.evaluate(traj.min_time).position
    np.testing.assert_allclose(p_before, p_start, atol=1e-9)
    p_after = traj.evaluate(traj.max_time + 100.0).position
    p_end = traj.evaluate(traj.max_time).position
    np.testing.assert_allclose(p_after, p_end, atol=1e-9)


# --- Initial state from first waypoint --------------------------------------


def test_generate_honours_first_waypoint_velocity() -> None:
    """First waypoint's velocity pins the integrator initial state."""
    from trajectory_generator_jerk_limited import Waypoint
    traj = Trajectory(_default_cfg())
    start = Waypoint(np.zeros(3))
    start.velocity = np.array([0.5, 0.0, 0.0])
    end = EndWaypoint(np.array([5.0, 0.0, 0.0]))
    assert traj.generate([start, end], max_speed=1.0)
    sample = traj.evaluate(traj.min_time)
    np.testing.assert_allclose(sample.velocity, [0.5, 0.0, 0.0], atol=1e-12)


def test_generate_honours_first_waypoint_acceleration() -> None:
    """First waypoint's acceleration is recorded in the first sample."""
    from trajectory_generator_jerk_limited import Waypoint
    traj = Trajectory(_default_cfg())
    start = Waypoint(np.zeros(3))
    start.acceleration = np.array([0.3, 0.0, 0.0])
    end = EndWaypoint(np.array([2.0, 0.0, 0.0]))
    assert traj.generate([start, end], max_speed=1.0)
    sample = traj.evaluate(traj.min_time)
    np.testing.assert_allclose(sample.acceleration, [0.3, 0.0, 0.0], atol=1e-12)


def test_generate_without_initial_state_matches_legacy() -> None:
    """A position-only first Waypoint yields the previous start-from-rest behaviour."""
    from trajectory_generator_jerk_limited import Waypoint
    traj = Trajectory(_default_cfg())
    start = Waypoint(np.zeros(3))  # no velocity / acceleration -> default zero
    end = EndWaypoint(np.array([2.0, 1.0, 0.5]))
    assert traj.generate([start, end], max_speed=1.0)
    sample = traj.evaluate(traj.min_time)
    np.testing.assert_allclose(sample.velocity, np.zeros(3), atol=1e-12)
    np.testing.assert_allclose(sample.acceleration, np.zeros(3), atol=1e-12)
