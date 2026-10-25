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

"""pytest suite for the high-level Trajectory wrapper."""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from gcopterpy import (
    CorridorConfig,
    DroneLimits,
    DroneParameters,
    OptimizationConfig,
    Trajectory,
    TrajectoryConfig,
    TrajectoryPoint,
)

import numpy as np
import pytest


def _make_config(
        corridor_margin: float = 2.0,
        waypoint_margin: float = 0.0,
        anchor_radius: float = 0.0) -> TrajectoryConfig:
    corridor = CorridorConfig(
        corridor_margin=corridor_margin,
        waypoint_margin=waypoint_margin,
        waypoint_anchor_radius=anchor_radius,
    )
    params = DroneParameters()
    params.mass = 0.61
    params.gravity = 9.8
    params.horizontal_drag = 0.70
    params.vertical_drag = 0.80
    params.parasitic_drag = 0.01
    params.speed_smooth_factor = 1.0e-4

    limits = DroneLimits()
    limits.max_velocity = 4.0
    limits.max_body_rate = 2.1
    limits.max_tilt_angle = 1.05
    limits.min_thrust = 2.0
    limits.max_thrust = 12.0

    opt = OptimizationConfig()
    opt.time_weight = 20.0
    opt.position_weight = 1.0e4
    opt.velocity_weight = 1.0e4
    opt.body_rate_weight = 1.0e4
    opt.tilt_weight = 1.0e4
    opt.thrust_weight = 1.0e5
    opt.smoothing_eps = 1.0e-2
    opt.integral_resolution = 16
    opt.rel_cost_tol = 1.0e-5

    return TrajectoryConfig(
        corridor=corridor, params=params, limits=limits, optimization=opt
    )


_POSITIONS = [
    np.array([0.0, 0.0, 1.0]),
    np.array([2.0, 0.0, 1.0]),
    np.array([2.0, 2.0, 1.0]),
]


def test_initial_state_is_invalid() -> None:
    traj = Trajectory(_make_config())
    assert not traj.is_valid
    assert traj.duration == 0.0
    assert traj.max_velocity == 0.0
    assert traj.max_acceleration == 0.0


def test_generate_raises_on_fewer_than_two_positions() -> None:
    traj = Trajectory(_make_config())
    with pytest.raises(ValueError):
        traj.generate([np.zeros(3)])


def test_generate_succeeds_and_sets_valid() -> None:
    traj = Trajectory(_make_config())
    traj.generate(_POSITIONS)
    assert traj.is_valid
    assert traj.duration > 0.0
    assert traj.max_velocity > 0.0
    assert traj.max_acceleration > 0.0


def test_evaluate_before_generate_raises() -> None:
    traj = Trajectory(_make_config())
    with pytest.raises(RuntimeError):
        traj.evaluate(0.0)


def test_evaluate_returns_trajectory_point() -> None:
    traj = Trajectory(_make_config())
    traj.generate(_POSITIONS)
    pt = traj.evaluate(0.5 * traj.duration)
    assert isinstance(pt, TrajectoryPoint)
    for vec in (pt.position, pt.velocity, pt.acceleration, pt.jerk):
        assert vec.shape == (3,)
        assert vec.dtype == np.float64


def test_evaluate_matches_endpoints() -> None:
    traj = Trajectory(_make_config())
    traj.generate(_POSITIONS)
    p0 = traj.evaluate(0.0).position
    pT = traj.evaluate(traj.duration).position
    np.testing.assert_allclose(p0, _POSITIONS[0], atol=1e-6)
    np.testing.assert_allclose(pT, _POSITIONS[-1], atol=1e-6)


def test_evaluate_clamps_time_below_zero() -> None:
    traj = Trajectory(_make_config())
    traj.generate(_POSITIONS)
    pt = traj.evaluate(-1.0)
    assert pt.time == pytest.approx(0.0)


def test_evaluate_clamps_time_above_duration() -> None:
    traj = Trajectory(_make_config())
    traj.generate(_POSITIONS)
    pt = traj.evaluate(traj.duration + 100.0)
    assert pt.time == pytest.approx(traj.duration)


def test_generate_uses_config_defaults() -> None:
    cfg = _make_config(corridor_margin=3.0)
    traj = Trajectory(cfg)
    traj.generate(_POSITIONS)
    assert traj.is_valid


def test_generate_override_corridor_margin() -> None:
    traj = Trajectory(_make_config())
    traj.generate(_POSITIONS, corridor_margin=1.0)
    assert traj.is_valid


def test_anchor_expansion_three_waypoints() -> None:
    cfg = _make_config(
        corridor_margin=2.0, waypoint_margin=0.5, anchor_radius=0.3
    )
    traj = Trajectory(cfg)
    traj.generate(_POSITIONS)
    assert traj.is_valid


def test_accepts_plain_python_lists_as_positions() -> None:
    traj = Trajectory(_make_config())
    positions = [[0.0, 0.0, 1.0], [2.0, 0.0, 1.0], [2.0, 2.0, 1.0]]
    traj.generate(positions)
    assert traj.is_valid
