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

"""pytest suite for TrajectoryGenerator binding."""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from gcopterpy import (
    DroneLimits,
    DroneParameters,
    GeneratorConfig,
    OptimizationConfig,
    Spline,
    TrajectoryGenerator,
    TrajectorySample,
    Waypoint,
)

import numpy as np

_REF_MAX_SPEED = 4.0


def _make_generator() -> TrajectoryGenerator:
    dp = DroneParameters()
    dp.mass = 0.61
    dp.gravity = 9.8
    dp.horizontal_drag = 0.70
    dp.vertical_drag = 0.80
    dp.parasitic_drag = 0.01
    dp.speed_smooth_factor = 1.0e-4

    dl = DroneLimits()
    dl.max_velocity = _REF_MAX_SPEED
    dl.max_body_rate = 2.1
    dl.max_tilt_angle = 1.05
    dl.min_thrust = 2.0
    dl.max_thrust = 12.0

    oc = OptimizationConfig()
    oc.time_weight = 20.0
    oc.position_weight = 1.0e4
    oc.velocity_weight = 1.0e4
    oc.body_rate_weight = 1.0e4
    oc.tilt_weight = 1.0e4
    oc.thrust_weight = 1.0e5
    oc.smoothing_eps = 1.0e-2
    oc.integral_resolution = 16
    oc.rel_cost_tol = 1.0e-5
    oc.corridor_margin = 2.0

    cfg = GeneratorConfig()
    cfg.params = dp
    cfg.limits = dl
    cfg.optimization = oc
    return TrajectoryGenerator(cfg)


def _simple_waypoints() -> list:
    return [
        Waypoint(np.array([0.0, 0.0, 1.0])),
        Waypoint(np.array([2.0, 0.0, 1.0])),
        Waypoint(np.array([2.0, 2.0, 1.0])),
    ]


def test_generate_returns_true_and_makes_generator_valid() -> None:
    gen = _make_generator()
    assert not gen.is_valid()
    assert gen.generate(_simple_waypoints(), _REF_MAX_SPEED)
    assert gen.is_valid()
    assert gen.duration() > 0.0
    assert gen.min_time() == 0.0
    assert gen.max_time() == gen.duration()


def test_trajectory_endpoints_match_waypoints() -> None:
    gen = _make_generator()
    waypoints = _simple_waypoints()
    assert gen.generate(waypoints, _REF_MAX_SPEED)

    p0 = gen.position(0.0)
    pT = gen.position(gen.duration())
    np.testing.assert_allclose(p0, waypoints[0].position, atol=1e-6)
    np.testing.assert_allclose(pT, waypoints[-1].position, atol=1e-6)


def test_trajectory_respects_max_velocity() -> None:
    gen = _make_generator()
    assert gen.generate(_simple_waypoints(), _REF_MAX_SPEED)
    assert gen.max_velocity() <= _REF_MAX_SPEED + 1e-3


def test_max_speed_argument_overrides_drone_limit() -> None:
    gen = _make_generator()
    v_call = 2.0
    assert gen.generate(_simple_waypoints(), v_call)
    assert gen.max_velocity() <= v_call * 1.05


def test_evaluate_returns_trajectory_sample() -> None:
    gen = _make_generator()
    assert gen.generate(_simple_waypoints(), _REF_MAX_SPEED)
    s = gen.evaluate(0.5 * gen.duration())
    assert isinstance(s, TrajectorySample)
    assert s.position.shape == (3,)
    assert s.velocity.shape == (3,)
    assert s.acceleration.shape == (3,)


def test_named_accessors_match_evaluate() -> None:
    gen = _make_generator()
    assert gen.generate(_simple_waypoints(), _REF_MAX_SPEED)
    t = 0.5 * gen.duration()
    s = gen.evaluate(t)
    np.testing.assert_allclose(s.position, gen.position(t), atol=1e-12)
    np.testing.assert_allclose(s.velocity, gen.velocity(t), atol=1e-12)
    np.testing.assert_allclose(s.acceleration, gen.acceleration(t), atol=1e-12)


def test_samples_return_3d_arrays() -> None:
    gen = _make_generator()
    assert gen.generate(_simple_waypoints(), _REF_MAX_SPEED)
    t = 0.5 * gen.duration()
    for fn in (gen.position, gen.velocity, gen.acceleration, gen.jerk):
        value = fn(t)
        assert value.shape == (3,)
        assert value.dtype == np.float64


def test_evaluate_clamps_out_of_range() -> None:
    gen = _make_generator()
    assert gen.generate(_simple_waypoints(), _REF_MAX_SPEED)
    below = gen.evaluate(-1.0).position
    start = gen.evaluate(0.0).position
    above = gen.evaluate(gen.duration() + 5.0).position
    end = gen.evaluate(gen.duration()).position
    np.testing.assert_allclose(below, start, atol=1e-9)
    np.testing.assert_allclose(above, end, atol=1e-9)


def test_spline_accessor_returns_segments() -> None:
    gen = _make_generator()
    assert gen.generate(_simple_waypoints(), _REF_MAX_SPEED)
    sp = gen.spline()
    assert isinstance(sp, Spline)
    assert len(sp.segments) >= 1
    assert sp.duration() > 0.0
    assert abs(sp.duration() - gen.duration()) < 1e-9
    # Polynomial coefficients are 3 axes × 6 coeffs (quintic) for gcopter.
    assert sp.segments[0].coefficients.shape == (3, 6)


def test_generate_with_per_segment_margins() -> None:
    gen = _make_generator()
    waypoints = _simple_waypoints()
    margins = [2.0, 2.0]
    assert gen.generate(waypoints, _REF_MAX_SPEED, margins)
    assert gen.is_valid()


def test_generate_with_zero_max_speed_fails() -> None:
    gen = _make_generator()
    assert not gen.generate(_simple_waypoints(), 0.0)
    assert not gen.is_valid()
