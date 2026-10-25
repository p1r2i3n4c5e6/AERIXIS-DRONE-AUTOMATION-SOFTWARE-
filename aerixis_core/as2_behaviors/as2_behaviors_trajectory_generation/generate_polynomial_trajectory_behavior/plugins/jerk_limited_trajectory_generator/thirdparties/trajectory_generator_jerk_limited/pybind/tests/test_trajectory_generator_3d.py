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

"""pytest suite for JerkLimitedTrajectoryGenerator3D bindings."""

from __future__ import annotations

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from dataclasses import dataclass

import numpy as np

from trajectory_generator_jerk_limited import JerkLimitedTrajectoryGenerator3D


@dataclass
class _Parameters:
    max_speed: float = 5.0
    max_acceleration: float = 3.0
    jerk: float = 4.0
    max_tracking_error: float = 2.0


def _make_generator(parameters: _Parameters) -> JerkLimitedTrajectoryGenerator3D:
    generator = JerkLimitedTrajectoryGenerator3D()
    generator.set_max_jerk(parameters.jerk)
    generator.set_max_speed(parameters.max_speed)
    generator.set_max_acceleration(parameters.max_acceleration)
    generator.set_max_tracking_error(parameters.max_tracking_error)
    return generator


def test_setpoints_are_numpy_3d_float64() -> None:
    parameters = _Parameters()
    generator = _make_generator(parameters)
    generator.reset(np.zeros(3), np.zeros(3), np.zeros(3))

    waypoint = np.array([1.0, 2.0, 1.0])
    setpoints = generator.generate(np.zeros(3), waypoint, np.zeros(3), 0.02)

    for field in (
        setpoints.position,
        setpoints.velocity,
        setpoints.acceleration,
        setpoints.jerk,
        setpoints.unsmoothed_velocity,
    ):
        assert isinstance(field, np.ndarray)
        assert field.shape == (3,)
        assert field.dtype == np.float64


def test_reaches_waypoint_with_zero_velocity() -> None:
    parameters = _Parameters()
    generator = _make_generator(parameters)
    generator.reset(np.zeros(3), np.zeros(3), np.zeros(3))

    waypoint = np.array([10.0, 5.0, 4.0])
    dt = 0.02
    num_iterations = 750

    vehicle_position = np.zeros(3)
    setpoints = None
    for _ in range(num_iterations):
        setpoints = generator.generate(
            vehicle_position, waypoint, np.zeros(3), dt
        )
        vehicle_position = setpoints.position

    assert setpoints is not None
    assert np.linalg.norm(setpoints.position - waypoint) < 1e-2
    assert np.linalg.norm(setpoints.velocity) < 1e-2


def test_respects_global_speed_acceleration_and_jerk_bounds() -> None:
    parameters = _Parameters()
    generator = _make_generator(parameters)
    generator.reset(np.zeros(3), np.zeros(3), np.zeros(3))

    waypoint = np.array([10.0, 5.0, 4.0])
    dt = 0.02
    num_iterations = 750

    vehicle_position = np.zeros(3)
    max_speed = 0.0
    max_acceleration = 0.0
    max_jerk = 0.0
    for _ in range(num_iterations):
        setpoints = generator.generate(
            vehicle_position, waypoint, np.zeros(3), dt
        )
        vehicle_position = setpoints.position
        max_speed = max(max_speed, float(np.linalg.norm(setpoints.velocity)))
        max_acceleration = max(
            max_acceleration, float(np.linalg.norm(setpoints.acceleration))
        )
        max_jerk = max(max_jerk, float(np.max(np.abs(setpoints.jerk))))

    assert max_speed <= parameters.max_speed + 1e-6
    assert max_acceleration <= parameters.max_acceleration + 1e-6
    assert max_jerk <= parameters.jerk + 1e-9


def test_diagonal_trajectory_respects_global_speed_bound() -> None:
    parameters = _Parameters(max_speed=2.0)
    generator = _make_generator(parameters)
    generator.reset(np.zeros(3), np.zeros(3), np.zeros(3))

    waypoint = np.array([30.0, 30.0, 30.0])
    dt = 0.02
    num_iterations = 1000

    vehicle_position = np.zeros(3)
    max_speed = 0.0
    for _ in range(num_iterations):
        setpoints = generator.generate(
            vehicle_position, waypoint, np.zeros(3), dt
        )
        vehicle_position = setpoints.position
        max_speed = max(max_speed, float(np.linalg.norm(setpoints.velocity)))

    assert max_speed <= parameters.max_speed + 1e-6


def test_force_zero_velocity_setpoint_matches_zero_unsmoothed() -> None:
    parameters = _Parameters()
    generator = _make_generator(parameters)
    generator.reset(np.zeros(3), np.zeros(3), np.zeros(3))

    setpoints = generator.generate(
        np.zeros(3), np.array([1.0, 0.0, 0.0]), np.zeros(3), 0.02,
        force_zero_velocity_setpoint=True,
    )
    assert np.allclose(setpoints.unsmoothed_velocity, np.zeros(3))


def test_zero_tracking_error_disables_time_stretch_gate() -> None:
    disabled_gate = _make_generator(_Parameters(max_tracking_error=0.0))
    enabled_gate = _make_generator(_Parameters(max_tracking_error=0.1))

    disabled_gate.reset(np.zeros(3), np.zeros(3), np.zeros(3))
    enabled_gate.reset(np.zeros(3), np.zeros(3), np.zeros(3))

    lagging_vehicle_position = np.array([-100.0, 0.0, 0.0])
    waypoint = np.array([10.0, 0.0, 0.0])
    dt = 0.02

    # First tick primes the per-axis durations.
    disabled_gate.generate(lagging_vehicle_position, waypoint, np.zeros(3), dt)
    enabled_gate.generate(lagging_vehicle_position, waypoint, np.zeros(3), dt)

    # Second tick highlights the gate effect.
    no_gate = disabled_gate.generate(lagging_vehicle_position, waypoint, np.zeros(3), dt)
    gated = enabled_gate.generate(lagging_vehicle_position, waypoint, np.zeros(3), dt)

    assert np.linalg.norm(no_gate.velocity) > 1e-6
    assert np.linalg.norm(gated.velocity) < 1e-9


def test_all_dynamic_limits_disabled_raises() -> None:
    generator = JerkLimitedTrajectoryGenerator3D()
    generator.reset(np.zeros(3), np.zeros(3), np.zeros(3))
    try:
        generator.generate(np.zeros(3), np.array([1.0, 0.0, 0.0]), np.zeros(3), 0.02)
    except ValueError:
        pass
    else:
        raise AssertionError(
            'Expected ValueError when max_speed, max_acceleration and max_jerk are all <= 0.')


# --- Multi-waypoint -----------------------------------------------------------


def test_multi_waypoint_repeated_target_matches_single_bit_for_bit() -> None:
    """Driving the multi overload with {wp, wp, wp} reproduces single byte-for-byte."""
    parameters = _Parameters()
    single = _make_generator(parameters)
    multi = _make_generator(parameters)
    multi.set_acceptance_radius(1.0)
    multi.set_trajectory_gain(1.5)
    single.reset(np.zeros(3), np.zeros(3), np.zeros(3))
    multi.reset(np.zeros(3), np.zeros(3), np.zeros(3))

    waypoint = np.array([10.0, 5.0, 4.0])
    triplet = np.tile(waypoint, (3, 1))
    ff = np.zeros(3)
    dt = 0.02

    pos_single = np.zeros(3)
    pos_multi = np.zeros(3)
    for _ in range(750):
        s = single.generate(pos_single, waypoint, ff, dt)
        m = multi.generate(pos_multi, triplet, ff, dt)
        np.testing.assert_array_equal(s.position, m.position)
        np.testing.assert_array_equal(s.velocity, m.velocity)
        np.testing.assert_array_equal(s.acceleration, m.acceleration)
        pos_single = s.position
        pos_multi = m.position


def test_multi_waypoint_accepts_3x3_ndarray_and_list_equivalently() -> None:
    parameters = _Parameters()
    gen_arr = _make_generator(parameters)
    gen_lst = _make_generator(parameters)
    gen_arr.set_acceptance_radius(1.0)
    gen_lst.set_acceptance_radius(1.0)
    gen_arr.reset(np.zeros(3), np.zeros(3), np.zeros(3))
    gen_lst.reset(np.zeros(3), np.zeros(3), np.zeros(3))

    triplet_arr = np.array([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [10.0, 10.0, 0.0]])
    triplet_lst = [
        np.array([0.0, 0.0, 0.0]),
        np.array([10.0, 0.0, 0.0]),
        np.array([10.0, 10.0, 0.0]),
    ]
    ff = np.zeros(3)
    dt = 0.02

    pos_a = np.zeros(3)
    pos_b = np.zeros(3)
    for _ in range(50):
        a = gen_arr.generate(pos_a, triplet_arr, ff, dt)
        b = gen_lst.generate(pos_b, triplet_lst, ff, dt)
        np.testing.assert_array_equal(a.position, b.position)
        pos_a = a.position
        pos_b = b.position


def test_multi_waypoint_zero_acceptance_radius_matches_single() -> None:
    """With acceptance_radius=0 the multi overload degrades to single behaviour."""
    parameters = _Parameters()
    single = _make_generator(parameters)
    multi = _make_generator(parameters)
    multi.set_acceptance_radius(0.0)
    single.reset(np.zeros(3), np.zeros(3), np.zeros(3))
    multi.reset(np.zeros(3), np.zeros(3), np.zeros(3))

    target = np.array([10.0, 0.0, 0.0])
    triplet = np.array([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [10.0, 10.0, 0.0]])
    ff = np.zeros(3)
    dt = 0.02

    pos_single = np.zeros(3)
    pos_multi = np.zeros(3)
    for _ in range(500):
        s = single.generate(pos_single, target, ff, dt)
        m = multi.generate(pos_multi, triplet, ff, dt)
        np.testing.assert_array_equal(s.position, m.position)
        pos_single = s.position
        pos_multi = m.position


def test_multi_waypoint_invalid_shape_raises() -> None:
    parameters = _Parameters()
    generator = _make_generator(parameters)
    generator.set_acceptance_radius(1.0)
    generator.reset(np.zeros(3), np.zeros(3), np.zeros(3))

    bad = np.zeros((4, 3))  # wrong number of waypoints
    try:
        generator.generate(np.zeros(3), bad, np.zeros(3), 0.02)
    except ValueError:
        pass
    else:
        raise AssertionError('Expected ValueError for waypoints shape != (3, 3).')
