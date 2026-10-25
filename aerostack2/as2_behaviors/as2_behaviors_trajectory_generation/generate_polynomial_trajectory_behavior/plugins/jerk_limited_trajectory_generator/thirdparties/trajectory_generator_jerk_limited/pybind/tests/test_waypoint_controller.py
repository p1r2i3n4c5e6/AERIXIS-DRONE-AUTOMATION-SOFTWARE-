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

"""pytest suite for WaypointTrajectoryController bindings."""

from __future__ import annotations

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import numpy as np

from trajectory_generator_jerk_limited import (
    TrajectoryParameters,
    WaypointTrajectoryController,
)


def test_default_parameters_are_sane() -> None:
    parameters = TrajectoryParameters()
    assert parameters.max_speed == 0.0
    assert parameters.max_acceleration == 0.0
    assert parameters.max_jerk == 0.0
    assert parameters.max_tracking_error == 0.0


def test_reaches_waypoint() -> None:
    parameters = TrajectoryParameters()
    parameters.max_speed = 5.0
    parameters.max_acceleration = 3.0
    parameters.max_jerk = 4.0
    parameters.max_tracking_error = 2.0
    controller = WaypointTrajectoryController(parameters)

    initial_position = np.array([0.0, 0.0, 0.0])
    waypoint = np.array([10.0, 5.0, 4.0])

    controller.reset(initial_position)
    vehicle_position = initial_position.copy()

    dt = 0.02
    num_iterations = 750

    setpoints = None
    for _ in range(num_iterations):
        setpoints = controller.update(dt, vehicle_position, waypoint)
        vehicle_position = setpoints.position

    assert setpoints is not None
    position_error = np.linalg.norm(setpoints.position - waypoint)
    velocity_norm = np.linalg.norm(setpoints.velocity)
    assert position_error < 1e-2
    assert velocity_norm < 1e-2


def test_all_dynamic_limits_disabled_raises() -> None:
    parameters = TrajectoryParameters()
    try:
        WaypointTrajectoryController(parameters)
    except ValueError:
        pass
    else:
        raise AssertionError(
            'Expected ValueError when max_speed, max_acceleration and max_jerk are all <= 0.')


def test_global_bounds_are_respected_when_climbing() -> None:
    parameters = TrajectoryParameters()
    parameters.max_speed = 2.5
    parameters.max_acceleration = 1.8

    controller = WaypointTrajectoryController(parameters)
    initial_position = np.array([0.0, 0.0, 0.0])
    waypoint = np.array([0.0, 0.0, 30.0])

    controller.reset(initial_position)
    vehicle_position = initial_position.copy()

    max_speed = 0.0
    max_acceleration = 0.0
    for _ in range(1500):
        setpoints = controller.update(0.02, vehicle_position, waypoint)
        vehicle_position = setpoints.position
        max_speed = max(max_speed, float(np.linalg.norm(setpoints.velocity)))
        max_acceleration = max(
            max_acceleration, float(np.linalg.norm(setpoints.acceleration))
        )

    assert max_speed <= parameters.max_speed + 1e-6
    assert max_acceleration <= parameters.max_acceleration + 1e-6
    assert max_speed > 0.5 * parameters.max_speed


def test_global_bounds_are_respected_when_descending() -> None:
    parameters = TrajectoryParameters()
    parameters.max_speed = 1.2
    parameters.max_acceleration = 1.0

    controller = WaypointTrajectoryController(parameters)
    initial_position = np.array([0.0, 0.0, 20.0])
    waypoint = np.array([0.0, 0.0, -10.0])

    controller.reset(initial_position)
    vehicle_position = initial_position.copy()

    max_speed = 0.0
    max_acceleration = 0.0
    for _ in range(2500):
        setpoints = controller.update(0.02, vehicle_position, waypoint)
        vehicle_position = setpoints.position
        max_speed = max(max_speed, float(np.linalg.norm(setpoints.velocity)))
        max_acceleration = max(
            max_acceleration, float(np.linalg.norm(setpoints.acceleration))
        )

    assert max_speed <= parameters.max_speed + 1e-6
    assert max_acceleration <= parameters.max_acceleration + 1e-6
    assert max_speed > 0.5 * parameters.max_speed


# --- Multi-waypoint -----------------------------------------------------------


def test_multi_waypoint_parameters_constructor_kwargs() -> None:
    parameters = TrajectoryParameters(
        max_speed=4.0,
        max_acceleration=2.0,
        max_jerk=8.0,
        acceptance_radius=1.5,
        trajectory_gain=0.75,
    )
    assert parameters.acceptance_radius == 1.5
    assert parameters.trajectory_gain == 0.75


def test_multi_waypoint_zero_acceptance_radius_matches_single() -> None:
    parameters = TrajectoryParameters(
        max_speed=4.0,
        max_acceleration=2.0,
        max_jerk=8.0,
        acceptance_radius=0.0,
    )
    single = WaypointTrajectoryController(parameters)
    multi = WaypointTrajectoryController(parameters)
    initial = np.zeros(3)
    single.reset(initial)
    multi.reset(initial)

    target = np.array([10.0, 0.0, 0.0])
    triplet = np.array([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [10.0, 10.0, 0.0]])
    dt = 0.02

    pos_single = initial.copy()
    pos_multi = initial.copy()
    for _ in range(500):
        s = single.update(dt, pos_single, target)
        m = multi.update(dt, pos_multi, triplet)
        np.testing.assert_array_equal(s.position, m.position)
        pos_single = s.position
        pos_multi = m.position


def test_multi_waypoint_with_external_rotation_covers_full_path() -> None:
    parameters = TrajectoryParameters(
        max_speed=4.0,
        max_acceleration=2.0,
        max_jerk=8.0,
        acceptance_radius=1.0,
        trajectory_gain=1.0,
    )
    controller = WaypointTrajectoryController(parameters)
    initial = np.zeros(3)
    controller.reset(initial)

    wp1 = np.array([10.0, 0.0, 0.0])
    wp2 = np.array([10.0, 10.0, 0.0])
    triplet = [initial, wp1, wp2]
    rotated = False
    dt = 0.02

    position = initial.copy()
    for _ in range(600):
        position = controller.update(dt, position, triplet).position
        if not rotated and float(np.linalg.norm(position - wp1)) < 3.0:
            triplet = [wp1, wp2, wp2]
            rotated = True
    assert rotated
    assert float(np.linalg.norm(position - wp2)) < 0.5


# --- Initial-state injection -------------------------------------------------


def test_reset_without_initial_state_matches_legacy() -> None:
    """The new reset(p, vel=None, accel=None) must behave like reset(p)."""
    parameters = TrajectoryParameters(max_speed=5.0, max_acceleration=3.0,
                                      max_jerk=4.0, max_tracking_error=2.0)
    legacy = WaypointTrajectoryController(parameters)
    extended = WaypointTrajectoryController(parameters)
    initial = np.zeros(3)
    legacy.reset(initial)
    extended.reset(initial)  # default kwargs hit the legacy path

    target = np.array([10.0, 5.0, 4.0])
    pos_legacy = initial.copy()
    pos_extended = initial.copy()
    for _ in range(500):
        leg = legacy.update(0.02, pos_legacy, target)
        ext = extended.update(0.02, pos_extended, target)
        np.testing.assert_array_equal(leg.position, ext.position)
        np.testing.assert_array_equal(leg.velocity, ext.velocity)
        pos_legacy = leg.position
        pos_extended = ext.position


def test_reset_with_explicit_zero_velocity_matches_legacy() -> None:
    """Explicit zero initial_velocity hits the extended path bit-for-bit equal to legacy."""
    parameters = TrajectoryParameters(max_speed=5.0, max_acceleration=3.0,
                                      max_jerk=4.0, max_tracking_error=2.0)
    legacy = WaypointTrajectoryController(parameters)
    extended = WaypointTrajectoryController(parameters)
    initial = np.zeros(3)
    legacy.reset(initial)
    extended.reset(initial, initial_velocity=np.zeros(3),
                   initial_acceleration=np.zeros(3))

    target = np.array([8.0, 0.0, 0.0])
    pos_legacy = initial.copy()
    pos_extended = initial.copy()
    for _ in range(300):
        leg = legacy.update(0.02, pos_legacy, target)
        ext = extended.update(0.02, pos_extended, target)
        np.testing.assert_array_equal(leg.position, ext.position)
        pos_legacy = leg.position
        pos_extended = ext.position


def test_reset_with_non_zero_initial_velocity_honours_it() -> None:
    parameters = TrajectoryParameters(max_speed=10.0, max_acceleration=3.0,
                                      max_jerk=4.0)
    controller = WaypointTrajectoryController(parameters)
    initial = np.zeros(3)
    v_init = np.array([2.0, 0.0, 0.0])
    controller.reset(initial, initial_velocity=v_init)

    sp = controller.update(0.02, initial, np.array([20.0, 0.0, 0.0]))
    # Emitted velocity should be close to v_init at the first tick (post-clip
    # is inert because v_init < max_speed).
    assert abs(float(sp.velocity[0]) - float(v_init[0])) < 0.2
    assert float(np.linalg.norm(sp.velocity)) <= parameters.max_speed + 1e-6


def test_reset_with_velocity_above_max_speed_gets_clipped() -> None:
    parameters = TrajectoryParameters(max_speed=1.0, max_acceleration=3.0,
                                      max_jerk=4.0)
    controller = WaypointTrajectoryController(parameters)
    initial = np.zeros(3)
    controller.reset(initial, initial_velocity=np.array([5.0, 0.0, 0.0]))

    sp = controller.update(0.02, initial, np.array([20.0, 0.0, 0.0]))
    assert float(np.linalg.norm(sp.velocity)) <= parameters.max_speed + 1e-6
