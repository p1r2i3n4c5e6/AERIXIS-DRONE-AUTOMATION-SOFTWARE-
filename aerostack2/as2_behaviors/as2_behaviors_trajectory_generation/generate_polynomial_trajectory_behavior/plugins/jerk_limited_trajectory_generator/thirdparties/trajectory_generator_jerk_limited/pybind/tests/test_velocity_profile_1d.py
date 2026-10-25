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

"""pytest suite for JerkLimitedVelocityProfile1D bindings."""

from __future__ import annotations

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import math

import pytest

from trajectory_generator_jerk_limited import JerkLimitedVelocityProfile1D


def _make_profile(
    max_jerk: float = 4.0,
    max_acceleration: float = 3.0,
    max_velocity: float = 5.0,
) -> JerkLimitedVelocityProfile1D:
    profile = JerkLimitedVelocityProfile1D()
    profile.set_max_jerk(max_jerk)
    profile.set_max_acceleration(max_acceleration)
    profile.set_max_velocity(max_velocity)
    profile.reset(0.0, 0.0, 0.0)
    return profile


def test_phase_durations_are_non_negative() -> None:
    profile = _make_profile()
    profile.update_durations(5.0)
    assert profile.phase1_duration() >= 0.0
    assert profile.phase2_duration() >= 0.0
    assert profile.phase3_duration() >= 0.0
    assert profile.total_time() > 0.0


def test_reaches_setpoint_with_zero_acceleration() -> None:
    profile = _make_profile()
    velocity_setpoint = 5.0
    profile.update_durations(velocity_setpoint)

    dt = 0.01
    num_steps = int((profile.total_time() + 0.5) / dt)
    for _ in range(num_steps):
        profile.update_trajectory(dt)

    assert profile.current_velocity() == pytest.approx(velocity_setpoint, abs=1e-6)
    assert profile.current_acceleration() == pytest.approx(0.0, abs=1e-6)


def test_respects_jerk_and_acceleration_bounds() -> None:
    max_jerk = 4.0
    max_acceleration = 3.0
    profile = _make_profile(max_jerk=max_jerk, max_acceleration=max_acceleration)
    profile.update_durations(5.0)

    dt = 0.01
    num_steps = int((profile.total_time() + 0.5) / dt)
    max_jerk_observed = 0.0
    max_acceleration_observed = 0.0
    for _ in range(num_steps):
        profile.update_trajectory(dt)
        max_jerk_observed = max(max_jerk_observed, abs(profile.current_jerk()))
        max_acceleration_observed = max(
            max_acceleration_observed, abs(profile.current_acceleration())
        )

    assert max_jerk_observed <= max_jerk + 1e-9
    assert max_acceleration_observed <= max_acceleration + 1e-6


def test_constructor_with_initial_state() -> None:
    profile = JerkLimitedVelocityProfile1D(0.5, 1.0, -2.0)
    assert math.isclose(profile.current_acceleration(), 0.5)
    assert math.isclose(profile.current_velocity(), 1.0)
    assert math.isclose(profile.current_position(), -2.0)
