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

"""pytest suite for Waypoint and EndWaypoint bindings."""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from gcopterpy import EndWaypoint, Waypoint

import numpy as np


def test_position_only_constructor_leaves_derivatives_unset() -> None:
    wp = Waypoint(np.array([1.0, 2.0, 3.0]))
    np.testing.assert_array_equal(wp.position, np.array([1.0, 2.0, 3.0]))
    assert wp.velocity is None
    assert wp.acceleration is None


def test_default_constructor_zero_position() -> None:
    wp = Waypoint()
    np.testing.assert_array_equal(wp.position, np.zeros(3))
    assert wp.velocity is None
    assert wp.acceleration is None


def test_properties_accept_assignment() -> None:
    wp = Waypoint(np.zeros(3))
    wp.position = np.array([4.0, 5.0, 6.0])
    wp.velocity = np.array([1.0, 1.0, 1.0])
    wp.acceleration = np.array([0.1, 0.2, 0.3])
    np.testing.assert_array_equal(wp.position, np.array([4.0, 5.0, 6.0]))
    np.testing.assert_array_equal(wp.velocity, np.array([1.0, 1.0, 1.0]))
    np.testing.assert_array_equal(wp.acceleration, np.array([0.1, 0.2, 0.3]))


def test_velocity_can_be_reset_to_none() -> None:
    wp = Waypoint(np.zeros(3))
    wp.velocity = np.array([1.0, 0.0, 0.0])
    assert wp.velocity is not None
    wp.velocity = None
    assert wp.velocity is None


def test_end_waypoint_pins_velocity_and_acceleration_to_zero() -> None:
    wp = EndWaypoint(np.array([0.0, 0.0, 1.0]))
    np.testing.assert_array_equal(wp.position, np.array([0.0, 0.0, 1.0]))
    assert wp.velocity is not None
    assert wp.acceleration is not None
    np.testing.assert_array_equal(wp.velocity, np.zeros(3))
    np.testing.assert_array_equal(wp.acceleration, np.zeros(3))
