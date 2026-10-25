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

"""pytest suite for build_aabb_corridor binding."""

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from gcopterpy import build_aabb_corridor

import numpy as np
import pytest


def test_uniform_margin_yields_one_polytope_per_segment() -> None:
    waypoints = [
        np.array([0.0, 0.0, 1.0]),
        np.array([2.0, 0.0, 1.0]),
        np.array([2.0, 2.0, 1.0]),
    ]
    corridor = build_aabb_corridor(waypoints, 0.5)
    assert len(corridor) == len(waypoints) - 1
    for polytope in corridor:
        # Each polytope is an Nx4 array of [a, b, c, d] half-spaces.
        assert polytope.shape[1] == 4
        # AABB has exactly 6 faces (upper and lower bounds on each axis).
        assert polytope.shape[0] == 6


def test_per_segment_margins_match_number_of_segments() -> None:
    waypoints = [
        np.array([0.0, 0.0, 1.0]),
        np.array([1.0, 0.0, 1.0]),
        np.array([1.0, 1.0, 1.0]),
        np.array([0.0, 1.0, 1.0]),
    ]
    margins = [0.3, 0.5, 0.4]
    corridor = build_aabb_corridor(waypoints, margins)
    assert len(corridor) == len(margins)


def test_per_segment_margins_length_mismatch_raises() -> None:
    waypoints = [
        np.array([0.0, 0.0, 1.0]),
        np.array([1.0, 0.0, 1.0]),
        np.array([1.0, 1.0, 1.0]),
    ]
    with pytest.raises((RuntimeError, ValueError, Exception)):
        build_aabb_corridor(waypoints, [0.3])
