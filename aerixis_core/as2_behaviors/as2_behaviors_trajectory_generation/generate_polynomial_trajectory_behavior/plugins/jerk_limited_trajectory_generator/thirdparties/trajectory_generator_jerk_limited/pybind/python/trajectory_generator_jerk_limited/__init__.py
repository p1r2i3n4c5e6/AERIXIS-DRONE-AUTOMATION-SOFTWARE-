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
Python bindings for trajectory_generator_jerk_limited.

Public API:

- **Streaming API** (legacy, used by realtime / online consumers):
  ``TrajectoryParameters``, ``TrajectorySetpoints``,
  ``WaypointTrajectoryController``,
  ``JerkLimitedTrajectoryGenerator3D``, ``JerkLimitedVelocityProfile1D``.

- **One-shot API** (offline planning):
  ``Waypoint``, ``EndWaypoint``, ``TrajectorySample``,
  ``SegmentPolynomial``, ``Spline``, ``TrajectoryGenerator``,
  plus the high-level Python wrappers ``Trajectory``,
  ``TrajectoryPoint``, ``GeneratorConfig`` and the YAML loader
  ``load_generator_config``.

For advanced use, the native pybind11 classes are also re-exported under
``_``-prefixed names.
"""


__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from ._trajectory_generator_jerk_limited_cpp import (
    _JerkLimitedTrajectoryGenerator3D,
    _JerkLimitedVelocityProfile1D,
    _TrajectoryParameters,
    _TrajectorySetpoints,
    _WaypointTrajectoryController,
    EndWaypoint,
    GeneratorConfig as _NativeGeneratorConfig,
    SegmentPolynomial,
    SimulatorConfig,
    Spline,
    TrajectoryGenerator,
    TrajectorySample,
    Waypoint,
)
from .trajectory import (
    GeneratorConfig,
    load_generator_config,
    Trajectory,
    TrajectoryPoint,
)
from .trajectory_generator import (
    JerkLimitedTrajectoryGenerator3D,
    JerkLimitedVelocityProfile1D,
    TrajectoryParameters,
    TrajectorySetpoints,
    WaypointTrajectoryController,
)

__all__ = [
    # One-shot API
    'EndWaypoint',
    'GeneratorConfig',
    'SegmentPolynomial',
    'SimulatorConfig',
    'Spline',
    'Trajectory',
    'TrajectoryGenerator',
    'TrajectoryPoint',
    'TrajectorySample',
    'Waypoint',
    'load_generator_config',
    # Streaming API
    'JerkLimitedTrajectoryGenerator3D',
    'JerkLimitedVelocityProfile1D',
    'TrajectoryParameters',
    'TrajectorySetpoints',
    'WaypointTrajectoryController',
    # Native bindings for advanced users
    '_JerkLimitedTrajectoryGenerator3D',
    '_JerkLimitedVelocityProfile1D',
    '_NativeGeneratorConfig',
    '_TrajectoryParameters',
    '_TrajectorySetpoints',
    '_WaypointTrajectoryController',
]
