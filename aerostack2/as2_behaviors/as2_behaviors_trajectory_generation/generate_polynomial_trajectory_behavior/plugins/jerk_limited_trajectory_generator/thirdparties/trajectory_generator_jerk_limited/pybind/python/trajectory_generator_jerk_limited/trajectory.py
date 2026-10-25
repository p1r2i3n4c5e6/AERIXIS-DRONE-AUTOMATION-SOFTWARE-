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
High-level Python helpers on top of the native bindings.

Provides:

- :class:`GeneratorConfig` — plain dataclass mirror of the native
  configuration struct, with a YAML loader.
- :class:`TrajectoryPoint` — dataclass returned by
  :meth:`Trajectory.evaluate`.
- :class:`Trajectory` — convenience wrapper around the native
  :class:`TrajectoryGenerator` with Pythonic input validation and
  output shaping.
"""

from __future__ import annotations

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence, Union

import numpy as np
import yaml

from ._trajectory_generator_jerk_limited_cpp import (
    GeneratorConfig as _NativeGeneratorConfig,
    TrajectoryGenerator,
    Waypoint,
)

PathLike = Union[str, Path]


@dataclass
class GeneratorConfig:
    """
    Plain dataclass mirror of the native :class:`GeneratorConfig`.

    Bundles the kinematic limits with the simulation-loop tunables
    consumed by :class:`TrajectoryGenerator.generate`.
    """

    max_speed: float = 0.0
    max_acceleration: float = 0.0
    max_jerk: float = 0.0
    max_tracking_error: float = 0.0
    acceptance_radius: float = 0.0
    trajectory_gain: float = 1.0
    step_dt: float = 0.01
    max_simulation_time: float = 600.0
    settle_velocity: float = 0.05
    polynomial_order: int = 16

    def to_native(self) -> _NativeGeneratorConfig:
        """Return an equivalent native :class:`GeneratorConfig`."""
        cfg = _NativeGeneratorConfig()
        cfg.params.max_speed = float(self.max_speed)
        cfg.params.max_acceleration = float(self.max_acceleration)
        cfg.params.max_jerk = float(self.max_jerk)
        cfg.params.max_tracking_error = float(self.max_tracking_error)
        cfg.params.acceptance_radius = float(self.acceptance_radius)
        cfg.params.trajectory_gain = float(self.trajectory_gain)
        cfg.simulator.step_dt = float(self.step_dt)
        cfg.simulator.max_simulation_time = float(self.max_simulation_time)
        cfg.simulator.settle_velocity = float(self.settle_velocity)
        cfg.simulator.polynomial_order = int(self.polynomial_order)
        return cfg


def load_generator_config(path: PathLike) -> GeneratorConfig:
    """
    Load a :class:`GeneratorConfig` from a YAML file.

    The YAML may contain a top-level ``trajectory_parameters`` mapping
    (kinematic bounds, matching the existing config schema) and an
    optional ``simulation`` mapping (``step_dt``,
    ``max_simulation_time``, ``settle_velocity``, ``polynomial_order``).
    Missing keys fall back to dataclass defaults; unknown keys are
    ignored for forward compatibility.

    :param path: Path to the YAML file.
    :type path: str or Path
    :return: Populated configuration.
    :rtype: GeneratorConfig
    """
    with open(path, 'r', encoding='utf-8') as f:
        doc = yaml.safe_load(f) or {}
    if not isinstance(doc, dict):
        raise ValueError(
            f'Expected mapping at root of {path}, got {type(doc).__name__}')

    cfg = GeneratorConfig()

    params = doc.get('trajectory_parameters', {}) or {}
    if not isinstance(params, dict):
        raise ValueError(
            f"Expected 'trajectory_parameters' to be a mapping in {path}, "
            f'got {type(params).__name__}')
    for key in ('max_speed', 'max_acceleration', 'max_jerk',
                'max_tracking_error', 'acceptance_radius', 'trajectory_gain'):
        if key in params:
            setattr(cfg, key, float(params[key]))

    sim = doc.get('simulation', {}) or {}
    if not isinstance(sim, dict):
        raise ValueError(
            f"Expected 'simulation' to be a mapping in {path}, "
            f'got {type(sim).__name__}')
    for key in ('step_dt', 'max_simulation_time', 'settle_velocity'):
        if key in sim:
            setattr(cfg, key, float(sim[key]))
    if 'polynomial_order' in sim:
        cfg.polynomial_order = int(sim['polynomial_order'])

    return cfg


@dataclass
class TrajectoryPoint:
    """A single point sampled from a trajectory."""

    time: float
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3))


class Trajectory:
    """High-level wrapper around :class:`TrajectoryGenerator`."""

    def __init__(
        self,
        config: Union[GeneratorConfig, _NativeGeneratorConfig, None] = None,
    ) -> None:
        """Construct the wrapper from a Python or native config (or defaults)."""
        if config is None:
            native_cfg = _NativeGeneratorConfig()
        elif isinstance(config, _NativeGeneratorConfig):
            native_cfg = config
        elif isinstance(config, GeneratorConfig):
            native_cfg = config.to_native()
        else:
            raise TypeError(
                'config must be None, GeneratorConfig or native GeneratorConfig, '
                f'got {type(config).__name__}')
        self._generator = TrajectoryGenerator(native_cfg)

    def generate(
        self,
        waypoints: Sequence[Union[Waypoint, np.ndarray]],
        max_speed: float,
    ) -> bool:
        """
        Generate a trajectory through ``waypoints`` at cruise ``max_speed`` [m/s].

        Each entry may be either a :class:`Waypoint` (or
        :class:`EndWaypoint`) or a 3D array-like position. Bare positions
        are wrapped as :class:`Waypoint` (position-only).

        :param waypoints: Ordered sequence of at least two waypoints.
        :param max_speed: Cruise speed cap [m/s], must be > 0.
        :return: True iff the simulation produced a valid trajectory.
        """
        if max_speed <= 0.0:
            raise ValueError(f'max_speed must be > 0, got {max_speed}')
        native: list[Waypoint] = []
        for wp in waypoints:
            if isinstance(wp, Waypoint):
                native.append(wp)
            else:
                native.append(Waypoint(np.asarray(wp, dtype=np.float64).reshape(3)))
        if len(native) < 2:
            raise ValueError(
                f'Need at least 2 waypoints to generate a trajectory, got {len(native)}')
        return self._generator.generate(native, float(max_speed))

    def evaluate(self, t: float) -> TrajectoryPoint:
        """Sample the trajectory at time ``t`` [s]."""
        sample = self._generator.evaluate(float(t))
        return TrajectoryPoint(
            time=float(t),
            position=np.asarray(sample.position, dtype=np.float64),
            velocity=np.asarray(sample.velocity, dtype=np.float64),
            acceleration=np.asarray(sample.acceleration, dtype=np.float64),
        )

    def spline(self):
        """Return the planned spline snapshot (Spline instance)."""
        return self._generator.spline()

    @property
    def is_valid(self) -> bool:
        """Return True iff the last :meth:`generate` call produced a usable trajectory."""
        return bool(self._generator.is_valid())

    @property
    def min_time(self) -> float:
        """Start time of the trajectory [s]."""
        return float(self._generator.min_time())

    @property
    def max_time(self) -> float:
        """End time of the trajectory [s]."""
        return float(self._generator.max_time())

    @property
    def duration(self) -> float:
        """Total duration of the trajectory [s]."""
        return float(self._generator.duration())

    @property
    def max_velocity(self) -> float:
        """Maximum speed magnitude attained along the trajectory [m/s]."""
        return float(self._generator.max_velocity())

    @property
    def max_acceleration(self) -> float:
        """Maximum acceleration magnitude attained along the trajectory [m/s^2]."""
        return float(self._generator.max_acceleration())
