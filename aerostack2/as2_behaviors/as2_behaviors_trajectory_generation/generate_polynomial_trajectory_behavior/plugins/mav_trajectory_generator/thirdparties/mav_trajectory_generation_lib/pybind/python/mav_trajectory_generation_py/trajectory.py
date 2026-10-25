# Copyright 2026 Universidad Politécnica de Madrid
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
  * A dataclass mirror of the C++ ``GeneratorConfig`` for ergonomic
    construction (``PyGeneratorConfig``).
  * A YAML loader (``load_generator_config``) returning a native
    :class:`GeneratorConfig`.
  * A ``TrajectoryPoint`` dataclass returned by ``Trajectory.evaluate``.
  * A convenience ``Trajectory`` class that wraps ``TrajectoryGenerator`` with
    Pythonic input validation and output shaping.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence, Union

import numpy as np
import yaml

from ._mav_trajectory_generation_bindings import (
    EndWaypoint,
    GeneratorConfig,
    OptimizationConfig,
    Solver,
    TrajectoryGenerator,
    Waypoint,
)

PathLike = Union[str, Path]


@dataclass
class PyGeneratorConfig:
    """
    Plain dataclass mirror of the native :class:`GeneratorConfig`.

    Flattens the ``optimization`` sub-fields for ergonomic construction. Use
    :meth:`to_native` to obtain an equivalent native :class:`GeneratorConfig`
    suitable for the :class:`TrajectoryGenerator` constructor.
    """

    derivative_to_optimize: int = 4
    solver: str = "linear"  # "linear" | "nonlinear"
    a_max: float = 4.0
    nl_max_iterations: int = 2000
    nl_f_rel: float = 0.05
    nl_x_rel: float = 0.1
    nl_time_penalty: float = 1000.0
    nl_initial_stepsize_rel: float = 0.1
    nl_inequality_constraint_tolerance: float = 0.2

    def to_native(self) -> GeneratorConfig:
        """Return an equivalent native :class:`GeneratorConfig`."""
        opt = OptimizationConfig()
        opt.derivative_to_optimize = int(self.derivative_to_optimize)
        opt.solver = _solver_from_string(self.solver)
        opt.a_max = float(self.a_max)
        opt.nl_max_iterations = int(self.nl_max_iterations)
        opt.nl_f_rel = float(self.nl_f_rel)
        opt.nl_x_rel = float(self.nl_x_rel)
        opt.nl_time_penalty = float(self.nl_time_penalty)
        opt.nl_initial_stepsize_rel = float(self.nl_initial_stepsize_rel)
        opt.nl_inequality_constraint_tolerance = float(self.nl_inequality_constraint_tolerance)
        cfg = GeneratorConfig()
        cfg.optimization = opt
        return cfg


def _solver_from_string(name: str) -> Solver:
    key = str(name).strip().lower()
    if key == "linear":
        return Solver.linear
    if key == "nonlinear":
        return Solver.nonlinear
    raise ValueError(f"Unknown solver '{name}'. Expected 'linear' or 'nonlinear'.")


def load_generator_config(path: PathLike) -> GeneratorConfig:
    """
    Load a native :class:`GeneratorConfig` from a YAML file.

    The YAML document is expected to contain an ``optimization`` mapping at
    its top level (missing fields are replaced by defaults). Any top-level
    keys other than ``optimization`` are ignored for forward compatibility.
    """
    with open(path, "r", encoding="utf-8") as f:
        doc = yaml.safe_load(f) or {}
    opt = doc.get("optimization", {}) if isinstance(doc, dict) else {}
    if not isinstance(opt, dict):
        raise ValueError(
            f"Expected 'optimization' to be a mapping in {path}, got {type(opt).__name__}")
    py_cfg = PyGeneratorConfig()
    for key, value in opt.items():
        if hasattr(py_cfg, key):
            setattr(py_cfg, key, value)
    return py_cfg.to_native()


@dataclass
class TrajectoryPoint:
    """A single point sampled from a trajectory."""

    time: float
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3))


class Trajectory:
    """High-level wrapper around :class:`TrajectoryGenerator`."""

    def __init__(self, config: Union[GeneratorConfig, PyGeneratorConfig, None] = None):
        if config is None:
            native_cfg = GeneratorConfig()
        elif isinstance(config, GeneratorConfig):
            native_cfg = config
        elif isinstance(config, PyGeneratorConfig):
            native_cfg = config.to_native()
        else:
            raise TypeError(
                "config must be None, GeneratorConfig or PyGeneratorConfig, "
                f"got {type(config).__name__}")
        self._generator = TrajectoryGenerator(native_cfg)

    def generate(
        self,
        waypoints: Sequence[Union[Waypoint, np.ndarray]],
        max_speed: float,
    ) -> bool:
        """
        Generate a trajectory through ``waypoints`` at cruise ``max_speed`` [m/s].

        Each entry may be either a :class:`Waypoint` (or :class:`EndWaypoint`) or
        a 3D array-like position. Bare positions are wrapped as unconstrained
        :class:`Waypoint` (position-only) for backward-compat; wrap the first
        and last entries in :class:`EndWaypoint` to pin velocity/acceleration
        to zero at the endpoints.
        """
        if max_speed <= 0.0:
            raise ValueError(f"max_speed must be > 0, got {max_speed}")
        native: list[Waypoint] = []
        for wp in waypoints:
            if isinstance(wp, Waypoint):
                native.append(wp)
            else:
                native.append(Waypoint(np.asarray(wp, dtype=np.float64).reshape(3)))
        if len(native) < 2:
            raise ValueError(
                f"Need at least 2 waypoints to generate a trajectory, got {len(native)}")
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

    @property
    def is_valid(self) -> bool:
        return bool(self._generator.is_valid())

    @property
    def min_time(self) -> float:
        return float(self._generator.min_time())

    @property
    def max_time(self) -> float:
        return float(self._generator.max_time())

    @property
    def duration(self) -> float:
        return float(self._generator.duration())
