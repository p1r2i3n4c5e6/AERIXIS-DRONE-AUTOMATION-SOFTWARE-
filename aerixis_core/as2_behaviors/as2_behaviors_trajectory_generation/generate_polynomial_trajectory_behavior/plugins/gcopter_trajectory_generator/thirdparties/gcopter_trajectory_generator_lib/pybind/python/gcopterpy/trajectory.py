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
YAML loaders and Python-friendly dataclasses for gcopter_lib.

Mirrors the structs and loaders in example/utils.hpp so that Python examples
use the same config files (config_example.yaml / config_trajectory.yaml) as
the C++ example.
"""
from __future__ import annotations

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import dataclasses
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import yaml

from ._gcopter_lib_cpp import (
    build_aabb_corridor,
    DroneLimits,
    DroneParameters,
    OptimizationConfig,
    Spline,
    TrajectoryGenerator,
    TrajectoryLogger,
    Waypoint,
)

__all__ = [
    'build_aabb_corridor',
    'CorridorConfig',
    'DroneLimits',
    'DroneParameters',
    'load_sim_config',
    'load_trajectory_config',
    'OptimizationConfig',
    'SimConfig',
    'Trajectory',
    'TrajectoryConfig',
    'TrajectoryGenerator',
    'TrajectoryLogger',
    'TrajectoryPoint',
    'Waypoint',
]


@dataclasses.dataclass
class SimConfig:
    """Loaded from sim_config block of config_example.yaml."""

    step_dt: float = 0.01
    waypoints: List[np.ndarray] = dataclasses.field(default_factory=list)


@dataclasses.dataclass
class CorridorConfig:
    """Loaded from trajectory_generator block of config_trajectory.yaml."""

    corridor_margin: float = 2.0
    waypoint_margin: float = 0.0
    waypoint_anchor_radius: float = 0.0


@dataclasses.dataclass
class TrajectoryConfig:
    """Full trajectory configuration loaded from config_trajectory.yaml."""

    corridor: CorridorConfig = dataclasses.field(default_factory=CorridorConfig)
    params: DroneParameters = dataclasses.field(default_factory=DroneParameters)
    limits: DroneLimits = dataclasses.field(default_factory=DroneLimits)
    optimization: OptimizationConfig = dataclasses.field(default_factory=OptimizationConfig)


def load_sim_config(path: str | Path) -> SimConfig:
    """Load SimConfig from config_example.yaml."""
    with open(path) as f:
        data = yaml.safe_load(f)
    sim = data['sim_config']
    cfg = SimConfig()
    cfg.step_dt = float(sim['step_dt'])
    if cfg.step_dt <= 0.0:
        raise ValueError('sim_config.step_dt must be > 0')
    cfg.waypoints = [np.array(wp, dtype=float) for wp in sim['waypoints']]
    if len(cfg.waypoints) < 2:
        raise ValueError('sim_config.waypoints must contain at least 2 entries')
    return cfg


def load_trajectory_config(path: str | Path) -> TrajectoryConfig:
    """Load TrajectoryConfig from config_trajectory.yaml."""
    with open(path) as f:
        data = yaml.safe_load(f)

    gen_node = data['trajectory_generator']
    gen = CorridorConfig()
    gen.corridor_margin = float(gen_node['corridor_margin'])
    gen.waypoint_margin = float(gen_node.get('waypoint_margin', 0.0))
    gen.waypoint_anchor_radius = float(gen_node.get('waypoint_anchor_radius', 0.0))
    if (gen.waypoint_margin > 0.0) != (gen.waypoint_anchor_radius > 0.0):
        raise ValueError(
            'trajectory_generator.waypoint_margin and waypoint_anchor_radius '
            'must both be > 0 or both zero'
        )

    params = DroneParameters()
    dp = data['drone_params']
    params.mass = float(dp['mass'])
    params.gravity = float(dp['gravity'])
    params.horizontal_drag = float(dp['horizontal_drag'])
    params.vertical_drag = float(dp['vertical_drag'])
    params.parasitic_drag = float(dp['parasitic_drag'])
    params.speed_smooth_factor = float(dp['speed_smooth_factor'])

    limits = DroneLimits()
    dl = data['drone_limits']
    limits.max_velocity = float(dl['max_velocity'])
    limits.max_body_rate = float(dl['max_body_rate'])
    limits.max_tilt_angle = float(dl['max_tilt_angle'])
    limits.min_thrust = float(dl['min_thrust'])
    limits.max_thrust = float(dl['max_thrust'])

    config = OptimizationConfig()
    opt = data['optimization']
    config.time_weight = float(opt['time_weight'])
    config.position_weight = float(opt['position_weight'])
    config.velocity_weight = float(opt['velocity_weight'])
    config.body_rate_weight = float(opt['body_rate_weight'])
    config.tilt_weight = float(opt['tilt_weight'])
    config.thrust_weight = float(opt['thrust_weight'])
    config.smoothing_eps = float(opt['smoothing_eps'])
    config.integral_resolution = int(opt['integral_resolution'])
    config.rel_cost_tol = float(opt['rel_cost_tol'])
    config.corridor_margin = gen.corridor_margin

    return TrajectoryConfig(corridor=gen, params=params, limits=limits, optimization=config)


@dataclasses.dataclass
class TrajectoryPoint:
    """
    A single sampled point on a generated trajectory.

    All vector quantities are expressed in the ENU world frame and SI units.

    :param time: Time from trajectory start (s).
    :param position: Position vector (m).
    :param velocity: Velocity vector (m/s).
    :param acceleration: Acceleration vector (m/s^2).
    :param jerk: Jerk vector (m/s^3).
    """

    time: float
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    jerk: np.ndarray


def _expand_path_with_anchors(
    waypoints: List[Waypoint],
    loose_margin: float,
    tight_margin: float,
    anchor_radius: float,
) -> Tuple[List[Waypoint], List[float]]:
    """
    Insert anchor points around each intermediate waypoint.

    For every intermediate waypoint i, two anchor points are inserted at
    ``anchor_radius`` along the incoming and outgoing directions, each capped
    at 45 % of the corresponding segment length. The returned margin list
    alternates between ``loose_margin`` (inter-waypoint segments) and
    ``tight_margin`` (the two short segments that pinch the corridor around
    the waypoint).

    Boundary waypoints preserve their velocity and acceleration constraints.

    :param waypoints: Ordered list with at least 2 entries.
    :type waypoints: List[Waypoint]
    :param loose_margin: AABB corridor half-width for long segments (m).
    :type loose_margin: float
    :param tight_margin: AABB corridor half-width for pinch segments (m).
    :type tight_margin: float
    :param anchor_radius: Nominal distance from waypoint to anchor point (m).
    :type anchor_radius: float
    :return: Tuple of (expanded_waypoints, per_segment_margins).
    :rtype: Tuple[List[Waypoint], List[float]]
    """
    positions = [wp.position for wp in waypoints]
    n = len(positions)

    if n < 3 or tight_margin <= 0.0 or anchor_radius <= 0.0:
        return waypoints, [loose_margin] * (n - 1)

    expanded_pos: List[np.ndarray] = [positions[0].copy()]

    for i in range(1, n - 1):
        prev = positions[i - 1]
        curr = positions[i]
        next_ = positions[i + 1]

        in_vec = curr - prev
        out_vec = next_ - curr
        in_norm = float(np.linalg.norm(in_vec))
        out_norm = float(np.linalg.norm(out_vec))

        if in_norm <= 0.0 or out_norm <= 0.0:
            expanded_pos.append(curr.copy())
            continue

        r_in = min(anchor_radius, 0.45 * in_norm)
        r_out = min(anchor_radius, 0.45 * out_norm)
        expanded_pos += [
            curr - in_vec * (r_in / in_norm),
            curr.copy(),
            curr + out_vec * (r_out / out_norm),
        ]

    expanded_pos.append(positions[-1].copy())

    m = n - 2
    margins: List[float] = [loose_margin]
    for i in range(m):
        margins += [tight_margin, tight_margin]
        if i + 1 < m:
            margins.append(loose_margin)
    margins.append(loose_margin)

    def _copy_or_none(value: Optional[np.ndarray]) -> Optional[np.ndarray]:
        return None if value is None else np.asarray(value, dtype=float).copy()

    expanded: List[Waypoint] = []
    for idx, pos in enumerate(expanded_pos):
        wp = Waypoint(pos)
        if idx == 0:
            wp.velocity = _copy_or_none(waypoints[0].velocity)
            wp.acceleration = _copy_or_none(waypoints[0].acceleration)
        elif idx == len(expanded_pos) - 1:
            wp.velocity = _copy_or_none(waypoints[-1].velocity)
            wp.acceleration = _copy_or_none(waypoints[-1].acceleration)
        expanded.append(wp)

    return expanded, margins


class Trajectory:
    """
    High-level wrapper around TrajectoryGenerator.

    Encapsulates waypoint conversion, anchor expansion, corridor generation
    and error checking into a single object. Default margin values are read
    from ``TrajectoryConfig.generator`` and can be overridden per call to
    :meth:`generate`.

    Not thread-safe: the underlying ``TrajectoryGenerator`` holds mutable
    state. Use one instance per thread or add external synchronisation.

    Example usage::

        cfg = load_trajectory_config('config_trajectory.yaml')
        traj = Trajectory(cfg)
        traj.generate([np.array([0.0, 0.0, 1.0]), np.array([5.0, 0.0, 1.0])])
        pt = traj.evaluate(0.5 * traj.duration)

    :param config: Full trajectory configuration.
    :type config: TrajectoryConfig
    """

    def __init__(self, config: TrajectoryConfig) -> None:
        """
        Initialize the Trajectory wrapper.

        :param config: Full trajectory configuration loaded from YAML or
            constructed programmatically.
        :type config: TrajectoryConfig
        """
        self._config = config
        self._generator = TrajectoryGenerator(self._make_native_config())

    def _make_native_config(self):
        """Build a native GeneratorConfig from the example-side TrajectoryConfig."""
        from gcopterpy import GeneratorConfig as _NativeGeneratorConfig
        cfg = _NativeGeneratorConfig()
        cfg.params = self._config.params
        cfg.limits = self._config.limits
        cfg.optimization = self._config.optimization
        return cfg

    def generate(
        self,
        positions: List[np.ndarray],
        *,
        corridor_margin: Optional[float] = None,
        waypoint_margin: Optional[float] = None,
        anchor_radius: Optional[float] = None,
    ) -> None:
        """
        Generate an optimised polynomial trajectory through ``positions``.

        Anchor expansion is applied automatically when both
        ``waypoint_margin`` and ``anchor_radius`` (either the passed value or
        the default from ``self._config.corridor``) are strictly positive.

        :param positions: Ordered list of 3-D position waypoints (m, ENU).
            Must contain at least 2 entries.
        :type positions: List[np.ndarray]
        :param corridor_margin: AABB corridor half-width for loose
            inter-waypoint segments (m). Defaults to
            ``config.generator.corridor_margin``.
        :type corridor_margin: float or None
        :param waypoint_margin: AABB corridor half-width for the pinch
            segments inserted around intermediate waypoints (m). Defaults to
            ``config.generator.waypoint_margin``. Set to 0 to disable anchor
            expansion.
        :type waypoint_margin: float or None
        :param anchor_radius: Distance from an intermediate waypoint to its
            anchor insertion point (m). Defaults to
            ``config.generator.waypoint_anchor_radius``.
        :type anchor_radius: float or None
        :raises ValueError: If fewer than 2 positions are provided.
        :raises RuntimeError: If the GCOPTER optimiser fails to find a
            feasible trajectory.
        :rtype: None
        """
        if len(positions) < 2:
            raise ValueError('generate() requires at least 2 positions.')

        if corridor_margin is None:
            corridor_margin = self._config.corridor.corridor_margin
        if waypoint_margin is None:
            waypoint_margin = self._config.corridor.waypoint_margin
        if anchor_radius is None:
            anchor_radius = self._config.corridor.waypoint_anchor_radius

        waypoints = [Waypoint(np.asarray(p, dtype=float)) for p in positions]

        # max_speed for this call: use the configured drone-level limit so the
        # corridor margins are the only varying inputs across the two paths.
        max_speed = float(self._config.limits.max_velocity)

        anchors_enabled = waypoint_margin > 0.0 and anchor_radius > 0.0
        if anchors_enabled:
            waypoints, margins = _expand_path_with_anchors(
                waypoints, corridor_margin, waypoint_margin, anchor_radius
            )
            ok = self._generator.generate(waypoints, max_speed, margins)
        else:
            # Push the requested corridor_margin into the optimisation config so
            # the single-margin overload uses it for this call.
            self._config.optimization.corridor_margin = corridor_margin
            self._generator = TrajectoryGenerator(self._make_native_config())
            ok = self._generator.generate(waypoints, max_speed)

        if not ok:
            raise RuntimeError(
                'Trajectory generation failed. Check waypoint spacing and '
                'corridor margins relative to drone limits.'
            )

    def evaluate(self, t: float) -> TrajectoryPoint:
        """
        Sample the trajectory at time *t*.

        The time is clamped to ``[0, duration]`` so callers do not need to
        guard against floating-point overshoot at the end of the trajectory.

        :param t: Query time (s, measured from trajectory start).
        :type t: float
        :return: Trajectory state at the clamped time.
        :rtype: TrajectoryPoint
        :raises RuntimeError: If called before a successful :meth:`generate`.
        """
        if not self._generator.is_valid():
            raise RuntimeError(
                'evaluate() called on an uninitialised trajectory. '
                'Call generate() first.'
            )
        t_clamped = float(np.clip(t, 0.0, self._generator.duration()))
        return TrajectoryPoint(
            time=t_clamped,
            position=self._generator.position(t_clamped),
            velocity=self._generator.velocity(t_clamped),
            acceleration=self._generator.acceleration(t_clamped),
            jerk=self._generator.jerk(t_clamped),
        )

    @property
    def is_valid(self) -> bool:
        """
        Return True if a trajectory has been successfully generated.

        :return: Validity flag.
        :rtype: bool
        """
        return self._generator.is_valid()

    @property
    def duration(self) -> float:
        """
        Return the total trajectory duration in seconds.

        :return: Duration, or 0.0 if no trajectory has been generated.
        :rtype: float
        """
        return self._generator.duration() if self._generator.is_valid() else 0.0

    @property
    def max_velocity(self) -> float:
        """
        Return the peak velocity magnitude along the trajectory in m/s.

        :return: Maximum velocity, or 0.0 if no trajectory has been generated.
        :rtype: float
        """
        return (
            self._generator.max_velocity() if self._generator.is_valid() else 0.0
        )

    @property
    def max_acceleration(self) -> float:
        """
        Return the peak acceleration magnitude along the trajectory in m/s^2.

        :return: Maximum acceleration, or 0.0 if no trajectory has been generated.
        :rtype: float
        """
        return (
            self._generator.max_acceleration() if self._generator.is_valid() else 0.0
        )

    def spline(self) -> Spline:
        """
        Return a snapshot of the planned spline with segment and polynomial data.

        :return: Piecewise polynomial trajectory; lowest-degree-first coeffs.
        :rtype: Spline
        :raises RuntimeError: If called before a successful :meth:`generate`.
        """
        return self._generator.spline()
