#!/usr/bin/env python3

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
Python interface for trajectory_generator_jerk_limited bindings.

Thin, documented wrappers around the native pybind11 classes. Every wrapper
forwards to its `_`-prefixed native counterpart kept on the private attribute
``self._parameters`` (for value-type parameter bundles) or ``self._native``
(for stateful objects). Eigen vectors are marshalled as ``numpy.ndarray``.
"""

from __future__ import annotations

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from typing import List, Optional, Sequence, Union

import numpy as np

from ._trajectory_generator_jerk_limited_cpp import (
    _JerkLimitedTrajectoryGenerator3D,
    _JerkLimitedVelocityProfile1D,
    _TrajectoryParameters,
    _TrajectorySetpoints,
    _WaypointTrajectoryController,
    Waypoint as _NativeWaypoint,
)


def _coerce_triplet(value) -> List[np.ndarray]:
    """
    Convert a multi-waypoint argument to a list of three 1D numpy arrays.

    Accepted forms:
      * ``numpy.ndarray`` of shape ``(3, 3)`` (rows are waypoints).
      * Sequence (list/tuple) of three array-like objects of shape ``(3,)``.

    :raises ValueError: If the argument does not match either form.
    """
    arr = np.asarray(value, dtype=float)
    if arr.ndim == 2 and arr.shape == (3, 3):
        return [np.ascontiguousarray(arr[i], dtype=float) for i in range(3)]
    if arr.ndim == 1 and arr.shape == (3,):
        raise ValueError(
            'Expected a multi-waypoint triplet of shape (3, 3); got a single (3,) array. '
            'Use the single-waypoint overload instead.'
        )
    if isinstance(value, Sequence) and len(value) == 3:
        coerced = [np.ascontiguousarray(np.asarray(wp, dtype=float)) for wp in value]
        if all(c.shape == (3,) for c in coerced):
            return coerced
    raise ValueError(
        'Multi-waypoint argument must be a numpy.ndarray of shape (3, 3) or a sequence '
        f'of three 3D arrays; got {type(value).__name__} with shape '
        f'{getattr(arr, "shape", None)}.'
    )


# -- TrajectoryParameters -----------------------------------------------------

class TrajectoryParameters:
    """
    Global motion limits for the jerk-limited trajectory generator.

    A non-positive value disables the corresponding bound. ``max_tracking_error``
    is the threshold above which the per-axis time-stretch gate engages.
    """

    def __init__(
            self,
            max_speed: float = 0.0,
            max_acceleration: float = 0.0,
            max_jerk: float = 0.0,
            max_tracking_error: float = 0.0,
            acceptance_radius: float = 0.0,
            trajectory_gain: float = 1.0) -> None:
        """
        Initialize the trajectory parameters.

        :param max_speed: Maximum linear speed (m/s). <= 0 disables the bound.
        :type max_speed: float
        :param max_acceleration: Maximum linear acceleration (m/s^2). <= 0 disables the bound.
        :type max_acceleration: float
        :param max_jerk: Maximum linear jerk (m/s^3). <= 0 disables the bound.
        :type max_jerk: float
        :param max_tracking_error: Tracking error threshold (m) that gates the
            time-stretch reset. <= 0 disables the gate.
        :type max_tracking_error: float
        :param acceptance_radius: Multi-waypoint corner / L1 acceptance radius (m).
            <= 0 forces the multi overload to behave as single-waypoint.
        :type acceptance_radius: float
        :param trajectory_gain: Dimensionless scale on max_acceleration inside the
            corner-speed cap of the multi-waypoint overload. Ignored otherwise.
        :type trajectory_gain: float
        """
        self._parameters = _TrajectoryParameters()
        self._parameters.max_speed = float(max_speed)
        self._parameters.max_acceleration = float(max_acceleration)
        self._parameters.max_jerk = float(max_jerk)
        self._parameters.max_tracking_error = float(max_tracking_error)
        self._parameters.acceptance_radius = float(acceptance_radius)
        self._parameters.trajectory_gain = float(trajectory_gain)

    @staticmethod
    def _from_native(native: _TrajectoryParameters) -> 'TrajectoryParameters':
        """
        Build a TrajectoryParameters wrapper from an existing native instance.

        :param native: Native binding instance.
        :type native: _TrajectoryParameters
        :return: Wrapper sharing the same underlying parameters.
        :rtype: TrajectoryParameters
        """
        wrapper = TrajectoryParameters.__new__(TrajectoryParameters)
        wrapper._parameters = native
        return wrapper

    @property
    def max_speed(self) -> float:
        """
        Get the maximum linear speed (m/s).

        :rtype: float
        """
        return self._parameters.max_speed

    @max_speed.setter
    def max_speed(self, value: float) -> None:
        """
        Set the maximum linear speed (m/s).

        :param value: Maximum speed. <= 0 disables the bound.
        :type value: float
        """
        self._parameters.max_speed = float(value)

    @property
    def max_acceleration(self) -> float:
        """
        Get the maximum linear acceleration (m/s^2).

        :rtype: float
        """
        return self._parameters.max_acceleration

    @max_acceleration.setter
    def max_acceleration(self, value: float) -> None:
        """
        Set the maximum linear acceleration (m/s^2).

        :param value: Maximum acceleration. <= 0 disables the bound.
        :type value: float
        """
        self._parameters.max_acceleration = float(value)

    @property
    def max_jerk(self) -> float:
        """
        Get the maximum linear jerk (m/s^3).

        :rtype: float
        """
        return self._parameters.max_jerk

    @max_jerk.setter
    def max_jerk(self, value: float) -> None:
        """
        Set the maximum linear jerk (m/s^3).

        :param value: Maximum jerk. <= 0 disables the bound.
        :type value: float
        """
        self._parameters.max_jerk = float(value)

    @property
    def max_tracking_error(self) -> float:
        """
        Get the tracking error threshold that gates time-stretch (m).

        :rtype: float
        """
        return self._parameters.max_tracking_error

    @max_tracking_error.setter
    def max_tracking_error(self, value: float) -> None:
        """
        Set the tracking error threshold (m).

        :param value: Threshold. <= 0 disables the gate.
        :type value: float
        """
        self._parameters.max_tracking_error = float(value)

    @property
    def acceptance_radius(self) -> float:
        """
        Multi-waypoint corner / L1 acceptance radius (m). 0 disables corner-cut.

        :rtype: float
        """
        return self._parameters.acceptance_radius

    @acceptance_radius.setter
    def acceptance_radius(self, value: float) -> None:
        """
        Set the multi-waypoint acceptance radius (m).

        :param value: Acceptance radius. <= 0 disables the corner-cut path.
        :type value: float
        """
        self._parameters.acceptance_radius = float(value)

    @property
    def trajectory_gain(self) -> float:
        """
        Get the dimensionless gain on ``max_acceleration`` for corner-speed.

        Applied inside the corner-speed cap of the multi-waypoint overload.

        :rtype: float
        """
        return self._parameters.trajectory_gain

    @trajectory_gain.setter
    def trajectory_gain(self, value: float) -> None:
        """
        Set the trajectory gain.

        :param value: Dimensionless scale on max_acceleration. < 0 is clamped to 0.
        :type value: float
        """
        self._parameters.trajectory_gain = float(value)


# -- TrajectorySetpoints ------------------------------------------------------

class TrajectorySetpoints:
    """
    One tick of trajectory setpoints emitted by the generator.

    All vectors are 3D numpy arrays in the world frame.
    """

    def __init__(self) -> None:
        """Initialize an empty setpoints struct (zeros)."""
        self._native = _TrajectorySetpoints()

    @staticmethod
    def _from_native(native: _TrajectorySetpoints) -> 'TrajectorySetpoints':
        """
        Build a TrajectorySetpoints wrapper from an existing native instance.

        :param native: Native binding instance.
        :type native: _TrajectorySetpoints
        :return: Wrapper sharing the same underlying data.
        :rtype: TrajectorySetpoints
        """
        wrapper = TrajectorySetpoints.__new__(TrajectorySetpoints)
        wrapper._native = native
        return wrapper

    @property
    def position(self) -> np.ndarray:
        """
        Get the position setpoint (m, world frame).

        :rtype: np.ndarray
        """
        return self._native.position

    @position.setter
    def position(self, value: np.ndarray) -> None:
        """
        Set the position setpoint (m, world frame).

        :param value: 3D position.
        :type value: np.ndarray
        """
        self._native.position = value

    @property
    def velocity(self) -> np.ndarray:
        """
        Get the velocity setpoint (m/s, world frame).

        :rtype: np.ndarray
        """
        return self._native.velocity

    @velocity.setter
    def velocity(self, value: np.ndarray) -> None:
        """
        Set the velocity setpoint (m/s, world frame).

        :param value: 3D velocity.
        :type value: np.ndarray
        """
        self._native.velocity = value

    @property
    def acceleration(self) -> np.ndarray:
        """
        Get the acceleration setpoint (m/s^2, world frame).

        :rtype: np.ndarray
        """
        return self._native.acceleration

    @acceleration.setter
    def acceleration(self, value: np.ndarray) -> None:
        """
        Set the acceleration setpoint (m/s^2, world frame).

        :param value: 3D acceleration.
        :type value: np.ndarray
        """
        self._native.acceleration = value

    @property
    def jerk(self) -> np.ndarray:
        """
        Get the jerk setpoint (m/s^3, world frame).

        :rtype: np.ndarray
        """
        return self._native.jerk

    @jerk.setter
    def jerk(self, value: np.ndarray) -> None:
        """
        Set the jerk setpoint (m/s^3, world frame).

        :param value: 3D jerk.
        :type value: np.ndarray
        """
        self._native.jerk = value

    @property
    def unsmoothed_velocity(self) -> np.ndarray:
        """
        Get the raw (unsmoothed) velocity reference used internally (m/s).

        Useful for diagnostics — it exposes the commanded velocity before the
        time-stretch gate is applied.

        :rtype: np.ndarray
        """
        return self._native.unsmoothed_velocity

    @unsmoothed_velocity.setter
    def unsmoothed_velocity(self, value: np.ndarray) -> None:
        """
        Set the raw (unsmoothed) velocity (m/s).

        :param value: 3D velocity.
        :type value: np.ndarray
        """
        self._native.unsmoothed_velocity = value


# -- JerkLimitedVelocityProfile1D ---------------------------------------------

class JerkLimitedVelocityProfile1D:
    """
    1D jerk-limited S-curve velocity profile.

    Integrates a three-phase S-curve defined by ``max_jerk``,
    ``max_acceleration`` and ``max_velocity``. Useful as a per-axis building
    block for higher-level generators.
    """

    def __init__(
            self,
            initial_acceleration: float = 0.0,
            initial_velocity: float = 0.0,
            initial_position: float = 0.0) -> None:
        """
        Initialize the 1D profile at a given state.

        :param initial_acceleration: Initial acceleration (m/s^2).
        :type initial_acceleration: float
        :param initial_velocity: Initial velocity (m/s).
        :type initial_velocity: float
        :param initial_position: Initial position (m).
        :type initial_position: float
        """
        self._native = _JerkLimitedVelocityProfile1D(
            float(initial_acceleration),
            float(initial_velocity),
            float(initial_position))

    @staticmethod
    def _from_native(
            native: _JerkLimitedVelocityProfile1D) -> 'JerkLimitedVelocityProfile1D':
        """
        Build a wrapper from an existing native instance.

        :param native: Native binding instance.
        :type native: _JerkLimitedVelocityProfile1D
        :return: Wrapper sharing the underlying state.
        :rtype: JerkLimitedVelocityProfile1D
        """
        wrapper = JerkLimitedVelocityProfile1D.__new__(JerkLimitedVelocityProfile1D)
        wrapper._native = native
        return wrapper

    # -- Reset ----------------------------------------------------------------

    def reset(self, acceleration: float, velocity: float, position: float) -> None:
        """
        Reset the integrator state.

        :param acceleration: Acceleration (m/s^2).
        :type acceleration: float
        :param velocity: Velocity (m/s).
        :type velocity: float
        :param position: Position (m).
        :type position: float
        :rtype: None
        """
        self._native.reset(float(acceleration), float(velocity), float(position))

    # -- Setters --------------------------------------------------------------

    def set_max_jerk(self, value: float) -> None:
        """
        Set the maximum absolute jerk (m/s^3).

        :param value: Maximum jerk.
        :type value: float
        :rtype: None
        """
        self._native.set_max_jerk(float(value))

    def set_max_acceleration(self, value: float) -> None:
        """
        Set the maximum absolute acceleration (m/s^2).

        :param value: Maximum acceleration.
        :type value: float
        :rtype: None
        """
        self._native.set_max_acceleration(float(value))

    def set_max_velocity(self, value: float) -> None:
        """
        Set the maximum absolute velocity (m/s).

        :param value: Maximum velocity.
        :type value: float
        :rtype: None
        """
        self._native.set_max_velocity(float(value))

    # -- Getters: bounds ------------------------------------------------------

    def max_jerk(self) -> float:
        """
        Get the current maximum jerk bound (m/s^3).

        :rtype: float
        """
        return self._native.max_jerk()

    def max_acceleration(self) -> float:
        """
        Get the current maximum acceleration bound (m/s^2).

        :rtype: float
        """
        return self._native.max_acceleration()

    def max_velocity(self) -> float:
        """
        Get the current maximum velocity bound (m/s).

        :rtype: float
        """
        return self._native.max_velocity()

    # -- Getters: state -------------------------------------------------------

    def current_jerk(self) -> float:
        """
        Get the current integrator jerk (m/s^3).

        :rtype: float
        """
        return self._native.current_jerk()

    def current_acceleration(self) -> float:
        """
        Get the current integrator acceleration (m/s^2).

        :rtype: float
        """
        return self._native.current_acceleration()

    def current_velocity(self) -> float:
        """
        Get the current integrator velocity (m/s).

        :rtype: float
        """
        return self._native.current_velocity()

    def current_position(self) -> float:
        """
        Get the current integrator position (m).

        :rtype: float
        """
        return self._native.current_position()

    # -- Getters: phases ------------------------------------------------------

    def phase1_duration(self) -> float:
        """
        Get the duration of phase 1 (jerk-up to a_max), in s.

        :rtype: float
        """
        return self._native.phase1_duration()

    def phase2_duration(self) -> float:
        """
        Get the duration of phase 2 (constant acceleration), in s.

        :rtype: float
        """
        return self._native.phase2_duration()

    def phase3_duration(self) -> float:
        """
        Get the duration of phase 3 (jerk-down to zero acceleration), in s.

        :rtype: float
        """
        return self._native.phase3_duration()

    def total_time(self) -> float:
        """
        Get the total duration of the three S-curve phases (s).

        :rtype: float
        """
        return self._native.total_time()

    # -- Integration ----------------------------------------------------------

    def update_durations(self, velocity_setpoint: float) -> None:
        """
        Recompute the three-phase durations for a new velocity setpoint.

        :param velocity_setpoint: Target velocity (m/s).
        :type velocity_setpoint: float
        :rtype: None
        """
        self._native.update_durations(float(velocity_setpoint))

    def update_trajectory(self, dt: float, time_stretch: float = 1.0) -> None:
        """
        Advance the integrator by ``dt`` seconds.

        :param dt: Integration step (s).
        :type dt: float
        :param time_stretch: Time-stretch factor (>= 1.0 slows down playback).
        :type time_stretch: float
        :rtype: None
        """
        self._native.update_trajectory(float(dt), float(time_stretch))


# -- JerkLimitedTrajectoryGenerator3D -----------------------------------------

class JerkLimitedTrajectoryGenerator3D:
    """
    3D jerk-limited trajectory generator (per-axis S-curve with time sync).

    Bounds are set individually (``set_max_*``) and the generator must be
    ``reset``-ed before the first ``generate`` call.
    """

    def __init__(self) -> None:
        """Initialize an empty 3D generator (bounds default to 0)."""
        self._native = _JerkLimitedTrajectoryGenerator3D()

    @staticmethod
    def _from_native(
            native: _JerkLimitedTrajectoryGenerator3D) -> 'JerkLimitedTrajectoryGenerator3D':
        """
        Build a wrapper from an existing native instance.

        :param native: Native binding instance.
        :type native: _JerkLimitedTrajectoryGenerator3D
        :return: Wrapper sharing the underlying state.
        :rtype: JerkLimitedTrajectoryGenerator3D
        """
        wrapper = JerkLimitedTrajectoryGenerator3D.__new__(JerkLimitedTrajectoryGenerator3D)
        wrapper._native = native
        return wrapper

    # -- Setters --------------------------------------------------------------

    def set_max_jerk(self, jerk: float) -> None:
        """
        Set the maximum jerk bound (m/s^3). <= 0 disables the bound.

        :param jerk: Maximum jerk.
        :type jerk: float
        :rtype: None
        """
        self._native.set_max_jerk(float(jerk))

    def set_max_speed(self, speed: float) -> None:
        """
        Set the maximum linear speed bound (m/s). <= 0 disables the bound.

        :param speed: Maximum speed.
        :type speed: float
        :rtype: None
        """
        self._native.set_max_speed(float(speed))

    def set_max_acceleration(self, acceleration: float) -> None:
        """
        Set the maximum linear acceleration bound (m/s^2). <= 0 disables the bound.

        :param acceleration: Maximum acceleration.
        :type acceleration: float
        :rtype: None
        """
        self._native.set_max_acceleration(float(acceleration))

    def set_max_tracking_error(self, error: float) -> None:
        """
        Set the tracking error threshold that gates time-stretch (m).

        :param error: Threshold. <= 0 disables the gate.
        :type error: float
        :rtype: None
        """
        self._native.set_max_tracking_error(float(error))

    def set_acceptance_radius(self, radius: float) -> None:
        """
        Set the multi-waypoint corner / L1 acceptance radius (m).

        :param radius: Radius. <= 0 disables corner handling (the multi
            overload then degrades to single-waypoint behaviour).
        :type radius: float
        :rtype: None
        """
        self._native.set_acceptance_radius(float(radius))

    def set_trajectory_gain(self, gain: float) -> None:
        """
        Set the dimensionless gain on ``max_acceleration`` for corner-speed.

        Applied inside the corner-speed cap of the multi-waypoint overload.

        :param gain: Trajectory gain. < 0 is clamped to 0 by the native side.
        :type gain: float
        :rtype: None
        """
        self._native.set_trajectory_gain(float(gain))

    # -- Getters --------------------------------------------------------------

    def current_position(self) -> np.ndarray:
        """
        Get the current internal position state (m, world frame).

        :rtype: np.ndarray
        """
        return self._native.current_position()

    def current_velocity(self) -> np.ndarray:
        """
        Get the current internal velocity state (m/s, world frame).

        :rtype: np.ndarray
        """
        return self._native.current_velocity()

    def current_speed(self) -> float:
        """
        Get the current internal speed (norm of velocity, m/s).

        :rtype: float
        """
        return self._native.current_speed()

    # -- Control --------------------------------------------------------------

    def reset(
            self,
            acceleration: np.ndarray,
            velocity: np.ndarray,
            position: np.ndarray) -> None:
        """
        Reset the integrator state.

        :param acceleration: Initial acceleration (m/s^2, world frame).
        :type acceleration: np.ndarray
        :param velocity: Initial velocity (m/s, world frame).
        :type velocity: np.ndarray
        :param position: Initial position (m, world frame).
        :type position: np.ndarray
        :rtype: None
        """
        self._native.reset(acceleration, velocity, position)

    def generate(
            self,
            current_position: np.ndarray,
            waypoint: Union[np.ndarray, Sequence],
            feedforward_velocity: np.ndarray,
            dt: float,
            force_zero_velocity_setpoint: bool = False) -> TrajectorySetpoints:
        """
        Advance the 3D S-curve by one tick and return the new setpoints.

        Single- or multi-waypoint depending on the shape of ``waypoint``:
          * Shape ``(3,)`` -> single-waypoint (legacy behaviour, no corner-cut).
          * Shape ``(3, 3)`` or sequence of three ``(3,)`` arrays -> multi-waypoint
            ``{prev, target, next}`` triplet (activates corner-cut and the
            corner-speed cap if ``acceptance_radius`` is positive).

        :param current_position: Current vehicle position (m, world frame).
        :type current_position: np.ndarray
        :param waypoint: Target waypoint (single) or triplet (multi).
        :type waypoint: np.ndarray or sequence
        :param feedforward_velocity: Feed-forward velocity (m/s, world frame).
        :type feedforward_velocity: np.ndarray
        :param dt: Integration step (s).
        :type dt: float
        :param force_zero_velocity_setpoint: If True, force the internal
            unsmoothed velocity setpoint to zero for this tick.
        :type force_zero_velocity_setpoint: bool
        :return: Trajectory setpoints for this tick.
        :rtype: TrajectorySetpoints
        :raises ValueError: If ``waypoint`` shape is neither (3,) nor (3, 3).
        """
        arr = np.asarray(waypoint, dtype=float)
        if arr.ndim == 1 and arr.shape == (3,):
            native = self._native.generate(
                current_position, arr, feedforward_velocity,
                float(dt), bool(force_zero_velocity_setpoint))
        else:
            triplet = _coerce_triplet(waypoint)
            native = self._native.generate(
                current_position, triplet, feedforward_velocity,
                float(dt), bool(force_zero_velocity_setpoint))
        return TrajectorySetpoints._from_native(native)


# -- WaypointTrajectoryController ---------------------------------------------

class WaypointTrajectoryController:
    """
    High-level point-to-point waypoint controller.

    Wraps a ``JerkLimitedTrajectoryGenerator3D`` with the motion limits from a
    ``TrajectoryParameters`` bundle. The typical flow is:

        1. Build ``TrajectoryParameters`` with the desired limits.
        2. Construct the controller and call ``reset(initial_position)``.
        3. Each tick, call ``update(dt, current_position, waypoint)`` and use
           the returned ``TrajectorySetpoints``.
    """

    def __init__(self, parameters: TrajectoryParameters) -> None:
        """
        Construct a new waypoint controller with the given motion limits.

        :param parameters: Trajectory parameters (motion limits).
        :type parameters: TrajectoryParameters
        :raises ValueError: If all of ``max_speed``, ``max_acceleration`` and
            ``max_jerk`` are <= 0.
        """
        self._native = _WaypointTrajectoryController(parameters._parameters)

    @staticmethod
    def _from_native(
            native: _WaypointTrajectoryController) -> 'WaypointTrajectoryController':
        """
        Build a wrapper from an existing native instance.

        :param native: Native binding instance.
        :type native: _WaypointTrajectoryController
        :return: Wrapper sharing the underlying state.
        :rtype: WaypointTrajectoryController
        """
        wrapper = WaypointTrajectoryController.__new__(WaypointTrajectoryController)
        wrapper._native = native
        return wrapper

    def reset(self,
              initial_position: np.ndarray,
              initial_velocity: Optional[np.ndarray] = None,
              initial_acceleration: Optional[np.ndarray] = None) -> None:
        """
        Reset the internal integrator to an explicit initial kinematic state.

        Two dispatch paths:

        * Position only (legacy): ``reset(p)`` seeds zero velocity and zero
          acceleration. Bit-for-bit identical to the previous behaviour.
        * Position + velocity (and optional acceleration): seeds the integrator
          with the supplied kinematic state. Useful for chaining a new
          trajectory onto a vehicle already in flight.

        Out-of-bounds values are accepted verbatim; the post-clip stage of
        the next ``update()`` call will clamp them to the parameters' bounds.
        Note that the reactive integrator does not propagate ``initial_acceleration``
        into the very first emitted setpoint (T1=T2=T3=0 immediately after
        reset, so the post-trajectory branch reports zero acceleration on the
        first tick); the seeded value is still recorded internally and shapes
        subsequent ticks.

        :param initial_position: Initial position (m, world frame).
        :type initial_position: np.ndarray
        :param initial_velocity: Optional initial velocity (m/s, world frame).
            ``None`` (default) means zero velocity (legacy behaviour).
        :type initial_velocity: np.ndarray or None
        :param initial_acceleration: Optional initial acceleration (m/s^2,
            world frame). ``None`` (default) means zero.
        :type initial_acceleration: np.ndarray or None
        :rtype: None
        """
        if initial_velocity is None and initial_acceleration is None:
            # Legacy path: bit-for-bit identical to the position-only overload.
            self._native.reset(initial_position)
            return
        wp = _NativeWaypoint(np.asarray(initial_position, dtype=float))
        if initial_velocity is not None:
            wp.velocity = np.asarray(initial_velocity, dtype=float)
        if initial_acceleration is not None:
            wp.acceleration = np.asarray(initial_acceleration, dtype=float)
        self._native.reset(wp)

    def update(
            self,
            dt: float,
            current_position: np.ndarray,
            waypoint: Union[np.ndarray, Sequence]) -> TrajectorySetpoints:
        """
        Advance the controller by one tick.

        Single- or multi-waypoint depending on the shape of ``waypoint``:
          * Shape ``(3,)`` -> single-waypoint (legacy behaviour).
          * Shape ``(3, 3)`` or sequence of three ``(3,)`` arrays -> multi-waypoint
            ``{prev, target, next}`` triplet. The caller is responsible for
            rotating the triplet (prev = old target, target = old next, ...)
            once the vehicle has crossed the current target's acceptance radius.

        :param dt: Integration step (s).
        :type dt: float
        :param current_position: Current vehicle position (m, world frame).
        :type current_position: np.ndarray
        :param waypoint: Target waypoint (single) or triplet (multi).
        :type waypoint: np.ndarray or sequence
        :return: Trajectory setpoints for this tick.
        :rtype: TrajectorySetpoints
        :raises ValueError: If ``waypoint`` shape is neither (3,) nor (3, 3).
        """
        arr = np.asarray(waypoint, dtype=float)
        if arr.ndim == 1 and arr.shape == (3,):
            native = self._native.update(float(dt), current_position, arr)
        else:
            triplet = _coerce_triplet(waypoint)
            native = self._native.update(float(dt), current_position, triplet)
        return TrajectorySetpoints._from_native(native)

    @property
    def parameters(self) -> TrajectoryParameters:
        """
        Get the trajectory parameters currently in use.

        :rtype: TrajectoryParameters
        """
        return TrajectoryParameters._from_native(self._native.parameters())


__all__: List[str] = [
    'JerkLimitedTrajectoryGenerator3D',
    'JerkLimitedVelocityProfile1D',
    'TrajectoryParameters',
    'TrajectorySetpoints',
    'WaypointTrajectoryController',
]
