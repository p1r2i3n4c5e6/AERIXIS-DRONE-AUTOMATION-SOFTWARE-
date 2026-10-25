// Copyright 2025 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * @file trajectory_generator_3d.hpp
 *
 * Three-axis jerk-limited trajectory generator toward a single waypoint.
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#ifndef TRAJECTORY_GENERATOR_JERK_LIMITED__TRAJECTORY_GENERATOR_3D_HPP_
#define TRAJECTORY_GENERATOR_JERK_LIMITED__TRAJECTORY_GENERATOR_3D_HPP_

#include <array>

#include <Eigen/Core>

#include "trajectory_generator_jerk_limited/velocity_profile_1d.hpp"

namespace trajectory_generator_jerk_limited {

/**
 * @brief Per-tick smoothed setpoints emitted by the 3D generator.
 *
 * The @p unsmoothed_velocity field is the instantaneous velocity command
 * before the per-axis S-curve filter, useful for diagnostics.
 */
struct TrajectorySetpoints {
  using Vector3 = Eigen::Vector3d;

  Vector3 jerk                = Vector3::Zero();
  Vector3 acceleration        = Vector3::Zero();
  Vector3 velocity            = Vector3::Zero();
  Vector3 position            = Vector3::Zero();
  Vector3 unsmoothed_velocity = Vector3::Zero();
};

/**
 * @brief Coordinates three jerk-limited 1D profiles (one per ENU axis) to
 *        produce a smooth 3D trajectory from the current state to a waypoint.
 *
 * On each tick the generator:
 *  1. Computes a feasible velocity command clamped by the global 3D braking
 *     distance bound and by a global speed bound.
 *  2. Integrates the polynomial of each 1D profile, with an optional time
 *     stretch based on the 3D tracking error.
 *  3. Recomputes phase durations for the new velocity command and synchronizes
 *     the three profiles so that all axes finish together.
 */
class JerkLimitedTrajectoryGenerator3D {
  using Vector3 = Eigen::Vector3d;

public:
  JerkLimitedTrajectoryGenerator3D() = default;

  // --- Limit setters --------------------------------------------------------

  /**
   * @brief Set the same maximum jerk on all three axes.
   *
   * @param jerk  Maximum jerk magnitude (m/s^3). <= 0 disables the jerk limit.
   */
  void setMaxJerk(const double jerk);

  /**
   * @brief Set the strict bound on 3D speed norm.
   *
   * @param speed  Maximum speed norm (m/s). <= 0 disables the speed bound.
   */
  void setMaxSpeed(const double speed);

  /**
   * @brief Set the strict bound on 3D acceleration norm.
   *
   * @param acceleration  Maximum acceleration norm (m/s^2). <= 0 disables the acceleration bound.
   */
  void setMaxAcceleration(const double acceleration);

  /**
   * @brief Set the 3D tracking error above which the trajectory is
   *        time-stretched to wait for the vehicle.
   *
   * @param error  Tracking error threshold (m). <= 0 disables the gate.
   */
  void setMaxTrackingError(const double error);

  /**
   * @brief Set the multi-waypoint acceptance radius used by the corner-speed
   *        cap and the L1 corner-cut crossing point.
   *
   * Has no effect on single-waypoint calls (their generate/update overloads
   * bypass the corner logic). <= 0 disables corner handling entirely so
   * multi-waypoint calls degrade to the single-waypoint behaviour.
   *
   * @param radius  Acceptance radius (m).
   */
  void setAcceptanceRadius(const double radius);

  /**
   * @brief Set the dimensionless gain that scales max_acceleration inside the
   *        corner-speed computation for the multi-waypoint path.
   *
   * Has no effect on single-waypoint calls. < 0 is clamped to 0.
   *
   * @param gain  Trajectory gain (dimensionless).
   */
  void setTrajectoryGain(const double gain);

  // --- State getters --------------------------------------------------------

  Vector3 currentPosition() const;
  Vector3 currentVelocity() const;
  double currentSpeed() const;

  /**
   * @brief Reset the generator to an explicit kinematic state.
   *
   * @param acceleration  Initial acceleration (m/s^2).
   * @param velocity      Initial velocity (m/s).
   * @param position      Initial position (m).
   */
  void reset(const Vector3& acceleration, const Vector3& velocity, const Vector3& position);

  /**
   * @brief Advance the trajectory by @p dt and return the emitted setpoints.
   *
   * @param current_position             Vehicle position, used for the time-stretch gate (m).
   * @param waypoint                     Target position (m).
   * @param feedforward_velocity         Feedforward velocity added on top (m/s).
   * @param dt                           Integration step (s).
   * @param force_zero_velocity_setpoint When true, the inner command becomes zero (stop).
   * @return Smoothed setpoints for this tick.
   * @throws std::invalid_argument If max speed, acceleration and jerk are all disabled.
   */
  TrajectorySetpoints generate(const Vector3& current_position,
                               const Vector3& waypoint,
                               const Vector3& feedforward_velocity,
                               const double dt,
                               const bool force_zero_velocity_setpoint = false);

  /**
   * @brief Multi-waypoint overload that consumes a {prev, target, next} triplet.
   *
   * Activates the L1 corner-cut, the corner-speed cap (computeSpeedFromWaypoints
   * with the @ref acceptance_radius and @ref trajectory_gain parameters) and the
   * speed lock that prevents re-acceleration in the middle of a turn.
   *
   * Single-waypoint behaviour can be restored at runtime by passing
   * {wp, wp, wp} or by leaving @ref acceptance_radius at 0.
   *
   * @param current_position             Vehicle position, used for the time-stretch gate (m).
   * @param waypoints                    Triplet {prev, target, next} (m).
   * @param feedforward_velocity         Feedforward velocity added on top (m/s).
   * @param dt                           Integration step (s).
   * @param force_zero_velocity_setpoint When true, the inner command becomes zero (stop).
   * @return Smoothed setpoints for this tick.
   * @throws std::invalid_argument If max speed, acceleration and jerk are all disabled.
   */
  TrajectorySetpoints generate(const Vector3& current_position,
                               const std::array<Vector3, 3>& waypoints,
                               const Vector3& feedforward_velocity,
                               const double dt,
                               const bool force_zero_velocity_setpoint = false);

private:
  // Common implementation for single- and multi-waypoint generate overloads.
  // When @p is_single_waypoint is true, the corner-cut and corner-speed lock
  // branches are bypassed and the single-waypoint path is taken (waypoints[1]
  // is the only waypoint consulted), guaranteeing bit-for-bit regression
  // against the pre-multi-waypoint behaviour.
  TrajectorySetpoints generateImpl(const Vector3& current_position,
                                   const std::array<Vector3, 3>& waypoints,
                                   const Vector3& feedforward_velocity,
                                   const double dt,
                                   const bool force_zero_velocity_setpoint,
                                   const bool is_single_waypoint);

  Vector3 computeVelocitySetpoint(const Vector3& waypoint,
                                  const Vector3& feedforward_velocity,
                                  const double dt) const;

  // Multi-waypoint variant. Computes the unsmoothed velocity setpoint using
  // the L1 crossing point and the multi-waypoint speed cap. Mutates
  // max_speed_previous_ to apply the in-turn speed lock.
  Vector3 computeVelocitySetpointMulti(const std::array<Vector3, 3>& waypoints,
                                       const Vector3& current_position,
                                       const Vector3& feedforward_velocity,
                                       const double dt);

  // Returns true when the vehicle is heading away from the straight-line
  // direction towards the target waypoint and far enough from it to justify
  // applying the L1 corner-cut. Convention follows PX4 PositionSmoothing.
  bool isTurning(const std::array<Vector3, 3>& waypoints, const Vector3& current_position) const;

  // L1-style guidance crossing point on the segment prev -> target,
  // projected from current_position and advanced by the L1 radius.
  Vector3 getL1Point(const Vector3& current_position,
                     const std::array<Vector3, 3>& waypoints) const;

  // Returns the target waypoint when the vehicle is not turning and the
  // L1 point otherwise. Used as the effective target for the unsmoothed
  // velocity command in the multi-waypoint path.
  Vector3 getCrossingPoint(const Vector3& current_position,
                           const std::array<Vector3, 3>& waypoints) const;

  void integrateTrajectory(const Vector3& current_position,
                           const Vector3& velocity_setpoint,
                           const double dt,
                           TrajectorySetpoints& out);

  // One 1D profile per ENU axis (X = 0, Y = 1, Z = 2).
  std::array<JerkLimitedVelocityProfile1D, 3> profiles_ = {};

  double max_speed_          = 0.0;
  double max_acceleration_   = 0.0;
  double max_tracking_error_ = 0.0;

  // Multi-waypoint state. Inert when @ref setAcceptanceRadius is left at zero.
  double acceptance_radius_  = 0.0;
  double trajectory_gain_    = 1.0;
  double max_speed_previous_ = 0.0;
};

}  // namespace trajectory_generator_jerk_limited

#endif  // TRAJECTORY_GENERATOR_JERK_LIMITED__TRAJECTORY_GENERATOR_3D_HPP_
