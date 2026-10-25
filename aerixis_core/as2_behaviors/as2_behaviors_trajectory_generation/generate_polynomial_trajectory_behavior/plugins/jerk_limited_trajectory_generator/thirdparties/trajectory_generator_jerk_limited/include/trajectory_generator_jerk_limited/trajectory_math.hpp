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
 * @file trajectory_math.hpp
 *
 * Free-standing math helpers used by the trajectory generator (sign, unit
 * vector, braking-distance speed bound and 3D magnitude clamp).
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#ifndef TRAJECTORY_GENERATOR_JERK_LIMITED__TRAJECTORY_MATH_HPP_
#define TRAJECTORY_GENERATOR_JERK_LIMITED__TRAJECTORY_MATH_HPP_

#include <array>

#include <Eigen/Core>

namespace trajectory_generator_jerk_limited {

/// Small tolerance for near-zero comparisons (float32 epsilon).
inline constexpr double kEpsilon = 1.1920929e-7;

/**
 * @brief Integer sign of a scalar value: +1, -1 or 0.
 *
 * @param value  Input scalar.
 * @return +1 if value > 0, -1 if value < 0, 0 otherwise.
 */
int signInt(const double value);

/**
 * @brief Return the unit vector of @p v, or the zero vector if @p v is tiny.
 *
 * @param v  Input 3D vector.
 * @return v / ||v|| when ||v|| > kEpsilon, otherwise (0, 0, 0).
 */
Eigen::Vector3d unitOrZero(const Eigen::Vector3d& v);

/**
 * @brief Maximum speed that still allows decelerating to @p final_speed over
 *        @p braking_distance with jerk @p jerk and acceleration @p acceleration.
 *
 * Assumes the constant-acceleration braking model with the standard
 * 2 * a / j jerk-ramp delay so the profile is feasible.
 *
 * @param jerk              Maximum jerk magnitude (m/s^3). Non-positive returns final_speed.
 * @param acceleration      Maximum acceleration magnitude (m/s^2). Non-positive returns
 * final_speed.
 * @param braking_distance  Remaining distance to brake (m).
 * @param final_speed       Target final speed (m/s).
 * @return Feasible maximum speed (m/s), clamped below by final_speed.
 */
double maxSpeedFromBrakingDistance(const double jerk,
                                   const double acceleration,
                                   const double braking_distance,
                                   const double final_speed);

/**
 * @brief Clamp the Euclidean norm of a 3D vector while preserving direction.
 *
 * If ||v|| exceeds @p max_norm, the whole vector is scaled so ||v|| equals
 * @p max_norm. If @p max_norm is non-positive, @p v becomes zero.
 *
 * @param v         [in,out] Vector to clamp in-place.
 * @param max_norm  Maximum allowed norm (m/s or m/s^2).
 * @param accuracy  Tolerance for near-zero comparisons.
 */
void clampVectorNorm(Eigen::Vector3d& v, const double max_norm, const double accuracy = kEpsilon);

/**
 * @brief Maximum tangential speed allowed when crossing a waypoint connecting
 *        two segments that form a "V" shape with internal angle @p alpha.
 *
 * Models the waypoint as the apex of a tangent circle that touches both
 * segments at the acceptance radius. The result is sqrt(a * d * tan(alpha/2))
 * where a is @p max_acceleration (lateral) and d is @p acceptance_radius.
 *
 * Convention for @p alpha: the angle between the two vectors that leave the
 * waypoint towards the previous and the next waypoint, i.e.
 *   alpha = acos(((target - prev).normalized()).dot((target - next).normalized()))
 * so a straight-line path gives alpha = pi (no curvature, no restriction)
 * and a sharp 180-degree reversal gives alpha = 0 (must stop).
 *
 * @param alpha               Angle between the V-shape segments (rad), in [0, pi].
 * @param max_acceleration    Maximum lateral acceleration (m/s^2). <= 0 disables.
 * @param acceptance_radius   Tangent circle radius (m). <= 0 disables.
 * @return Maximum tangential speed (m/s). +infinity when the constraint is
 *         disabled or the path is straight; 0 when alpha is at or beyond pi
 *         (sharp reversal).
 */
double computeMaxSpeedInWaypoint(const double alpha,
                                 const double max_acceleration,
                                 const double acceptance_radius);

/**
 * @brief Maximum speed at @p start that still allows reaching @p target with
 *        @p exit_speed and respecting the geometry of the next turn (target -> next).
 *
 * Combines three caps and returns their minimum:
 *  - @p max_speed (when > 0).
 *  - Corner-speed at @p target via @ref computeMaxSpeedInWaypoint with the
 *    angle between (target - start) and (target - next).
 *  - Braking-speed via @ref maxSpeedFromBrakingDistance over ||target - start||
 *    with @p exit_speed as the post-braking final speed.
 *
 * @p trajectory_gain is NOT applied here; the caller scales @p max_acceleration
 * if needed.
 *
 * @param start              Position from which the speed is computed (m).
 * @param target             Waypoint to be crossed next (m).
 * @param next               Waypoint following @p target (m).
 * @param exit_speed         Required speed at @p target (m/s). Use 0 for stop.
 * @param max_jerk           Jerk bound (m/s^3). <= 0 disables the braking cap.
 * @param max_acceleration   Acceleration bound (m/s^2). <= 0 disables both caps.
 * @param max_speed          Speed bound (m/s). <= 0 disables this cap.
 * @param acceptance_radius  Acceptance radius for the corner cap (m). <= 0 disables it.
 * @return Minimum of the three (or two) active caps (m/s). +infinity if all
 *         caps are disabled.
 */
double computeStartSpeedFromWaypoints(const Eigen::Vector3d& start,
                                      const Eigen::Vector3d& target,
                                      const Eigen::Vector3d& next,
                                      const double exit_speed,
                                      const double max_jerk,
                                      const double max_acceleration,
                                      const double max_speed,
                                      const double acceptance_radius);

/**
 * @brief Backward sweep over a triplet {start, target, next} returning the
 *        maximum allowed speed at @p waypoints[0].
 *
 * Step 1: compute the speed at @p waypoints[1] that allows decelerating from
 *         there to @p waypoints[2] with final speed @p exit_speed.
 * Step 2: compute the speed at @p waypoints[0] that allows decelerating from
 *         there to @p waypoints[1] with the speed obtained in step 1.
 *
 * The sweep is fixed at N=3 to match the {prev, target, next} triplet API
 * exposed by the multi-waypoint trajectory generator.
 *
 * @param waypoints          Triplet {start, target, next} (m).
 * @param max_jerk           Jerk bound (m/s^3). <= 0 disables the braking caps.
 * @param max_acceleration   Acceleration bound (m/s^2). <= 0 disables corner+braking caps.
 * @param max_speed          Speed bound (m/s). <= 0 disables this cap.
 * @param acceptance_radius  Acceptance radius for the corner cap (m). <= 0 disables it.
 * @param exit_speed         Required speed at @p waypoints[2] (m/s). Default 0 (stop).
 * @return Maximum allowed speed at @p waypoints[0] (m/s).
 */
double computeSpeedFromWaypoints(const std::array<Eigen::Vector3d, 3>& waypoints,
                                 const double max_jerk,
                                 const double max_acceleration,
                                 const double max_speed,
                                 const double acceptance_radius,
                                 const double exit_speed = 0.0);

}  // namespace trajectory_generator_jerk_limited

#endif  // TRAJECTORY_GENERATOR_JERK_LIMITED__TRAJECTORY_MATH_HPP_
