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
 * @file waypoint_controller.hpp
 *
 * High-level single-waypoint trajectory controller.
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#ifndef TRAJECTORY_GENERATOR_JERK_LIMITED__WAYPOINT_CONTROLLER_HPP_
#define TRAJECTORY_GENERATOR_JERK_LIMITED__WAYPOINT_CONTROLLER_HPP_

#include <array>

#include <Eigen/Core>

#include "trajectory_generator_jerk_limited/parameters.hpp"
#include "trajectory_generator_jerk_limited/trajectory_generator_3d.hpp"
#include "trajectory_generator_jerk_limited/types.hpp"

namespace trajectory_generator_jerk_limited {

/**
 * @brief Wraps a @ref JerkLimitedTrajectoryGenerator3D with a
 *        @ref TrajectoryParameters set.
 */
class WaypointTrajectoryController {
  using Vector3 = Eigen::Vector3d;

public:
  /**
   * @brief Construct the controller with a parameter set.
   *
   * @param parameters  Motion limits. Copied and kept internally.
   * @throws std::invalid_argument If max speed, acceleration and jerk are all disabled.
   */
  explicit WaypointTrajectoryController(const TrajectoryParameters& parameters);

  /**
   * @brief Reset the internal generator to @p initial_position, zero velocity
   *        and zero acceleration.
   *
   * @param initial_position  Starting position (m).
   */
  void reset(const Vector3& initial_position);

  /**
   * @brief Reset the internal generator to the kinematic state carried by a
   *        @ref Waypoint.
   *
   * The waypoint's `position` is always honoured. Its optional `velocity` and
   * `acceleration` fields, when set, seed the integrator's initial velocity
   * and acceleration; when unset, both default to zero. Use this overload to
   * chain a new trajectory onto a vehicle that is already in flight without
   * introducing discontinuities in the velocity setpoint at the join.
   *
   * The seeded values are written verbatim into the per-axis profile state.
   * They are NOT pre-clipped against @p TrajectoryParameters::max_speed or
   * @p TrajectoryParameters::max_acceleration: if the supplied state exceeds
   * those bounds, the post-clip + re-inject step inside the next
   * @ref update() call will clamp it consistently with the rest of the
   * library's contract. The first emitted setpoint will therefore reflect the
   * clamped state, not the supplied one.
   *
   * **Caveat on the seeded acceleration**: the underlying S-curve integrator
   * is *reactive*: it derives every tick's polynomial phase durations from
   * the current state and a freshly computed velocity setpoint. Immediately
   * after `reset()` no S-curve phase is active yet (T1 = T2 = T3 = 0), so
   * the first emitted acceleration falls into the integrator's
   * post-trajectory branch and is reported as zero, not the seeded value.
   * The seeded value is still written into the internal state so subsequent
   * calls to @ref update() compute the next S-curve phase from a consistent
   * starting point; it just does not appear directly in the first setpoint.
   * For flows where matching the initial acceleration of a previous trajectory
   * is critical, prefer chaining a few warm-up ticks before reading the
   * emitted acceleration. The first emitted *velocity* and *position* always
   * honour the seeded values.
   *
   * @param initial_state  Initial kinematic state. `position` is required;
   *                       `velocity` and `acceleration` default to zero when
   *                       unset.
   */
  void reset(const Waypoint& initial_state);

  /**
   * @brief Advance the trajectory by one tick.
   *
   * The generator is lazily initialised to @p current_position on the first
   * call, so the caller does not need to invoke reset() explicitly.
   *
   * @param dt                Time step (s).
   * @param current_position  Vehicle position, used for the time-stretch gate (m).
   * @param waypoint          Target position (m).
   * @return Smoothed setpoints for this tick.
   */
  TrajectorySetpoints update(const double dt,
                             const Vector3& current_position,
                             const Vector3& waypoint);

  /**
   * @brief Multi-waypoint overload that consumes a {prev, target, next} triplet.
   *
   * Activates the L1 corner-cut and the corner-speed cap in the underlying
   * generator. The caller is responsible for rotating the triplet (e.g.
   * setting prev = old_target, target = old_next, next = next_next) once the
   * vehicle has crossed the current target's acceptance radius; the generator
   * itself is stateless about that scheduling.
   *
   * @param dt                Time step (s).
   * @param current_position  Vehicle position, used for the time-stretch gate (m).
   * @param waypoints         Triplet {prev, target, next} (m).
   * @return Smoothed setpoints for this tick.
   */
  TrajectorySetpoints update(const double dt,
                             const Vector3& current_position,
                             const std::array<Vector3, 3>& waypoints);

  // Access the current parameters.
  const TrajectoryParameters& parameters() const { return parameters_; }

private:
  TrajectoryParameters parameters_;
  JerkLimitedTrajectoryGenerator3D generator_;
  bool is_initialized_ = false;
};

}  // namespace trajectory_generator_jerk_limited

#endif  // TRAJECTORY_GENERATOR_JERK_LIMITED__WAYPOINT_CONTROLLER_HPP_
