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
 * @file parameters.hpp
 *
 * Motion-limit parameters for the jerk-limited trajectory generator.
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#ifndef TRAJECTORY_GENERATOR_JERK_LIMITED__PARAMETERS_HPP_
#define TRAJECTORY_GENERATOR_JERK_LIMITED__PARAMETERS_HPP_

namespace trajectory_generator_jerk_limited {

/**
 * @brief Motion limits for the trajectory generator.
 *
 * Global 3D bounds for speed, acceleration and jerk, plus the 3D tracking
 * error threshold above which the trajectory is time-stretched to wait for
 * the vehicle. Units are SI throughout. Frame convention is ENU.
 *
 * Each limit is disabled when <= 0. At least one of max_speed, max_acceleration
 * or max_jerk must remain enabled, otherwise trajectory generation throws.
 *
 * The acceptance_radius and trajectory_gain fields only affect the
 * multi-waypoint generate / update overloads. Single-waypoint calls ignore
 * them, so existing single-waypoint configurations are unchanged.
 */
struct TrajectoryParameters {
  // Strict bound on 3D speed norm: ||v|| <= max_speed. <= 0 disables.
  double max_speed = 0.0;  // [m/s]

  // Strict bound on 3D acceleration norm: ||a|| <= max_acceleration. <= 0 disables.
  double max_acceleration = 0.0;  // [m/s^2]

  // Jerk limit applied to all three 1D profiles. <= 0 disables.
  double max_jerk = 0.0;  // [m/s^3]

  // Tracking gate: slow the trajectory when 3D lag grows too much. <= 0 disables.
  double max_tracking_error = 0.0;  // [m]

  // Multi-waypoint corner / L1 acceptance radius. <= 0 forces the multi-waypoint
  // path to behave as single-waypoint (no corner-cut, no corner-speed cap).
  double acceptance_radius = 0.0;  // [m]

  // Position tolerance used ONLY to declare termination at the last waypoint
  // (in TrajectoryGenerator::generate()). Decoupled from acceptance_radius so
  // that callers can keep a generous L1 corner-cut on intermediate waypoints
  // while still requiring the trajectory to settle tightly at the goal.
  // <= 0 disables the override and reuses max(acceptance_radius, internal
  // floor) as before, preserving the legacy behaviour.
  double final_acceptance_radius = 0.0;  // [m]

  // Multi-waypoint scale applied to max_acceleration inside the corner-speed
  // computation. > 1 allows tighter corners at higher speed; < 1 widens them.
  // Has no effect on single-waypoint calls.
  double trajectory_gain = 1.0;  // [-]
};

}  // namespace trajectory_generator_jerk_limited

#endif  // TRAJECTORY_GENERATOR_JERK_LIMITED__PARAMETERS_HPP_
