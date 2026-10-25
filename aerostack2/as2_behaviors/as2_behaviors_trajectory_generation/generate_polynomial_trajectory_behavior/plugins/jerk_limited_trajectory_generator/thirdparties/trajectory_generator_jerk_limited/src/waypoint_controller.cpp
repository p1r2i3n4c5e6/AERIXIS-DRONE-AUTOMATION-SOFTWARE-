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
 * @file waypoint_controller.cpp
 *
 * High-level single-waypoint trajectory controller implementation.
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#include "trajectory_generator_jerk_limited/waypoint_controller.hpp"

#include <stdexcept>

namespace trajectory_generator_jerk_limited {

WaypointTrajectoryController::WaypointTrajectoryController(const TrajectoryParameters& parameters)
    : parameters_(parameters) {
  if (parameters_.max_speed <= 0.0 && parameters_.max_acceleration <= 0.0 &&
      parameters_.max_jerk <= 0.0) {
    throw std::invalid_argument(
        "Invalid trajectory limits: max_speed, max_acceleration and max_jerk cannot all be <= 0.");
  }

  generator_.setMaxJerk(parameters_.max_jerk);
  generator_.setMaxSpeed(parameters_.max_speed);
  generator_.setMaxAcceleration(parameters_.max_acceleration);
  generator_.setMaxTrackingError(parameters_.max_tracking_error);
  generator_.setAcceptanceRadius(parameters_.acceptance_radius);
  generator_.setTrajectoryGain(parameters_.trajectory_gain);
}

void WaypointTrajectoryController::reset(const Vector3& initial_position) {
  reset(Waypoint(initial_position));
}

void WaypointTrajectoryController::reset(const Waypoint& initial_state) {
  generator_.reset(initial_state.acceleration.value_or(Vector3::Zero()),
                   initial_state.velocity.value_or(Vector3::Zero()), initial_state.position);
  is_initialized_ = true;
}

TrajectorySetpoints WaypointTrajectoryController::update(const double dt,
                                                         const Vector3& current_position,
                                                         const Vector3& waypoint) {
  if (!is_initialized_) {
    reset(current_position);
  }
  const Vector3 feedforward_velocity = Vector3::Zero();
  return generator_.generate(current_position, waypoint, feedforward_velocity, dt,
                             /*force_zero_velocity_setpoint=*/false);
}

TrajectorySetpoints WaypointTrajectoryController::update(const double dt,
                                                         const Vector3& current_position,
                                                         const std::array<Vector3, 3>& waypoints) {
  if (!is_initialized_) {
    reset(current_position);
  }
  const Vector3 feedforward_velocity = Vector3::Zero();
  return generator_.generate(current_position, waypoints, feedforward_velocity, dt,
                             /*force_zero_velocity_setpoint=*/false);
}

}  // namespace trajectory_generator_jerk_limited
