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
 * @file trajectory_generator_3d.cpp
 *
 * Three-axis jerk-limited trajectory generator implementation.
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#include "trajectory_generator_jerk_limited/trajectory_generator_3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

#include "trajectory_generator_jerk_limited/trajectory_math.hpp"

namespace trajectory_generator_jerk_limited {

void JerkLimitedTrajectoryGenerator3D::setMaxJerk(const double jerk) {
  const double clamped_jerk = std::max(0.0, jerk);
  for (auto& profile : profiles_) {
    profile.setMaxJerk(clamped_jerk);
  }
}

void JerkLimitedTrajectoryGenerator3D::setMaxSpeed(const double speed) {
  max_speed_ = std::max(0.0, speed);
  for (auto& profile : profiles_) {
    profile.setMaxVelocity(max_speed_);
  }
  // Keep the multi-waypoint speed lock consistent with the new ceiling.
  if (max_speed_previous_ <= 0.0 || max_speed_previous_ > max_speed_) {
    max_speed_previous_ = max_speed_;
  }
}

void JerkLimitedTrajectoryGenerator3D::setMaxAcceleration(const double acceleration) {
  max_acceleration_ = std::max(0.0, acceleration);
  for (auto& profile : profiles_) {
    profile.setMaxAcceleration(max_acceleration_);
  }
}

void JerkLimitedTrajectoryGenerator3D::setMaxTrackingError(const double error) {
  max_tracking_error_ = std::max(0.0, error);
}

void JerkLimitedTrajectoryGenerator3D::setAcceptanceRadius(const double radius) {
  acceptance_radius_ = std::max(0.0, radius);
}

void JerkLimitedTrajectoryGenerator3D::setTrajectoryGain(const double gain) {
  trajectory_gain_ = std::max(0.0, gain);
}

JerkLimitedTrajectoryGenerator3D::Vector3 JerkLimitedTrajectoryGenerator3D::currentPosition()
    const {
  return Vector3(profiles_[0].currentPosition(), profiles_[1].currentPosition(),
                 profiles_[2].currentPosition());
}

JerkLimitedTrajectoryGenerator3D::Vector3 JerkLimitedTrajectoryGenerator3D::currentVelocity()
    const {
  return Vector3(profiles_[0].currentVelocity(), profiles_[1].currentVelocity(),
                 profiles_[2].currentVelocity());
}

double JerkLimitedTrajectoryGenerator3D::currentSpeed() const { return currentVelocity().norm(); }

void JerkLimitedTrajectoryGenerator3D::reset(const Vector3& acceleration,
                                             const Vector3& velocity,
                                             const Vector3& position) {
  for (std::size_t i = 0; i < 3; ++i) {
    profiles_[i].reset(acceleration(i), velocity(i), position(i));
  }
  // Re-arm the multi-waypoint speed lock to the current speed ceiling.
  max_speed_previous_ = max_speed_;
}

JerkLimitedTrajectoryGenerator3D::Vector3 JerkLimitedTrajectoryGenerator3D::computeVelocitySetpoint(
    const Vector3& waypoint,
    const Vector3& feedforward_velocity,
    const double dt) const {
  // Direction is computed from the generator's internal position, not the
  // vehicle's real one, to keep the command path causal.
  const Vector3 trajectory_position = currentPosition();
  const Vector3 error               = waypoint - trajectory_position;

  const double max_jerk                = profiles_[0].maxJerk();
  const bool speed_limit_active        = max_speed_ > kEpsilon;
  const bool acceleration_limit_active = max_acceleration_ > kEpsilon;
  const bool jerk_limit_active         = max_jerk > kEpsilon;
  const bool braking_limit_active      = jerk_limit_active && acceleration_limit_active;

  Vector3 constrained_velocity = Vector3::Zero();
  if (speed_limit_active || braking_limit_active) {
    double constrained_speed = std::numeric_limits<double>::infinity();
    if (speed_limit_active) {
      constrained_speed = std::min(constrained_speed, max_speed_);
    }
    if (braking_limit_active) {
      const double distance_to_waypoint = error.norm();
      const double speed_from_braking =
          maxSpeedFromBrakingDistance(max_jerk, max_acceleration_, distance_to_waypoint, 0.0);
      constrained_speed = std::min(constrained_speed, speed_from_braking);
    }
    constrained_velocity = unitOrZero(error) * constrained_speed;
  } else {
    const double safe_dt = std::max(dt, kEpsilon);
    constrained_velocity = error / safe_dt;
  }

  Vector3 velocity_setpoint = feedforward_velocity;
  for (std::size_t i = 0; i < 3; ++i) {
    if (std::isfinite(velocity_setpoint(i))) {
      velocity_setpoint(i) += constrained_velocity(i);
    } else {
      velocity_setpoint(i) = constrained_velocity(i);
    }
  }
  if (speed_limit_active) {
    clampVectorNorm(velocity_setpoint, max_speed_);
  }
  return velocity_setpoint;
}

void JerkLimitedTrajectoryGenerator3D::integrateTrajectory(const Vector3& current_position,
                                                           const Vector3& velocity_setpoint,
                                                           const double dt,
                                                           TrajectorySetpoints& out) {
  if (!velocity_setpoint.allFinite()) {
    return;
  }

  const Vector3 trajectory_position = currentPosition();
  const Vector3 trajectory_velocity = currentVelocity();
  const Vector3 delta               = trajectory_position - current_position;
  const double tracking_error       = delta.norm();
  const double dot_with_velocity    = delta.dot(trajectory_velocity);

  double time_stretch = 1.0;
  if (max_tracking_error_ > kEpsilon && dot_with_velocity >= 0.0) {
    time_stretch = 1.0 - std::clamp(tracking_error / max_tracking_error_, 0.0, 1.0);
  }
  if (dot_with_velocity < 0.0) {
    time_stretch = 1.0;
  }

  for (std::size_t i = 0; i < 3; ++i) {
    if (profiles_[i].maxJerk() <= kEpsilon) {
      profiles_[i].updateDurations(velocity_setpoint(i));
    }
    profiles_[i].updateTrajectory(dt, time_stretch);
    out.jerk(i)         = profiles_[i].currentJerk();
    out.acceleration(i) = profiles_[i].currentAcceleration();
    out.velocity(i)     = profiles_[i].currentVelocity();
    out.position(i)     = profiles_[i].currentPosition();
  }

  // Hard global 3D bounds on emitted setpoints.
  const bool speed_limit_active        = max_speed_ > kEpsilon;
  const bool acceleration_limit_active = max_acceleration_ > kEpsilon;
  if (speed_limit_active) {
    clampVectorNorm(out.velocity, max_speed_);
  }
  if (acceleration_limit_active) {
    clampVectorNorm(out.acceleration, max_acceleration_);
  }

  // Keep internal profile state consistent with the clamped output.
  for (std::size_t i = 0; i < 3; ++i) {
    profiles_[i].setCurrentVelocity(out.velocity(i));
    profiles_[i].setCurrentAcceleration(out.acceleration(i));
  }

  for (std::size_t i = 0; i < 3; ++i) {
    profiles_[i].updateDurations(velocity_setpoint(i));
  }

  std::vector<JerkLimitedVelocityProfile1D*> profile_pointers = {&profiles_[0], &profiles_[1],
                                                                 &profiles_[2]};
  JerkLimitedVelocityProfile1D::synchronize(profile_pointers);
}

TrajectorySetpoints JerkLimitedTrajectoryGenerator3D::generate(
    const Vector3& current_position,
    const Vector3& waypoint,
    const Vector3& feedforward_velocity,
    const double dt,
    const bool force_zero_velocity_setpoint) {
  // Single-waypoint API: delegate to the common implementation with the
  // is_single_waypoint flag set, which bypasses the multi-waypoint branches
  // and guarantees bit-for-bit regression against the pre-refactor behaviour.
  const std::array<Vector3, 3> waypoints = {waypoint, waypoint, waypoint};
  return generateImpl(current_position, waypoints, feedforward_velocity, dt,
                      force_zero_velocity_setpoint, /*is_single_waypoint=*/true);
}

TrajectorySetpoints JerkLimitedTrajectoryGenerator3D::generate(
    const Vector3& current_position,
    const std::array<Vector3, 3>& waypoints,
    const Vector3& feedforward_velocity,
    const double dt,
    const bool force_zero_velocity_setpoint) {
  return generateImpl(current_position, waypoints, feedforward_velocity, dt,
                      force_zero_velocity_setpoint, /*is_single_waypoint=*/false);
}

TrajectorySetpoints JerkLimitedTrajectoryGenerator3D::generateImpl(
    const Vector3& current_position,
    const std::array<Vector3, 3>& waypoints,
    const Vector3& feedforward_velocity,
    const double dt,
    const bool force_zero_velocity_setpoint,
    const bool is_single_waypoint) {
  const bool speed_limit_disabled        = max_speed_ <= kEpsilon;
  const bool acceleration_limit_disabled = max_acceleration_ <= kEpsilon;
  const bool jerk_limit_disabled         = profiles_[0].maxJerk() <= kEpsilon;
  if (speed_limit_disabled && acceleration_limit_disabled && jerk_limit_disabled) {
    throw std::invalid_argument(
        "Invalid trajectory limits: max_speed, max_acceleration and max_jerk cannot all be <= 0.");
  }

  // The multi-waypoint logic is opt-in: it requires both a non-trivial triplet
  // (target != next or target != prev) and a positive acceptance_radius.
  // Otherwise the call degrades cleanly to the single-waypoint path so the
  // caller never has to worry about parameters interacting at the boundary.
  const bool multi_waypoint_active =
      !is_single_waypoint && acceptance_radius_ > kEpsilon &&
      ((waypoints[0] - waypoints[1]).squaredNorm() > kEpsilon * kEpsilon ||
       (waypoints[2] - waypoints[1]).squaredNorm() > kEpsilon * kEpsilon);

  if (!multi_waypoint_active) {
    // Single-waypoint code path. The lock is re-armed so a subsequent
    // multi-waypoint call does not inherit a stale value.
    max_speed_previous_ = max_speed_;
  }

  TrajectorySetpoints out;
  Vector3 velocity_setpoint;
  if (force_zero_velocity_setpoint) {
    velocity_setpoint = Vector3::Zero();
  } else if (multi_waypoint_active) {
    velocity_setpoint =
        computeVelocitySetpointMulti(waypoints, current_position, feedforward_velocity, dt);
  } else {
    velocity_setpoint = computeVelocitySetpoint(waypoints[1], feedforward_velocity, dt);
  }
  out.unsmoothed_velocity = velocity_setpoint;
  integrateTrajectory(current_position, velocity_setpoint, dt, out);
  return out;
}

namespace {

// Tuning constants ported from PX4 PositionSmoothing.cpp.
inline constexpr double kTurningMinSpeed        = 0.2;   // [m/s]
inline constexpr double kTurningCosineThreshold = 0.98;  // dimensionless
inline constexpr double kL1MinRadius            = 5.0;   // [m]

}  // namespace

bool JerkLimitedTrajectoryGenerator3D::isTurning(const std::array<Vector3, 3>& waypoints,
                                                 const Vector3& current_position) const {
  const Vector3 vel       = currentVelocity();
  const double speed      = vel.norm();
  const Vector3 to_target = waypoints[1] - current_position;
  const double distance   = to_target.norm();

  if (speed <= kTurningMinSpeed || distance <= acceptance_radius_) {
    return false;
  }
  const Vector3 vel_unit       = unitOrZero(vel);
  const Vector3 to_target_unit = unitOrZero(to_target);
  const double cos_align       = vel_unit.dot(to_target_unit);
  return cos_align < kTurningCosineThreshold;
}

JerkLimitedTrajectoryGenerator3D::Vector3 JerkLimitedTrajectoryGenerator3D::getL1Point(
    const Vector3& current_position,
    const std::array<Vector3, 3>& waypoints) const {
  const Vector3 prev_to_target = waypoints[1] - waypoints[0];
  const double segment_length  = prev_to_target.norm();
  if (segment_length < kEpsilon) {
    // Degenerate prev == target: nothing to project on, fall back to target.
    return waypoints[1];
  }
  const Vector3 segment_unit       = prev_to_target / segment_length;
  const Vector3 prev_to_position   = current_position - waypoints[0];
  const double along_track         = prev_to_position.dot(segment_unit);
  const Vector3 closest_on_segment = waypoints[0] + along_track * segment_unit;
  const double cross_track         = (closest_on_segment - current_position).norm();

  const double l1_radius = std::max(acceptance_radius_, kL1MinRadius);
  double along_advance   = 0.0;
  if (l1_radius > cross_track) {
    along_advance = std::sqrt(l1_radius * l1_radius - cross_track * cross_track);
  }
  // PX4 returns the L1 point as-is, even past the target. Keeping the pull
  // beyond the target is what lets the trajectory keep momentum across the
  // corner; clamping here would force the vehicle to stop at the waypoint.
  return closest_on_segment + along_advance * segment_unit;
}

JerkLimitedTrajectoryGenerator3D::Vector3 JerkLimitedTrajectoryGenerator3D::getCrossingPoint(
    const Vector3& current_position,
    const std::array<Vector3, 3>& waypoints) const {
  if (!isTurning(waypoints, current_position)) {
    return waypoints[1];
  }
  return getL1Point(current_position, waypoints);
}

JerkLimitedTrajectoryGenerator3D::Vector3
JerkLimitedTrajectoryGenerator3D::computeVelocitySetpointMulti(
    const std::array<Vector3, 3>& waypoints,
    const Vector3& current_position,
    const Vector3& feedforward_velocity,
    const double dt) {
  const Vector3 trajectory_position = currentPosition();
  const Vector3 crossing            = getCrossingPoint(current_position, waypoints);
  const Vector3 error               = crossing - trajectory_position;

  const double max_jerk                = profiles_[0].maxJerk();
  const bool speed_limit_active        = max_speed_ > kEpsilon;
  const bool acceleration_limit_active = max_acceleration_ > kEpsilon;
  const bool jerk_limit_active         = max_jerk > kEpsilon;
  const bool braking_limit_active      = jerk_limit_active && acceleration_limit_active;

  // Fly-through vs stop discriminator: callers signal "stop at waypoints[1]"
  // by collapsing the triplet (waypoints[2] == waypoints[1]). When the next
  // waypoint exists distinctly, waypoints[1] is intermediate and the vehicle
  // should keep momentum across it. The exit speed at waypoints[1] is then
  // the corner cap derived from the V-shape (prev = waypoints[0],
  // next = waypoints[2]); for a collinear straight line alpha = pi yields an
  // unrestricted cap, so the carry-through speed is just max_speed.
  const bool stop_at_target   = (waypoints[2] - waypoints[1]).squaredNorm() < kEpsilon * kEpsilon;
  double exit_speed_at_target = 0.0;
  if (!stop_at_target) {
    const Vector3 to_prev      = unitOrZero(waypoints[0] - waypoints[1]);
    const Vector3 to_next      = unitOrZero(waypoints[2] - waypoints[1]);
    const double effective_acc = max_acceleration_ * trajectory_gain_;
    double corner_cap          = std::numeric_limits<double>::infinity();
    if (to_prev.squaredNorm() > kEpsilon && to_next.squaredNorm() > kEpsilon) {
      const double cos_alpha = std::clamp(to_prev.dot(to_next), -1.0, 1.0);
      const double alpha     = std::acos(cos_alpha);
      corner_cap             = computeMaxSpeedInWaypoint(alpha, effective_acc, acceptance_radius_);
    }
    exit_speed_at_target =
        std::isfinite(corner_cap) ? corner_cap : std::numeric_limits<double>::infinity();
    if (speed_limit_active) {
      exit_speed_at_target = std::min(exit_speed_at_target, max_speed_);
    }
    if (!std::isfinite(exit_speed_at_target)) {
      exit_speed_at_target = 0.0;
    }
  }

  // Multi-waypoint speed cap: the smaller of the corner geometry and the
  // straight-line braking distance to the actual target (not the L1 point);
  // the sweep is run on the original triplet so the corner at waypoints[2]
  // also contributes when relevant. trajectory_gain_ scales the lateral
  // acceleration the corner formula assumes. The sweep's exit_speed at
  // waypoints[2] is exit_speed_at_target as well: when waypoints[1] is a
  // fly-through there is no visibility past waypoints[2] either, so we
  // apply the same carry-through assumption.
  const double effective_corner_accel = max_acceleration_ * trajectory_gain_;
  const double sweep_speed = computeSpeedFromWaypoints(waypoints, max_jerk, effective_corner_accel,
                                                       /*max_speed=*/0.0, acceptance_radius_,
                                                       /*exit_speed=*/exit_speed_at_target);

  // Lock the speed during a turn so we never accelerate back up halfway
  // through the corner. Outside of turns the lock simply tracks the cap.
  double constrained_speed = std::numeric_limits<double>::infinity();
  if (speed_limit_active) {
    constrained_speed = std::min(constrained_speed, max_speed_);
  }
  if (std::isfinite(sweep_speed)) {
    constrained_speed = std::min(constrained_speed, sweep_speed);
  }
  if (isTurning(waypoints, current_position)) {
    constrained_speed = std::min(constrained_speed, max_speed_previous_);
  } else {
    max_speed_previous_ = std::isfinite(constrained_speed) ? constrained_speed : max_speed_;
  }

  Vector3 constrained_velocity = Vector3::Zero();
  if (speed_limit_active || braking_limit_active) {
    if (braking_limit_active) {
      const double distance_to_crossing = error.norm();
      const double speed_from_braking   = maxSpeedFromBrakingDistance(
            max_jerk, max_acceleration_, distance_to_crossing, exit_speed_at_target);
      constrained_speed = std::min(constrained_speed, speed_from_braking);
    }
    constrained_velocity = unitOrZero(error) * constrained_speed;
  } else {
    const double safe_dt = std::max(dt, kEpsilon);
    constrained_velocity = error / safe_dt;
  }

  Vector3 velocity_setpoint = feedforward_velocity;
  for (std::size_t i = 0; i < 3; ++i) {
    if (std::isfinite(velocity_setpoint(i))) {
      velocity_setpoint(i) += constrained_velocity(i);
    } else {
      velocity_setpoint(i) = constrained_velocity(i);
    }
  }
  if (speed_limit_active) {
    clampVectorNorm(velocity_setpoint, max_speed_);
  }
  return velocity_setpoint;
}

}  // namespace trajectory_generator_jerk_limited
