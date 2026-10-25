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
 * @file trajectory_math.cpp
 *
 * Math helpers implementation.
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#include "trajectory_generator_jerk_limited/trajectory_math.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace trajectory_generator_jerk_limited {

int signInt(const double value) {
  if (value > 0.0) return 1;
  if (value < 0.0) return -1;
  return 0;
}

Eigen::Vector3d unitOrZero(const Eigen::Vector3d& v) {
  const double norm = v.norm();
  if (norm < kEpsilon) {
    return Eigen::Vector3d::Zero();
  }
  return v / norm;
}

double maxSpeedFromBrakingDistance(const double jerk,
                                   const double acceleration,
                                   const double braking_distance,
                                   const double final_speed) {
  if (jerk <= kEpsilon || acceleration <= kEpsilon) {
    return final_speed;
  }
  const double b            = 4.0 * acceleration * acceleration / jerk;
  const double c            = -2.0 * acceleration * braking_distance - final_speed * final_speed;
  const double discriminant = b * b - 4.0 * c;
  if (discriminant < 0.0) {
    return final_speed;
  }
  const double max_speed = 0.5 * (-b + std::sqrt(discriminant));
  return std::max(max_speed, final_speed);
}

void clampVectorNorm(Eigen::Vector3d& v, const double max_norm, const double accuracy) {
  if (max_norm <= accuracy) {
    v.setZero();
    return;
  }

  const double norm = v.norm();
  if (norm <= max_norm + accuracy || norm <= accuracy) {
    return;
  }

  v *= (max_norm / norm);
}

double computeMaxSpeedInWaypoint(const double alpha,
                                 const double max_acceleration,
                                 const double acceptance_radius) {
  // Constraint disabled when either parameter is non-positive: the corner cap
  // contributes "no restriction" (caller takes the min over multiple caps).
  if (max_acceleration <= kEpsilon || acceptance_radius <= kEpsilon) {
    return std::numeric_limits<double>::infinity();
  }
  // Straight-line path: alpha = pi means the V-shape is fully open. Returning
  // infinity keeps the corner cap inert.
  if (alpha >= M_PI - kEpsilon) {
    return std::numeric_limits<double>::infinity();
  }
  // Sharp 180-degree reversal: alpha = 0 means tan(alpha/2) = 0, so the
  // tangent circle radius is zero: must stop. Guard separately because the
  // formula below also returns 0 here, but we keep the explicit branch for
  // numerical clarity at very small alpha.
  if (alpha <= kEpsilon) {
    return 0.0;
  }
  return std::sqrt(max_acceleration * acceptance_radius * std::tan(0.5 * alpha));
}

double computeStartSpeedFromWaypoints(const Eigen::Vector3d& start,
                                      const Eigen::Vector3d& target,
                                      const Eigen::Vector3d& next,
                                      const double exit_speed,
                                      const double max_jerk,
                                      const double max_acceleration,
                                      const double max_speed,
                                      const double acceptance_radius) {
  // Geometry of the V-shape at @p target. Convention: vectors LEAVE the target
  // toward the previous and next waypoints (PX4 convention). A straight line
  // gives alpha = pi, a sharp reversal alpha = 0.
  const Eigen::Vector3d to_prev      = unitOrZero(start - target);
  const Eigen::Vector3d to_next      = unitOrZero(next - target);
  const bool prev_segment_degenerate = to_prev.squaredNorm() < kEpsilon;
  const bool next_segment_degenerate = to_next.squaredNorm() < kEpsilon;

  // Step 1: speed permitted AT the target (not at start). The corner cap and
  // the incoming exit_speed are both expressed at the target; we take the
  // min so the trajectory stays feasible no matter which of the two bites.
  double speed_at_target = std::numeric_limits<double>::infinity();
  if (!prev_segment_degenerate && !next_segment_degenerate) {
    const double cos_alpha = std::clamp(to_prev.dot(to_next), -1.0, 1.0);
    const double alpha     = std::acos(cos_alpha);
    const double corner_speed =
        computeMaxSpeedInWaypoint(alpha, max_acceleration, acceptance_radius);
    speed_at_target = std::min(speed_at_target, corner_speed);
  }
  if (exit_speed >= 0.0) {
    speed_at_target = std::min(speed_at_target, exit_speed);
  }
  if (max_speed > kEpsilon) {
    speed_at_target = std::min(speed_at_target, max_speed);
  }
  if (!std::isfinite(speed_at_target)) {
    speed_at_target = 0.0;
  }

  // Step 2: speed permitted AT start such that we can decelerate to
  // speed_at_target by the time we reach the target. Then clamp with the
  // global max_speed (which is independent of the segment).
  const double distance = (target - start).norm();
  const double braking_speed =
      maxSpeedFromBrakingDistance(max_jerk, max_acceleration, distance, speed_at_target);
  double result = braking_speed;
  if (max_speed > kEpsilon) {
    result = std::min(result, max_speed);
  }
  return result;
}

double computeSpeedFromWaypoints(const std::array<Eigen::Vector3d, 3>& waypoints,
                                 const double max_jerk,
                                 const double max_acceleration,
                                 const double max_speed,
                                 const double acceptance_radius,
                                 const double exit_speed) {
  // Backward sweep on the triplet. Step 1 establishes the speed at waypoints[1]
  // such that the segment waypoints[1]->waypoints[2] can decelerate from that
  // speed to @p exit_speed. We pass @p next == @p target so the corner cap at
  // waypoints[2] is treated as a straight extension (alpha = pi -> no corner
  // restriction); only the braking cap and @p max_speed apply there.
  const double speed_at_target =
      computeStartSpeedFromWaypoints(waypoints[1], waypoints[2], waypoints[2], exit_speed, max_jerk,
                                     max_acceleration, max_speed, acceptance_radius);

  // Step 2 propagates one segment back: speed at waypoints[0] must allow
  // decelerating to @p speed_at_target by the time we reach waypoints[1], and
  // must respect the corner geometry at waypoints[1] formed with waypoints[2].
  return computeStartSpeedFromWaypoints(waypoints[0], waypoints[1], waypoints[2], speed_at_target,
                                        max_jerk, max_acceleration, max_speed, acceptance_radius);
}

}  // namespace trajectory_generator_jerk_limited
