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
//    * Neither the name of the Universidad Politécnica de Madrid nor the names
//      of its contributors may be used to endorse or promote products derived
//      from this software without specific prior written permission.
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
 * @file dynamic_trajectory_pybind.hpp
 *
 * @brief Python binding wrapper for the DynamicTrajectory class.
 *
 * @author Rafael Pérez Seguí <r.psegui@upm.es>
 */

#ifndef DYNAMIC_TRAJECTORY_GENERATOR_PYBIND__DYNAMIC_TRAJECTORY_PYBIND_HPP_
#define DYNAMIC_TRAJECTORY_GENERATOR_PYBIND__DYNAMIC_TRAJECTORY_PYBIND_HPP_

#include <cmath>

#include <memory>
#include <stdexcept>
#include <tuple>
#include <vector>

#include <Eigen/Dense>

#include "dynamic_trajectory_generator/dynamic_trajectory.hpp"
#include "dynamic_trajectory_generator/dynamic_waypoint.hpp"

namespace dynamic_traj_generator
{

/**
 * @brief Python-friendly wrapper around ::dynamic_traj_generator::DynamicTrajectory.
 *
 * Provides a simplified interface that:
 *   - Accepts plain Eigen waypoints instead of DynamicWaypoint objects.
 *   - Returns trajectory references as a tuple suitable for pybind11.
 *   - Optionally computes a path-facing yaw reference.
 *
 * Units are SI (meters, radians, seconds). The inertial frame follows ENU.
 */
class DynamicTrajectoryBind
{
public:
  DynamicTrajectoryBind()
  : trajectory_generator_(std::make_shared<DynamicTrajectory>()) {}

  ~DynamicTrajectoryBind() = default;

  /**
   * @brief Generate a trajectory from the current vehicle position through a set of waypoints.
   *
   * Blocks until the trajectory optimization worker produces a valid solution.
   *
   * @param initial_position Initial vehicle position (m), used as the trajectory start.
   * @param initial_yaw Initial yaw reference (rad).
   * @param waypoints Sequence of 3D waypoints (m). Must contain at least two entries.
   * @param max_velocity Maximum linear velocity along the trajectory (m/s). Must be > 0.
   *
   * @throws std::runtime_error If the waypoint set or velocity are invalid.
   */
  void generateTrajectory(
    const Eigen::Vector3d & initial_position,
    const float initial_yaw,
    const std::vector<Eigen::Vector3d> & waypoints,
    const float max_velocity)
  {
    if (waypoints.size() < 2) {
      throw std::runtime_error("At least two waypoints are required to generate a trajectory");
    }
    if (max_velocity <= 0.0f) {
      throw std::runtime_error("Maximum velocity must be strictly positive");
    }

    trajectory_generator_->updateVehiclePosition(initial_position);
    trajectory_generator_->setSpeed(static_cast<double>(max_velocity));
    trajectory_generator_->setWaypoints(eigenToDynamicWaypoints(waypoints));

    yaw_reference_ = initial_yaw;

    // Block until the trajectory optimization worker produces a valid solution.
    max_time_ = trajectory_generator_->getMaxTime();
    min_time_ = trajectory_generator_->getMinTime();
  }

  /**
   * @brief Evaluate the trajectory at a given time.
   *
   * @param time Evaluation time (s). Must lie within [min_time, max_time].
   * @return Tuple (position, velocity, acceleration, yaw).
   *
   * @throws std::runtime_error If time is outside the valid trajectory range.
   */
  std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, float> evaluateTrajectory(
    const float time)
  {
    if (time < min_time_ || time > max_time_) {
      throw std::runtime_error("Evaluation time is outside the valid trajectory range");
    }

    References refs;
    trajectory_generator_->evaluateTrajectory(time, refs);

    if (path_facing_) {
      yaw_reference_ = computePathFacingYaw(refs.velocity);
    }

    return std::make_tuple(refs.position, refs.velocity, refs.acceleration, yaw_reference_);
  }

  /**
   * @brief Enable or disable automatic yaw alignment with the trajectory velocity.
   *
   * When enabled, the yaw reference returned by evaluateTrajectory() is computed from the
   * horizontal velocity direction, falling back to the last known yaw when the horizontal
   * speed is too low to provide a reliable heading.
   */
  void setPathFacing(const bool enable)
  {
    path_facing_ = enable;
  }

  double getMaxTime() {return trajectory_generator_->getMaxTime();}
  double getMinTime() {return trajectory_generator_->getMinTime();}

  /**
   * @brief Return whether the trajectory has been regenerated since the last query.
   */
  bool getWasTrajectoryRegenerated()
  {
    return trajectory_generator_->getWasTrajectoryRegenerated();
  }

private:
  std::shared_ptr<DynamicTrajectory> trajectory_generator_;
  bool path_facing_ = false;
  float yaw_reference_ = 0.0f;
  double max_time_ = 0.0;
  double min_time_ = 0.0;

  static constexpr double kMinHorizontalSpeedForYaw = 0.1;

  float computePathFacingYaw(const Eigen::Vector3d & velocity) const
  {
    const Eigen::Vector2d velocity_xy(velocity.x(), velocity.y());
    if (velocity_xy.norm() > kMinHorizontalSpeedForYaw) {
      return static_cast<float>(std::atan2(velocity.y(), velocity.x()));
    }
    return yaw_reference_;
  }

  static DynamicWaypoint::Vector eigenToDynamicWaypoints(
    const std::vector<Eigen::Vector3d> & waypoints)
  {
    DynamicWaypoint::Vector result;
    result.reserve(waypoints.size());
    for (const auto & position : waypoints) {
      DynamicWaypoint waypoint;
      waypoint.resetWaypoint(position);
      result.emplace_back(std::move(waypoint));
    }
    return result;
  }
};

}  // namespace dynamic_traj_generator

#endif  // DYNAMIC_TRAJECTORY_GENERATOR_PYBIND__DYNAMIC_TRAJECTORY_PYBIND_HPP_
