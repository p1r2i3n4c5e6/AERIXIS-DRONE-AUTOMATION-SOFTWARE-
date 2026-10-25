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
 * @file trajectory_generator.hpp
 * @brief Pure C++ wrapper around GCOPTER's polytope-SFC trajectory optimiser.
 *
 * The wrapper marshals caller-friendly structs (DroneParameters, DroneLimits,
 * OptimizationConfig, Waypoint) into the Eigen vectors and polytope corridor
 * expected by gcopter::GCOPTER_PolytopeSFC, runs the L-BFGS optimisation and
 * exposes the resulting polynomial trajectory both as time-indexed
 * position/velocity/acceleration/jerk queries and as an inspectable
 * piecewise-polynomial Spline.
 *
 * Frame and units: everything is in SI and in a single fixed world frame
 * chosen by the caller (any right-handed frame, e.g. ENU). The generator is
 * stateless between calls to generate().
 *
 * Thread safety: not thread-safe. Serialise access across threads.
 *
 * Failure modes:
 *  - generate() returns false if the corridor is malformed, if fewer than two
 *    waypoints are supplied, if max_speed is non-positive, or if the L-BFGS
 *    solver diverges (cost = INF).
 *  - After a failed generate(), all accessors throw std::runtime_error.
 */

#ifndef GCOPTER_LIB__TRAJECTORY_GENERATOR_HPP_
#define GCOPTER_LIB__TRAJECTORY_GENERATOR_HPP_

#include <Eigen/Eigen>
#include <memory>
#include <vector>

#include "gcopter_lib/types.hpp"

namespace gcopter_lib {

class TrajectoryGenerator {
public:
  /**
   * @brief Construct a generator from a single @ref GeneratorConfig.
   */
  explicit TrajectoryGenerator(const GeneratorConfig& cfg = {});
  ~TrajectoryGenerator();

  TrajectoryGenerator(const TrajectoryGenerator&)            = delete;
  TrajectoryGenerator& operator=(const TrajectoryGenerator&) = delete;
  TrajectoryGenerator(TrajectoryGenerator&&) noexcept;
  TrajectoryGenerator& operator=(TrajectoryGenerator&&) noexcept;

  /**
   * @brief Optimise a trajectory through the given waypoints at @p max_speed.
   *
   * The first and last waypoint provide the boundary conditions (position,
   * and velocity / acceleration when explicitly set; otherwise zero — start /
   * end at rest). Intermediate waypoints only contribute their position,
   * which is used to grow the AABB Safe Flight Corridor.
   *
   * @param waypoints  Ordered list, size >= 2.
   * @param max_speed  Cruise speed [m/s], must be > 0. For this single call
   *                   it transiently overrides `DroneLimits::max_velocity`
   *                   used as the magnitude bound passed to GCOPTER.
   * @return true on successful optimisation, false otherwise.
   *
   * The default corridor margin is taken from `OptimizationConfig::corridor_margin`
   * (default 2.0 m). To set per-segment margins, use the second overload.
   */
  bool generate(const std::vector<Waypoint>& waypoints, double max_speed);

  /**
   * @brief Optimise a trajectory with a per-segment corridor margin.
   *
   * gcopter-specific extension. Lets the caller tighten the corridor around
   * specific segments (e.g. to encourage the trajectory to pass closer to
   * intermediate waypoints by using a small margin on the segments that
   * touch them).
   *
   * @param waypoints         Ordered list, size >= 2.
   * @param max_speed         Cruise speed [m/s], must be > 0. Same semantics
   *                          as the two-argument overload.
   * @param corridor_margins  Half-margin per segment [m]. Size must equal
   *                          `waypoints.size() - 1`; every entry must be > 0.
   * @return true on successful optimisation, false otherwise.
   */
  bool generate(const std::vector<Waypoint>& waypoints,
                double max_speed,
                const std::vector<double>& corridor_margins);

  /// True if the last generate() produced a valid trajectory.
  bool isValid() const noexcept;

  /// Start time of the trajectory [s]. Always 0 when valid.
  /// Throws std::runtime_error if `!isValid()`.
  double minTime() const;

  /// End time of the trajectory [s]. Equal to `duration()` when valid.
  /// Throws std::runtime_error if `!isValid()`.
  double maxTime() const;

  /// Total duration of the trajectory [s]. Throws if `!isValid()`.
  double duration() const;

  /**
   * @brief Evaluate position/velocity/acceleration at time @p t.
   *
   * The input @p t is clamped to `[minTime(), maxTime()]`. Throws
   * std::runtime_error if `!isValid()`.
   */
  TrajectorySample evaluate(double t) const;

  /**
   * @brief Evaluate the @p derivative_order -th derivative at time @p t.
   *
   * Supports orders 0..3 (POSITION, VELOCITY, ACCELERATION, JERK); higher
   * orders return zero. The input @p t is clamped to `[minTime(), maxTime()]`.
   * Throws std::runtime_error if `!isValid()`.
   */
  Eigen::Vector3d evaluateDerivative(double t, int derivative_order) const;

  /// Position at time t in [0, duration()] [m]. Throws if `!isValid()`.
  Eigen::Vector3d position(double t) const;
  /// Velocity at time t [m/s]. Throws if `!isValid()`.
  Eigen::Vector3d velocity(double t) const;
  /// Acceleration at time t [m/s^2]. Throws if `!isValid()`.
  Eigen::Vector3d acceleration(double t) const;
  /// Jerk at time t [m/s^3]. Throws if `!isValid()`.
  Eigen::Vector3d jerk(double t) const;

  /// Maximum speed magnitude attained along the trajectory [m/s].
  /// Throws if `!isValid()`.
  double maxVelocity() const;
  /// Maximum acceleration magnitude attained along the trajectory [m/s^2].
  /// Throws if `!isValid()`.
  double maxAcceleration() const;

  /**
   * @brief Snapshot of the current planned spline.
   *
   * The returned @ref Spline owns its polynomial coefficients in
   * lowest-degree-first order (gcopter_lib public convention) and is
   * independent of the generator. Throws std::runtime_error if `!isValid()`.
   */
  Spline spline() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gcopter_lib

#endif  // GCOPTER_LIB__TRAJECTORY_GENERATOR_HPP_
