// Copyright 2026 Universidad Politécnica de Madrid
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

#ifndef MAV_TRAJECTORY_GENERATION_CPP_TRAJECTORY_GENERATOR_HPP_
#define MAV_TRAJECTORY_GENERATION_CPP_TRAJECTORY_GENERATOR_HPP_

#include <Eigen/Dense>

#include <memory>
#include <vector>

#include "mav_trajectory_generation_cpp/types.hpp"  // Waypoint, EndWaypoint, ...

namespace mav_trajectory_generation_cpp {

/**
 * @brief Pure-C++ facade around ETH-ASL `mav_trajectory_generation`.
 *
 * Generates smooth polynomial trajectories through a sequence of 3D
 * `Waypoint`s and exposes continuous evaluation of position, velocity,
 * acceleration and jerk.
 *
 * The lib-specific bits (Linear vs. Nonlinear solver selector, NLopt
 * knobs) live in @ref OptimizationConfig.
 *
 * Each `Waypoint` carries an optional velocity and acceleration. When
 * present, those are enforced as hard constraints at that vertex. When
 * absent, the optimiser is free to pick their values. Use `EndWaypoint`
 * at the first and last entries to pin velocity and acceleration to zero
 * (the typical "start / end at rest" pattern).
 *
 * The generator is reusable: call `generate()` multiple times on the same
 * instance; each call resets internal state and replaces the trajectory.
 *
 * Failure modes:
 *  - generate() returns false if fewer than two waypoints are supplied,
 *    if max_speed is non-positive, or if the optimiser fails to converge.
 *  - After a failed (or never-called) generate(), all accessors throw
 *    std::runtime_error. Use isValid() to check first.
 *
 * Implementation uses the Pimpl idiom so that this header does not leak
 * upstream includes (and transitively glog / nlopt) into consumers.
 */
class TrajectoryGenerator {
public:
  /**
   * @brief Construct a generator from a single @ref GeneratorConfig.
   */
  explicit TrajectoryGenerator(const GeneratorConfig& cfg = {});
  ~TrajectoryGenerator();

  TrajectoryGenerator(TrajectoryGenerator&&) noexcept;
  TrajectoryGenerator& operator=(TrajectoryGenerator&&) noexcept;

  TrajectoryGenerator(const TrajectoryGenerator&)            = delete;
  TrajectoryGenerator& operator=(const TrajectoryGenerator&) = delete;

  /**
   * @brief Generate a 3D polynomial trajectory through @p waypoints.
   *
   * Constraint handling per vertex:
   *   - Position is always pinned.
   *   - Intermediate vertices: velocity/acceleration pinned only when the
   *     corresponding `std::optional` fields are set; otherwise free.
   *   - Endpoints (first / last): all derivatives up to
   *     `cfg.derivative_to_optimize` must be pinned for a well-posed problem.
   *     The caller's explicit `Waypoint::velocity` / `::acceleration` wins;
   *     any unset derivative defaults to zero.
   *
   * Convenience: `EndWaypoint` is a `Waypoint` with velocity = 0 and
   * acceleration = 0. At an endpoint vertex it is functionally equivalent
   * to a bare `Waypoint(pos)` (which also defaults to zero there). It exists
   * as an explicit-intent marker and to document boundary behaviour.
   *
   * @param waypoints  Sequence of `Waypoint`s (implicit-convertible from
   *                   `Eigen::Vector3d` for position-only usage). Must have
   *                   at least two entries.
   * @param max_speed  Cruise speed used for segment-time allocation
   *                   (`estimateSegmentTimes()`) and, for
   *                   `Solver::Nonlinear`, as the velocity-magnitude
   *                   upper bound. Must be strictly positive [m/s].
   * @return  true iff a valid trajectory was produced. On failure the
   *          object transitions to an invalid state (`isValid() == false`).
   */
  bool generate(const std::vector<Waypoint>& waypoints, double max_speed);

  /// True iff the last `generate()` call produced a usable trajectory.
  bool isValid() const noexcept;

  /// Start time of the trajectory [s]. Always 0 when valid.
  /// Throws std::runtime_error if `!isValid()`.
  double minTime() const;

  /// End time of the trajectory [s]. Sum of segment durations when valid.
  /// Throws std::runtime_error if `!isValid()`.
  double maxTime() const;

  /// Convenience: `maxTime() - minTime()`. Throws if `!isValid()`.
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
   * Supports any derivative order exposed by the upstream polynomial
   * (0 = POSITION, 1 = VELOCITY, 2 = ACCELERATION, 3 = JERK, 4 = SNAP).
   * The input @p t is clamped to `[minTime(), maxTime()]`. Throws
   * std::runtime_error if `!isValid()`.
   */
  Eigen::Vector3d evaluateDerivative(double t, int derivative_order) const;

  /// Position at time t in [minTime(), maxTime()] [m]. Throws if `!isValid()`.
  Eigen::Vector3d position(double t) const;
  /// Velocity at time t [m/s]. Throws if `!isValid()`.
  Eigen::Vector3d velocity(double t) const;
  /// Acceleration at time t [m/s^2]. Throws if `!isValid()`.
  Eigen::Vector3d acceleration(double t) const;
  /// Jerk at time t [m/s^3]. Throws if `!isValid()`.
  Eigen::Vector3d jerk(double t) const;

  /// Maximum speed magnitude attained along the trajectory [m/s].
  /// Computed by sampling the underlying spline. Throws if `!isValid()`.
  double maxVelocity() const;
  /// Maximum acceleration magnitude attained along the trajectory [m/s^2].
  /// Computed by sampling the underlying spline. Throws if `!isValid()`.
  double maxAcceleration() const;

  /**
   * @brief Snapshot of the current planned spline.
   *
   * The returned @ref Spline owns its polynomial coefficients and is
   * independent of the generator — it can be stored, serialised, or
   * fed into another generator via @ref setSpline(). Throws
   * std::runtime_error if `!isValid()`.
   */
  Spline spline() const;

  /**
   * @brief Replace the current trajectory with an externally-provided spline.
   *
   * Useful for reusing a pre-computed / cached trajectory without running
   * the solver again (e.g. loaded from a CSV via @ref Spline::loadFromCsv).
   * Validates each segment's coefficient shape against the facade's fixed
   * `(dimension=3, polynomial_order=10)`. On failure the generator
   * transitions to an invalid state (`isValid() == false`).
   *
   * Note: `Spline::start_time` is retained in the input but the upstream
   * `mav_trajectory_generation::Trajectory` always exposes `minTime() == 0`,
   * so after `setSpline()` the generator's `minTime()` is `0` regardless
   * of `spline.start_time`.
   *
   * @return true iff the spline was accepted.
   */
  bool setSpline(const Spline& spline);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mav_trajectory_generation_cpp

#endif  // MAV_TRAJECTORY_GENERATION_CPP_TRAJECTORY_GENERATOR_HPP_
