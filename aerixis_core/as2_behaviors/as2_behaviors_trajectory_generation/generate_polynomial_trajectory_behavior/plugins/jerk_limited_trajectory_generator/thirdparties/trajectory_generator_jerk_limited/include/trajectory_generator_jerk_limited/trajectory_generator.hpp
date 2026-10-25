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
 * @brief One-shot trajectory generator that wraps the streaming
 *        WaypointTrajectoryController into an offline planning API.
 *
 * Internally drives a `WaypointTrajectoryController` through the full
 * waypoint sequence at a fixed simulation step, records every tick, and
 * exposes:
 *
 *   - `evaluate(t)` and the per-derivative accessors, which interpolate
 *     the recorded samples (exact at sample times).
 *   - `spline()`, which returns a piecewise-polynomial least-squares fit
 *     of the samples, one segment per pair of adjacent waypoints. The
 *     spline is intended for serialisation and visualisation; for exact
 *     evaluation prefer `evaluate(t)`.
 *
 * The streaming `WaypointTrajectoryController` API remains the canonical
 * way to consume this generator from a real-time control loop. This
 * one-shot wrapper exists for offline tooling (CSV logging, plotting,
 * batch trajectory comparisons).
 */

#ifndef TRAJECTORY_GENERATOR_JERK_LIMITED__TRAJECTORY_GENERATOR_HPP_
#define TRAJECTORY_GENERATOR_JERK_LIMITED__TRAJECTORY_GENERATOR_HPP_

#include <Eigen/Core>

#include <memory>
#include <vector>

#include "trajectory_generator_jerk_limited/parameters.hpp"
#include "trajectory_generator_jerk_limited/types.hpp"

namespace trajectory_generator_jerk_limited {

/**
 * @brief Implementation knobs of the internal simulation loop.
 *
 * These fields tune how `TrajectoryGenerator::generate()` drives its
 * streaming integrator and how `spline()` fits a polynomial to the
 * recorded samples.
 */
struct SimulatorConfig {
  /// Internal simulation step [s]. Must be > 0.
  double step_dt = 0.01;

  /// Maximum allowed simulation time [s]. If the integrator does not
  /// settle within this horizon, `generate()` returns false.
  double max_simulation_time = 600.0;

  /// Velocity threshold below which the trajectory is considered
  /// "at rest" once the last waypoint has been reached [m/s]. Stops
  /// the simulation loop early.
  double settle_velocity = 0.05;

  /// Floor applied to the per-waypoint advance threshold so that very
  /// small `acceptance_radius` values can take effect on the integrator's
  /// active-waypoint advance check. The effective advance radius used is
  /// `max(params.acceptance_radius, advance_radius_min)`. <= 0 disables
  /// the floor entirely (advance happens strictly within
  /// `params.acceptance_radius`); values below `step_dt * max_speed` are
  /// ill-conditioned (the drone may overshoot the waypoint before the
  /// advance check triggers). [m]
  double advance_radius_min = 0.1;

  /// Number of polynomial coefficients per axis per segment fitted in
  /// `spline()`. The polynomial degree is `polynomial_order - 1`. Must
  /// be >= 2 (linear fit) and <= number of samples in the shortest
  /// segment + 1.
  ///
  /// Higher orders reduce the per-segment fit residual against the
  /// underlying S-curve. For multi-phase S-curves typical of this
  /// library, residuals plateau around 1-2 mm at order >= 16; the
  /// default below targets that regime so the spline is visually
  /// indistinguishable from the integrator output.
  int polynomial_order = 16;
};

/**
 * @brief Tunables for @ref TrajectoryGenerator.
 *
 * Bundles the streaming `TrajectoryParameters` (kinematic bounds and
 * acceptance radius) with the implementation-specific
 * @ref SimulatorConfig.
 */
struct GeneratorConfig {
  /// Kinematic bounds and multi-waypoint corner / radius parameters.
  TrajectoryParameters params{};

  /// Internal simulation-loop and spline-fit knobs.
  SimulatorConfig simulator{};
};

/**
 * @brief One-shot multi-waypoint trajectory generator.
 *
 * Construct with a @ref GeneratorConfig (or default-construct), call
 * `generate(waypoints, max_speed)` once per trajectory, and then sample
 * the result via `evaluate(t)` or any of the per-derivative accessors.
 *
 * Failure modes:
 *  - `generate()` returns false if fewer than two waypoints are
 *    supplied, if `max_speed` is non-positive, if `step_dt` is
 *    non-positive, or if the simulation does not settle within
 *    `max_simulation_time`.
 *  - After a failed (or never-called) `generate()`, all accessors throw
 *    std::runtime_error. Use `isValid()` to check first.
 *
 * Thread safety: not thread-safe. Serialise access across threads.
 */
class TrajectoryGenerator {
public:
  explicit TrajectoryGenerator(const GeneratorConfig& cfg = GeneratorConfig{});
  ~TrajectoryGenerator();

  TrajectoryGenerator(TrajectoryGenerator&&) noexcept;
  TrajectoryGenerator& operator=(TrajectoryGenerator&&) noexcept;

  TrajectoryGenerator(const TrajectoryGenerator&)            = delete;
  TrajectoryGenerator& operator=(const TrajectoryGenerator&) = delete;

  /**
   * @brief Simulate a multi-waypoint trajectory through @p waypoints at
   *        cruise @p max_speed.
   *
   * Internally instantiates a fresh @ref WaypointTrajectoryController
   * with the configured @ref TrajectoryParameters (overridden so that
   * `params.max_speed = max_speed` for this call), resets it to
   * `waypoints[0].position` and integrates until the last waypoint is
   * reached and the velocity falls below `cfg.settle_velocity`.
   *
   * @param waypoints  Sequence of `Waypoint`s. Must have at least two
   *                   entries. Implicit-convertible from `Eigen::Vector3d`.
   * @param max_speed  Cruise speed [m/s]. Must be strictly positive.
   * @return true iff a valid trajectory was produced. On failure the
   *         object transitions to an invalid state (`isValid() == false`).
   */
  bool generate(const std::vector<Waypoint>& waypoints, double max_speed);

  /// True iff the last `generate()` call produced a usable trajectory.
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
   * Interpolates between recorded ticks. The input @p t is clamped to
   * `[minTime(), maxTime()]`. Throws std::runtime_error if `!isValid()`.
   */
  TrajectorySample evaluate(double t) const;

  /**
   * @brief Evaluate the @p derivative_order -th derivative at time @p t.
   *
   * Supports orders 0..3 (POSITION, VELOCITY, ACCELERATION, JERK); higher
   * orders return zero. The input @p t is clamped to
   * `[minTime(), maxTime()]`. Throws std::runtime_error if `!isValid()`.
   */
  Eigen::Vector3d evaluateDerivative(double t, int derivative_order) const;

  /// Position at time t [m]. Throws if `!isValid()`.
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
   * @brief Snapshot of the planned spline.
   *
   * The returned @ref Spline owns its polynomial coefficients and is
   * independent of the generator. Throws std::runtime_error if
   * `!isValid()`. See the class documentation for the fit semantics.
   */
  Spline spline() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace trajectory_generator_jerk_limited

#endif  // TRAJECTORY_GENERATOR_JERK_LIMITED__TRAJECTORY_GENERATOR_HPP_
