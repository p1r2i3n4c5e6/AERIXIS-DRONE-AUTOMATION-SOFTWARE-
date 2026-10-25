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

#ifndef MAV_TRAJECTORY_GENERATION_CPP_TYPES_HPP_
#define MAV_TRAJECTORY_GENERATION_CPP_TYPES_HPP_

#include <Eigen/Dense>

#include <optional>
#include <string>
#include <vector>

namespace mav_trajectory_generation_cpp {

/**
 * @brief Solver backend for the polynomial trajectory optimisation.
 *
 * `Linear` runs the closed-form least-squares solver
 * (`PolynomialOptimization<N>::solveLinear()`). It is deterministic and
 * typically orders of magnitude faster than the nonlinear variant, but it
 * does not enforce velocity/acceleration bounds — the cruise speed is only
 * used to allocate segment durations via `estimateSegmentTimes()`.
 *
 * `Nonlinear` runs NLopt-backed free-endpoint optimisation
 * (`PolynomialOptimizationNonLinear<N>::optimize()`) and actively enforces
 * velocity and acceleration magnitude bounds. It is slower, may fail to
 * converge on degenerate waypoint layouts, and in those cases `generate()`
 * returns false.
 */
enum class Solver {
  Linear,
  Nonlinear,
};

/**
 * @brief Tunables for the polynomial trajectory optimiser.
 *
 * All fields have sensible defaults tuned for MAV polynomial trajectory
 * generation.
 * Units: SI (metres, seconds, rad, m/s, m/s^2).
 */
struct OptimizationConfig {
  /// Derivative order to minimise along the trajectory.
  /// 0 = POSITION, 1 = VELOCITY, 2 = ACCELERATION, 3 = JERK, 4 = SNAP.
  int derivative_to_optimize = 4;

  /// Solver backend selector.
  Solver solver = Solver::Linear;

  /// Maximum acceleration magnitude [m/s^2]. Used by `estimateSegmentTimes()`
  /// for timing allocation and, for `Solver::Nonlinear`, as the upper bound
  /// on the acceleration magnitude constraint.
  double a_max = 4.0;

  /// NLopt stop criterion: maximum number of iterations.
  /// Only used when `solver == Solver::Nonlinear`.
  int nl_max_iterations = 2000;

  /// NLopt stop criterion: relative tolerance on the objective.
  /// Only used when `solver == Solver::Nonlinear`.
  double nl_f_rel = 0.05;

  /// NLopt stop criterion: relative tolerance on the parameter vector.
  /// Only used when `solver == Solver::Nonlinear`.
  double nl_x_rel = 0.1;

  /// NLopt penalty on total trajectory time (free-endpoint variant).
  /// Only used when `solver == Solver::Nonlinear`.
  double nl_time_penalty = 1000.0;

  /// NLopt initial step size (relative to the parameter range).
  /// Only used when `solver == Solver::Nonlinear`.
  double nl_initial_stepsize_rel = 0.1;

  /// NLopt tolerance on inequality constraint violation.
  /// Only used when `solver == Solver::Nonlinear`.
  double nl_inequality_constraint_tolerance = 0.2;
};

/**
 * @brief Top-level configuration of @ref TrajectoryGenerator.
 *
 * Wraps the lower-level @ref OptimizationConfig in a single struct
 * consumed by `TrajectoryGenerator(const GeneratorConfig&)`. Future
 * additions (e.g. logging or numerical knobs that are not part of
 * `OptimizationConfig`) would land here.
 */
struct GeneratorConfig {
  OptimizationConfig optimization{};  ///< Solver / NLopt tuning.
};

/**
 * @brief Polynomial trajectory evaluated at a single time instant.
 *
 * Frame: world (caller-defined). Units: metres, m/s, m/s^2.
 */
struct TrajectorySample {
  Eigen::Vector3d position     = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity     = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
};

/**
 * @brief Waypoint along a trajectory with optional per-derivative constraints.
 *
 * - `position` is always a hard constraint.
 * - `velocity` / `acceleration`, when set, are hard constraints too. Leaving
 *   them as `std::nullopt` means the optimiser is free to pick their values
 *   at this vertex.
 *
 * Intermediate waypoints typically only constrain position. Endpoints often
 * want velocity/acceleration fixed to zero — use @ref EndWaypoint for that.
 *
 * Chaining: to stitch a new trajectory to the end of a previous one without
 * coming to rest, build a starting `Waypoint` whose `velocity` equals
 * `gen.evaluate(gen.maxTime()).velocity` of the previous plan.
 */
struct Waypoint {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  std::optional<Eigen::Vector3d> velocity;
  std::optional<Eigen::Vector3d> acceleration;
  /// Per-segment kinematic limits for the segment ARRIVING at this waypoint
  /// (segment i->i+1 is timed by waypoint[i+1]). `std::nullopt`
  /// inherits the generator's global limit (`generate()`'s `max_speed` /
  /// `OptimizationConfig::a_max`).
  std::optional<double> max_speed;
  std::optional<double> max_acceleration;

  Waypoint() = default;
  /* implicit */ Waypoint(const Eigen::Vector3d& pos)  // NOLINT(runtime/explicit)
      : position(pos) {}
};

/**
 * @brief Waypoint for start/end vertices: velocity and acceleration pinned to zero.
 *
 * Inherits from @ref Waypoint with no additional members, so slicing when
 * pushed into a `std::vector<Waypoint>` preserves the intended constraints
 * (vel = 0, acc = 0). Use to mark the start and end of a trajectory that
 * must come to rest.
 */
struct EndWaypoint : public Waypoint {
  explicit EndWaypoint(const Eigen::Vector3d& pos) : Waypoint(pos) {
    velocity     = Eigen::Vector3d::Zero();
    acceleration = Eigen::Vector3d::Zero();
  }
};

/**
 * @brief One polynomial segment of the planned spline.
 *
 * Each segment is defined on a local time `tau ∈ [0, duration]`. For axis
 * `d` the scalar polynomial is
 *
 *     p_d(tau) = sum_{k=0..N-1} coefficients(d, k) * tau^k
 *
 * Rows of `coefficients` are axes (x, y, z); columns are polynomial
 * coefficients in increasing power of `tau` (`c0, c1, ..., c_{N-1}`).
 */
struct SegmentPolynomial {
  double duration = 0.0;         //< Segment duration [s]
  Eigen::MatrixXd coefficients;  //< shape (3, N); row per axis, col per power
};

/**
 * @brief Piecewise polynomial trajectory (spline).
 *
 * Stores the minimum information needed to reconstruct / evaluate the
 * spline: an absolute `start_time` and the ordered list of polynomial
 * segments. Segment durations live in each `SegmentPolynomial::duration`
 * (no duplication); breakpoints and total duration are computed on
 * demand via the accessors below.
 *
 * The facade exposes this struct via `TrajectoryGenerator::spline()` and
 * `TrajectoryGenerator::setSpline()`, so a spline can be snapshotted,
 * transported (e.g. serialised to CSV) and reused as the trajectory
 * backing another generator instance without re-running the optimiser.
 */
struct Spline {
  double start_time = 0.0;                  //< Absolute time of the first breakpoint [s]
  std::vector<SegmentPolynomial> segments;  //< Segments in temporal order

  /// Segment durations [s] (i.e. `{segments[0].duration, ..., segments[K-1].duration}`).
  std::vector<double> segmentTimes() const;

  /// Absolute-time breakpoints [s]. Returns `segments.size() + 1` values:
  /// `{start_time, start_time + d0, start_time + d0 + d1, ..., maxTime()}`.
  /// Empty if `segments` is empty.
  std::vector<double> breakpoints() const;

  /// Total duration [s] = sum of segment durations. Zero if `segments` is empty.
  double duration() const;

  /// Convenience alias: `start_time`.
  double minTime() const { return start_time; }

  /// Convenience alias: `start_time + duration()`.
  double maxTime() const { return start_time + duration(); }

  /**
   * @brief Serialise this spline to a CSV file.
   *
   * Format:
   *   - 4 comment lines (`# key: value`) with `spline_format_version`,
   *     `start_time`, `dimension`, and `polynomial_order`.
   *   - One header row: `duration,cx0,cx1,...,cx{N-1},cy0,...,cz{N-1}`.
   *   - One data row per segment, in temporal order.
   * Uses `std::numeric_limits<double>::max_digits10` digits so that
   * `loadFromCsv(saveToCsv())` is bit-exact for all IEEE-754 doubles.
   *
   * @return true on success; false on I/O error.
   */
  bool saveToCsv(const std::string& path) const;

  /**
   * @brief Load a spline from a CSV previously produced by `saveToCsv()`
   *        (or manually authored with the same schema).
   *
   * On success, populates `*this` and returns true. On missing file,
   * parse error, unknown `spline_format_version`, or shape mismatch
   * (`dimension != 3` or `polynomial_order` inconsistent with the data
   * columns), leaves `*this` unchanged and returns false.
   */
  bool loadFromCsv(const std::string& path);
};

}  // namespace mav_trajectory_generation_cpp

#endif  // MAV_TRAJECTORY_GENERATION_CPP_TYPES_HPP_
