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
 * @file types.hpp
 * @brief Waypoint and trajectory POD structs used by the one-shot
 *        TrajectoryGenerator wrapper.
 *
 * All quantities are in SI units and in a fixed world frame (ENU).
 *
 * Important note: the underlying generator is a tick-by-tick S-curve
 * integrator, not a polynomial optimiser. Therefore the @ref Spline
 * produced by TrajectoryGenerator::spline() is a high-degree polynomial
 * fit (one segment per pair of waypoints, fitted by least squares from
 * the per-tick samples). It is intended for serialisation and
 * visualisation; for exact evaluation use TrajectoryGenerator::evaluate(t),
 * which interpolates the per-tick samples directly.
 */

#ifndef TRAJECTORY_GENERATOR_JERK_LIMITED__TYPES_HPP_
#define TRAJECTORY_GENERATOR_JERK_LIMITED__TYPES_HPP_

#include <Eigen/Core>

#include <optional>
#include <string>
#include <vector>

namespace trajectory_generator_jerk_limited {

/**
 * @brief Waypoint along a trajectory with optional per-derivative constraints.
 *
 * - `position` is always honoured.
 * - `velocity` and `acceleration` of the **first** waypoint pin the initial
 *   kinematic state of the integrator (so the trajectory can be chained
 *   continuously onto a vehicle already in flight). When unset, both default
 *   to zero, which is the standard start-from-rest case.
 * - `velocity` and `acceleration` of the **last** waypoint are metadata; the
 *   trajectory is considered to have arrived once the integrator settles to
 *   rest. Use @ref EndWaypoint to mark a stop-at-rest endpoint explicitly.
 * - `velocity` and `acceleration` at **intermediate** waypoints are ignored:
 *   this library is a tick-by-tick S-curve integrator, not a polynomial
 *   solver, and cannot pin internal derivatives along the path.
 */
struct Waypoint {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  std::optional<Eigen::Vector3d> velocity;
  std::optional<Eigen::Vector3d> acceleration;

  Waypoint() = default;
  /* implicit */ Waypoint(const Eigen::Vector3d& pos)  // NOLINT(runtime/explicit)
      : position(pos) {}
};

/**
 * @brief Waypoint for start/end vertices: velocity and acceleration pinned to zero.
 *
 * Inherits from @ref Waypoint with no additional members so slicing when
 * pushed into a `std::vector<Waypoint>` preserves the intended constraints.
 * Use to mark the start and end of a trajectory that must come to rest.
 */
struct EndWaypoint : public Waypoint {
  explicit EndWaypoint(const Eigen::Vector3d& pos) : Waypoint(pos) {
    velocity     = Eigen::Vector3d::Zero();
    acceleration = Eigen::Vector3d::Zero();
  }
};

/**
 * @brief Trajectory state evaluated at a single time instant.
 *
 * Frame: world (caller-defined). Units: metres, m/s, m/s^2.
 */
struct TrajectorySample {
  Eigen::Vector3d position     = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity     = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
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
 * In this library the spline is **derived** from the per-tick integrator
 * samples by least-squares fitting one polynomial per pair of adjacent
 * waypoints (degree configurable via GeneratorConfig::polynomial_order).
 * It is therefore an approximation of the underlying S-curve trajectory,
 * but with sufficient polynomial order the residual is negligible
 * (typically below the CSV print precision).
 *
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
   * If segments have different polynomial orders, `N` is the maximum order
   * across segments and missing high-order terms are serialized as zeros.
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
   * (`dimension != 3`), leaves `*this` unchanged and returns false.
   */
  bool loadFromCsv(const std::string& path);
};

}  // namespace trajectory_generator_jerk_limited

#endif  // TRAJECTORY_GENERATOR_JERK_LIMITED__TYPES_HPP_
