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
 * @brief Parameter, waypoint and trajectory POD structs used by gcopter_lib.
 *
 * All quantities are expressed in SI units and in a fixed world frame
 * (convention: ENU — x east, y north, z up). The library is frame-agnostic:
 * as long as the caller is consistent across waypoints, limits and parameters
 * the resulting trajectory is expressed in the same frame.
 *
 * The Waypoint / TrajectorySample / SegmentPolynomial / Spline structs define
 * the public data model of gcopter_lib and are kept as plain POD-like types
 * for straightforward use from C++ and language bindings.
 */

#ifndef GCOPTER_LIB__TYPES_HPP_
#define GCOPTER_LIB__TYPES_HPP_

#include <Eigen/Eigen>

#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace gcopter_lib {

/**
 * @brief Physical parameters of the multirotor used by the flatness model
 *        inside GCOPTER (see gcopter/flatness.hpp).
 */
struct DroneParameters {
  double mass;                 ///< Vehicle mass [kg]
  double gravity;              ///< Gravitational acceleration magnitude [m/s^2]
  double horizontal_drag;      ///< Horizontal drag coefficient c_h [-]
  double vertical_drag;        ///< Vertical drag coefficient c_v [-]
  double parasitic_drag;       ///< Parasitic drag coefficient c_p [-]
  double speed_smooth_factor;  ///< Speed smoothing epsilon_v [-]
};

/**
 * @brief Hard magnitude bounds enforced by GCOPTER during optimisation.
 *        Order matches magnitudeBounds = [v_max, omg_max, theta_max,
 *        thrust_min, thrust_max] expected by GCOPTER_PolytopeSFC::setup.
 *
 * Note: `max_velocity` may be transiently overridden by the `max_speed`
 * argument of `TrajectoryGenerator::generate()` for a single call.
 */
struct DroneLimits {
  double max_velocity;    ///< Maximum linear speed [m/s]
  double max_body_rate;   ///< Maximum body rate magnitude [rad/s]
  double max_tilt_angle;  ///< Maximum tilt angle from vertical [rad]
  double min_thrust;      ///< Minimum collective thrust [N]
  double max_thrust;      ///< Maximum collective thrust [N]
};

/**
 * @brief Tuning knobs of the GCOPTER L-BFGS optimiser.
 *
 * The five *_weight fields map to penaltyWeights[0..4] in GCOPTER ordering:
 * [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight].
 *
 * `corridor_margin` is the default AABB half-margin used by the
 * single-margin `generate(waypoints, max_speed)` overload to build the
 * Safe Flight Corridor. Per-segment margins can still be provided via the
 * `generate(waypoints, max_speed, corridor_margins)` overload.
 */
struct OptimizationConfig {
  double time_weight{512.0};         ///< rho — trajectory duration penalty
  double position_weight{10000.0};   ///< chi[0] — position constraint penalty
  double velocity_weight{10000.0};   ///< chi[1] — velocity constraint penalty
  double body_rate_weight{10000.0};  ///< chi[2] — body-rate constraint penalty
  double tilt_weight{10000.0};       ///< chi[3] — tilt angle constraint penalty
  double thrust_weight{10000.0};     ///< chi[4] — thrust constraint penalty
  double smoothing_eps{1.0e-2};      ///< Smoothing epsilon for inequality
                                     ///< constraints
  int integral_resolution{16};       ///< Quadrature resolution per piece
  double rel_cost_tol{1.0e-3};       ///< L-BFGS relative cost tolerance
  double length_per_piece{           ///< Desired arclength per polynomial piece
                          std::numeric_limits<double>::infinity()};
  double corridor_margin{2.0};  ///< Default AABB half-margin per face [m]

  /// XY-plane offset applied to the *initial* waypoint when the segment is
  /// purely vertical (i.e. when the start and end positions differ only in
  /// world-Z). Units: metres.
  ///
  /// Why is this needed:
  ///   GCOPTER drives a MINCO spline + flatness-map cost through L-BFGS. For
  ///   any segment perfectly aligned with the world Z axis the optimiser
  ///   aborts with `LBFGSERR_INVALIDPARAMETERS` ("A logic error (negative
  ///   line-search step) occurred."). Two reinforcing factors cause it:
  ///     (a) GCOPTER's `setInitial()` distributes the initial inner knots
  ///         along the straight line between head and tail PVA, leaving every
  ///         knot exactly on the Z axis when the segment is pure-vertical;
  ///     (b) the flatness map injects gravity only on the Z component
  ///         (zu_z = a_z + drag_z + g, zu_x/y = a_x/y + drag_x/y), so the
  ///         initial thrust direction is exactly +Z and the cost gradient
  ///         vanishes in the XY subspace at the initial guess. The BFGS
  ///         Hessian estimate inherits a rank-deficient XY block and the
  ///         first non-trivial search direction yields a non-positive line-
  ///         search step that the Lewis-Overton routine refuses.
  ///   Pure +X / +Y / diagonal segments do not hit this because gravity
  ///   already breaks the symmetry along the thrust direction.
  ///
  /// What this knob does:
  ///   When ||p_end.xy - p_start.xy|| < `vertical_alignment_threshold`,
  ///   `runOptimization()` shifts the *initial* waypoint by
  ///   `(vertical_perturbation, 0, 0)` before invoking the L-BFGS solver.
  ///   That breaks the Z-axis symmetry without altering the requested final
  ///   waypoint: the optimisation converges and the resulting trajectory
  ///   ends exactly at the user's `tail` position. The trajectory now
  ///   *starts* at `head + (vertical_perturbation, 0, 0)`, which is
  ///   invisible at flight scale (cm-range over a m-range hop) and is
  ///   absorbed by the closed-loop tracker within a few control periods.
  ///
  /// Set to 0 to disable the workaround (purely vertical segments will then
  /// fail to plan, just like upstream GCOPTER).
  double vertical_perturbation{0.05};  ///< [m] (5 cm)

  /// Threshold on ||p_end.xy - p_start.xy|| under which a segment is
  /// considered purely vertical and the perturbation above is applied.
  /// Default 1 mm: anything above already produces enough horizontal
  /// gradient component to keep L-BFGS healthy.
  double vertical_alignment_threshold{1.0e-3};  ///< [m] (1 mm)
};

/**
 * @brief Top-level configuration of @ref TrajectoryGenerator.
 *
 * Bundles the three configuration sub-structs into a single argument
 * consumed by `TrajectoryGenerator(const GeneratorConfig&)`.
 */
struct GeneratorConfig {
  DroneParameters params{};           ///< Vehicle physical parameters (mass, drag, ...).
  DroneLimits limits{};               ///< Hard magnitude bounds passed to GCOPTER.
  OptimizationConfig optimization{};  ///< L-BFGS solver tuning.
};

/**
 * @brief Waypoint along a trajectory with optional per-derivative constraints.
 *
 * - `position` is always a hard constraint.
 * - `velocity` / `acceleration`, when set, fix the boundary derivatives
 *   at that vertex. They are only used at the first and last waypoints
 *   (which provide the boundary conditions of the optimisation) and are
 *   ignored on intermediate waypoints.
 * - When `velocity` / `acceleration` are left as `std::nullopt` at the
 *   first / last waypoint, they default to zero (start / end at rest).
 *   Use @ref EndWaypoint as an explicit-intent marker for that case.
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
 * Inherits from @ref Waypoint with no additional members, so slicing when
 * pushed into a `std::vector<Waypoint>` preserves the intended constraints
 * (vel = 0, acc = 0). Use to mark the start and end of a trajectory that
 * must come to rest. Functionally equivalent to a bare `Waypoint(pos)` at
 * an endpoint (since unset endpoint derivatives default to zero), but
 * clearer in intent.
 */
struct EndWaypoint : public Waypoint {
  explicit EndWaypoint(const Eigen::Vector3d& pos) : Waypoint(pos) {
    velocity     = Eigen::Vector3d::Zero();
    acceleration = Eigen::Vector3d::Zero();
  }
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
 * @brief One polynomial segment of the planned spline.
 *
 * Each segment is defined on a local time `tau ∈ [0, duration]`. For axis
 * `d` the scalar polynomial is
 *
 *     p_d(tau) = sum_{k=0..N-1} coefficients(d, k) * tau^k
 *
 * Rows of `coefficients` are axes (x, y, z); columns are polynomial
 * coefficients in increasing power of `tau` (`c0, c1, ..., c_{N-1}`).
 * The internal MINCO trajectory used by GCOPTER stores quintic pieces
 * (degree 5), so `coefficients` is shape (3, 6).
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
 * The facade exposes this struct via `TrajectoryGenerator::spline()`, so
 * a spline can be snapshotted and serialised (e.g. to CSV) for later
 * inspection or re-use without re-running the optimiser.
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
   * (`dimension != 3`), leaves `*this` unchanged and returns false.
   */
  bool loadFromCsv(const std::string& path);
};

}  // namespace gcopter_lib

#endif  // GCOPTER_LIB__TYPES_HPP_
