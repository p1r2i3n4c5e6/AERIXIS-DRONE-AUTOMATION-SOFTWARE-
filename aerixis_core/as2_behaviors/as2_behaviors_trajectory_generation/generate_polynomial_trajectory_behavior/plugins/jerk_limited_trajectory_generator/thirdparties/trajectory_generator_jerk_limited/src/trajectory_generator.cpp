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
 * @file trajectory_generator.cpp
 *
 * Implementation of the one-shot @ref TrajectoryGenerator wrapper. The
 * trajectory is produced by driving a @ref WaypointTrajectoryController
 * through the full waypoint sequence at fixed step_dt and recording every
 * tick. Per-tick samples back the time-domain accessors (`evaluate(t)`,
 * etc.); a least-squares polynomial fit (one segment per pair of adjacent
 * waypoints) backs `spline()`.
 */

#include "trajectory_generator_jerk_limited/trajectory_generator.hpp"

#include <Eigen/Dense>
#include <Eigen/QR>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "trajectory_generator_jerk_limited/types.hpp"
#include "trajectory_generator_jerk_limited/waypoint_controller.hpp"

namespace trajectory_generator_jerk_limited {

namespace {

// One recorded simulation tick.
struct Sample {
  double time                  = 0.0;
  Eigen::Vector3d position     = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity     = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d jerk         = Eigen::Vector3d::Zero();
};

// Fit a single polynomial of order `n_poly` (degree `n_poly - 1`) to the
// scalar series (taus[i], values[i]) by least squares. Returns the
// `n_poly` coefficients in ascending power order (c0 + c1*tau + ...).
Eigen::VectorXd fitPolynomial(const Eigen::VectorXd& taus,
                              const Eigen::VectorXd& values,
                              int n_poly) {
  const Eigen::Index n_samples = taus.size();
  Eigen::MatrixXd vandermonde(n_samples, n_poly);
  for (Eigen::Index i = 0; i < n_samples; ++i) {
    double t_pow = 1.0;
    for (int k = 0; k < n_poly; ++k) {
      vandermonde(i, k) = t_pow;
      t_pow *= taus(i);
    }
  }
  return vandermonde.householderQr().solve(values);
}

}  // namespace

struct TrajectoryGenerator::Impl {
  GeneratorConfig cfg;
  bool valid = false;
  std::vector<Sample> samples;               // ordered by time, time[0] == 0.
  std::vector<std::size_t> arrival_indices;  // sample index of each waypoint arrival
  Spline spline_snapshot;
  double max_velocity_norm     = 0.0;
  double max_acceleration_norm = 0.0;
};

TrajectoryGenerator::TrajectoryGenerator(const GeneratorConfig& cfg)
    : impl_(std::make_unique<Impl>()) {
  impl_->cfg = cfg;
}

TrajectoryGenerator::~TrajectoryGenerator() = default;

TrajectoryGenerator::TrajectoryGenerator(TrajectoryGenerator&&) noexcept            = default;
TrajectoryGenerator& TrajectoryGenerator::operator=(TrajectoryGenerator&&) noexcept = default;

bool TrajectoryGenerator::generate(const std::vector<Waypoint>& waypoints, double max_speed) {
  // Reset state up-front so a previous successful run does not bleed into a failed one.
  impl_->valid = false;
  impl_->samples.clear();
  impl_->arrival_indices.clear();
  impl_->spline_snapshot       = Spline{};
  impl_->max_velocity_norm     = 0.0;
  impl_->max_acceleration_norm = 0.0;

  if (waypoints.size() < 2) {
    std::cerr << "[trajectory_generator_jerk_limited] generate(): need >= 2 waypoints, got "
              << waypoints.size() << "\n";
    return false;
  }
  if (!(max_speed > 0.0)) {
    std::cerr << "[trajectory_generator_jerk_limited] generate(): max_speed must be > 0\n";
    return false;
  }
  if (!(impl_->cfg.simulator.step_dt > 0.0)) {
    std::cerr << "[trajectory_generator_jerk_limited] generate(): step_dt must be > 0\n";
    return false;
  }

  // Build the streaming controller with the requested max_speed (overriding cfg.params.max_speed).
  TrajectoryParameters effective = impl_->cfg.params;
  effective.max_speed            = max_speed;

  // The first waypoint carries the integrator's initial kinematic state via
  // its optional `velocity` / `acceleration` fields. When unset they default
  // to zero, so callers using the legacy `Waypoint(pos)` constructor get
  // exactly the previous start-from-rest behaviour.
  const Waypoint& start = waypoints.front();

  WaypointTrajectoryController ctrl(effective);
  ctrl.reset(start);

  const double dt = impl_->cfg.simulator.step_dt;
  // Effective advance threshold between waypoints. The user-facing knob is
  // `params.acceptance_radius`, but a positive `simulator.advance_radius_min`
  // acts as a floor so very small acceptance_radius values still take effect
  // (the integrator never advances strictly closer than the floor). A
  // non-positive floor disables the lower bound entirely.
  const double radius_floor   = std::max(impl_->cfg.simulator.advance_radius_min, 0.0);
  const double advance_radius = std::max(impl_->cfg.params.acceptance_radius, radius_floor);
  // Termination tolerance for the last waypoint. When the user provides a
  // strictly positive `final_acceptance_radius`, use it verbatim so the
  // trajectory can settle arbitrarily close to the goal regardless of the L1
  // corner-cut radius applied to intermediate waypoints. Otherwise fall back
  // to `advance_radius` to preserve the legacy single-knob behaviour.
  const double final_radius    = (impl_->cfg.params.final_acceptance_radius > 0.0)
                                     ? impl_->cfg.params.final_acceptance_radius
                                     : advance_radius;
  const double settle_velocity = impl_->cfg.simulator.settle_velocity;
  const double max_t           = impl_->cfg.simulator.max_simulation_time;
  const std::size_t n_wps      = waypoints.size();

  // First waypoint is the start; integrate towards waypoints[1..].
  std::size_t active = 1;

  // Record the start sample (t=0) honouring the seeded velocity / acceleration.
  Sample first;
  first.time         = 0.0;
  first.position     = start.position;
  first.velocity     = start.velocity.value_or(Eigen::Vector3d::Zero());
  first.acceleration = start.acceleration.value_or(Eigen::Vector3d::Zero());
  impl_->samples.push_back(first);
  impl_->arrival_indices.push_back(0);  // arrival at waypoints[0] is sample 0.

  impl_->max_velocity_norm     = first.velocity.norm();
  impl_->max_acceleration_norm = first.acceleration.norm();

  Eigen::Vector3d vehicle = start.position;
  std::size_t tick        = 0;
  bool settled            = false;
  while (true) {
    const double t_now = static_cast<double>(tick + 1) * dt;
    if (t_now > max_t) {
      std::cerr << "[trajectory_generator_jerk_limited] generate(): max_simulation_time (" << max_t
                << " s) reached without settling\n";
      return false;
    }

    // Build {prev, target, next} triplet for the active segment. On the
    // final segment we collapse the triplet (next = target), which the
    // multi-waypoint controller path already interprets as "stop at this
    // waypoint" via its fly-through-vs-stop discriminator
    // (waypoints[2] == waypoints[1] => exit_speed_at_target = 0). The
    // tight settling tolerance is then enforced externally by
    // final_acceptance_radius in the termination check below.
    const Eigen::Vector3d prev   = waypoints[active - 1].position;
    const Eigen::Vector3d target = waypoints[active].position;
    const Eigen::Vector3d next   = (active + 1 < n_wps) ? waypoints[active + 1].position : target;
    const std::array<Eigen::Vector3d, 3> triplet = {prev, target, next};

    const TrajectorySetpoints sp = ctrl.update(dt, vehicle, triplet);
    vehicle                      = sp.position;
    ++tick;

    Sample s;
    s.time         = t_now;
    s.position     = sp.position;
    s.velocity     = sp.velocity;
    s.acceleration = sp.acceleration;
    s.jerk         = sp.jerk;
    impl_->samples.push_back(s);

    impl_->max_velocity_norm     = std::max(impl_->max_velocity_norm, sp.velocity.norm());
    impl_->max_acceleration_norm = std::max(impl_->max_acceleration_norm, sp.acceleration.norm());

    // Waypoint advance, mirrors run_example.cpp manager-style pattern.
    if (active + 1 < n_wps && (vehicle - target).norm() < advance_radius) {
      impl_->arrival_indices.push_back(impl_->samples.size() - 1);
      ++active;
    }

    // Termination: last waypoint reached AND velocity has settled to rest.
    if (active + 1 == n_wps) {
      const bool near_last = (vehicle - waypoints.back().position).norm() < final_radius;
      const bool slow      = sp.velocity.norm() < settle_velocity;
      if (near_last && slow) {
        impl_->arrival_indices.push_back(impl_->samples.size() - 1);
        settled = true;
        break;
      }
    }
  }

  if (!settled) {
    std::cerr << "[trajectory_generator_jerk_limited] generate(): did not settle\n";
    return false;
  }
  if (impl_->arrival_indices.size() != n_wps) {
    // Defensive: something went wrong with bookkeeping.
    std::cerr << "[trajectory_generator_jerk_limited] generate(): unexpected arrival count "
              << impl_->arrival_indices.size() << " (expected " << n_wps << ")\n";
    return false;
  }

  // Build the spline: one segment per pair of adjacent waypoints.
  Spline sp_out;
  sp_out.start_time = 0.0;
  sp_out.segments.reserve(n_wps - 1);

  const int requested_order = std::max(2, impl_->cfg.simulator.polynomial_order);

  for (std::size_t k = 0; k + 1 < n_wps; ++k) {
    const std::size_t i_start = impl_->arrival_indices[k];
    const std::size_t i_end   = impl_->arrival_indices[k + 1];
    if (i_end <= i_start) {
      std::cerr << "[trajectory_generator_jerk_limited] generate(): empty segment between "
                   "waypoints "
                << k << " and " << k + 1 << "\n";
      return false;
    }
    const std::size_t n_pts = i_end - i_start + 1;
    const int n_poly        = std::min<int>(requested_order, static_cast<int>(n_pts));
    const double t0         = impl_->samples[i_start].time;
    const double t1         = impl_->samples[i_end].time;
    const double duration   = t1 - t0;

    Eigen::VectorXd taus(n_pts);
    Eigen::Matrix3Xd positions(3, n_pts);
    for (std::size_t i = 0; i < n_pts; ++i) {
      const Sample& s  = impl_->samples[i_start + i];
      taus(i)          = s.time - t0;
      positions.col(i) = s.position;
    }

    SegmentPolynomial seg;
    seg.duration     = duration;
    seg.coefficients = Eigen::MatrixXd::Zero(3, n_poly);
    for (int axis = 0; axis < 3; ++axis) {
      Eigen::VectorXd values     = positions.row(axis).transpose();
      Eigen::VectorXd coefs      = fitPolynomial(taus, values, n_poly);
      seg.coefficients.row(axis) = coefs.transpose();
    }
    sp_out.segments.push_back(std::move(seg));
  }

  impl_->spline_snapshot = std::move(sp_out);
  impl_->valid           = true;
  return true;
}

bool TrajectoryGenerator::isValid() const noexcept { return impl_->valid; }

namespace {
constexpr const char* kInvalidTrajectoryMsg =
    "[trajectory_generator_jerk_limited] TrajectoryGenerator: accessor called on invalid "
    "trajectory; call generate() and check its return value first";
}  // namespace

double TrajectoryGenerator::minTime() const {
  if (!impl_->valid) throw std::runtime_error(kInvalidTrajectoryMsg);
  return impl_->samples.front().time;
}

double TrajectoryGenerator::maxTime() const {
  if (!impl_->valid) throw std::runtime_error(kInvalidTrajectoryMsg);
  return impl_->samples.back().time;
}

double TrajectoryGenerator::duration() const {
  if (!impl_->valid) throw std::runtime_error(kInvalidTrajectoryMsg);
  return impl_->samples.back().time - impl_->samples.front().time;
}

namespace {
// Linearly interpolate between two samples at fraction alpha in [0, 1].
Sample interpolate(const Sample& a, const Sample& b, double alpha) {
  Sample out;
  out.time         = a.time + (b.time - a.time) * alpha;
  out.position     = a.position + (b.position - a.position) * alpha;
  out.velocity     = a.velocity + (b.velocity - a.velocity) * alpha;
  out.acceleration = a.acceleration + (b.acceleration - a.acceleration) * alpha;
  out.jerk         = a.jerk + (b.jerk - a.jerk) * alpha;
  return out;
}

Sample evaluateInterp(const std::vector<Sample>& samples, double t) {
  // Clamp.
  if (t <= samples.front().time) return samples.front();
  if (t >= samples.back().time) return samples.back();
  // Bisection on time.
  std::size_t lo = 0;
  std::size_t hi = samples.size() - 1;
  while (hi - lo > 1) {
    const std::size_t mid = (lo + hi) / 2;
    if (samples[mid].time <= t)
      lo = mid;
    else
      hi = mid;
  }
  const double dt    = samples[hi].time - samples[lo].time;
  const double alpha = (dt > 0.0) ? (t - samples[lo].time) / dt : 0.0;
  return interpolate(samples[lo], samples[hi], alpha);
}
}  // namespace

TrajectorySample TrajectoryGenerator::evaluate(double t) const {
  if (!impl_->valid) throw std::runtime_error(kInvalidTrajectoryMsg);
  const Sample s = evaluateInterp(impl_->samples, t);
  TrajectorySample out;
  out.position     = s.position;
  out.velocity     = s.velocity;
  out.acceleration = s.acceleration;
  return out;
}

Eigen::Vector3d TrajectoryGenerator::evaluateDerivative(double t, int derivative_order) const {
  if (!impl_->valid) throw std::runtime_error(kInvalidTrajectoryMsg);
  const Sample s = evaluateInterp(impl_->samples, t);
  switch (derivative_order) {
    case 0:
      return s.position;
    case 1:
      return s.velocity;
    case 2:
      return s.acceleration;
    case 3:
      return s.jerk;
    default:
      return Eigen::Vector3d::Zero();
  }
}

Eigen::Vector3d TrajectoryGenerator::position(double t) const { return evaluateDerivative(t, 0); }

Eigen::Vector3d TrajectoryGenerator::velocity(double t) const { return evaluateDerivative(t, 1); }

Eigen::Vector3d TrajectoryGenerator::acceleration(double t) const {
  return evaluateDerivative(t, 2);
}

Eigen::Vector3d TrajectoryGenerator::jerk(double t) const { return evaluateDerivative(t, 3); }

double TrajectoryGenerator::maxVelocity() const {
  if (!impl_->valid) throw std::runtime_error(kInvalidTrajectoryMsg);
  return impl_->max_velocity_norm;
}

double TrajectoryGenerator::maxAcceleration() const {
  if (!impl_->valid) throw std::runtime_error(kInvalidTrajectoryMsg);
  return impl_->max_acceleration_norm;
}

Spline TrajectoryGenerator::spline() const {
  if (!impl_->valid) throw std::runtime_error(kInvalidTrajectoryMsg);
  return impl_->spline_snapshot;
}

}  // namespace trajectory_generator_jerk_limited
