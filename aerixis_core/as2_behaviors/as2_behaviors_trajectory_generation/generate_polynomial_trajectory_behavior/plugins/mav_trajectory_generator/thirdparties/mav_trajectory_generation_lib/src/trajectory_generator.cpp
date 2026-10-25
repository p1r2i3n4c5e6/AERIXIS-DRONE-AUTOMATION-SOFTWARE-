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

#include "mav_trajectory_generation_cpp/trajectory_generator.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include "mav_trajectory_generation/motion_defines.h"
#include "mav_trajectory_generation/polynomial_optimization_linear.h"
#include "mav_trajectory_generation/polynomial_optimization_nonlinear.h"
#include "mav_trajectory_generation/trajectory.h"
#include "mav_trajectory_generation/vertex.h"

#include "segment_timing.hpp"

namespace mav_trajectory_generation_cpp {

namespace {

// Polynomial order used throughout (degree 10), matching the ETH-ASL
// mav_trajectory_generation reference implementation.
constexpr int kPolynomialN = 10;

// 3D trajectories only — matches the public TrajectorySample API.
constexpr int kDimension = 3;

}  // namespace

struct TrajectoryGenerator::Impl {
  OptimizationConfig cfg;
  mav_trajectory_generation::Trajectory trajectory;
  bool valid = false;

  void ensureValid(const char* caller) const {
    if (!valid) {
      throw std::runtime_error(
          std::string("mav_trajectory_generation_cpp::TrajectoryGenerator::") + caller +
          ": no valid trajectory (generate() must succeed before accessor calls)");
    }
  }

  double clampTime(double t) const {
    const double t_min = trajectory.getMinTime();
    const double t_max = trajectory.getMaxTime();
    if (t < t_min) return t_min;
    if (t > t_max) return t_max;
    return t;
  }
};

TrajectoryGenerator::TrajectoryGenerator(const GeneratorConfig& cfg)
    : impl_(std::make_unique<Impl>()) {
  impl_->cfg = cfg.optimization;
}

TrajectoryGenerator::~TrajectoryGenerator()                                         = default;
TrajectoryGenerator::TrajectoryGenerator(TrajectoryGenerator&&) noexcept            = default;
TrajectoryGenerator& TrajectoryGenerator::operator=(TrajectoryGenerator&&) noexcept = default;

bool TrajectoryGenerator::generate(const std::vector<Waypoint>& waypoints, double max_speed) {
  impl_->valid = false;
  impl_->trajectory.clear();

  if (waypoints.size() < 2) {
    std::cerr << "[mav_trajectory_generation_cpp] generate(): need at least 2 waypoints, got "
              << waypoints.size() << "\n";
    return false;
  }
  if (!(max_speed > 0.0)) {
    std::cerr << "[mav_trajectory_generation_cpp] generate(): max_speed must be > 0, got "
              << max_speed << "\n";
    return false;
  }

  using mav_trajectory_generation::derivative_order::ACCELERATION;
  using mav_trajectory_generation::derivative_order::POSITION;
  using mav_trajectory_generation::derivative_order::VELOCITY;

  // Build Vertex::Vector.
  //   Intermediate vertices: only the constraints the user set.
  //   Endpoint vertices: the polynomial optimiser requires all derivatives
  //   up to derivative_to_optimize to be pinned for a well-posed problem
  //   (matches the canonical `makeStartOrEnd` boundary conditions). The
  //   user's explicit constraints win; any unset derivative at an endpoint
  //   is zero by default (→ start/end at rest unless the caller says
  //   otherwise, e.g. to chain trajectories with non-zero initial velocity).
  mav_trajectory_generation::Vertex::Vector vertices;
  vertices.reserve(waypoints.size());
  const int up_to_derivative = impl_->cfg.derivative_to_optimize;

  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    const Waypoint& wp     = waypoints[i];
    const bool is_endpoint = (i == 0) || (i + 1 == waypoints.size());

    mav_trajectory_generation::Vertex v(kDimension);
    v.addConstraint(POSITION, wp.position);

    if (wp.velocity)
      v.addConstraint(VELOCITY, *wp.velocity);
    else if (is_endpoint)
      v.addConstraint(VELOCITY, Eigen::Vector3d::Zero());

    if (wp.acceleration)
      v.addConstraint(ACCELERATION, *wp.acceleration);
    else if (is_endpoint)
      v.addConstraint(ACCELERATION, Eigen::Vector3d::Zero());

    if (is_endpoint) {
      for (int d = 3; d <= up_to_derivative; ++d) {
        v.addConstraint(d, Eigen::Vector3d::Zero());
      }
    }

    vertices.push_back(v);
  }

  // Segment-time allocation
  const std::size_t n_segments = waypoints.size() > 0 ? waypoints.size() - 1 : 0;
  std::vector<double> v_seg(n_segments, max_speed);
  std::vector<double> a_seg(n_segments, impl_->cfg.a_max);
  for (std::size_t i = 0; i < n_segments; ++i) {
    const Waypoint& target = waypoints[i + 1];
    v_seg[i] = target.max_speed.value_or(max_speed);
    a_seg[i] = target.max_acceleration.value_or(impl_->cfg.a_max);
  }
  const std::vector<double> segment_times =
      estimateSegmentTimesFromLimits(vertices, v_seg, a_seg);

  switch (impl_->cfg.solver) {
    case Solver::Linear: {
      mav_trajectory_generation::PolynomialOptimization<kPolynomialN> opt(kDimension);
      if (!opt.setupFromVertices(vertices, segment_times, up_to_derivative)) {
        std::cerr << "[mav_trajectory_generation_cpp] Linear setupFromVertices failed\n";
        return false;
      }
      if (!opt.solveLinear()) {
        std::cerr << "[mav_trajectory_generation_cpp] solveLinear() failed\n";
        return false;
      }
      opt.getTrajectory(&impl_->trajectory);
      break;
    }
    case Solver::Nonlinear: {
      mav_trajectory_generation::NonlinearOptimizationParameters params;
      params.max_iterations                  = impl_->cfg.nl_max_iterations;
      params.f_rel                           = impl_->cfg.nl_f_rel;
      params.x_rel                           = impl_->cfg.nl_x_rel;
      params.time_penalty                    = impl_->cfg.nl_time_penalty;
      params.initial_stepsize_rel            = impl_->cfg.nl_initial_stepsize_rel;
      params.inequality_constraint_tolerance = impl_->cfg.nl_inequality_constraint_tolerance;

      mav_trajectory_generation::PolynomialOptimizationNonLinear<kPolynomialN> opt(kDimension,
                                                                                   params);
      if (!opt.setupFromVertices(vertices, segment_times, up_to_derivative)) {
        std::cerr << "[mav_trajectory_generation_cpp] Nonlinear setupFromVertices failed\n";
        return false;
      }
      opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                                        max_speed);
      opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION,
                                        impl_->cfg.a_max);
      // NLopt return codes < 0 signal failure, >= 0 success / convergence-criterion reached.
      const int rc = opt.optimize();
      if (rc < 0) {
        std::cerr << "[mav_trajectory_generation_cpp] Nonlinear optimize() failed, rc=" << rc
                  << "\n";
        return false;
      }
      opt.getTrajectory(&impl_->trajectory);
      break;
    }
  }

  impl_->valid = true;
  return true;
}

bool TrajectoryGenerator::isValid() const noexcept { return impl_->valid; }

double TrajectoryGenerator::minTime() const {
  impl_->ensureValid("minTime");
  return impl_->trajectory.getMinTime();
}

double TrajectoryGenerator::maxTime() const {
  impl_->ensureValid("maxTime");
  return impl_->trajectory.getMaxTime();
}

double TrajectoryGenerator::duration() const {
  impl_->ensureValid("duration");
  return impl_->trajectory.getMaxTime() - impl_->trajectory.getMinTime();
}

TrajectorySample TrajectoryGenerator::evaluate(double t) const {
  impl_->ensureValid("evaluate");
  const double t_clamped = impl_->clampTime(t);
  using mav_trajectory_generation::derivative_order::ACCELERATION;
  using mav_trajectory_generation::derivative_order::POSITION;
  using mav_trajectory_generation::derivative_order::VELOCITY;
  const Eigen::VectorXd p = impl_->trajectory.evaluate(t_clamped, POSITION);
  const Eigen::VectorXd v = impl_->trajectory.evaluate(t_clamped, VELOCITY);
  const Eigen::VectorXd a = impl_->trajectory.evaluate(t_clamped, ACCELERATION);
  TrajectorySample sample;
  sample.position     = p.head<kDimension>();
  sample.velocity     = v.head<kDimension>();
  sample.acceleration = a.head<kDimension>();
  return sample;
}

Eigen::Vector3d TrajectoryGenerator::evaluateDerivative(double t, int derivative_order) const {
  impl_->ensureValid("evaluateDerivative");
  const double t_clamped  = impl_->clampTime(t);
  const Eigen::VectorXd d = impl_->trajectory.evaluate(t_clamped, derivative_order);
  return d.head<kDimension>();
}

Eigen::Vector3d TrajectoryGenerator::position(double t) const {
  return evaluateDerivative(t, mav_trajectory_generation::derivative_order::POSITION);
}

Eigen::Vector3d TrajectoryGenerator::velocity(double t) const {
  return evaluateDerivative(t, mav_trajectory_generation::derivative_order::VELOCITY);
}

Eigen::Vector3d TrajectoryGenerator::acceleration(double t) const {
  return evaluateDerivative(t, mav_trajectory_generation::derivative_order::ACCELERATION);
}

Eigen::Vector3d TrajectoryGenerator::jerk(double t) const {
  return evaluateDerivative(t, mav_trajectory_generation::derivative_order::JERK);
}

namespace {

// Sample the trajectory at @p n_samples uniformly spaced times and return the
// peak magnitude of the requested derivative. Used by maxVelocity() and
// maxAcceleration() — adequate for typical poly orders (10) without pulling in
// the Jenkins-Traub root finder used by the upstream Segment helpers.
double peakMagnitude(const mav_trajectory_generation::Trajectory& trajectory,
                     int derivative_order,
                     int n_samples) {
  const double t_min = trajectory.getMinTime();
  const double t_max = trajectory.getMaxTime();
  if (n_samples < 2) n_samples = 2;
  const double dt = (t_max - t_min) / static_cast<double>(n_samples - 1);
  double peak     = 0.0;
  for (int i = 0; i < n_samples; ++i) {
    const double t          = t_min + i * dt;
    const Eigen::VectorXd d = trajectory.evaluate(t, derivative_order);
    const double m          = d.head<kDimension>().norm();
    if (m > peak) peak = m;
  }
  return peak;
}

}  // namespace

double TrajectoryGenerator::maxVelocity() const {
  impl_->ensureValid("maxVelocity");
  // 200 samples per segment gives sub-percent accuracy on N=10 polynomials
  // for typical drone trajectories. Cap at a sane upper bound.
  const int n_segments = static_cast<int>(impl_->trajectory.K());
  const int n_samples  = std::clamp(n_segments * 200, 200, 20000);
  return peakMagnitude(impl_->trajectory, mav_trajectory_generation::derivative_order::VELOCITY,
                       n_samples);
}

double TrajectoryGenerator::maxAcceleration() const {
  impl_->ensureValid("maxAcceleration");
  const int n_segments = static_cast<int>(impl_->trajectory.K());
  const int n_samples  = std::clamp(n_segments * 200, 200, 20000);
  return peakMagnitude(impl_->trajectory, mav_trajectory_generation::derivative_order::ACCELERATION,
                       n_samples);
}

Spline TrajectoryGenerator::spline() const {
  impl_->ensureValid("spline");
  Spline out;
  out.start_time = impl_->trajectory.getMinTime();

  mav_trajectory_generation::Segment::Vector raw;
  impl_->trajectory.getSegments(&raw);
  out.segments.reserve(raw.size());
  for (const auto& seg : raw) {
    SegmentPolynomial sp;
    sp.duration              = seg.getTime();
    const auto& polys        = seg.getPolynomialsRef();
    const int n_coeffs       = (polys.empty() ? 0 : polys.front().N());
    sp.coefficients          = Eigen::MatrixXd::Zero(kDimension, n_coeffs);
    const int axes_available = std::min<int>(kDimension, static_cast<int>(polys.size()));
    for (int d = 0; d < axes_available; ++d) {
      sp.coefficients.row(d) = polys[d].getCoefficients(0).transpose();
    }
    out.segments.push_back(std::move(sp));
  }
  return out;
}

bool TrajectoryGenerator::setSpline(const Spline& s) {
  if (s.segments.empty()) {
    std::cerr << "[mav_trajectory_generation_cpp] setSpline(): empty spline\n";
    impl_->valid = false;
    impl_->trajectory.clear();
    return false;
  }
  mav_trajectory_generation::Segment::Vector raw;
  raw.reserve(s.segments.size());
  for (std::size_t i = 0; i < s.segments.size(); ++i) {
    const auto& sp = s.segments[i];
    if (sp.coefficients.rows() != kDimension || sp.coefficients.cols() != kPolynomialN) {
      std::cerr << "[mav_trajectory_generation_cpp] setSpline(): segment " << i
                << " coefficients shape is (" << sp.coefficients.rows() << ","
                << sp.coefficients.cols() << "), expected (" << kDimension << "," << kPolynomialN
                << ")\n";
      impl_->valid = false;
      impl_->trajectory.clear();
      return false;
    }
    mav_trajectory_generation::Segment seg(kPolynomialN, kDimension);
    seg.setTime(sp.duration);
    for (int d = 0; d < kDimension; ++d) {
      // Polynomial::setCoefficients takes a column vector of size N.
      seg[d].setCoefficients(sp.coefficients.row(d).transpose());
    }
    raw.push_back(std::move(seg));
  }
  impl_->trajectory.setSegments(raw);
  impl_->valid = true;
  return true;
}

// --- Spline methods ---------------------------------------------------------

std::vector<double> Spline::segmentTimes() const {
  std::vector<double> out;
  out.reserve(segments.size());
  for (const auto& sp : segments) out.push_back(sp.duration);
  return out;
}

std::vector<double> Spline::breakpoints() const {
  if (segments.empty()) return {};
  std::vector<double> out;
  out.reserve(segments.size() + 1);
  double t = start_time;
  out.push_back(t);
  for (const auto& sp : segments) {
    t += sp.duration;
    out.push_back(t);
  }
  return out;
}

double Spline::duration() const {
  double total = 0.0;
  for (const auto& sp : segments) total += sp.duration;
  return total;
}

namespace {

constexpr int kSplineFormatVersion = 1;

// Parse a `# key: value` metadata line. Returns false if the line does not
// match that shape, and leaves @p key / @p value unmodified.
bool parseMetaLine(const std::string& line, std::string* key, std::string* value) {
  if (line.empty() || line.front() != '#') return false;
  const auto colon = line.find(':');
  if (colon == std::string::npos) return false;
  std::string k = line.substr(1, colon - 1);
  std::string v = line.substr(colon + 1);
  auto trim     = [](std::string& s) {
    const char* ws = " \t\r\n";
    const auto l   = s.find_first_not_of(ws);
    s              = (l == std::string::npos) ? std::string{} : s.substr(l);
    const auto r   = s.find_last_not_of(ws);
    if (r != std::string::npos) s.erase(r + 1);
  };
  trim(k);
  trim(v);
  if (k.empty() || v.empty()) return false;
  *key   = k;
  *value = v;
  return true;
}

// Split a CSV line on ',' into doubles. Returns false on parse error.
bool parseCsvRow(const std::string& line, std::vector<double>* out) {
  out->clear();
  std::stringstream ss(line);
  std::string cell;
  while (std::getline(ss, cell, ',')) {
    try {
      out->push_back(std::stod(cell));
    } catch (const std::exception&) {
      return false;
    }
  }
  return !out->empty();
}

}  // namespace

bool Spline::saveToCsv(const std::string& path) const {
  std::ofstream out(path);
  if (!out) {
    std::cerr << "[mav_trajectory_generation_cpp] Spline::saveToCsv(): cannot open '" << path
              << "'\n";
    return false;
  }

  // Infer dimension and polynomial order from the first segment (if any).
  const int dim    = segments.empty() ? 3 : static_cast<int>(segments.front().coefficients.rows());
  const int n_poly = segments.empty() ? 0 : static_cast<int>(segments.front().coefficients.cols());

  out << "# spline_format_version: " << kSplineFormatVersion << "\n";
  out.precision(std::numeric_limits<double>::max_digits10);
  out << "# start_time: " << start_time << "\n";
  out << "# dimension: " << dim << "\n";
  out << "# polynomial_order: " << n_poly << "\n";

  // Header row.
  out << "duration";
  const char axis_name[] = {'x', 'y', 'z'};
  for (int d = 0; d < dim; ++d) {
    for (int k = 0; k < n_poly; ++k) {
      const char axis = (d < 3) ? axis_name[d] : '?';
      out << ",c" << axis << k;
    }
  }
  out << "\n";

  // Data rows.
  for (const auto& sp : segments) {
    if (sp.coefficients.rows() != dim || sp.coefficients.cols() != n_poly) {
      std::cerr << "[mav_trajectory_generation_cpp] Spline::saveToCsv(): inconsistent segment "
                   "shapes; file may be malformed\n";
      return false;
    }
    out << sp.duration;
    for (int d = 0; d < dim; ++d) {
      for (int k = 0; k < n_poly; ++k) {
        out << "," << sp.coefficients(d, k);
      }
    }
    out << "\n";
  }
  return static_cast<bool>(out);
}

bool Spline::loadFromCsv(const std::string& path) {
  std::ifstream in(path);
  if (!in) {
    std::cerr << "[mav_trajectory_generation_cpp] Spline::loadFromCsv(): cannot open '" << path
              << "'\n";
    return false;
  }

  std::string line;
  int version = -1, dim = -1, n_poly = -1;
  double start_time_meta = 0.0;

  // 1) Read comment-prefixed metadata until the header row appears.
  while (std::getline(in, line)) {
    if (!line.empty() && line.front() == '#') {
      std::string key, value;
      if (!parseMetaLine(line, &key, &value)) continue;
      try {
        if (key == "spline_format_version")
          version = std::stoi(value);
        else if (key == "start_time")
          start_time_meta = std::stod(value);
        else if (key == "dimension")
          dim = std::stoi(value);
        else if (key == "polynomial_order")
          n_poly = std::stoi(value);
      } catch (const std::exception&) {
        std::cerr << "[mav_trajectory_generation_cpp] Spline::loadFromCsv(): bad metadata '" << line
                  << "'\n";
        return false;
      }
    } else {
      break;  // First non-comment line is the header.
    }
  }
  if (version != kSplineFormatVersion) {
    std::cerr << "[mav_trajectory_generation_cpp] Spline::loadFromCsv(): unknown format version "
              << version << " (expected " << kSplineFormatVersion << ")\n";
    return false;
  }
  if (dim != 3) {
    std::cerr << "[mav_trajectory_generation_cpp] Spline::loadFromCsv(): unsupported dimension "
              << dim << " (only 3 is supported)\n";
    return false;
  }
  if (n_poly <= 0) {
    std::cerr << "[mav_trajectory_generation_cpp] Spline::loadFromCsv(): invalid polynomial_order "
              << n_poly << "\n";
    return false;
  }
  // `line` holds the header row at this point — we just rely on the data row
  // count matching the metadata; no need to parse the column names.

  // 2) Read data rows.
  const int expected_cols = 1 + dim * n_poly;
  std::vector<SegmentPolynomial> parsed;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    std::vector<double> cells;
    if (!parseCsvRow(line, &cells)) {
      std::cerr << "[mav_trajectory_generation_cpp] Spline::loadFromCsv(): malformed row '" << line
                << "'\n";
      return false;
    }
    if (static_cast<int>(cells.size()) != expected_cols) {
      std::cerr << "[mav_trajectory_generation_cpp] Spline::loadFromCsv(): row has " << cells.size()
                << " cells, expected " << expected_cols << "\n";
      return false;
    }
    SegmentPolynomial sp;
    sp.duration     = cells[0];
    sp.coefficients = Eigen::MatrixXd::Zero(dim, n_poly);
    std::size_t idx = 1;
    for (int d = 0; d < dim; ++d) {
      for (int k = 0; k < n_poly; ++k) {
        sp.coefficients(d, k) = cells[idx++];
      }
    }
    parsed.push_back(std::move(sp));
  }
  if (parsed.empty()) {
    std::cerr << "[mav_trajectory_generation_cpp] Spline::loadFromCsv(): no segments found in '"
              << path << "'\n";
    return false;
  }

  // 3) Commit (only now that parsing has succeeded).
  this->start_time = start_time_meta;
  this->segments   = std::move(parsed);
  return true;
}

}  // namespace mav_trajectory_generation_cpp
