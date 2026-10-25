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

#include "gcopter_lib/trajectory_generator.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "gcopter_lib/corridor_builder.hpp"

// Upstream GCOPTER headers (ROS-free algorithmic core).
// Note 1: gcopter.hpp uses geo_utils symbols without including the header
// itself, so we include geo_utils.hpp first (kept in its own include block so
// clang-format does not reorder it behind gcopter.hpp).
// Note 2: we use a locally patched copy of geo_utils.hpp (see
// third_party/gcopter/geo_utils.hpp) because the upstream filterLess::operator()
// is non-const and fails C++17's std::set Compare requirement.
#include "third_party/gcopter/geo_utils.hpp"

#include "gcopter/gcopter.hpp"
#include "gcopter/trajectory.hpp"

namespace gcopter_lib {

namespace {

// Internal MINCO trajectory degree used by GCOPTER (quintic pieces).
constexpr int kPieceDegree = 5;
// 3D trajectories only — matches the public TrajectorySample API.
constexpr int kDimension = 3;

}  // namespace

struct TrajectoryGenerator::Impl {
  DroneParameters drone_params;
  DroneLimits drone_limits;
  OptimizationConfig config;
  Trajectory<kPieceDegree> trajectory;
  bool valid{false};

  Impl(const DroneParameters& dp, const DroneLimits& dl, const OptimizationConfig& cfg)
      : drone_params(dp), drone_limits(dl), config(cfg) {}

  void ensureValid(const char* caller) const {
    if (!valid) {
      throw std::runtime_error(std::string("gcopter_lib::TrajectoryGenerator::") + caller +
                               ": no valid trajectory (generate() must succeed"
                               " before accessor calls)");
    }
  }

  // Clamp t to [0, totalDuration]; assumes valid trajectory.
  double clampTime(double t) const {
    const double T = trajectory.getTotalDuration();
    if (t < 0.0) return 0.0;
    if (t > T) return T;
    return t;
  }

  bool runOptimization(const std::vector<Waypoint>& waypoints,
                       const PolyhedraH& corridor,
                       double v_max_override);
};

namespace {

// Build the (pos, vel, acc) 3x3 boundary-state matrix expected by GCOPTER.
// Unset optional vel/acc default to zero (start/end at rest).
Eigen::Matrix3d boundaryStateMatrix(const Waypoint& wp) {
  Eigen::Matrix3d m;
  m.col(0) = wp.position;
  m.col(1) = wp.velocity ? *wp.velocity : Eigen::Vector3d::Zero();
  m.col(2) = wp.acceleration ? *wp.acceleration : Eigen::Vector3d::Zero();
  return m;
}

Eigen::VectorXd magnitudeBoundsVec(const DroneLimits& limits, double v_max_override) {
  Eigen::VectorXd v(5);
  v(0) = v_max_override;
  v(1) = limits.max_body_rate;
  v(2) = limits.max_tilt_angle;
  v(3) = limits.min_thrust;
  v(4) = limits.max_thrust;
  return v;
}

Eigen::VectorXd penaltyWeightsVec(const OptimizationConfig& cfg) {
  Eigen::VectorXd v(5);
  v(0) = cfg.position_weight;
  v(1) = cfg.velocity_weight;
  v(2) = cfg.body_rate_weight;
  v(3) = cfg.tilt_weight;
  v(4) = cfg.thrust_weight;
  return v;
}

Eigen::VectorXd physicalParamsVec(const DroneParameters& params) {
  Eigen::VectorXd v(6);
  v(0) = params.mass;
  v(1) = params.gravity;
  v(2) = params.horizontal_drag;
  v(3) = params.vertical_drag;
  v(4) = params.parasitic_drag;
  v(5) = params.speed_smooth_factor;
  return v;
}

}  // namespace

bool TrajectoryGenerator::Impl::runOptimization(const std::vector<Waypoint>& waypoints,
                                                const PolyhedraH& corridor,
                                                double v_max_override) {
  Eigen::Matrix3d ini_state       = boundaryStateMatrix(waypoints.front());
  const Eigen::Matrix3d fin_state = boundaryStateMatrix(waypoints.back());

  // Pure-vertical workaround: when *every* waypoint of the requested path
  // shares the same XY column (i.e. the entire planned segment lives on a
  // line parallel to world-Z), GCOPTER's L-BFGS aborts with
  // LBFGSERR_INVALIDPARAMETERS. The initial guess produced by `setInitial()`
  // is then fully aligned with the Z axis, and the flatness-map cost
  // gradient vanishes in the XY subspace (gravity only enters z, so the
  // BFGS Hessian estimate is rank-deficient there and the first search
  // direction yields a non-positive step). See
  // `OptimizationConfig::vertical_perturbation` for the full diagnosis.
  // Shifting only the *initial* waypoint by a small in-plane offset breaks
  // the symmetry while preserving the user-requested final position: the
  // resulting trajectory ends exactly at `tail`. The trajectory enters at
  // `head + (perturbation, 0, 0)`, which is centimetre-scale and absorbed
  // by any closed-loop tracker within a few control periods.
  if (config.vertical_perturbation > 0.0 && config.vertical_alignment_threshold > 0.0) {
    const Eigen::Vector2d head_xy = ini_state.col(0).head<2>();
    bool all_on_z_line            = true;
    for (const Waypoint& wp : waypoints) {
      if ((wp.position.head<2>() - head_xy).norm() >= config.vertical_alignment_threshold) {
        all_on_z_line = false;
        break;
      }
    }
    if (all_on_z_line) {
      ini_state(0, 0) += config.vertical_perturbation;
    }
  }

  const Eigen::VectorXd magnitude_bounds = magnitudeBoundsVec(drone_limits, v_max_override);
  const Eigen::VectorXd penalty_weights  = penaltyWeightsVec(config);
  const Eigen::VectorXd physical_params  = physicalParamsVec(drone_params);

  gcopter::GCOPTER_PolytopeSFC solver;
  if (!solver.setup(config.time_weight, ini_state, fin_state, corridor, config.length_per_piece,
                    config.smoothing_eps, config.integral_resolution, magnitude_bounds,
                    penalty_weights, physical_params)) {
    return false;
  }

  const double cost = solver.optimize(trajectory, config.rel_cost_tol);
  if (std::isinf(cost) || trajectory.getPieceNum() == 0) {
    trajectory.clear();
    return false;
  }
  valid = true;
  return true;
}

TrajectoryGenerator::TrajectoryGenerator(const GeneratorConfig& cfg)
    : impl_(std::make_unique<Impl>(cfg.params, cfg.limits, cfg.optimization)) {}

TrajectoryGenerator::~TrajectoryGenerator() = default;

TrajectoryGenerator::TrajectoryGenerator(TrajectoryGenerator&&) noexcept            = default;
TrajectoryGenerator& TrajectoryGenerator::operator=(TrajectoryGenerator&&) noexcept = default;

bool TrajectoryGenerator::generate(const std::vector<Waypoint>& waypoints, double max_speed) {
  impl_->valid = false;
  impl_->trajectory.clear();

  if (waypoints.size() < 2) {
    std::cerr << "[gcopter_lib] generate(): need at least 2 waypoints, got " << waypoints.size()
              << "\n";
    return false;
  }
  if (!(max_speed > 0.0)) {
    std::cerr << "[gcopter_lib] generate(): max_speed must be > 0, got " << max_speed << "\n";
    return false;
  }

  std::vector<Eigen::Vector3d> positions;
  positions.reserve(waypoints.size());
  for (const Waypoint& wp : waypoints) {
    positions.emplace_back(wp.position);
  }

  PolyhedraH corridor;
  try {
    corridor = buildAabbCorridor(positions, impl_->config.corridor_margin);
  } catch (const std::invalid_argument&) {
    return false;
  }
  return impl_->runOptimization(waypoints, corridor, max_speed);
}

bool TrajectoryGenerator::generate(const std::vector<Waypoint>& waypoints,
                                   double max_speed,
                                   const std::vector<double>& corridor_margins) {
  impl_->valid = false;
  impl_->trajectory.clear();

  if (waypoints.size() < 2) {
    std::cerr << "[gcopter_lib] generate(): need at least 2 waypoints, got " << waypoints.size()
              << "\n";
    return false;
  }
  if (!(max_speed > 0.0)) {
    std::cerr << "[gcopter_lib] generate(): max_speed must be > 0, got " << max_speed << "\n";
    return false;
  }
  if (corridor_margins.size() != waypoints.size() - 1) {
    std::cerr << "[gcopter_lib] generate(): corridor_margins.size() = " << corridor_margins.size()
              << " must equal waypoints.size() - 1 = " << (waypoints.size() - 1) << "\n";
    return false;
  }

  std::vector<Eigen::Vector3d> positions;
  positions.reserve(waypoints.size());
  for (const Waypoint& wp : waypoints) {
    positions.emplace_back(wp.position);
  }

  PolyhedraH corridor;
  try {
    corridor = buildAabbCorridor(positions, corridor_margins);
  } catch (const std::invalid_argument&) {
    return false;
  }
  return impl_->runOptimization(waypoints, corridor, max_speed);
}

bool TrajectoryGenerator::isValid() const noexcept { return impl_ && impl_->valid; }

double TrajectoryGenerator::minTime() const {
  impl_->ensureValid("minTime");
  return 0.0;
}

double TrajectoryGenerator::maxTime() const {
  impl_->ensureValid("maxTime");
  return impl_->trajectory.getTotalDuration();
}

double TrajectoryGenerator::duration() const {
  impl_->ensureValid("duration");
  return impl_->trajectory.getTotalDuration();
}

TrajectorySample TrajectoryGenerator::evaluate(double t) const {
  impl_->ensureValid("evaluate");
  const double tc = impl_->clampTime(t);
  TrajectorySample s;
  s.position     = impl_->trajectory.getPos(tc);
  s.velocity     = impl_->trajectory.getVel(tc);
  s.acceleration = impl_->trajectory.getAcc(tc);
  return s;
}

Eigen::Vector3d TrajectoryGenerator::evaluateDerivative(double t, int derivative_order) const {
  impl_->ensureValid("evaluateDerivative");
  const double tc = impl_->clampTime(t);
  switch (derivative_order) {
    case 0:
      return impl_->trajectory.getPos(tc);
    case 1:
      return impl_->trajectory.getVel(tc);
    case 2:
      return impl_->trajectory.getAcc(tc);
    case 3:
      return impl_->trajectory.getJer(tc);
    default:
      // Higher derivatives (snap, crackle) are not exposed by Piece<5>.
      return Eigen::Vector3d::Zero();
  }
}

Eigen::Vector3d TrajectoryGenerator::position(double t) const {
  impl_->ensureValid("position");
  return impl_->trajectory.getPos(impl_->clampTime(t));
}

Eigen::Vector3d TrajectoryGenerator::velocity(double t) const {
  impl_->ensureValid("velocity");
  return impl_->trajectory.getVel(impl_->clampTime(t));
}

Eigen::Vector3d TrajectoryGenerator::acceleration(double t) const {
  impl_->ensureValid("acceleration");
  return impl_->trajectory.getAcc(impl_->clampTime(t));
}

Eigen::Vector3d TrajectoryGenerator::jerk(double t) const {
  impl_->ensureValid("jerk");
  return impl_->trajectory.getJer(impl_->clampTime(t));
}

double TrajectoryGenerator::maxVelocity() const {
  impl_->ensureValid("maxVelocity");
  return impl_->trajectory.getMaxVelRate();
}

double TrajectoryGenerator::maxAcceleration() const {
  impl_->ensureValid("maxAcceleration");
  return impl_->trajectory.getMaxAccRate();
}

Spline TrajectoryGenerator::spline() const {
  impl_->ensureValid("spline");
  Spline out;
  out.start_time = 0.0;
  const int N    = impl_->trajectory.getPieceNum();
  out.segments.reserve(N);
  for (int i = 0; i < N; ++i) {
    const auto& piece      = impl_->trajectory[i];
    const auto& coeff      = piece.getCoeffMat();  // shape (3, D+1)
    constexpr int n_coeffs = kPieceDegree + 1;
    SegmentPolynomial sp;
    sp.duration     = piece.getDuration();
    sp.coefficients = Eigen::MatrixXd::Zero(kDimension, n_coeffs);
    // GCOPTER stores coefficients highest-degree-first (col 0 = t^D, col D = t^0).
    // Public Spline convention is lowest-degree-first (col 0 = t^0, col k = t^k),
    // as used by gcopter_lib::SegmentPolynomial. Reverse columns.
    for (int k = 0; k < n_coeffs; ++k) {
      sp.coefficients.col(k) = coeff.col(kPieceDegree - k);
    }
    out.segments.push_back(std::move(sp));
  }
  return out;
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
    std::cerr << "[gcopter_lib] Spline::saveToCsv(): cannot open '" << path << "'\n";
    return false;
  }
  const int dim =
      segments.empty() ? kDimension : static_cast<int>(segments.front().coefficients.rows());
  const int n_poly = segments.empty() ? 0 : static_cast<int>(segments.front().coefficients.cols());

  out << "# spline_format_version: " << kSplineFormatVersion << "\n";
  out.precision(std::numeric_limits<double>::max_digits10);
  out << "# start_time: " << start_time << "\n";
  out << "# dimension: " << dim << "\n";
  out << "# polynomial_order: " << n_poly << "\n";

  out << "duration";
  const char axis_name[] = {'x', 'y', 'z'};
  for (int d = 0; d < dim; ++d) {
    for (int k = 0; k < n_poly; ++k) {
      const char axis = (d < 3) ? axis_name[d] : '?';
      out << ",c" << axis << k;
    }
  }
  out << "\n";

  for (const auto& sp : segments) {
    if (sp.coefficients.rows() != dim || sp.coefficients.cols() != n_poly) {
      std::cerr << "[gcopter_lib] Spline::saveToCsv(): inconsistent segment shapes; "
                   "file may be malformed\n";
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
    std::cerr << "[gcopter_lib] Spline::loadFromCsv(): cannot open '" << path << "'\n";
    return false;
  }

  std::string line;
  int version = -1, dim = -1, n_poly = -1;
  double start_time_meta = 0.0;

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
        std::cerr << "[gcopter_lib] Spline::loadFromCsv(): bad metadata '" << line << "'\n";
        return false;
      }
    } else {
      break;
    }
  }
  if (version != kSplineFormatVersion) {
    std::cerr << "[gcopter_lib] Spline::loadFromCsv(): unknown format version " << version
              << " (expected " << kSplineFormatVersion << ")\n";
    return false;
  }
  if (dim != kDimension) {
    std::cerr << "[gcopter_lib] Spline::loadFromCsv(): unsupported dimension " << dim << " (only "
              << kDimension << " is supported)\n";
    return false;
  }
  if (n_poly <= 0) {
    std::cerr << "[gcopter_lib] Spline::loadFromCsv(): invalid polynomial_order " << n_poly << "\n";
    return false;
  }

  const int expected_cols = 1 + dim * n_poly;
  std::vector<SegmentPolynomial> parsed;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    std::vector<double> cells;
    if (!parseCsvRow(line, &cells)) {
      std::cerr << "[gcopter_lib] Spline::loadFromCsv(): malformed row '" << line << "'\n";
      return false;
    }
    if (static_cast<int>(cells.size()) != expected_cols) {
      std::cerr << "[gcopter_lib] Spline::loadFromCsv(): row has " << cells.size()
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
    std::cerr << "[gcopter_lib] Spline::loadFromCsv(): no segments found in '" << path << "'\n";
    return false;
  }

  this->start_time = start_time_meta;
  this->segments   = std::move(parsed);
  return true;
}

}  // namespace gcopter_lib
