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
 * @file run_example.cpp
 * @brief End-to-end example for gcopter_lib.
 *
 * Loads the simulation and drone YAML configurations, builds a
 * TrajectoryGenerator, optimises a trajectory that traverses the configured
 * waypoints and samples the resulting polynomial at a fixed step_dt. Each
 * sample (time, position, velocity, acceleration) is appended to a CSV file via
 * gcopter_lib::TrajectoryLogger.
 *
 * Usage:
 *   ./gcopter_lib_run_example \
 *     -c example/config_example.yaml \
 *     -d example/config_trajectory.yaml   \
 *     -f example/trajectory_logs/trajectory.csv
 *
 * Frames and units: all quantities are SI and expressed in a single fixed
 * world frame (ENU by convention).
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#include <algorithm>
#include <cmath>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "gcopter_lib/trajectory_generator.hpp"
#include "gcopter_lib/trajectory_logger.hpp"
#include "gcopter_lib/types.hpp"
#include "utils.hpp"

namespace {

// Pretty-print a Spline: segment timing table + per-axis polynomial coefficients.
void printSpline(const gcopter_lib::Spline& sp) {
  if (sp.segments.empty()) {
    std::cout << "(empty spline)\n";
    return;
  }
  const auto knots         = sp.breakpoints();
  const std::streamsize ow = std::cout.precision();
  const auto fl            = std::cout.flags();

  std::cout << "\nSpline (" << sp.segments.size() << " segments, total duration " << std::fixed
            << std::setprecision(4) << sp.duration() << " s):\n";
  std::cout << "  seg   t_start     t_end   duration\n";
  std::cout << "  ---  --------  --------  --------\n";
  for (std::size_t i = 0; i < sp.segments.size(); ++i) {
    std::cout << "  " << std::setw(3) << i << "  " << std::setw(8) << knots[i] << "  "
              << std::setw(8) << knots[i + 1] << "  " << std::setw(8) << sp.segments[i].duration
              << "\n";
  }

  std::cout << "\nPolynomials  p_d(tau) = c0 + c1*tau + c2*tau^2 + ... + c_{N-1}*tau^{N-1},"
               "  tau in [0, T_seg]:\n";
  const char* axis_name[] = {"px", "py", "pz"};
  for (std::size_t i = 0; i < sp.segments.size(); ++i) {
    const auto& seg = sp.segments[i];
    std::cout << "  Segment " << i << "  (T=" << seg.duration << " s):\n";
    for (int d = 0; d < 3; ++d) {
      std::cout << "    " << axis_name[d] << ":";
      for (int k = 0; k < seg.coefficients.cols(); ++k) {
        std::cout << "  " << std::showpos << std::setw(11) << std::setprecision(4)
                  << seg.coefficients(d, k) << std::noshowpos;
      }
      std::cout << "\n";
    }
  }
  std::cout << std::endl;
  std::cout.flags(fl);
  std::cout.precision(ow);
}

// Derive the spline CSV path from the sample CSV path: replace a trailing
// ".csv" with ".spline.csv"; otherwise append ".spline.csv".
std::string splineCsvPathFor(const std::string& csv_path) {
  const std::string suffix = ".csv";
  if (csv_path.size() >= suffix.size() &&
      csv_path.compare(csv_path.size() - suffix.size(), suffix.size(), suffix) == 0) {
    return csv_path.substr(0, csv_path.size() - suffix.size()) + ".spline.csv";
  }
  return csv_path + ".spline.csv";
}

std::vector<gcopter_lib::Waypoint> buildWaypoints(const std::vector<Eigen::Vector3d>& positions) {
  std::vector<gcopter_lib::Waypoint> wps;
  wps.reserve(positions.size());
  for (const Eigen::Vector3d& p : positions) {
    gcopter_lib::Waypoint wp;
    wp.position = p;
    // velocity and acceleration default to zero on the first and last waypoint,
    // yielding a stationary start and stop. Intermediate waypoints only use
    // position (velocity/acceleration there are ignored by the generator).
    wps.emplace_back(std::move(wp));
  }
  return wps;
}

// Insert anchor points at ±radius around every intermediate waypoint and
// assign a tight margin to the two pinch segments that touch each waypoint,
// keeping the loose margin elsewhere.
//
// For N input waypoints (N >= 3), the expanded path has 3*(N-2) + 2 points
// and 3*N - 5 segments. For N = 2 the mechanism is skipped (no intermediate
// waypoints to anchor).
struct ExpandedPath {
  std::vector<Eigen::Vector3d> positions;
  std::vector<double> margins;
};

ExpandedPath expandPathWithAnchors(const std::vector<Eigen::Vector3d>& waypoints,
                                   double loose_margin,
                                   double tight_margin,
                                   double anchor_radius) {
  ExpandedPath out;
  const std::size_t N = waypoints.size();
  if (N < 3 || tight_margin <= 0.0 || anchor_radius <= 0.0) {
    out.positions = waypoints;
    out.margins.assign(N - 1, loose_margin);
    return out;
  }

  out.positions.reserve(3 * (N - 2) + 2);
  out.positions.emplace_back(waypoints.front());

  for (std::size_t i = 1; i + 1 < N; ++i) {
    const Eigen::Vector3d& prev = waypoints[i - 1];
    const Eigen::Vector3d& curr = waypoints[i];
    const Eigen::Vector3d& next = waypoints[i + 1];

    const Eigen::Vector3d in_vec  = curr - prev;
    const Eigen::Vector3d out_vec = next - curr;
    const double in_norm          = in_vec.norm();
    const double out_norm         = out_vec.norm();
    if (in_norm <= 0.0 || out_norm <= 0.0) {
      // Degenerate: two consecutive waypoints coincide. Skip anchoring
      // around this waypoint and keep the waypoint itself in the path.
      out.positions.emplace_back(curr);
      continue;
    }
    // Clamp the radius so consecutive anchors never cross each other.
    const double r_in  = std::min(anchor_radius, 0.45 * in_norm);
    const double r_out = std::min(anchor_radius, 0.45 * out_norm);

    out.positions.emplace_back(curr - in_vec * (r_in / in_norm));
    out.positions.emplace_back(curr);
    out.positions.emplace_back(curr + out_vec * (r_out / out_norm));
  }
  out.positions.emplace_back(waypoints.back());

  // Segment margin pattern for the expanded path:
  //   [loose]                                           (wp_0 -> first anchor)
  //   for each intermediate wp i:
  //     [tight, tight]                                  (anchor_in -> wp -> anchor_out)
  //     if not the last intermediate: [loose]           (anchor_out -> next anchor_in)
  //   [loose]                                           (last anchor -> wp_{N-1})
  const std::size_t M = N - 2;  // number of intermediate waypoints
  out.margins.reserve(3 * M + 1);
  out.margins.emplace_back(loose_margin);
  for (std::size_t i = 0; i < M; ++i) {
    out.margins.emplace_back(tight_margin);
    out.margins.emplace_back(tight_margin);
    if (i + 1 < M) {
      out.margins.emplace_back(loose_margin);
    }
  }
  out.margins.emplace_back(loose_margin);
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  using namespace gcopter_lib::examples;

  CliArgs args;
  SimConfig sim;
  TrajectoryConfig trajectory_cfg;

  try {
    args           = parseCli(argc, argv);
    sim            = loadSimConfig(args.sim_config_path);
    trajectory_cfg = loadTrajectoryConfig(args.trajectory_config_path);
  } catch (const std::exception& e) {
    std::cerr << "Configuration error: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  const bool anchors_enabled = trajectory_cfg.generator.waypoint_margin > 0.0 &&
                               trajectory_cfg.generator.waypoint_anchor_radius > 0.0;

  const ExpandedPath expanded = expandPathWithAnchors(
      sim.waypoints, trajectory_cfg.generator.corridor_margin,
      trajectory_cfg.generator.waypoint_margin, trajectory_cfg.generator.waypoint_anchor_radius);

  std::cout << "Configuration\n"
            << "  sim:        " << args.sim_config_path << "\n"
            << "  trajectory: " << args.trajectory_config_path << "\n"
            << "  csv:        " << args.output_csv_path << "\n"
            << "  waypoints:        " << sim.waypoints.size() << "\n"
            << "  step_dt:          " << sim.step_dt << " s\n"
            << "  corridor_margin:  " << trajectory_cfg.generator.corridor_margin << " m\n";
  if (anchors_enabled) {
    std::cout << "  waypoint_margin:  " << trajectory_cfg.generator.waypoint_margin << " m\n"
              << "  anchor_radius:    " << trajectory_cfg.generator.waypoint_anchor_radius << " m\n"
              << "  expanded points:  " << expanded.positions.size() << "\n";
  }
  std::cout << "  max_velocity:     " << trajectory_cfg.limits.max_velocity << " m/s" << std::endl;

  gcopter_lib::GeneratorConfig native_cfg;
  native_cfg.params       = trajectory_cfg.params;
  native_cfg.limits       = trajectory_cfg.limits;
  native_cfg.optimization = trajectory_cfg.optimization;
  gcopter_lib::TrajectoryGenerator generator(native_cfg);

  const std::vector<gcopter_lib::Waypoint> waypoints = buildWaypoints(expanded.positions);

  const double max_speed = trajectory_cfg.limits.max_velocity;
  const bool ok = anchors_enabled ? generator.generate(waypoints, max_speed, expanded.margins)
                                  : generator.generate(waypoints, max_speed);
  if (!ok) {
    std::cerr << "Trajectory generation failed (check waypoints, limits and "
                 "corridor margins)."
              << std::endl;
    return EXIT_FAILURE;
  }

  const double duration = generator.duration();
  std::cout << std::fixed << std::setprecision(3) << "\nTrajectory generated:\n"
            << "  duration:         " << duration << " s\n"
            << "  max velocity:     " << generator.maxVelocity() << " m/s\n"
            << "  max acceleration: " << generator.maxAcceleration() << " m/s^2\n"
            << std::endl;

  // Dump the planned spline (segment timing table + per-axis polynomial coeffs).
  printSpline(generator.spline());

  const std::filesystem::path csv_path(args.output_csv_path);
  const std::string spline_csv_path = splineCsvPathFor(args.output_csv_path);
  if (csv_path.has_parent_path()) {
    std::error_code ec;
    std::filesystem::create_directories(csv_path.parent_path(), ec);
    if (ec) {
      std::cerr << "Failed to create log directory '" << csv_path.parent_path().string()
                << "': " << ec.message() << std::endl;
      return EXIT_FAILURE;
    }
  }
  if (generator.spline().saveToCsv(spline_csv_path)) {
    std::cout << "Spline written to:    " << spline_csv_path << std::endl;
  } else {
    std::cerr << "WARNING: failed to write spline CSV to " << spline_csv_path << std::endl;
  }
  gcopter_lib::TrajectoryLogger logger(args.output_csv_path);

  // Sample at [0, step_dt, ..., N*step_dt] and add a final sample exactly at
  // t = duration so the end waypoint is always present in the CSV.
  const int steps          = static_cast<int>(std::floor(duration / sim.step_dt)) + 1;
  const int progress_every = std::max(1, steps / 100);

  for (int i = 0; i < steps; ++i) {
    const double t = std::min(i * sim.step_dt, duration);
    logger.save(t, generator.position(t), generator.velocity(t), generator.acceleration(t));
    if (i % progress_every == 0) {
      printProgress(t / duration);
    }
  }
  // Ensure the final sample is exactly at t = duration.
  logger.save(duration, generator.position(duration), generator.velocity(duration),
              generator.acceleration(duration));
  printProgress(1.0);
  std::cout << std::endl;

  logger.close();

  std::cout << "Done. CSV written to: " << args.output_csv_path << std::endl;
  return EXIT_SUCCESS;
}
