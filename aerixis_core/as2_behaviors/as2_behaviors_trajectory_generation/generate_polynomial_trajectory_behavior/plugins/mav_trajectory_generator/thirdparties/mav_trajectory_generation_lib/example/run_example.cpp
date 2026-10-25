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

/**
 * @file run_example.cpp
 * @brief End-to-end example for mav_trajectory_generation_cpp.
 *
 * Loads two YAMLs (sim_config + optimization), builds a TrajectoryGenerator,
 * plans a single trajectory through the configured waypoints and samples the
 * resulting polynomial at a fixed step_dt. Each sample
 * (time, position, velocity, acceleration) is appended to a 10-column CSV.
 *
 * Usage:
 *   ./mav_trajectory_generation_cpp_run_example \
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
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "mav_trajectory_generation_cpp/trajectory_generator.hpp"
#include "mav_trajectory_generation_cpp/types.hpp"
#include "utils.hpp"

namespace {

namespace mtg = mav_trajectory_generation_cpp;

// Pretty-print a Spline: segment timing table + per-axis polynomial coefficients.
void printSpline(const mtg::Spline& sp) {
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

// Derive the spline CSV path: `<base>.csv` -> `<base>.spline.csv`.
std::string splineCsvPathFor(const std::string& csv_path) {
  const std::string suffix = ".csv";
  if (csv_path.size() >= suffix.size() &&
      csv_path.compare(csv_path.size() - suffix.size(), suffix.size(), suffix) == 0) {
    return csv_path.substr(0, csv_path.size() - suffix.size()) + ".spline.csv";
  }
  return csv_path + ".spline.csv";
}

// Compose the waypoint list. The first and last are pinned to rest
// (EndWaypoint); intermediates only constrain position.
std::vector<mtg::Waypoint> buildWaypoints(const std::vector<Eigen::Vector3d>& positions) {
  std::vector<mtg::Waypoint> wps;
  wps.reserve(positions.size());
  if (positions.empty()) {
    return wps;
  }
  wps.emplace_back(mtg::EndWaypoint(positions.front()));
  for (std::size_t i = 1; i + 1 < positions.size(); ++i) {
    wps.emplace_back(mtg::Waypoint(positions[i]));
  }
  if (positions.size() >= 2) {
    wps.emplace_back(mtg::EndWaypoint(positions.back()));
  }
  return wps;
}

}  // namespace

int main(int argc, char** argv) {
  namespace examples = mtg::examples;

  examples::CliArgs args;
  examples::SimConfig sim;
  examples::TrajectoryConfig trajectory_cfg;

  try {
    args           = examples::parseCli(argc, argv);
    sim            = examples::loadSimConfig(args.sim_config_path);
    trajectory_cfg = examples::loadTrajectoryConfig(args.trajectory_config_path);
  } catch (const std::exception& e) {
    std::cerr << "Configuration error: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  // Use a sensible default cruise speed: the optimisation a_max is the only
  // hard kinematic bound declared; without an explicit max_speed in YAML we
  // pick a conservative value tied to the configured a_max.
  const double max_speed = std::max(0.1, trajectory_cfg.optimization.a_max);

  std::cout << "Configuration\n"
            << "  sim:                " << args.sim_config_path << "\n"
            << "  trajectory:         " << args.trajectory_config_path << "\n"
            << "  csv:                " << args.output_csv_path << "\n"
            << "  step_dt:            " << sim.step_dt << " s\n"
            << "  waypoints:          " << sim.waypoints.size() << "\n"
            << "  max_speed:          " << max_speed << " m/s\n"
            << "  a_max:              " << trajectory_cfg.optimization.a_max << " m/s^2\n"
            << "  solver:             "
            << (trajectory_cfg.optimization.solver == mtg::Solver::Linear ? "linear" : "nonlinear")
            << "\n"
            << std::endl;

  mtg::GeneratorConfig cfg;
  cfg.optimization = trajectory_cfg.optimization;

  mtg::TrajectoryGenerator generator(cfg);

  const std::vector<mtg::Waypoint> waypoints = buildWaypoints(sim.waypoints);
  if (!generator.generate(waypoints, max_speed)) {
    std::cerr << "Trajectory generation failed (check waypoints, limits, and step_dt)."
              << std::endl;
    return EXIT_FAILURE;
  }

  const double duration = generator.duration();
  std::cout << std::fixed << std::setprecision(3) << "Trajectory generated:\n"
            << "  duration:         " << duration << " s\n"
            << "  max velocity:     " << generator.maxVelocity() << " m/s\n"
            << "  max acceleration: " << generator.maxAcceleration() << " m/s^2\n"
            << std::endl;

  printSpline(generator.spline());

  const std::filesystem::path csv_path(args.output_csv_path);
  if (csv_path.has_parent_path()) {
    std::error_code ec;
    std::filesystem::create_directories(csv_path.parent_path(), ec);
    if (ec) {
      std::cerr << "Failed to create log directory '" << csv_path.parent_path().string()
                << "': " << ec.message() << std::endl;
      return EXIT_FAILURE;
    }
  }

  const std::string spline_csv_path = splineCsvPathFor(args.output_csv_path);
  if (generator.spline().saveToCsv(spline_csv_path)) {
    std::cout << "Spline written to:    " << spline_csv_path << std::endl;
  } else {
    std::cerr << "WARNING: failed to write spline CSV to " << spline_csv_path << std::endl;
  }

  std::ofstream csv(args.output_csv_path);
  if (!csv) {
    std::cerr << "error: cannot open " << args.output_csv_path << " for writing\n";
    return EXIT_FAILURE;
  }
  csv << std::setprecision(10);
  csv << "t,px,py,pz,vx,vy,vz,ax,ay,az\n";

  // Sample at [0, step_dt, ..., N*step_dt] and add a final sample exactly at
  // t = duration so the end waypoint is always present in the CSV.
  const std::size_t steps = static_cast<std::size_t>(std::floor(duration / sim.step_dt)) + 1;
  const std::size_t progress_every = std::max<std::size_t>(1, steps / 100);

  for (std::size_t i = 0; i < steps; ++i) {
    const double t                = std::min(static_cast<double>(i) * sim.step_dt, duration);
    const mtg::TrajectorySample s = generator.evaluate(t);
    csv << t << "," << s.position.x() << "," << s.position.y() << "," << s.position.z() << ","
        << s.velocity.x() << "," << s.velocity.y() << "," << s.velocity.z() << ","
        << s.acceleration.x() << "," << s.acceleration.y() << "," << s.acceleration.z() << "\n";
    if (i % progress_every == 0) {
      examples::printProgress(t / duration);
    }
  }
  // Final exact sample at t = duration.
  const mtg::TrajectorySample s_end = generator.evaluate(duration);
  csv << duration << "," << s_end.position.x() << "," << s_end.position.y() << ","
      << s_end.position.z() << "," << s_end.velocity.x() << "," << s_end.velocity.y() << ","
      << s_end.velocity.z() << "," << s_end.acceleration.x() << "," << s_end.acceleration.y() << ","
      << s_end.acceleration.z() << "\n";
  examples::printProgress(1.0);
  std::cout << std::endl;

  std::cout << "Done. CSV written to: " << args.output_csv_path << std::endl;
  return EXIT_SUCCESS;
}
