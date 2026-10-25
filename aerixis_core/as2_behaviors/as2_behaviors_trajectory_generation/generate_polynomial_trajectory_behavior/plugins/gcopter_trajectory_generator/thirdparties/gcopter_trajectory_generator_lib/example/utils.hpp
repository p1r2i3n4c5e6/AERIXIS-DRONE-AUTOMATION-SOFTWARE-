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
 * @file utils.hpp
 * @brief Helpers used by the gcopter_lib example executable.
 *
 * Header-only: declares and defines inline a minimal CLI parser, YAML config
 * loaders for the simulation and trajectory configuration files, and a small
 * progress bar. Intended for use only by example/run_example.cpp.
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#ifndef GCOPTER_LIB_EXAMPLE_UTILS_HPP_
#define GCOPTER_LIB_EXAMPLE_UTILS_HPP_

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "gcopter_lib/types.hpp"

namespace gcopter_lib::examples {

struct SimConfig {
  double step_dt{0.01};
  std::vector<Eigen::Vector3d> waypoints;  // size >= 2, world frame (ENU)
};

struct GeneratorConfig {
  double corridor_margin{2.0};
  // Optional: if both > 0, the trajectory is built with anchor points inserted
  // at +/- waypoint_anchor_radius around every intermediate waypoint, and the
  // pinch segments (anchor_in -> waypoint and waypoint -> anchor_out) use
  // waypoint_margin instead of corridor_margin. This forces the trajectory to
  // pass close to each intermediate waypoint while keeping the rest loose.
  // Set to 0 to disable the mechanism (uniform corridor_margin everywhere).
  double waypoint_margin{0.0};
  double waypoint_anchor_radius{0.0};
};

struct TrajectoryConfig {
  GeneratorConfig generator;
  gcopter_lib::DroneParameters params;
  gcopter_lib::DroneLimits limits;
  gcopter_lib::OptimizationConfig optimization;
};

struct CliArgs {
  std::string sim_config_path{"example/config_example.yaml"};
  std::string trajectory_config_path{"example/config_trajectory.yaml"};
  std::string output_csv_path{"example/trajectory_logs/trajectory.csv"};
};

namespace detail {

inline YAML::Node loadYamlRoot(const std::string &path) {
  if (!std::filesystem::exists(path)) {
    throw std::invalid_argument("Config file not found at " +
                                std::filesystem::absolute(path).string() + ".");
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception &e) {
    throw std::invalid_argument("Failed to parse YAML file '" + path + "': " + e.what());
  }

  if (!root || !root.IsMap()) {
    throw std::invalid_argument("Root of '" + path + "' must be a YAML mapping.");
  }
  return root;
}

inline double readDouble(const YAML::Node &node, const std::string &path) {
  if (!node) {
    throw std::invalid_argument("Missing required key: " + path);
  }
  try {
    return node.as<double>();
  } catch (const YAML::Exception &) {
    throw std::invalid_argument(path + " must be a numeric value.");
  }
}

inline int readInt(const YAML::Node &node, const std::string &path) {
  if (!node) {
    throw std::invalid_argument("Missing required key: " + path);
  }
  try {
    return node.as<int>();
  } catch (const YAML::Exception &) {
    throw std::invalid_argument(path + " must be an integer value.");
  }
}

inline Eigen::Vector3d readVector3(const YAML::Node &node, const std::string &path) {
  if (!node || !node.IsSequence() || node.size() != 3) {
    throw std::invalid_argument(path + " must be a 3-element sequence [x, y, z].");
  }
  Eigen::Vector3d v;
  for (int i = 0; i < 3; ++i) {
    try {
      v(i) = node[i].as<double>();
    } catch (const YAML::Exception &) {
      throw std::invalid_argument(path + " must contain numeric values.");
    }
  }
  return v;
}

}  // namespace detail

inline SimConfig loadSimConfig(const std::string &path) {
  const YAML::Node root = detail::loadYamlRoot(path);
  const YAML::Node sim  = root["sim_config"];
  if (!sim || !sim.IsMap()) {
    throw std::invalid_argument("sim_config must be a mapping in " + path);
  }

  SimConfig cfg;
  cfg.step_dt = detail::readDouble(sim["step_dt"], "sim_config.step_dt");

  if (cfg.step_dt <= 0.0) {
    throw std::invalid_argument("sim_config.step_dt must be > 0.");
  }

  const YAML::Node waypoints = sim["waypoints"];
  if (!waypoints || !waypoints.IsSequence()) {
    throw std::invalid_argument("sim_config.waypoints must be a sequence.");
  }

  cfg.waypoints.reserve(waypoints.size());
  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    cfg.waypoints.emplace_back(
        detail::readVector3(waypoints[i], "sim_config.waypoints[" + std::to_string(i) + "]"));
  }

  if (cfg.waypoints.size() < 2) {
    throw std::invalid_argument("sim_config.waypoints must contain at least 2 waypoints.");
  }
  return cfg;
}

inline TrajectoryConfig loadTrajectoryConfig(const std::string &path) {
  const YAML::Node root      = detail::loadYamlRoot(path);
  const YAML::Node generator = root["trajectory_generator"];
  const YAML::Node params    = root["drone_params"];
  const YAML::Node limits    = root["drone_limits"];
  const YAML::Node opt       = root["optimization"];
  if (!generator || !generator.IsMap()) {
    throw std::invalid_argument("trajectory_generator must be a mapping in " + path);
  }
  if (!params || !params.IsMap()) {
    throw std::invalid_argument("drone_params must be a mapping in " + path);
  }
  if (!limits || !limits.IsMap()) {
    throw std::invalid_argument("drone_limits must be a mapping in " + path);
  }
  if (!opt || !opt.IsMap()) {
    throw std::invalid_argument("optimization must be a mapping in " + path);
  }

  TrajectoryConfig cfg;
  cfg.generator.corridor_margin =
      detail::readDouble(generator["corridor_margin"], "trajectory_generator.corridor_margin");
  if (cfg.generator.corridor_margin <= 0.0) {
    throw std::invalid_argument("trajectory_generator.corridor_margin must be > 0.");
  }

  if (generator["waypoint_margin"]) {
    cfg.generator.waypoint_margin =
        detail::readDouble(generator["waypoint_margin"], "trajectory_generator.waypoint_margin");
    if (cfg.generator.waypoint_margin < 0.0) {
      throw std::invalid_argument("trajectory_generator.waypoint_margin must be >= 0.");
    }
  }
  if (generator["waypoint_anchor_radius"]) {
    cfg.generator.waypoint_anchor_radius = detail::readDouble(
        generator["waypoint_anchor_radius"], "trajectory_generator.waypoint_anchor_radius");
    if (cfg.generator.waypoint_anchor_radius < 0.0) {
      throw std::invalid_argument("trajectory_generator.waypoint_anchor_radius must be >= 0.");
    }
  }
  if ((cfg.generator.waypoint_margin > 0.0) != (cfg.generator.waypoint_anchor_radius > 0.0)) {
    throw std::invalid_argument(
        "trajectory_generator.waypoint_margin and "
        "trajectory_generator.waypoint_anchor_radius must either both be > 0 "
        "(to enable per-waypoint tightening) or both be zero/omitted.");
  }

  cfg.params.mass    = detail::readDouble(params["mass"], "drone_params.mass");
  cfg.params.gravity = detail::readDouble(params["gravity"], "drone_params.gravity");
  cfg.params.horizontal_drag =
      detail::readDouble(params["horizontal_drag"], "drone_params.horizontal_drag");
  cfg.params.vertical_drag =
      detail::readDouble(params["vertical_drag"], "drone_params.vertical_drag");
  cfg.params.parasitic_drag =
      detail::readDouble(params["parasitic_drag"], "drone_params.parasitic_drag");
  cfg.params.speed_smooth_factor =
      detail::readDouble(params["speed_smooth_factor"], "drone_params.speed_smooth_factor");

  cfg.limits.max_velocity = detail::readDouble(limits["max_velocity"], "drone_limits.max_velocity");
  cfg.limits.max_body_rate =
      detail::readDouble(limits["max_body_rate"], "drone_limits.max_body_rate");
  cfg.limits.max_tilt_angle =
      detail::readDouble(limits["max_tilt_angle"], "drone_limits.max_tilt_angle");
  cfg.limits.min_thrust = detail::readDouble(limits["min_thrust"], "drone_limits.min_thrust");
  cfg.limits.max_thrust = detail::readDouble(limits["max_thrust"], "drone_limits.max_thrust");

  cfg.optimization.time_weight = detail::readDouble(opt["time_weight"], "optimization.time_weight");
  cfg.optimization.position_weight =
      detail::readDouble(opt["position_weight"], "optimization.position_weight");
  cfg.optimization.velocity_weight =
      detail::readDouble(opt["velocity_weight"], "optimization.velocity_weight");
  cfg.optimization.body_rate_weight =
      detail::readDouble(opt["body_rate_weight"], "optimization.body_rate_weight");
  cfg.optimization.tilt_weight = detail::readDouble(opt["tilt_weight"], "optimization.tilt_weight");
  cfg.optimization.thrust_weight =
      detail::readDouble(opt["thrust_weight"], "optimization.thrust_weight");
  cfg.optimization.smoothing_eps =
      detail::readDouble(opt["smoothing_eps"], "optimization.smoothing_eps");
  cfg.optimization.integral_resolution =
      detail::readInt(opt["integral_resolution"], "optimization.integral_resolution");
  cfg.optimization.rel_cost_tol =
      detail::readDouble(opt["rel_cost_tol"], "optimization.rel_cost_tol");
  cfg.optimization.corridor_margin = cfg.generator.corridor_margin;
  return cfg;
}

inline void printCliHelp(const char *program_name) {
  std::cout << "Usage: " << program_name << " [options]\n"
            << "\n"
            << "Options:\n"
            << "  -c <path>   Sim config YAML   (default: example/config_example.yaml)\n"
            << "  -d <path>   Trajectory config YAML (default: example/config_trajectory.yaml)\n"
            << "  -f <path>   Output CSV path   (default: example/trajectory_logs/trajectory.csv)\n"
            << "  -h          Show this help\n";
}

inline CliArgs parseCli(int argc, char **argv) {
  CliArgs args;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next_value       = [&](const std::string &flag) -> std::string {
      if (i + 1 >= argc) {
        throw std::invalid_argument("Missing value for " + flag);
      }
      return argv[++i];
    };

    if (arg == "-c") {
      args.sim_config_path = next_value(arg);
    } else if (arg == "-d") {
      args.trajectory_config_path = next_value(arg);
    } else if (arg == "-f") {
      args.output_csv_path = next_value(arg);
    } else if (arg == "-h" || arg == "--help") {
      printCliHelp(argv[0]);
      std::exit(EXIT_SUCCESS);
    } else {
      throw std::invalid_argument("Unknown argument: " + arg);
    }
  }
  return args;
}

inline void printProgress(double fraction) {
  constexpr int kWidth = 40;
  const double clamped = std::clamp(fraction, 0.0, 1.0);
  const int pos        = static_cast<int>(kWidth * clamped);

  std::cout << "\r[";
  for (int i = 0; i < kWidth; ++i) {
    std::cout << (i < pos ? '=' : (i == pos ? '>' : ' '));
  }
  std::cout << "] " << static_cast<int>(clamped * 100.0) << "%" << std::flush;
}

}  // namespace gcopter_lib::examples

#endif  // GCOPTER_LIB_EXAMPLE_UTILS_HPP_
