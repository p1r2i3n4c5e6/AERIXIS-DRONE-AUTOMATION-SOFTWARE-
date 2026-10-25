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
 * @file utils.hpp
 * @brief Helpers used by the mav_trajectory_generation_cpp example executable.
 *
 * Header-only: declares and defines inline a minimal CLI parser, YAML config
 * loaders for the simulation and trajectory configuration files, and a small
 * progress bar. Intended for use only by example/run_example.cpp.
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#ifndef MAV_TRAJECTORY_GENERATION_CPP_EXAMPLE_UTILS_HPP_
#define MAV_TRAJECTORY_GENERATION_CPP_EXAMPLE_UTILS_HPP_

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

#include "mav_trajectory_generation_cpp/types.hpp"

namespace mav_trajectory_generation_cpp::examples {

struct SimConfig {
  double step_dt{0.01};
  std::vector<Eigen::Vector3d> waypoints;  // size >= 2, world frame (ENU)
};

/// Example-side wrapper around the library's @ref OptimizationConfig. The lib
/// has no extra example-specific knobs, so this struct only carries the
/// optimisation tuning.
struct TrajectoryConfig {
  mav_trajectory_generation_cpp::OptimizationConfig optimization{};
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

inline Solver readSolver(const YAML::Node &node, const std::string &path) {
  if (!node) {
    throw std::invalid_argument("Missing required key: " + path);
  }
  const std::string s = node.as<std::string>();
  if (s == "linear" || s == "Linear") return Solver::Linear;
  if (s == "nonlinear" || s == "Nonlinear") return Solver::Nonlinear;
  throw std::invalid_argument(path + " must be 'linear' or 'nonlinear', got '" + s + "'.");
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
  const YAML::Node root = detail::loadYamlRoot(path);
  const YAML::Node opt  = root["optimization"];
  if (!opt || !opt.IsMap()) {
    throw std::invalid_argument("optimization must be a mapping in " + path);
  }

  TrajectoryConfig cfg;
  if (opt["derivative_to_optimize"]) {
    cfg.optimization.derivative_to_optimize =
        detail::readInt(opt["derivative_to_optimize"], "optimization.derivative_to_optimize");
  }
  if (opt["solver"]) {
    cfg.optimization.solver = detail::readSolver(opt["solver"], "optimization.solver");
  }
  if (opt["a_max"]) {
    cfg.optimization.a_max = detail::readDouble(opt["a_max"], "optimization.a_max");
  }
  if (opt["nl_max_iterations"]) {
    cfg.optimization.nl_max_iterations =
        detail::readInt(opt["nl_max_iterations"], "optimization.nl_max_iterations");
  }
  if (opt["nl_f_rel"]) {
    cfg.optimization.nl_f_rel = detail::readDouble(opt["nl_f_rel"], "optimization.nl_f_rel");
  }
  if (opt["nl_x_rel"]) {
    cfg.optimization.nl_x_rel = detail::readDouble(opt["nl_x_rel"], "optimization.nl_x_rel");
  }
  if (opt["nl_time_penalty"]) {
    cfg.optimization.nl_time_penalty =
        detail::readDouble(opt["nl_time_penalty"], "optimization.nl_time_penalty");
  }
  if (opt["nl_initial_stepsize_rel"]) {
    cfg.optimization.nl_initial_stepsize_rel =
        detail::readDouble(opt["nl_initial_stepsize_rel"], "optimization.nl_initial_stepsize_rel");
  }
  if (opt["nl_inequality_constraint_tolerance"]) {
    cfg.optimization.nl_inequality_constraint_tolerance =
        detail::readDouble(opt["nl_inequality_constraint_tolerance"],
                           "optimization.nl_inequality_constraint_tolerance");
  }
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

}  // namespace mav_trajectory_generation_cpp::examples

#endif  // MAV_TRAJECTORY_GENERATION_CPP_EXAMPLE_UTILS_HPP_
