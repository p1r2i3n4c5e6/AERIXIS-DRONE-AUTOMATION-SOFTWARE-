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

#pragma once

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>  // std::vector, std::optional conversions

#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "gcopter_lib/corridor_builder.hpp"
#include "gcopter_lib/trajectory_generator.hpp"
#include "gcopter_lib/trajectory_logger.hpp"
#include "gcopter_lib/types.hpp"

namespace py = pybind11;

namespace gcopter_lib {

inline void bindGcopterLib(py::module& m) {
  // ──────────────────────────────────────────────────────────────
  // DroneParameters
  // ──────────────────────────────────────────────────────────────
  py::class_<DroneParameters>(m, "DroneParameters")
      .def(py::init<>())
      .def_readwrite("mass", &DroneParameters::mass)
      .def_readwrite("gravity", &DroneParameters::gravity)
      .def_readwrite("horizontal_drag", &DroneParameters::horizontal_drag)
      .def_readwrite("vertical_drag", &DroneParameters::vertical_drag)
      .def_readwrite("parasitic_drag", &DroneParameters::parasitic_drag)
      .def_readwrite("speed_smooth_factor", &DroneParameters::speed_smooth_factor)
      .def("__repr__", [](const DroneParameters& p) {
        return "DroneParameters(mass=" + std::to_string(p.mass) +
               ", gravity=" + std::to_string(p.gravity) + ")";
      });

  // ──────────────────────────────────────────────────────────────
  // DroneLimits
  // ──────────────────────────────────────────────────────────────
  py::class_<DroneLimits>(m, "DroneLimits")
      .def(py::init<>())
      .def_readwrite("max_velocity", &DroneLimits::max_velocity)
      .def_readwrite("max_body_rate", &DroneLimits::max_body_rate)
      .def_readwrite("max_tilt_angle", &DroneLimits::max_tilt_angle)
      .def_readwrite("min_thrust", &DroneLimits::min_thrust)
      .def_readwrite("max_thrust", &DroneLimits::max_thrust)
      .def("__repr__", [](const DroneLimits& l) {
        return "DroneLimits(max_velocity=" + std::to_string(l.max_velocity) +
               ", max_thrust=" + std::to_string(l.max_thrust) + ")";
      });

  // ──────────────────────────────────────────────────────────────
  // OptimizationConfig
  // ──────────────────────────────────────────────────────────────
  py::class_<OptimizationConfig>(m, "OptimizationConfig")
      .def(py::init<>())
      .def_readwrite("time_weight", &OptimizationConfig::time_weight)
      .def_readwrite("position_weight", &OptimizationConfig::position_weight)
      .def_readwrite("velocity_weight", &OptimizationConfig::velocity_weight)
      .def_readwrite("body_rate_weight", &OptimizationConfig::body_rate_weight)
      .def_readwrite("tilt_weight", &OptimizationConfig::tilt_weight)
      .def_readwrite("thrust_weight", &OptimizationConfig::thrust_weight)
      .def_readwrite("smoothing_eps", &OptimizationConfig::smoothing_eps)
      .def_readwrite("integral_resolution", &OptimizationConfig::integral_resolution)
      .def_readwrite("rel_cost_tol", &OptimizationConfig::rel_cost_tol)
      .def_readwrite("length_per_piece", &OptimizationConfig::length_per_piece)
      .def_readwrite("corridor_margin", &OptimizationConfig::corridor_margin,
                     "Default AABB half-margin per face [m] used by the single-margin "
                     "generate(waypoints, max_speed) overload.");

  // ──────────────────────────────────────────────────────────────
  // GeneratorConfig
  // ──────────────────────────────────────────────────────────────
  py::class_<GeneratorConfig>(m, "GeneratorConfig",
                              "Top-level configuration of TrajectoryGenerator.")
      .def(py::init<>())
      .def_readwrite("params", &GeneratorConfig::params,
                     "Vehicle physical parameters (DroneParameters).")
      .def_readwrite("limits", &GeneratorConfig::limits,
                     "Hard magnitude bounds passed to GCOPTER (DroneLimits).")
      .def_readwrite("optimization", &GeneratorConfig::optimization,
                     "L-BFGS solver tuning (OptimizationConfig).");

  // ──────────────────────────────────────────────────────────────
  // Waypoint / EndWaypoint
  // ──────────────────────────────────────────────────────────────
  py::class_<Waypoint>(m, "Waypoint", "Waypoint with optional velocity/acceleration constraints.")
      .def(py::init<>())
      .def(py::init<const Eigen::Vector3d&>(), py::arg("position"),
           "Construct from a 3D position; velocity and acceleration stay unconstrained.")
      .def_readwrite("position", &Waypoint::position)
      .def_readwrite("velocity", &Waypoint::velocity,
                     "Optional velocity constraint [m/s]. None → defaults to zero at endpoints, "
                     "ignored at intermediate waypoints.")
      .def_readwrite("acceleration", &Waypoint::acceleration,
                     "Optional acceleration constraint [m/s^2]. None → defaults to zero at "
                     "endpoints, ignored at intermediate waypoints.")
      .def("__repr__", [](const Waypoint& w) {
        std::ostringstream os;
        os << "Waypoint(position=[" << w.position.transpose() << "]";
        if (w.velocity) os << ", velocity=[" << w.velocity->transpose() << "]";
        if (w.acceleration) os << ", acceleration=[" << w.acceleration->transpose() << "]";
        os << ")";
        return os.str();
      });

  py::class_<EndWaypoint, Waypoint>(m, "EndWaypoint",
                                    "Waypoint with velocity and acceleration pinned to zero.")
      .def(py::init<const Eigen::Vector3d&>(), py::arg("position"),
           "Build an EndWaypoint at the given position (vel = 0, acc = 0).");

  // ──────────────────────────────────────────────────────────────
  // TrajectorySample
  // ──────────────────────────────────────────────────────────────
  py::class_<TrajectorySample>(m, "TrajectorySample",
                               "Polynomial trajectory state at a single time instant.")
      .def(py::init<>())
      .def_readonly("position", &TrajectorySample::position)
      .def_readonly("velocity", &TrajectorySample::velocity)
      .def_readonly("acceleration", &TrajectorySample::acceleration)
      .def("__repr__", [](const TrajectorySample& s) {
        std::ostringstream os;
        os << "TrajectorySample(position=[" << s.position.transpose() << "], velocity=["
           << s.velocity.transpose() << "], acceleration=[" << s.acceleration.transpose() << "])";
        return os.str();
      });

  // ──────────────────────────────────────────────────────────────
  // SegmentPolynomial / Spline
  // ──────────────────────────────────────────────────────────────
  py::class_<SegmentPolynomial>(m, "SegmentPolynomial",
                                "One polynomial segment: duration [s] and per-axis "
                                "coefficients (rows = axes, columns = powers of tau).")
      .def(py::init<>())
      .def_readwrite("duration", &SegmentPolynomial::duration)
      .def_readwrite("coefficients", &SegmentPolynomial::coefficients)
      .def("__repr__", [](const SegmentPolynomial& s) {
        std::ostringstream os;
        os << "SegmentPolynomial(duration=" << s.duration << ", coefficients_shape=("
           << s.coefficients.rows() << ", " << s.coefficients.cols() << "))";
        return os.str();
      });

  py::class_<Spline>(m, "Spline", "Piecewise polynomial trajectory: start_time + ordered segments.")
      .def(py::init<>())
      .def_readwrite("start_time", &Spline::start_time,
                     "Absolute time of the first breakpoint [s].")
      .def_readwrite("segments", &Spline::segments, "Ordered list of SegmentPolynomial entries.")
      .def("segment_times", &Spline::segmentTimes, "Segment durations [s] in temporal order.")
      .def("breakpoints", &Spline::breakpoints, "Absolute-time knots [s], length len(segments)+1.")
      .def("duration", &Spline::duration, "Total duration [s].")
      .def("min_time", &Spline::minTime, "Alias for start_time.")
      .def("max_time", &Spline::maxTime, "start_time + duration().")
      .def("save_to_csv", &Spline::saveToCsv, py::arg("path"),
           "Serialise this spline to a CSV file. Returns False on I/O error.")
      .def("load_from_csv", &Spline::loadFromCsv, py::arg("path"),
           "Load a spline from a CSV previously written by save_to_csv(). "
           "Populates self and returns True on success.")
      .def("__repr__", [](const Spline& s) {
        std::ostringstream os;
        os << "Spline(start_time=" << s.start_time << ", segments=" << s.segments.size()
           << ", duration=" << s.duration() << ")";
        return os.str();
      });

  // ──────────────────────────────────────────────────────────────
  // TrajectoryGenerator
  // ──────────────────────────────────────────────────────────────
  py::class_<TrajectoryGenerator>(m, "TrajectoryGenerator")
      .def(py::init<const GeneratorConfig&>(), py::arg("config") = GeneratorConfig{},
           "Construct a TrajectoryGenerator from a GeneratorConfig.")
      // generate(waypoints, max_speed)
      .def("generate",
           py::overload_cast<const std::vector<Waypoint>&, double>(&TrajectoryGenerator::generate),
           py::arg("waypoints"), py::arg("max_speed"),
           "Generate a 3D polynomial trajectory through the given waypoints at max_speed [m/s]. "
           "Uses OptimizationConfig.corridor_margin as the AABB half-margin.")
      // generate(waypoints, max_speed, corridor_margins)  — gcopter-only extension
      .def("generate",
           py::overload_cast<const std::vector<Waypoint>&, double, const std::vector<double>&>(
               &TrajectoryGenerator::generate),
           py::arg("waypoints"), py::arg("max_speed"), py::arg("corridor_margins"),
           "Generate with per-segment AABB margins. corridor_margins.size() must equal "
           "waypoints.size() - 1.")
      .def("is_valid", &TrajectoryGenerator::isValid)
      .def("min_time", &TrajectoryGenerator::minTime, "Start time of the trajectory [s].")
      .def("max_time", &TrajectoryGenerator::maxTime, "End time of the trajectory [s].")
      .def("duration", &TrajectoryGenerator::duration)
      .def("evaluate", &TrajectoryGenerator::evaluate, py::arg("t"),
           "Evaluate position/velocity/acceleration at time t (clamped to [min_time, max_time]).")
      .def("evaluate_derivative", &TrajectoryGenerator::evaluateDerivative, py::arg("t"),
           py::arg("derivative_order"),
           "Evaluate a single derivative order (0..3) at time t; higher orders return zero.")
      .def("position", &TrajectoryGenerator::position, py::arg("t"))
      .def("velocity", &TrajectoryGenerator::velocity, py::arg("t"))
      .def("acceleration", &TrajectoryGenerator::acceleration, py::arg("t"))
      .def("jerk", &TrajectoryGenerator::jerk, py::arg("t"))
      .def("max_velocity", &TrajectoryGenerator::maxVelocity)
      .def("max_acceleration", &TrajectoryGenerator::maxAcceleration)
      .def("spline", &TrajectoryGenerator::spline,
           "Snapshot of the current spline; throws if is_valid() is False.");

  // ──────────────────────────────────────────────────────────────
  // TrajectoryLogger
  // ──────────────────────────────────────────────────────────────
  py::class_<TrajectoryLogger>(m, "TrajectoryLogger")
      .def(py::init<const std::string&>(), py::arg("file_name"))
      .def("save", &TrajectoryLogger::save, py::arg("t"), py::arg("position"), py::arg("velocity"),
           py::arg("acceleration"))
      .def("close", &TrajectoryLogger::close);

  // ──────────────────────────────────────────────────────────────
  // Free functions
  // ──────────────────────────────────────────────────────────────
  m.def("build_aabb_corridor",
        py::overload_cast<const std::vector<Eigen::Vector3d>&, double>(&buildAabbCorridor),
        py::arg("waypoints"), py::arg("margin"),
        "Build an AABB Safe Flight Corridor with a uniform margin per segment.");
  m.def("build_aabb_corridor",
        py::overload_cast<const std::vector<Eigen::Vector3d>&, const std::vector<double>&>(
            &buildAabbCorridor),
        py::arg("waypoints"), py::arg("margins"),
        "Build an AABB Safe Flight Corridor with per-segment margins.");
}

}  // namespace gcopter_lib
