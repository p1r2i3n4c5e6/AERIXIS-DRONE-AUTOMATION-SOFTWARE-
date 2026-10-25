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

#ifndef MAV_TRAJECTORY_GENERATION_CPP_PYBIND_MAV_TRAJECTORY_GENERATION_CPP_PYBIND_HPP_
#define MAV_TRAJECTORY_GENERATION_CPP_PYBIND_MAV_TRAJECTORY_GENERATION_CPP_PYBIND_HPP_

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>  // std::vector, std::optional conversions

#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "mav_trajectory_generation_cpp/trajectory_generator.hpp"
#include "mav_trajectory_generation_cpp/types.hpp"

namespace mav_trajectory_generation_cpp {

inline void bindMavTrajGenCpp(pybind11::module& m) {
  namespace py = pybind11;

  py::enum_<Solver>(m, "Solver", "Solver backend for the polynomial trajectory optimisation.")
      .value("linear", Solver::Linear, "Closed-form linear least-squares solver.")
      .value("nonlinear", Solver::Nonlinear,
             "NLopt-backed free-endpoint solver with magnitude constraints.");

  py::class_<OptimizationConfig>(m, "OptimizationConfig",
                                 "Tunables for the polynomial trajectory optimiser.")
      .def(py::init<>())
      .def_readwrite("derivative_to_optimize", &OptimizationConfig::derivative_to_optimize,
                     "0=POSITION, 1=VELOCITY, 2=ACCELERATION, 3=JERK, 4=SNAP.")
      .def_readwrite("solver", &OptimizationConfig::solver)
      .def_readwrite("a_max", &OptimizationConfig::a_max, "Maximum acceleration magnitude [m/s^2].")
      .def_readwrite("nl_max_iterations", &OptimizationConfig::nl_max_iterations)
      .def_readwrite("nl_f_rel", &OptimizationConfig::nl_f_rel)
      .def_readwrite("nl_x_rel", &OptimizationConfig::nl_x_rel)
      .def_readwrite("nl_time_penalty", &OptimizationConfig::nl_time_penalty)
      .def_readwrite("nl_initial_stepsize_rel", &OptimizationConfig::nl_initial_stepsize_rel)
      .def_readwrite("nl_inequality_constraint_tolerance",
                     &OptimizationConfig::nl_inequality_constraint_tolerance)
      .def("__repr__", [](const OptimizationConfig& c) {
        std::ostringstream os;
        os << "OptimizationConfig(derivative_to_optimize=" << c.derivative_to_optimize
           << ", solver=" << (c.solver == Solver::Linear ? "linear" : "nonlinear")
           << ", a_max=" << c.a_max << ", nl_max_iterations=" << c.nl_max_iterations
           << ", nl_f_rel=" << c.nl_f_rel << ")";
        return os.str();
      });

  py::class_<GeneratorConfig>(m, "GeneratorConfig",
                              "Top-level configuration of TrajectoryGenerator.")
      .def(py::init<>())
      .def_readwrite("optimization", &GeneratorConfig::optimization,
                     "Solver / NLopt tuning (OptimizationConfig).")
      .def("__repr__", [](const GeneratorConfig& c) {
        std::ostringstream os;
        os << "GeneratorConfig(optimization="
           << "OptimizationConfig(solver="
           << (c.optimization.solver == Solver::Linear ? "linear" : "nonlinear")
           << ", a_max=" << c.optimization.a_max << "))";
        return os.str();
      });

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

  py::class_<Waypoint>(m, "Waypoint", "Waypoint with optional velocity/acceleration constraints.")
      .def(py::init<>())
      .def(py::init<const Eigen::Vector3d&>(), py::arg("position"),
           "Construct from a 3D position; velocity and acceleration stay unconstrained.")
      .def_readwrite("position", &Waypoint::position)
      .def_readwrite("velocity", &Waypoint::velocity,
                     "Optional velocity constraint [m/s]. None → free at this vertex.")
      .def_readwrite("acceleration", &Waypoint::acceleration,
                     "Optional acceleration constraint [m/s^2]. None → free at this vertex.")
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

  py::class_<TrajectoryGenerator>(m, "TrajectoryGenerator",
                                  "ROS-free wrapper around the ETH-ASL polynomial optimiser.")
      .def(py::init<const GeneratorConfig&>(), py::arg("config") = GeneratorConfig{},
           "Create a trajectory generator with the given GeneratorConfig.")
      .def("generate", &TrajectoryGenerator::generate, py::arg("waypoints"), py::arg("max_speed"),
           "Generate a 3D polynomial trajectory through the given waypoints at max_speed [m/s]. "
           "Each waypoint may carry optional velocity/acceleration constraints.")
      .def("is_valid", &TrajectoryGenerator::isValid,
           "Whether the last generate() call produced a usable trajectory.")
      .def("min_time", &TrajectoryGenerator::minTime, "Start time of the trajectory [s].")
      .def("max_time", &TrajectoryGenerator::maxTime, "End time of the trajectory [s].")
      .def("duration", &TrajectoryGenerator::duration,
           "Total duration of the planned trajectory [s].")
      .def("evaluate", &TrajectoryGenerator::evaluate, py::arg("t"),
           "Evaluate position/velocity/acceleration at time t (clamped to [min_time, max_time]).")
      .def("evaluate_derivative", &TrajectoryGenerator::evaluateDerivative, py::arg("t"),
           py::arg("derivative_order"), "Evaluate a single derivative order (0..4) at time t.")
      .def("position", &TrajectoryGenerator::position, py::arg("t"))
      .def("velocity", &TrajectoryGenerator::velocity, py::arg("t"))
      .def("acceleration", &TrajectoryGenerator::acceleration, py::arg("t"))
      .def("jerk", &TrajectoryGenerator::jerk, py::arg("t"))
      .def("max_velocity", &TrajectoryGenerator::maxVelocity,
           "Peak velocity magnitude along the trajectory [m/s].")
      .def("max_acceleration", &TrajectoryGenerator::maxAcceleration,
           "Peak acceleration magnitude along the trajectory [m/s^2].")
      .def("spline", &TrajectoryGenerator::spline,
           "Snapshot of the current spline; throws if is_valid() is False.")
      .def("set_spline", &TrajectoryGenerator::setSpline, py::arg("spline"),
           "Replace the current trajectory with an externally-provided Spline. "
           "Returns False on shape mismatch.");
}

}  // namespace mav_trajectory_generation_cpp

#endif  // MAV_TRAJECTORY_GENERATION_CPP_PYBIND_MAV_TRAJECTORY_GENERATION_CPP_PYBIND_HPP_
