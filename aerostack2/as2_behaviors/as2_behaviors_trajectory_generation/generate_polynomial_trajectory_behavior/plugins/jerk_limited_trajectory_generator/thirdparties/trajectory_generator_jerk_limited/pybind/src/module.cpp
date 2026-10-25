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
 * @file module.cpp
 *
 * Python bindings for trajectory_generator_jerk_limited. Exposes the full
 * C++ API (parameters, 1D profile, 3D generator, waypoint controller) in a
 * single module. Eigen vectors are marshalled to/from numpy.ndarray via
 * pybind11/eigen.h.
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#include <array>
#include <sstream>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "trajectory_generator_jerk_limited/parameters.hpp"
#include "trajectory_generator_jerk_limited/trajectory_generator.hpp"
#include "trajectory_generator_jerk_limited/trajectory_generator_3d.hpp"
#include "trajectory_generator_jerk_limited/types.hpp"
#include "trajectory_generator_jerk_limited/velocity_profile_1d.hpp"
#include "trajectory_generator_jerk_limited/waypoint_controller.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_trajectory_generator_jerk_limited_cpp, m) {
  namespace tg = trajectory_generator_jerk_limited;

  m.doc() = "Python bindings for trajectory_generator_jerk_limited";

  // --- TrajectoryParameters -------------------------------------------------
  py::class_<tg::TrajectoryParameters>(m, "_TrajectoryParameters")
      .def(py::init<>())
      .def_readwrite("max_speed", &tg::TrajectoryParameters::max_speed)
      .def_readwrite("max_acceleration", &tg::TrajectoryParameters::max_acceleration)
      .def_readwrite("max_jerk", &tg::TrajectoryParameters::max_jerk)
      .def_readwrite("max_tracking_error", &tg::TrajectoryParameters::max_tracking_error)
      .def_readwrite("acceptance_radius", &tg::TrajectoryParameters::acceptance_radius)
      .def_readwrite("trajectory_gain", &tg::TrajectoryParameters::trajectory_gain);

  // --- TrajectorySetpoints --------------------------------------------------
  py::class_<tg::TrajectorySetpoints>(m, "_TrajectorySetpoints")
      .def(py::init<>())
      .def_readwrite("jerk", &tg::TrajectorySetpoints::jerk)
      .def_readwrite("acceleration", &tg::TrajectorySetpoints::acceleration)
      .def_readwrite("velocity", &tg::TrajectorySetpoints::velocity)
      .def_readwrite("position", &tg::TrajectorySetpoints::position)
      .def_readwrite("unsmoothed_velocity", &tg::TrajectorySetpoints::unsmoothed_velocity);

  // --- JerkLimitedVelocityProfile1D -----------------------------------------
  py::class_<tg::JerkLimitedVelocityProfile1D>(m, "_JerkLimitedVelocityProfile1D")
      .def(py::init<>())
      .def(py::init<double, double, double>(), py::arg("initial_acceleration"),
           py::arg("initial_velocity"), py::arg("initial_position"))
      .def("reset", &tg::JerkLimitedVelocityProfile1D::reset, py::arg("acceleration"),
           py::arg("velocity"), py::arg("position"))
      .def("set_max_jerk", &tg::JerkLimitedVelocityProfile1D::setMaxJerk, py::arg("value"))
      .def("set_max_acceleration", &tg::JerkLimitedVelocityProfile1D::setMaxAcceleration,
           py::arg("value"))
      .def("set_max_velocity", &tg::JerkLimitedVelocityProfile1D::setMaxVelocity, py::arg("value"))
      .def("max_jerk", &tg::JerkLimitedVelocityProfile1D::maxJerk)
      .def("max_acceleration", &tg::JerkLimitedVelocityProfile1D::maxAcceleration)
      .def("max_velocity", &tg::JerkLimitedVelocityProfile1D::maxVelocity)
      .def("current_jerk", &tg::JerkLimitedVelocityProfile1D::currentJerk)
      .def("current_acceleration", &tg::JerkLimitedVelocityProfile1D::currentAcceleration)
      .def("current_velocity", &tg::JerkLimitedVelocityProfile1D::currentVelocity)
      .def("current_position", &tg::JerkLimitedVelocityProfile1D::currentPosition)
      .def("phase1_duration", &tg::JerkLimitedVelocityProfile1D::phase1Duration)
      .def("phase2_duration", &tg::JerkLimitedVelocityProfile1D::phase2Duration)
      .def("phase3_duration", &tg::JerkLimitedVelocityProfile1D::phase3Duration)
      .def("total_time", &tg::JerkLimitedVelocityProfile1D::totalTime)
      .def("update_durations", &tg::JerkLimitedVelocityProfile1D::updateDurations,
           py::arg("velocity_setpoint"))
      .def("update_trajectory", &tg::JerkLimitedVelocityProfile1D::updateTrajectory, py::arg("dt"),
           py::arg("time_stretch") = 1.0);

  // --- JerkLimitedTrajectoryGenerator3D -------------------------------------
  py::class_<tg::JerkLimitedTrajectoryGenerator3D>(m, "_JerkLimitedTrajectoryGenerator3D")
      .def(py::init<>())
      .def("set_max_jerk", &tg::JerkLimitedTrajectoryGenerator3D::setMaxJerk, py::arg("jerk"))
      .def("set_max_speed", &tg::JerkLimitedTrajectoryGenerator3D::setMaxSpeed, py::arg("speed"))
      .def("set_max_acceleration", &tg::JerkLimitedTrajectoryGenerator3D::setMaxAcceleration,
           py::arg("acceleration"))
      .def("set_max_tracking_error", &tg::JerkLimitedTrajectoryGenerator3D::setMaxTrackingError,
           py::arg("error"))
      .def("set_acceptance_radius", &tg::JerkLimitedTrajectoryGenerator3D::setAcceptanceRadius,
           py::arg("radius"))
      .def("set_trajectory_gain", &tg::JerkLimitedTrajectoryGenerator3D::setTrajectoryGain,
           py::arg("gain"))
      .def("current_position", &tg::JerkLimitedTrajectoryGenerator3D::currentPosition)
      .def("current_velocity", &tg::JerkLimitedTrajectoryGenerator3D::currentVelocity)
      .def("current_speed", &tg::JerkLimitedTrajectoryGenerator3D::currentSpeed)
      .def("reset", &tg::JerkLimitedTrajectoryGenerator3D::reset, py::arg("acceleration"),
           py::arg("velocity"), py::arg("position"))
      .def("generate",
           py::overload_cast<const Eigen::Vector3d&, const Eigen::Vector3d&, const Eigen::Vector3d&,
                             const double, const bool>(
               &tg::JerkLimitedTrajectoryGenerator3D::generate),
           py::arg("current_position"), py::arg("waypoint"), py::arg("feedforward_velocity"),
           py::arg("dt"), py::arg("force_zero_velocity_setpoint") = false)
      .def("generate",
           py::overload_cast<const Eigen::Vector3d&, const std::array<Eigen::Vector3d, 3>&,
                             const Eigen::Vector3d&, const double, const bool>(
               &tg::JerkLimitedTrajectoryGenerator3D::generate),
           py::arg("current_position"), py::arg("waypoints"), py::arg("feedforward_velocity"),
           py::arg("dt"), py::arg("force_zero_velocity_setpoint") = false);

  // --- Public Waypoint / Spline / Sample types ------------------------------
  py::class_<tg::Waypoint>(m, "Waypoint",
                           "Waypoint with optional velocity/acceleration constraints.")
      .def(py::init<>())
      .def(py::init<const Eigen::Vector3d&>(), py::arg("position"),
           "Construct a position-only waypoint.")
      .def_readwrite("position", &tg::Waypoint::position)
      .def_readwrite("velocity", &tg::Waypoint::velocity,
                     "Optional velocity constraint at this waypoint (None = unconstrained).")
      .def_readwrite("acceleration", &tg::Waypoint::acceleration,
                     "Optional acceleration constraint at this waypoint (None = unconstrained).")
      .def("__repr__", [](const tg::Waypoint& w) {
        std::ostringstream os;
        os << "Waypoint(position=[" << w.position.transpose() << "])";
        return os.str();
      });

  py::class_<tg::EndWaypoint, tg::Waypoint>(m, "EndWaypoint",
                                            "Endpoint waypoint with velocity and acceleration "
                                            "pinned to zero.")
      .def(py::init<const Eigen::Vector3d&>(), py::arg("position"));

  // --- WaypointTrajectoryController -----------------------------------------
  py::class_<tg::WaypointTrajectoryController>(m, "_WaypointTrajectoryController")
      .def(py::init<const tg::TrajectoryParameters&>(), py::arg("parameters"))
      .def("reset",
           py::overload_cast<const Eigen::Vector3d&>(&tg::WaypointTrajectoryController::reset),
           py::arg("initial_position"))
      .def("reset",
           py::overload_cast<const tg::Waypoint&>(&tg::WaypointTrajectoryController::reset),
           py::arg("initial_state"))
      .def("update",
           py::overload_cast<const double, const Eigen::Vector3d&, const Eigen::Vector3d&>(
               &tg::WaypointTrajectoryController::update),
           py::arg("dt"), py::arg("current_position"), py::arg("waypoint"))
      .def("update",
           py::overload_cast<const double, const Eigen::Vector3d&,
                             const std::array<Eigen::Vector3d, 3>&>(
               &tg::WaypointTrajectoryController::update),
           py::arg("dt"), py::arg("current_position"), py::arg("waypoints"))
      .def("parameters", &tg::WaypointTrajectoryController::parameters,
           py::return_value_policy::reference_internal);

  py::class_<tg::TrajectorySample>(m, "TrajectorySample",
                                   "Trajectory state at a single time instant "
                                   "(position, velocity, acceleration).")
      .def(py::init<>())
      .def_readwrite("position", &tg::TrajectorySample::position)
      .def_readwrite("velocity", &tg::TrajectorySample::velocity)
      .def_readwrite("acceleration", &tg::TrajectorySample::acceleration);

  py::class_<tg::SegmentPolynomial>(m, "SegmentPolynomial",
                                    "Single piecewise-polynomial segment "
                                    "(duration + per-axis coefficients).")
      .def(py::init<>())
      .def_readwrite("duration", &tg::SegmentPolynomial::duration)
      .def_readwrite("coefficients", &tg::SegmentPolynomial::coefficients);

  py::class_<tg::Spline>(m, "Spline",
                         "Piecewise polynomial trajectory: start_time + ordered segments.")
      .def(py::init<>())
      .def_readwrite("start_time", &tg::Spline::start_time)
      .def_readwrite("segments", &tg::Spline::segments)
      .def("segment_times", &tg::Spline::segmentTimes)
      .def("breakpoints", &tg::Spline::breakpoints)
      .def("duration", &tg::Spline::duration)
      .def("min_time", &tg::Spline::minTime)
      .def("max_time", &tg::Spline::maxTime)
      .def("save_to_csv", &tg::Spline::saveToCsv, py::arg("path"),
           "Serialise the spline to a CSV file. Returns true on success.")
      .def("load_from_csv", &tg::Spline::loadFromCsv, py::arg("path"),
           "Load the spline from a CSV. Returns true on success.");

  // --- SimulatorConfig -------------------------------------------------------
  py::class_<tg::SimulatorConfig>(m, "SimulatorConfig",
                                  "Implementation knobs of the internal simulation loop.")
      .def(py::init<>())
      .def_readwrite("step_dt", &tg::SimulatorConfig::step_dt)
      .def_readwrite("max_simulation_time", &tg::SimulatorConfig::max_simulation_time)
      .def_readwrite("settle_velocity", &tg::SimulatorConfig::settle_velocity)
      .def_readwrite("polynomial_order", &tg::SimulatorConfig::polynomial_order);

  // --- GeneratorConfig -------------------------------------------------------
  py::class_<tg::GeneratorConfig>(m, "GeneratorConfig",
                                  "Tunables for the one-shot TrajectoryGenerator.")
      .def(py::init<>())
      .def_readwrite("params", &tg::GeneratorConfig::params)
      .def_readwrite("simulator", &tg::GeneratorConfig::simulator);

  // --- TrajectoryGenerator (one-shot wrapper) --------------------------------
  py::class_<tg::TrajectoryGenerator>(m, "TrajectoryGenerator",
                                      "One-shot multi-waypoint trajectory generator.")
      .def(py::init<const tg::GeneratorConfig&>(), py::arg("config") = tg::GeneratorConfig{})
      .def("generate", &tg::TrajectoryGenerator::generate, py::arg("waypoints"),
           py::arg("max_speed"), "Simulate the multi-waypoint trajectory. Returns true on success.")
      .def("is_valid", &tg::TrajectoryGenerator::isValid)
      .def("min_time", &tg::TrajectoryGenerator::minTime)
      .def("max_time", &tg::TrajectoryGenerator::maxTime)
      .def("duration", &tg::TrajectoryGenerator::duration)
      .def("evaluate", &tg::TrajectoryGenerator::evaluate, py::arg("t"))
      .def("evaluate_derivative", &tg::TrajectoryGenerator::evaluateDerivative, py::arg("t"),
           py::arg("derivative_order"))
      .def("position", &tg::TrajectoryGenerator::position, py::arg("t"))
      .def("velocity", &tg::TrajectoryGenerator::velocity, py::arg("t"))
      .def("acceleration", &tg::TrajectoryGenerator::acceleration, py::arg("t"))
      .def("jerk", &tg::TrajectoryGenerator::jerk, py::arg("t"))
      .def("max_velocity", &tg::TrajectoryGenerator::maxVelocity)
      .def("max_acceleration", &tg::TrajectoryGenerator::maxAcceleration)
      .def("spline", &tg::TrajectoryGenerator::spline);
}
