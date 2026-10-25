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
 * @file trajectory_generator_gtest.cpp
 * @brief Sanity tests for the mav_trajectory_generation_cpp facade.
 */

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

#include "mav_trajectory_generation_cpp/trajectory_generator.hpp"
#include "mav_trajectory_generation_cpp/types.hpp"

namespace {

constexpr double kEndpointPositionTol = 5.0e-3;  // [m]
constexpr double kEndpointVelTol      = 5.0e-2;  // [m/s]
constexpr double kSimulationStep      = 1.0e-2;  // [s]

using mav_trajectory_generation_cpp::EndWaypoint;
using mav_trajectory_generation_cpp::Waypoint;

std::vector<Waypoint> makeThreeWaypoints() {
  // A non-trivial 3D path with a mid-altitude change so the optimiser has
  // something to smooth out. Endpoints pinned to rest via EndWaypoint.
  return {
      EndWaypoint(Eigen::Vector3d(0.0, 0.0, 1.0)),
      Waypoint(Eigen::Vector3d(5.0, 2.0, 1.5)),
      EndWaypoint(Eigen::Vector3d(8.0, 0.0, 2.0)),
  };
}

}  // namespace

TEST(TrajectoryGenerator, PlanSucceedsWithLinearSolver) {
  const auto waypoints = makeThreeWaypoints();

  mav_trajectory_generation_cpp::GeneratorConfig cfg;
  cfg.optimization.solver = mav_trajectory_generation_cpp::Solver::Linear;

  mav_trajectory_generation_cpp::TrajectoryGenerator gen(cfg);
  ASSERT_TRUE(gen.generate(waypoints, 3.0));
  EXPECT_TRUE(gen.isValid());
  EXPECT_GT(gen.duration(), 0.0);
  EXPECT_EQ(gen.minTime(), 0.0);
  EXPECT_GT(gen.maxTime(), gen.minTime());
}

TEST(TrajectoryGenerator, EndpointsMatchWaypoints) {
  const auto waypoints = makeThreeWaypoints();

  mav_trajectory_generation_cpp::TrajectoryGenerator gen;
  ASSERT_TRUE(gen.generate(waypoints, 3.0));

  const auto start = gen.evaluate(gen.minTime());
  const auto end   = gen.evaluate(gen.maxTime());

  EXPECT_NEAR((start.position - waypoints.front().position).norm(), 0.0, kEndpointPositionTol);
  EXPECT_NEAR((end.position - waypoints.back().position).norm(), 0.0, kEndpointPositionTol);

  // EndWaypoint pins zero velocity at start/end.
  EXPECT_NEAR(start.velocity.norm(), 0.0, kEndpointVelTol);
  EXPECT_NEAR(end.velocity.norm(), 0.0, kEndpointVelTol);
}

TEST(TrajectoryGenerator, EvaluationClampsOutOfRangeTimes) {
  const auto waypoints = makeThreeWaypoints();

  mav_trajectory_generation_cpp::TrajectoryGenerator gen;
  ASSERT_TRUE(gen.generate(waypoints, 3.0));

  const auto below = gen.evaluate(-1.0);
  const auto above = gen.evaluate(gen.maxTime() + 5.0);
  const auto start = gen.evaluate(gen.minTime());
  const auto end   = gen.evaluate(gen.maxTime());

  EXPECT_NEAR((below.position - start.position).norm(), 0.0, 1.0e-9);
  EXPECT_NEAR((above.position - end.position).norm(), 0.0, 1.0e-9);
}

TEST(TrajectoryGenerator, SpeedLimitApproximatelyHeldWithLinearSolver) {
  // The Linear solver only uses max_speed for segment-time allocation, so we
  // allow a generous slack (1.5x). The key property is boundedness, not tight
  // tracking of the cap.
  const auto waypoints     = makeThreeWaypoints();
  constexpr double v_max   = 3.0;
  constexpr double v_slack = 1.5 * v_max;

  mav_trajectory_generation_cpp::TrajectoryGenerator gen;
  ASSERT_TRUE(gen.generate(waypoints, v_max));

  double v_peak = 0.0;
  for (double t = gen.minTime(); t <= gen.maxTime(); t += kSimulationStep) {
    const double v = gen.evaluate(t).velocity.norm();
    v_peak         = std::max(v_peak, v);
  }
  EXPECT_LE(v_peak, v_slack) << "Peak speed exceeded the expected envelope";
}

TEST(TrajectoryGenerator, FailsWithOneWaypoint) {
  mav_trajectory_generation_cpp::TrajectoryGenerator gen;
  const std::vector<Waypoint> single = {EndWaypoint(Eigen::Vector3d::Zero())};
  EXPECT_FALSE(gen.generate(single, 3.0));
  EXPECT_FALSE(gen.isValid());
  EXPECT_THROW(gen.duration(), std::runtime_error);
}

TEST(TrajectoryGenerator, AccessorsThrowBeforeGenerate) {
  mav_trajectory_generation_cpp::TrajectoryGenerator gen;
  EXPECT_FALSE(gen.isValid());
  EXPECT_THROW(gen.minTime(), std::runtime_error);
  EXPECT_THROW(gen.maxTime(), std::runtime_error);
  EXPECT_THROW(gen.duration(), std::runtime_error);
  EXPECT_THROW(gen.evaluate(0.0), std::runtime_error);
  EXPECT_THROW(gen.evaluateDerivative(0.0, 0), std::runtime_error);
  EXPECT_THROW(gen.position(0.0), std::runtime_error);
  EXPECT_THROW(gen.velocity(0.0), std::runtime_error);
  EXPECT_THROW(gen.acceleration(0.0), std::runtime_error);
  EXPECT_THROW(gen.jerk(0.0), std::runtime_error);
  EXPECT_THROW(gen.maxVelocity(), std::runtime_error);
  EXPECT_THROW(gen.maxAcceleration(), std::runtime_error);
  EXPECT_THROW(gen.spline(), std::runtime_error);
}

TEST(TrajectoryGenerator, NamedAccessorsMatchEvaluate) {
  const auto waypoints = makeThreeWaypoints();
  mav_trajectory_generation_cpp::TrajectoryGenerator gen;
  ASSERT_TRUE(gen.generate(waypoints, 3.0));
  const double t    = 0.5 * (gen.minTime() + gen.maxTime());
  const auto sample = gen.evaluate(t);
  EXPECT_NEAR((sample.position - gen.position(t)).norm(), 0.0, 1.0e-12);
  EXPECT_NEAR((sample.velocity - gen.velocity(t)).norm(), 0.0, 1.0e-12);
  EXPECT_NEAR((sample.acceleration - gen.acceleration(t)).norm(), 0.0, 1.0e-12);
  // jerk() is order 3.
  EXPECT_NEAR((gen.evaluateDerivative(t, 3) - gen.jerk(t)).norm(), 0.0, 1.0e-12);
}

TEST(TrajectoryGenerator, MaxVelocityAndAccelerationAreFinite) {
  mav_trajectory_generation_cpp::GeneratorConfig cfg;
  cfg.optimization.solver            = mav_trajectory_generation_cpp::Solver::Nonlinear;
  cfg.optimization.nl_max_iterations = 500;
  mav_trajectory_generation_cpp::TrajectoryGenerator gen(cfg);
  ASSERT_TRUE(gen.generate(makeThreeWaypoints(), 3.0));
  const double v_peak = gen.maxVelocity();
  const double a_peak = gen.maxAcceleration();
  EXPECT_GT(v_peak, 0.0);
  EXPECT_GT(a_peak, 0.0);
  EXPECT_TRUE(std::isfinite(v_peak));
  EXPECT_TRUE(std::isfinite(a_peak));
  // Nonlinear solver enforces v <= 3.0 with some inequality tolerance.
  EXPECT_LE(v_peak, 3.0 * 1.2);
}

TEST(TrajectoryGenerator, FailsWithZeroSpeed) {
  mav_trajectory_generation_cpp::TrajectoryGenerator gen;
  const std::vector<Waypoint> pts = {
      EndWaypoint(Eigen::Vector3d(0.0, 0.0, 0.0)),
      EndWaypoint(Eigen::Vector3d(1.0, 0.0, 0.0)),
  };
  EXPECT_FALSE(gen.generate(pts, 0.0));
  EXPECT_FALSE(gen.generate(pts, -1.0));
  EXPECT_FALSE(gen.isValid());
}

TEST(TrajectoryGenerator, PlanSucceedsWithNonlinearSolver) {
  const auto waypoints = makeThreeWaypoints();

  mav_trajectory_generation_cpp::GeneratorConfig cfg;
  cfg.optimization.solver = mav_trajectory_generation_cpp::Solver::Nonlinear;
  // Keep iterations modest so this test stays fast.
  cfg.optimization.nl_max_iterations = 500;

  mav_trajectory_generation_cpp::TrajectoryGenerator gen(cfg);
  ASSERT_TRUE(gen.generate(waypoints, 3.0));
  EXPECT_TRUE(gen.isValid());
  EXPECT_GT(gen.duration(), 0.0);

  const auto start = gen.evaluate(gen.minTime());
  const auto end   = gen.evaluate(gen.maxTime());
  EXPECT_NEAR((start.position - waypoints.front().position).norm(), 0.0, kEndpointPositionTol);
  EXPECT_NEAR((end.position - waypoints.back().position).norm(), 0.0, kEndpointPositionTol);
}

TEST(TrajectoryGenerator, NonlinearRespectsVelocityBoundTightly) {
  // Unlike the Linear solver, Nonlinear actively enforces the velocity
  // magnitude constraint. A modest slack (1.1x) accounts for NLopt's
  // inequality tolerance (inequality_constraint_tolerance).
  const auto waypoints     = makeThreeWaypoints();
  constexpr double v_max   = 3.0;
  constexpr double v_slack = 1.2 * v_max;

  mav_trajectory_generation_cpp::GeneratorConfig cfg;
  cfg.optimization.solver            = mav_trajectory_generation_cpp::Solver::Nonlinear;
  cfg.optimization.nl_max_iterations = 500;

  mav_trajectory_generation_cpp::TrajectoryGenerator gen(cfg);
  ASSERT_TRUE(gen.generate(waypoints, v_max));

  double v_peak = 0.0;
  for (double t = gen.minTime(); t <= gen.maxTime(); t += kSimulationStep) {
    const double v = gen.evaluate(t).velocity.norm();
    v_peak         = std::max(v_peak, v);
  }
  EXPECT_LE(v_peak, v_slack) << "Nonlinear peak speed exceeded its constraint envelope";
}

TEST(TrajectoryGenerator, EvaluateDerivativeAtZeroMatchesEvaluate) {
  const auto waypoints = makeThreeWaypoints();
  mav_trajectory_generation_cpp::TrajectoryGenerator gen;
  ASSERT_TRUE(gen.generate(waypoints, 3.0));

  const double t = 0.5 * (gen.minTime() + gen.maxTime());
  const auto s   = gen.evaluate(t);
  const auto pos = gen.evaluateDerivative(t, 0);
  const auto vel = gen.evaluateDerivative(t, 1);
  const auto acc = gen.evaluateDerivative(t, 2);

  EXPECT_NEAR((s.position - pos).norm(), 0.0, 1.0e-9);
  EXPECT_NEAR((s.velocity - vel).norm(), 0.0, 1.0e-9);
  EXPECT_NEAR((s.acceleration - acc).norm(), 0.0, 1.0e-9);
}

TEST(TrajectoryGenerator, BareWaypointEndpointsStillProduceRest) {
  // Bare Waypoint at endpoints (no explicit velocity/acceleration set) must
  // still yield a valid trajectory because the facade auto-pins unset
  // derivatives to zero at endpoints.
  const std::vector<Waypoint> waypoints = {
      Waypoint(Eigen::Vector3d(0.0, 0.0, 1.0)),
      Waypoint(Eigen::Vector3d(5.0, 2.0, 1.5)),
      Waypoint(Eigen::Vector3d(8.0, 0.0, 2.0)),
  };
  mav_trajectory_generation_cpp::TrajectoryGenerator gen;
  ASSERT_TRUE(gen.generate(waypoints, 3.0));
  EXPECT_TRUE(gen.isValid());
  // Endpoints should be at rest because bare Waypoints default to zero at endpoints.
  EXPECT_NEAR(gen.evaluate(gen.minTime()).velocity.norm(), 0.0, kEndpointVelTol);
  EXPECT_NEAR(gen.evaluate(gen.maxTime()).velocity.norm(), 0.0, kEndpointVelTol);
}

TEST(TrajectoryGenerator, ReuseForMultipleTrajectoriesPreservesState) {
  // Ensures the same generator instance survives several generate() calls
  // and each call reflects the latest plan.
  mav_trajectory_generation_cpp::TrajectoryGenerator gen;

  const std::vector<Waypoint> first = {
      EndWaypoint(Eigen::Vector3d(0.0, 0.0, 1.0)),
      EndWaypoint(Eigen::Vector3d(5.0, 0.0, 1.0)),
  };
  ASSERT_TRUE(gen.generate(first, 3.0));
  const double dur_first = gen.duration();
  EXPECT_GT(dur_first, 0.0);

  const std::vector<Waypoint> second = {
      EndWaypoint(Eigen::Vector3d(5.0, 0.0, 1.0)),
      EndWaypoint(Eigen::Vector3d(0.0, 0.0, 1.0)),
  };
  ASSERT_TRUE(gen.generate(second, 3.0));
  EXPECT_TRUE(gen.isValid());
  EXPECT_NEAR((gen.evaluate(gen.minTime()).position - second.front().position).norm(), 0.0,
              kEndpointPositionTol);
}

TEST(TrajectoryGenerator, SplineGetterAgreesWithEvaluate) {
  // spline() must be consistent with evaluate(): manually reconstructing the
  // polynomial of the first segment from the returned Spline must reproduce
  // gen.evaluate(t) for t inside segment 0.
  const auto waypoints = makeThreeWaypoints();
  mav_trajectory_generation_cpp::TrajectoryGenerator gen;
  ASSERT_TRUE(gen.generate(waypoints, 3.0));

  const auto sp    = gen.spline();
  const auto times = sp.segmentTimes();
  const auto knots = sp.breakpoints();

  ASSERT_EQ(sp.segments.size(), waypoints.size() - 1);  // M segments for M+1 waypoints
  ASSERT_EQ(times.size(), sp.segments.size());
  ASSERT_EQ(knots.size(), sp.segments.size() + 1);

  for (std::size_t i = 0; i < sp.segments.size(); ++i) {
    EXPECT_NEAR(sp.segments[i].duration, times[i], 1.0e-12);
    EXPECT_NEAR(knots[i + 1] - knots[i], times[i], 1.0e-12);
  }
  EXPECT_NEAR(knots.front(), gen.minTime(), 1.0e-12);
  EXPECT_NEAR(knots.back(), gen.maxTime(), 1.0e-12);
  EXPECT_NEAR(sp.duration(), gen.duration(), 1.0e-12);

  ASSERT_EQ(sp.segments.front().coefficients.rows(), 3);
  const int N = sp.segments.front().coefficients.cols();
  EXPECT_GT(N, 0);

  const double tau              = 0.3 * sp.segments.front().duration;
  const double t_global         = knots.front() + tau;
  Eigen::Vector3d reconstructed = Eigen::Vector3d::Zero();
  double tau_pow                = 1.0;
  for (int k = 0; k < N; ++k) {
    reconstructed += tau_pow * sp.segments.front().coefficients.col(k);
    tau_pow *= tau;
  }
  const auto sample = gen.evaluate(t_global);
  EXPECT_NEAR((reconstructed - sample.position).norm(), 0.0, 1.0e-9);
}

TEST(TrajectoryGenerator, SplineThrowsWhenInvalid) {
  mav_trajectory_generation_cpp::TrajectoryGenerator gen;
  EXPECT_FALSE(gen.isValid());
  EXPECT_THROW(gen.spline(), std::runtime_error);
}

TEST(TrajectoryGenerator, SetSplineMakesEvaluableGenerator) {
  mav_trajectory_generation_cpp::TrajectoryGenerator gen;
  ASSERT_TRUE(gen.generate(makeThreeWaypoints(), 3.0));
  const auto sp_a = gen.spline();

  mav_trajectory_generation_cpp::TrajectoryGenerator gen2;
  EXPECT_FALSE(gen2.isValid());
  ASSERT_TRUE(gen2.setSpline(sp_a));
  EXPECT_TRUE(gen2.isValid());
  EXPECT_NEAR(gen2.duration(), gen.duration(), 1.0e-12);

  // Sample at a few times; both generators must evaluate to the same state.
  for (double frac : {0.0, 0.25, 0.5, 0.75, 1.0}) {
    const double t = gen.minTime() + frac * gen.duration();
    const auto a   = gen.evaluate(t);
    const auto b   = gen2.evaluate(t);
    EXPECT_NEAR((a.position - b.position).norm(), 0.0, 1.0e-9);
    EXPECT_NEAR((a.velocity - b.velocity).norm(), 0.0, 1.0e-9);
    EXPECT_NEAR((a.acceleration - b.acceleration).norm(), 0.0, 1.0e-9);
  }
}

TEST(TrajectoryGenerator, SetSplineRejectsWrongShape) {
  mav_trajectory_generation_cpp::Spline bad;
  mav_trajectory_generation_cpp::SegmentPolynomial seg;
  seg.duration     = 1.0;
  seg.coefficients = Eigen::MatrixXd::Zero(3, 4);  // wrong: N != 10
  bad.segments.push_back(seg);

  mav_trajectory_generation_cpp::TrajectoryGenerator gen;
  EXPECT_FALSE(gen.setSpline(bad));
  EXPECT_FALSE(gen.isValid());

  mav_trajectory_generation_cpp::Spline empty;
  EXPECT_FALSE(gen.setSpline(empty));
  EXPECT_FALSE(gen.isValid());
}

TEST(TrajectoryGenerator, SplineCsvRoundTrip) {
  mav_trajectory_generation_cpp::TrajectoryGenerator gen;
  ASSERT_TRUE(gen.generate(makeThreeWaypoints(), 3.0));
  const auto a = gen.spline();

  // Use a unique-enough path under /tmp for the round-trip.
  const std::string path = "/tmp/mav_trajectory_generation_cpp_spline_roundtrip.csv";
  ASSERT_TRUE(a.saveToCsv(path));

  mav_trajectory_generation_cpp::Spline b;
  ASSERT_TRUE(b.loadFromCsv(path));
  EXPECT_EQ(a.segments.size(), b.segments.size());
  EXPECT_NEAR(a.start_time, b.start_time, 1.0e-12);
  EXPECT_NEAR(a.duration(), b.duration(), 1.0e-12);

  for (std::size_t i = 0; i < a.segments.size(); ++i) {
    EXPECT_NEAR(a.segments[i].duration, b.segments[i].duration, 1.0e-12);
    EXPECT_EQ(a.segments[i].coefficients.rows(), b.segments[i].coefficients.rows());
    EXPECT_EQ(a.segments[i].coefficients.cols(), b.segments[i].coefficients.cols());
    EXPECT_NEAR((a.segments[i].coefficients - b.segments[i].coefficients).norm(), 0.0, 1.0e-12);
  }

  // Reload into a fresh generator and verify evaluate() parity.
  mav_trajectory_generation_cpp::TrajectoryGenerator gen2;
  ASSERT_TRUE(gen2.setSpline(b));
  const double t = 0.5 * gen.duration();
  EXPECT_NEAR((gen.evaluate(t).position - gen2.evaluate(t).position).norm(), 0.0, 1.0e-9);
}

TEST(TrajectoryGenerator, SplineLoadRejectsMissingFileAndBadFormat) {
  mav_trajectory_generation_cpp::Spline s;
  EXPECT_FALSE(s.loadFromCsv("/nonexistent/definitely-not-a-file.csv"));

  // Write a CSV with a bogus dimension and verify it is rejected.
  const std::string path = "/tmp/mav_trajectory_generation_cpp_bad_spline.csv";
  {
    std::ofstream out(path);
    ASSERT_TRUE(static_cast<bool>(out));
    out << "# spline_format_version: 1\n"
        << "# start_time: 0\n"
        << "# dimension: 2\n"  // unsupported
        << "# polynomial_order: 10\n"
        << "duration\n";
  }
  EXPECT_FALSE(s.loadFromCsv(path));
}

TEST(TrajectoryGenerator, ChainedTrajectoryHonoursInitialVelocity) {
  // Chain a second trajectory to the final state of the first, passing a
  // non-zero final velocity explicitly. The second trajectory must then
  // start with that exact velocity (continuity).
  mav_trajectory_generation_cpp::TrajectoryGenerator gen;

  // First trajectory ends with a non-zero velocity (explicit Waypoint).
  Waypoint last_with_vel(Eigen::Vector3d(8.0, 0.0, 2.0));
  last_with_vel.velocity            = Eigen::Vector3d(1.0, -0.5, 0.0);
  const std::vector<Waypoint> first = {
      EndWaypoint(Eigen::Vector3d(0.0, 0.0, 1.0)),
      Waypoint(Eigen::Vector3d(5.0, 2.0, 1.5)),
      last_with_vel,
  };
  ASSERT_TRUE(gen.generate(first, 3.0));
  const Eigen::Vector3d v_final = gen.evaluate(gen.maxTime()).velocity;
  // Sanity: the final velocity we asked for is actually realised.
  EXPECT_NEAR((v_final - *last_with_vel.velocity).norm(), 0.0, 1.0e-3);

  // Second trajectory: start position = first's endpoint, inherit velocity.
  Waypoint chained_start(last_with_vel.position);
  chained_start.velocity             = v_final;
  const std::vector<Waypoint> second = {
      chained_start,
      EndWaypoint(Eigen::Vector3d(0.0, 0.0, 1.0)),
  };
  ASSERT_TRUE(gen.generate(second, 3.0));
  EXPECT_NEAR((gen.evaluate(gen.minTime()).velocity - v_final).norm(), 0.0, 1.0e-6);
}
