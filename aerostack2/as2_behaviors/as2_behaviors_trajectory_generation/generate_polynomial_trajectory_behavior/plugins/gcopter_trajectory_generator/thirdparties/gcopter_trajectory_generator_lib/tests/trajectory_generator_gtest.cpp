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
 * @file trajectory_generator_gtest.cpp
 * @brief End-to-end tests for gcopter_lib.
 *
 * Validates the unified TrajectoryGenerator API: generate(waypoints, max_speed),
 * scalar accessors, evaluate(t), spline() snapshot and CSV round-trip.
 */

#include <gtest/gtest.h>

#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "gcopter_lib/trajectory_generator.hpp"
#include "gcopter_lib/trajectory_logger.hpp"
#include "gcopter_lib/types.hpp"

namespace {

constexpr double kPositionTolerance = 1.0e-3;  // [m]
constexpr double kBoundaryVelTol    = 1.0e-2;  // [m/s]
constexpr double kSimulationStep    = 1.0e-2;  // [s]
constexpr double kMaxVelTolerance   = 0.05;    // 5% slack on v_max
constexpr double kReferenceMaxSpeed = 4.0;     // [m/s]

gcopter_lib::DroneParameters makeReferenceDrone() {
  // Values taken from GCOPTER/gcopter/config/global_planning.yaml.
  return gcopter_lib::DroneParameters{
      /*mass*/ 0.61,
      /*gravity*/ 9.8,
      /*horizontal_drag*/ 0.70,
      /*vertical_drag*/ 0.80,
      /*parasitic_drag*/ 0.01,
      /*speed_smooth_factor*/ 1.0e-4,
  };
}

gcopter_lib::DroneLimits makeReferenceLimits() {
  return gcopter_lib::DroneLimits{
      /*max_velocity*/ kReferenceMaxSpeed,
      /*max_body_rate*/ 2.1,
      /*max_tilt_angle*/ 1.05,
      /*min_thrust*/ 2.0,
      /*max_thrust*/ 12.0,
  };
}

gcopter_lib::GeneratorConfig makeReferenceGeneratorConfig() {
  gcopter_lib::GeneratorConfig cfg;
  cfg.params       = makeReferenceDrone();
  cfg.limits       = makeReferenceLimits();
  cfg.optimization = [] {
    gcopter_lib::OptimizationConfig opt;
    opt.time_weight         = 20.0;
    opt.position_weight     = 1.0e4;
    opt.velocity_weight     = 1.0e4;
    opt.body_rate_weight    = 1.0e4;
    opt.tilt_weight         = 1.0e4;
    opt.thrust_weight       = 1.0e5;
    opt.smoothing_eps       = 1.0e-2;
    opt.integral_resolution = 16;
    opt.rel_cost_tol        = 1.0e-5;
    opt.corridor_margin     = 2.0;
    return opt;
  }();
  return cfg;
}

std::vector<gcopter_lib::Waypoint> makeFiveWaypoints() {
  // A loose rectangular climb: 5 points forming a staircase-like pattern.
  // Large inter-point spacing (>3 m) so that an AABB corridor with a 2 m
  // margin leaves the solver ample room to converge.
  const std::vector<Eigen::Vector3d> positions = {
      {0.0, 0.0, 1.0}, {5.0, 0.0, 1.5}, {5.0, 5.0, 2.0}, {0.0, 5.0, 2.5}, {0.0, 0.0, 3.0},
  };
  std::vector<gcopter_lib::Waypoint> waypoints;
  waypoints.reserve(positions.size());
  for (const Eigen::Vector3d& p : positions) {
    waypoints.push_back(gcopter_lib::Waypoint{p});
  }
  return waypoints;
}

std::string logFilePath() {
  const char* env = std::getenv("GCOPTER_LIB_TEST_LOG");
  if (env != nullptr && env[0] != '\0') {
    return std::string(env);
  }
  namespace fs = std::filesystem;
  return (fs::temp_directory_path() / "gcopter_lib_trajectory.csv").string();
}

}  // namespace

TEST(TrajectoryGenerator, AccessorsThrowBeforeGenerate) {
  gcopter_lib::TrajectoryGenerator gen(makeReferenceGeneratorConfig());
  EXPECT_FALSE(gen.isValid());
  EXPECT_THROW(gen.duration(), std::runtime_error);
  EXPECT_THROW(gen.minTime(), std::runtime_error);
  EXPECT_THROW(gen.maxTime(), std::runtime_error);
  EXPECT_THROW(gen.position(0.0), std::runtime_error);
  EXPECT_THROW(gen.evaluate(0.0), std::runtime_error);
  EXPECT_THROW(gen.evaluateDerivative(0.0, 0), std::runtime_error);
  EXPECT_THROW(gen.maxVelocity(), std::runtime_error);
  EXPECT_THROW(gen.maxAcceleration(), std::runtime_error);
  EXPECT_THROW(gen.spline(), std::runtime_error);
}

TEST(TrajectoryGenerator, GenerateFailsWithSingleWaypoint) {
  gcopter_lib::TrajectoryGenerator gen(makeReferenceGeneratorConfig());
  std::vector<gcopter_lib::Waypoint> single = {gcopter_lib::Waypoint{Eigen::Vector3d::Zero()}};
  EXPECT_FALSE(gen.generate(single, kReferenceMaxSpeed));
  EXPECT_FALSE(gen.isValid());
}

TEST(TrajectoryGenerator, GenerateFailsWithZeroOrNegativeMaxSpeed) {
  gcopter_lib::TrajectoryGenerator gen(makeReferenceGeneratorConfig());
  const auto waypoints = makeFiveWaypoints();
  EXPECT_FALSE(gen.generate(waypoints, 0.0));
  EXPECT_FALSE(gen.generate(waypoints, -1.0));
  EXPECT_FALSE(gen.isValid());
}

TEST(TrajectoryGenerator, FiveWaypointsEndToEnd) {
  const auto waypoints = makeFiveWaypoints();
  gcopter_lib::TrajectoryGenerator gen(makeReferenceGeneratorConfig());

  ASSERT_TRUE(gen.generate(waypoints, kReferenceMaxSpeed))
      << "GCOPTER optimiser failed to converge.";
  ASSERT_TRUE(gen.isValid());

  const double T = gen.duration();
  ASSERT_GT(T, 0.0);
  ASSERT_LT(T, 60.0) << "Duration suspiciously large (" << T << " s)";

  EXPECT_DOUBLE_EQ(gen.minTime(), 0.0);
  EXPECT_DOUBLE_EQ(gen.maxTime(), T);

  // Boundary conditions.
  EXPECT_LE((gen.position(0.0) - waypoints.front().position).norm(), kPositionTolerance);
  EXPECT_LE((gen.position(T) - waypoints.back().position).norm(), kPositionTolerance);
  EXPECT_LE(gen.velocity(0.0).norm(), kBoundaryVelTol);
  EXPECT_LE(gen.velocity(T).norm(), kBoundaryVelTol);

  // Magnitude limits (allow small smoothing slack).
  EXPECT_LE(gen.maxVelocity(), kReferenceMaxSpeed * (1.0 + kMaxVelTolerance));
  EXPECT_TRUE(std::isfinite(gen.maxAcceleration()));

  // Sample and log.
  const std::string csv_path = logFilePath();
  gcopter_lib::TrajectoryLogger logger(csv_path);

  std::size_t n_samples = 0;
  double last_t         = 0.0;
  for (double t = 0.0; t <= T; t += kSimulationStep) {
    const double tc = std::min(t, T);
    logger.save(tc, gen.position(tc), gen.velocity(tc), gen.acceleration(tc));
    ++n_samples;
    last_t = tc;
  }
  if (last_t < T) {
    logger.save(T, gen.position(T), gen.velocity(T), gen.acceleration(T));
    ++n_samples;
  }
  logger.close();

  const std::size_t expected_min_samples = static_cast<std::size_t>(T / kSimulationStep);
  EXPECT_GE(n_samples, expected_min_samples);

  std::ifstream f(csv_path);
  ASSERT_TRUE(f.is_open()) << "Cannot reopen CSV at " << csv_path;
  std::string header;
  std::getline(f, header);
  EXPECT_EQ(header, "t,px,py,pz,vx,vy,vz,ax,ay,az");
  std::string first_row;
  std::getline(f, first_row);
  EXPECT_FALSE(first_row.empty());

  std::cout << "[gcopter_lib] trajectory duration = " << T << " s, " << n_samples
            << " samples written to " << csv_path << std::endl;
}

TEST(TrajectoryGenerator, EvaluationClampsOutOfRangeTimes) {
  const auto waypoints = makeFiveWaypoints();
  gcopter_lib::TrajectoryGenerator gen(makeReferenceGeneratorConfig());
  ASSERT_TRUE(gen.generate(waypoints, kReferenceMaxSpeed));

  const auto below = gen.evaluate(-1.0);
  const auto above = gen.evaluate(gen.maxTime() + 5.0);
  const auto start = gen.evaluate(gen.minTime());
  const auto end   = gen.evaluate(gen.maxTime());

  EXPECT_NEAR((below.position - start.position).norm(), 0.0, 1.0e-9);
  EXPECT_NEAR((above.position - end.position).norm(), 0.0, 1.0e-9);
}

TEST(TrajectoryGenerator, EvaluateMatchesNamedAccessors) {
  const auto waypoints = makeFiveWaypoints();
  gcopter_lib::TrajectoryGenerator gen(makeReferenceGeneratorConfig());
  ASSERT_TRUE(gen.generate(waypoints, kReferenceMaxSpeed));

  const double t    = 0.5 * gen.duration();
  const auto sample = gen.evaluate(t);
  EXPECT_NEAR((sample.position - gen.position(t)).norm(), 0.0, 1.0e-12);
  EXPECT_NEAR((sample.velocity - gen.velocity(t)).norm(), 0.0, 1.0e-12);
  EXPECT_NEAR((sample.acceleration - gen.acceleration(t)).norm(), 0.0, 1.0e-12);

  EXPECT_NEAR((gen.evaluateDerivative(t, 0) - gen.position(t)).norm(), 0.0, 1.0e-12);
  EXPECT_NEAR((gen.evaluateDerivative(t, 1) - gen.velocity(t)).norm(), 0.0, 1.0e-12);
  EXPECT_NEAR((gen.evaluateDerivative(t, 2) - gen.acceleration(t)).norm(), 0.0, 1.0e-12);
  EXPECT_NEAR((gen.evaluateDerivative(t, 3) - gen.jerk(t)).norm(), 0.0, 1.0e-12);
  EXPECT_NEAR(gen.evaluateDerivative(t, 4).norm(), 0.0, 1.0e-12);
}

TEST(TrajectoryGenerator, SplineGetterAgreesWithEvaluate) {
  // spline() must be consistent with evaluate(): manually reconstructing the
  // polynomial of the first segment from the returned Spline must reproduce
  // gen.evaluate(t) for t inside segment 0.
  const auto waypoints = makeFiveWaypoints();
  gcopter_lib::TrajectoryGenerator gen(makeReferenceGeneratorConfig());
  ASSERT_TRUE(gen.generate(waypoints, kReferenceMaxSpeed));

  const auto sp    = gen.spline();
  const auto times = sp.segmentTimes();
  const auto knots = sp.breakpoints();

  ASSERT_GE(sp.segments.size(), 1u);
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
  EXPECT_EQ(N, 6);  // GCOPTER quintic pieces

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

TEST(TrajectoryGenerator, SplineCsvRoundTrip) {
  const auto waypoints = makeFiveWaypoints();
  gcopter_lib::TrajectoryGenerator gen(makeReferenceGeneratorConfig());
  ASSERT_TRUE(gen.generate(waypoints, kReferenceMaxSpeed));
  const auto a = gen.spline();

  const std::string path = "/tmp/gcopter_lib_spline_roundtrip.csv";
  ASSERT_TRUE(a.saveToCsv(path));

  gcopter_lib::Spline b;
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
}

TEST(TrajectoryGenerator, EndWaypointMatchesDefaultedWaypoint) {
  // EndWaypoint should be functionally identical to a bare Waypoint at endpoints
  // (since unset endpoint vel/acc default to zero anyway).
  const std::vector<gcopter_lib::Waypoint> with_endwp = {
      gcopter_lib::EndWaypoint(Eigen::Vector3d(0.0, 0.0, 1.0)),
      gcopter_lib::Waypoint(Eigen::Vector3d(2.0, 0.0, 1.0)),
      gcopter_lib::EndWaypoint(Eigen::Vector3d(2.0, 2.0, 1.0)),
  };
  const std::vector<gcopter_lib::Waypoint> bare = {
      gcopter_lib::Waypoint(Eigen::Vector3d(0.0, 0.0, 1.0)),
      gcopter_lib::Waypoint(Eigen::Vector3d(2.0, 0.0, 1.0)),
      gcopter_lib::Waypoint(Eigen::Vector3d(2.0, 2.0, 1.0)),
  };

  gcopter_lib::TrajectoryGenerator g1(makeReferenceGeneratorConfig());
  gcopter_lib::TrajectoryGenerator g2(makeReferenceGeneratorConfig());
  ASSERT_TRUE(g1.generate(with_endwp, kReferenceMaxSpeed));
  ASSERT_TRUE(g2.generate(bare, kReferenceMaxSpeed));

  // Same boundary conditions ⇒ essentially the same trajectory; allow a small
  // slack because L-BFGS is deterministic but its rel_cost_tol gives it room
  // to land on numerically-close (not bit-identical) optima.
  EXPECT_NEAR(g1.duration(), g2.duration(), 1.0e-3);
}

TEST(TrajectoryGenerator, MaxSpeedOverridesDroneLimit) {
  // Lower max_speed on a generate() call should cap the realised peak speed
  // below the (much larger) DroneLimits.max_velocity baked into construction.
  const auto waypoints = makeFiveWaypoints();
  gcopter_lib::TrajectoryGenerator gen(makeReferenceGeneratorConfig());
  constexpr double v_call = 2.0;  // half of DroneLimits.max_velocity = 4.0
  ASSERT_TRUE(gen.generate(waypoints, v_call));
  EXPECT_LE(gen.maxVelocity(), v_call * (1.0 + kMaxVelTolerance));
}

TEST(TrajectoryGenerator, GenerateWithPerSegmentMargins) {
  const auto waypoints = makeFiveWaypoints();
  gcopter_lib::TrajectoryGenerator gen(makeReferenceGeneratorConfig());
  std::vector<double> margins(waypoints.size() - 1, 1.5);
  ASSERT_TRUE(gen.generate(waypoints, kReferenceMaxSpeed, margins));
  EXPECT_TRUE(gen.isValid());
}

TEST(TrajectoryGenerator, GenerateWithMismatchedMarginsFails) {
  const auto waypoints = makeFiveWaypoints();
  gcopter_lib::TrajectoryGenerator gen(makeReferenceGeneratorConfig());
  std::vector<double> wrong_size(waypoints.size(), 1.0);  // off by one
  EXPECT_FALSE(gen.generate(waypoints, kReferenceMaxSpeed, wrong_size));
  EXPECT_FALSE(gen.isValid());
}

// Pure-vertical 2-waypoint hop: GCOPTER's L-BFGS aborts with
// LBFGSERR_INVALIDPARAMETERS on a Z-aligned segment unless the optimisation
// is seeded with a slightly off-axis initial waypoint. The
// `vertical_perturbation` knob in OptimizationConfig drives that workaround;
// when enabled (its default), the trajectory must converge and land *exactly*
// at the user-requested terminal position.
//
// Uses a higher `max_thrust` than the multi-waypoint fixture so the 10 m
// vertical hop is dynamically feasible (default fixture leaves only 0.82 N
// of thrust margin over weight, which is not enough to climb 10 m at
// kReferenceMaxSpeed = 4 m/s).
namespace {
gcopter_lib::GeneratorConfig makeVerticalHopConfig() {
  gcopter_lib::GeneratorConfig cfg = makeReferenceGeneratorConfig();
  cfg.limits.max_thrust            = 25.0;  // ample margin over m·g = 11.2 N
  return cfg;
}
constexpr double kVerticalHopMaxSpeed = 3.0;  // [m/s]
}  // namespace

TEST(TrajectoryGenerator, PureVerticalHopConvergesWithPerturbation) {
  const std::vector<gcopter_lib::Waypoint> waypoints = {
      gcopter_lib::EndWaypoint(Eigen::Vector3d(0.0, 0.0, 0.0)),
      gcopter_lib::EndWaypoint(Eigen::Vector3d(0.0, 0.0, 10.0)),
  };
  gcopter_lib::TrajectoryGenerator gen(makeVerticalHopConfig());
  ASSERT_TRUE(gen.generate(waypoints, kVerticalHopMaxSpeed))
      << "Pure-vertical hop must converge with the default vertical perturbation.";
  ASSERT_TRUE(gen.isValid());

  const double T = gen.duration();
  ASSERT_GT(T, 0.0);
  // Tail is preserved exactly: the perturbation only shifts the *initial* PVA.
  EXPECT_LE((gen.position(T) - waypoints.back().position).norm(), kPositionTolerance);
  EXPECT_LE(gen.velocity(T).norm(), kBoundaryVelTol);
  // The trajectory enters at head + (perturbation, 0, 0); confirm the offset
  // matches the configured value (within a few mm of solver slack).
  const double perturbation   = makeVerticalHopConfig().optimization.vertical_perturbation;
  const Eigen::Vector3d entry = gen.position(0.0) - waypoints.front().position;
  EXPECT_NEAR(entry.x(), perturbation, 5.0e-3);
  EXPECT_NEAR(entry.y(), 0.0, 5.0e-3);
  EXPECT_NEAR(entry.z(), 0.0, 5.0e-3);
  // Magnitude bound is still respected.
  EXPECT_LE(gen.maxVelocity(), kVerticalHopMaxSpeed * (1.0 + kMaxVelTolerance));
}

TEST(TrajectoryGenerator, PureVerticalHopFailsWhenPerturbationDisabled) {
  // Documents the upstream failure mode the workaround compensates for.
  // Disabling the perturbation must let the solver fail (otherwise the test
  // is silently masking a regression).
  const std::vector<gcopter_lib::Waypoint> waypoints = {
      gcopter_lib::EndWaypoint(Eigen::Vector3d(0.0, 0.0, 0.0)),
      gcopter_lib::EndWaypoint(Eigen::Vector3d(0.0, 0.0, 10.0)),
  };
  gcopter_lib::GeneratorConfig cfg       = makeVerticalHopConfig();
  cfg.optimization.vertical_perturbation = 0.0;
  gcopter_lib::TrajectoryGenerator gen(cfg);
  EXPECT_FALSE(gen.generate(waypoints, kVerticalHopMaxSpeed));
  EXPECT_FALSE(gen.isValid());
}
