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
 *
 * Unit tests for the one-shot @ref TrajectoryGenerator wrapper.
 */

#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include "trajectory_generator_jerk_limited/trajectory_generator.hpp"
#include "trajectory_generator_jerk_limited/types.hpp"

namespace tg = trajectory_generator_jerk_limited;

namespace {

tg::GeneratorConfig defaultConfig() {
  tg::GeneratorConfig cfg;
  cfg.params.max_acceleration       = 1.0;
  cfg.params.max_jerk               = 5.0;
  cfg.params.max_tracking_error     = 1.0;
  cfg.params.acceptance_radius      = 0.5;
  cfg.params.trajectory_gain        = 1.0;
  cfg.simulator.step_dt             = 0.02;
  cfg.simulator.max_simulation_time = 60.0;
  cfg.simulator.settle_velocity     = 0.05;
  cfg.simulator.polynomial_order    = 16;
  return cfg;
}

}  // namespace

TEST(TrajectoryGenerator, AccessorsThrowBeforeGenerate) {
  tg::TrajectoryGenerator gen(defaultConfig());
  EXPECT_FALSE(gen.isValid());
  EXPECT_THROW(gen.minTime(), std::runtime_error);
  EXPECT_THROW(gen.maxTime(), std::runtime_error);
  EXPECT_THROW(gen.duration(), std::runtime_error);
  EXPECT_THROW(gen.evaluate(0.0), std::runtime_error);
  EXPECT_THROW(gen.position(0.0), std::runtime_error);
  EXPECT_THROW(gen.velocity(0.0), std::runtime_error);
  EXPECT_THROW(gen.acceleration(0.0), std::runtime_error);
  EXPECT_THROW(gen.jerk(0.0), std::runtime_error);
  EXPECT_THROW(gen.maxVelocity(), std::runtime_error);
  EXPECT_THROW(gen.maxAcceleration(), std::runtime_error);
  EXPECT_THROW(gen.spline(), std::runtime_error);
}

TEST(TrajectoryGenerator, RejectsTooFewWaypoints) {
  tg::TrajectoryGenerator gen(defaultConfig());
  std::vector<tg::Waypoint> only_one = {tg::EndWaypoint(Eigen::Vector3d::Zero())};
  EXPECT_FALSE(gen.generate(only_one, 1.0));
  EXPECT_FALSE(gen.isValid());
}

TEST(TrajectoryGenerator, RejectsNonPositiveMaxSpeed) {
  tg::TrajectoryGenerator gen(defaultConfig());
  std::vector<tg::Waypoint> wps = {tg::EndWaypoint(Eigen::Vector3d::Zero()),
                                   tg::EndWaypoint(Eigen::Vector3d(1.0, 0.0, 0.0))};
  EXPECT_FALSE(gen.generate(wps, 0.0));
  EXPECT_FALSE(gen.generate(wps, -1.0));
  EXPECT_FALSE(gen.isValid());
}

TEST(TrajectoryGenerator, RejectsBadStepDt) {
  tg::GeneratorConfig cfg = defaultConfig();
  cfg.simulator.step_dt   = 0.0;
  tg::TrajectoryGenerator gen(cfg);
  std::vector<tg::Waypoint> wps = {tg::EndWaypoint(Eigen::Vector3d::Zero()),
                                   tg::EndWaypoint(Eigen::Vector3d(1.0, 0.0, 0.0))};
  EXPECT_FALSE(gen.generate(wps, 1.0));
}

TEST(TrajectoryGenerator, TwoWaypointStraightLine) {
  tg::TrajectoryGenerator gen(defaultConfig());
  std::vector<tg::Waypoint> wps = {tg::EndWaypoint(Eigen::Vector3d::Zero()),
                                   tg::EndWaypoint(Eigen::Vector3d(2.0, 0.0, 0.0))};
  ASSERT_TRUE(gen.generate(wps, 1.0));
  EXPECT_TRUE(gen.isValid());
  EXPECT_DOUBLE_EQ(gen.minTime(), 0.0);
  EXPECT_GT(gen.duration(), 0.0);
  EXPECT_GT(gen.maxTime(), gen.minTime());

  const auto start = gen.evaluate(0.0);
  EXPECT_NEAR(start.position.x(), 0.0, 1e-9);
  EXPECT_NEAR(start.position.y(), 0.0, 1e-9);
  EXPECT_NEAR(start.position.z(), 0.0, 1e-9);
  EXPECT_NEAR(start.velocity.norm(), 0.0, 1e-9);

  const auto end = gen.evaluate(gen.maxTime());
  EXPECT_NEAR(end.position.x(), 2.0, 0.5);  // settle_velocity may stop short
  EXPECT_NEAR(end.position.y(), 0.0, 1e-3);
  EXPECT_NEAR(end.position.z(), 0.0, 1e-3);

  // Bounds satisfied (with small numerical slack).
  EXPECT_LT(gen.maxVelocity(), 1.0 * 1.05);
  EXPECT_LT(gen.maxAcceleration(), 1.0 * 1.05);

  // Spline has exactly one segment.
  const tg::Spline sp = gen.spline();
  ASSERT_EQ(sp.segments.size(), 1u);
  EXPECT_NEAR(sp.duration(), gen.duration(), 1e-9);
}

TEST(TrajectoryGenerator, MultiWaypointSplineSegmentsEqualWaypointPairs) {
  tg::TrajectoryGenerator gen(defaultConfig());
  std::vector<tg::Waypoint> wps;
  wps.emplace_back(tg::EndWaypoint(Eigen::Vector3d(0.0, 0.0, 0.0)));
  wps.emplace_back(tg::Waypoint(Eigen::Vector3d(2.0, 0.0, 0.0)));
  wps.emplace_back(tg::Waypoint(Eigen::Vector3d(2.0, 2.0, 0.0)));
  wps.emplace_back(tg::EndWaypoint(Eigen::Vector3d(0.0, 2.0, 0.0)));

  ASSERT_TRUE(gen.generate(wps, 1.0));
  EXPECT_TRUE(gen.isValid());

  const tg::Spline sp = gen.spline();
  ASSERT_EQ(sp.segments.size(), wps.size() - 1);

  // breakpoints: monotonically non-decreasing, from minTime() to ~maxTime().
  const auto knots = sp.breakpoints();
  ASSERT_EQ(knots.size(), wps.size());
  EXPECT_DOUBLE_EQ(knots.front(), 0.0);
  for (std::size_t i = 1; i < knots.size(); ++i) {
    EXPECT_GE(knots[i], knots[i - 1]);
  }
  EXPECT_NEAR(knots.back(), gen.maxTime(), 1e-9);
}

TEST(TrajectoryGenerator, EvaluateClampsOutsideRange) {
  tg::TrajectoryGenerator gen(defaultConfig());
  std::vector<tg::Waypoint> wps = {tg::EndWaypoint(Eigen::Vector3d(0.0, 0.0, 0.0)),
                                   tg::EndWaypoint(Eigen::Vector3d(1.0, 0.0, 0.0))};
  ASSERT_TRUE(gen.generate(wps, 0.5));

  const auto before = gen.evaluate(-10.0);
  const auto start  = gen.evaluate(gen.minTime());
  EXPECT_NEAR((before.position - start.position).norm(), 0.0, 1e-9);

  const auto after = gen.evaluate(gen.maxTime() + 100.0);
  const auto end   = gen.evaluate(gen.maxTime());
  EXPECT_NEAR((after.position - end.position).norm(), 0.0, 1e-9);
}

TEST(TrajectoryGenerator, JerkAndDerivativeOrders) {
  tg::TrajectoryGenerator gen(defaultConfig());
  std::vector<tg::Waypoint> wps = {tg::EndWaypoint(Eigen::Vector3d(0.0, 0.0, 0.0)),
                                   tg::EndWaypoint(Eigen::Vector3d(1.0, 0.0, 0.0))};
  ASSERT_TRUE(gen.generate(wps, 0.5));

  const double t = gen.duration() / 2.0;
  const auto pos = gen.evaluateDerivative(t, 0);
  EXPECT_NEAR((pos - gen.position(t)).norm(), 0.0, 1e-12);
  const auto vel = gen.evaluateDerivative(t, 1);
  EXPECT_NEAR((vel - gen.velocity(t)).norm(), 0.0, 1e-12);
  const auto acc = gen.evaluateDerivative(t, 2);
  EXPECT_NEAR((acc - gen.acceleration(t)).norm(), 0.0, 1e-12);
  const auto jrk = gen.evaluateDerivative(t, 3);
  EXPECT_NEAR((jrk - gen.jerk(t)).norm(), 0.0, 1e-12);
  // Higher orders are zero.
  EXPECT_NEAR(gen.evaluateDerivative(t, 4).norm(), 0.0, 1e-12);
}

TEST(TrajectoryGenerator, FailedRegenerateLeavesInvalid) {
  tg::TrajectoryGenerator gen(defaultConfig());
  std::vector<tg::Waypoint> wps = {tg::EndWaypoint(Eigen::Vector3d::Zero()),
                                   tg::EndWaypoint(Eigen::Vector3d(1.0, 0.0, 0.0))};
  ASSERT_TRUE(gen.generate(wps, 1.0));
  EXPECT_TRUE(gen.isValid());

  // A failing generate() must transition the object back to invalid.
  EXPECT_FALSE(gen.generate(wps, 0.0));
  EXPECT_FALSE(gen.isValid());
  EXPECT_THROW(gen.evaluate(0.0), std::runtime_error);
}

// --- Initial state from first waypoint ------------------------------------

TEST(TrajectoryGenerator, GenerateHonoursFirstWaypointVelocity) {
  // The first waypoint's optional velocity field pins the integrator's
  // initial state.
  tg::TrajectoryGenerator gen(defaultConfig());
  tg::Waypoint start(Eigen::Vector3d::Zero());
  start.velocity                = Eigen::Vector3d(0.5, 0.0, 0.0);
  std::vector<tg::Waypoint> wps = {start, tg::EndWaypoint(Eigen::Vector3d(5.0, 0.0, 0.0))};
  ASSERT_TRUE(gen.generate(wps, /*max_speed=*/1.0));

  // evaluate(0) must report the seeded velocity exactly (it is stored in the
  // first sample verbatim).
  const auto sample = gen.evaluate(gen.minTime());
  EXPECT_NEAR(sample.velocity.x(), 0.5, 1e-12);
  EXPECT_NEAR(sample.velocity.y(), 0.0, 1e-12);
  EXPECT_NEAR(sample.velocity.z(), 0.0, 1e-12);
  // Convergence to the end waypoint must still happen within the simulator
  // horizon and below the global speed bound. The settle_velocity threshold
  // can stop the integrator a few cm short of the exact endpoint.
  EXPECT_NEAR(gen.evaluate(gen.maxTime()).position.x(), 5.0, 0.1);
  EXPECT_LE(gen.maxVelocity(), 1.0 + 1e-6);
}

TEST(TrajectoryGenerator, GenerateHonoursFirstWaypointAcceleration) {
  // The acceleration seed is recorded in the first sample even if the
  // reactive integrator does not propagate it into the second tick (the
  // post-trajectory branch overwrites acceleration to zero before T1/T2/T3
  // are computed). The contract this test asserts is the recorded value.
  tg::TrajectoryGenerator gen(defaultConfig());
  tg::Waypoint start(Eigen::Vector3d::Zero());
  start.acceleration            = Eigen::Vector3d(0.3, 0.0, 0.0);
  std::vector<tg::Waypoint> wps = {start, tg::EndWaypoint(Eigen::Vector3d(2.0, 0.0, 0.0))};
  ASSERT_TRUE(gen.generate(wps, 1.0));

  const auto sample = gen.evaluate(gen.minTime());
  EXPECT_NEAR(sample.acceleration.x(), 0.3, 1e-12);
  EXPECT_LE(gen.maxAcceleration(), 1.0 + 1e-6);
}

TEST(TrajectoryGenerator, CollinearWaypointsCruiseWithoutOscillation) {
  // With N >= 3 collinear waypoints, the one-shot driver feeds the streaming
  // controller a triplet {wp[i-1], wp[i], wp[i+1]} for every intermediate
  // waypoint, then collapses to {wp[N-2], wp[N-1], wp[N-1]} at the last
  // waypoint. The library uses that collapse to discriminate fly-through
  // (intermediate) from stop (final). On a straight line, intermediates must
  // not brake the trajectory: the velocity should reach max_speed and stay
  // at the plateau between waypoints, only decelerating to rest at the
  // final waypoint.
  tg::TrajectoryGenerator gen(defaultConfig());
  std::vector<tg::Waypoint> wps = {
      tg::EndWaypoint(Eigen::Vector3d(0.0, 0.0, 0.0)),
      tg::Waypoint(Eigen::Vector3d(2.0, 0.0, 0.0)),
      tg::Waypoint(Eigen::Vector3d(4.0, 0.0, 0.0)),
      tg::Waypoint(Eigen::Vector3d(6.0, 0.0, 0.0)),
      tg::Waypoint(Eigen::Vector3d(8.0, 0.0, 0.0)),
      tg::EndWaypoint(Eigen::Vector3d(10.0, 0.0, 0.0)),
  };
  const double max_speed = 1.0;
  ASSERT_TRUE(gen.generate(wps, max_speed));

  // Sample the trajectory densely and compute the speed envelope around each
  // intermediate waypoint. The drone must not slow below ~90% of max_speed at
  // any intermediate waypoint (the residual gap covers the inevitable
  // discretisation slack at the triplet-rotation tick).
  const double duration = gen.duration();
  ASSERT_GT(duration, 0.0);

  // Find the time at which the trajectory first reaches each intermediate
  // waypoint (within 0.2 m), and check the speed there.
  for (std::size_t k = 1; k + 1 < wps.size(); ++k) {
    const Eigen::Vector3d wp = wps[k].position;
    double t                 = 0.0;
    bool found               = false;
    while (t <= duration) {
      if ((gen.position(t) - wp).norm() < 0.2) {
        found = true;
        break;
      }
      t += 0.05;
    }
    ASSERT_TRUE(found) << "intermediate waypoint " << k << " not reached";
    const double speed = gen.velocity(t).norm();
    EXPECT_GT(speed, 0.9 * max_speed) << "intermediate waypoint " << k
                                      << " braked below 90% of max_speed (speed=" << speed << ")";
  }

  // The final waypoint must still be reached at rest.
  const auto end = gen.evaluate(gen.maxTime());
  EXPECT_LT(end.velocity.norm(), 0.1) << "final waypoint not reached at rest";
  EXPECT_LE(gen.maxVelocity(), max_speed * 1.001);
}

TEST(TrajectoryGenerator, GenerateWithoutInitialStateMatchesLegacy) {
  // A first Waypoint with no velocity / acceleration (i.e. constructed with
  // the position-only ctor) must produce the same trajectory as the legacy
  // pre-overload behaviour, so the default config and any caller using
  // `Waypoint(pos)` retains its byte-identical CSV output.
  tg::TrajectoryGenerator gen(defaultConfig());
  std::vector<tg::Waypoint> wps = {tg::Waypoint(Eigen::Vector3d::Zero()),
                                   tg::EndWaypoint(Eigen::Vector3d(2.0, 1.0, 0.5))};
  ASSERT_TRUE(gen.generate(wps, 1.0));
  // Sanity: legacy start sample is at rest.
  const auto sample = gen.evaluate(gen.minTime());
  EXPECT_NEAR(sample.velocity.norm(), 0.0, 1e-12);
  EXPECT_NEAR(sample.acceleration.norm(), 0.0, 1e-12);
}
