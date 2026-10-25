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
 * @file waypoint_controller_gtest.cpp
 *
 * GoogleTest unit tests for WaypointTrajectoryController.
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

#include <gtest/gtest.h>
#include <Eigen/Core>

#include "trajectory_generator_jerk_limited/parameters.hpp"
#include "trajectory_generator_jerk_limited/waypoint_controller.hpp"

namespace trajectory_generator_jerk_limited {
namespace {

struct MotionStats {
  double max_speed        = 0.0;
  double max_acceleration = 0.0;
};

MotionStats simulate(const TrajectoryParameters& parameters,
                     const Eigen::Vector3d& initial_position,
                     const Eigen::Vector3d& waypoint,
                     const double dt,
                     const std::size_t num_iterations) {
  WaypointTrajectoryController controller(parameters);
  controller.reset(initial_position);

  Eigen::Vector3d vehicle_position = initial_position;
  MotionStats stats;
  for (std::size_t i = 0; i < num_iterations; ++i) {
    const TrajectorySetpoints setpoints = controller.update(dt, vehicle_position, waypoint);
    vehicle_position                    = setpoints.position;

    stats.max_speed        = std::max(stats.max_speed, setpoints.velocity.norm());
    stats.max_acceleration = std::max(stats.max_acceleration, setpoints.acceleration.norm());
  }
  return stats;
}

TEST(WaypointController, DefaultParametersDisableAllLimits) {
  const TrajectoryParameters parameters;
  EXPECT_DOUBLE_EQ(parameters.max_speed, 0.0);
  EXPECT_DOUBLE_EQ(parameters.max_acceleration, 0.0);
  EXPECT_DOUBLE_EQ(parameters.max_jerk, 0.0);
  EXPECT_DOUBLE_EQ(parameters.max_tracking_error, 0.0);
}

TEST(WaypointController, ReachesWaypoint) {
  TrajectoryParameters parameters;
  parameters.max_speed          = 5.0;
  parameters.max_acceleration   = 3.0;
  parameters.max_jerk           = 4.0;
  parameters.max_tracking_error = 2.0;
  WaypointTrajectoryController controller(parameters);

  const Eigen::Vector3d initial_position(0.0, 0.0, 0.0);
  const Eigen::Vector3d waypoint(10.0, 5.0, 4.0);

  controller.reset(initial_position);
  Eigen::Vector3d vehicle_position = initial_position;

  const double dt                  = 0.02;
  const std::size_t num_iterations = 750;
  TrajectorySetpoints setpoints;
  for (std::size_t i = 0; i < num_iterations; ++i) {
    setpoints        = controller.update(dt, vehicle_position, waypoint);
    vehicle_position = setpoints.position;
  }

  EXPECT_LT((setpoints.position - waypoint).norm(), 1e-2);
  EXPECT_LT(setpoints.velocity.norm(), 1e-2);
}

TEST(WaypointController, AllDynamicLimitsDisabledThrows) {
  const TrajectoryParameters parameters;
  EXPECT_THROW({ WaypointTrajectoryController controller(parameters); }, std::invalid_argument);
}

TEST(WaypointController, RespectsGlobalBoundsWhenClimbing) {
  TrajectoryParameters parameters;
  parameters.max_speed        = 2.5;
  parameters.max_acceleration = 1.8;

  const Eigen::Vector3d initial_position(0.0, 0.0, 0.0);
  const Eigen::Vector3d waypoint(0.0, 0.0, 30.0);

  const MotionStats stats = simulate(parameters, initial_position, waypoint,
                                     /*dt=*/0.02, /*num_iterations=*/1500);

  EXPECT_LE(stats.max_speed, parameters.max_speed + 1e-6);
  EXPECT_LE(stats.max_acceleration, parameters.max_acceleration + 1e-6);
  EXPECT_GT(stats.max_speed, 0.5 * parameters.max_speed);
}

TEST(WaypointController, RespectsGlobalBoundsWhenDescending) {
  TrajectoryParameters parameters;
  parameters.max_speed        = 1.2;
  parameters.max_acceleration = 1.0;

  const Eigen::Vector3d initial_position(0.0, 0.0, 20.0);
  const Eigen::Vector3d waypoint(0.0, 0.0, -10.0);

  const MotionStats stats = simulate(parameters, initial_position, waypoint,
                                     /*dt=*/0.02, /*num_iterations=*/2500);

  EXPECT_LE(stats.max_speed, parameters.max_speed + 1e-6);
  EXPECT_LE(stats.max_acceleration, parameters.max_acceleration + 1e-6);
  EXPECT_GT(stats.max_speed, 0.5 * parameters.max_speed);
}

// --- Multi-waypoint ------------------------------------------------------

TEST(WaypointController, MultiWaypointZeroAcceptanceRadiusMatchesSingle) {
  // With acceptance_radius == 0 the controller's multi update overload must
  // produce the same trajectory as the single overload, even with a
  // non-trivial triplet. Same regression contract as the generator test.
  TrajectoryParameters parameters;
  parameters.max_speed         = 4.0;
  parameters.max_acceleration  = 2.0;
  parameters.max_jerk          = 8.0;
  parameters.acceptance_radius = 0.0;

  WaypointTrajectoryController single(parameters);
  WaypointTrajectoryController multi(parameters);
  const Eigen::Vector3d initial(0.0, 0.0, 0.0);
  single.reset(initial);
  multi.reset(initial);

  const Eigen::Vector3d target(10.0, 0.0, 0.0);
  const std::array<Eigen::Vector3d, 3> triplet = {Eigen::Vector3d::Zero(), target,
                                                  Eigen::Vector3d(10.0, 10.0, 0.0)};
  const double dt                              = 0.02;

  Eigen::Vector3d pos_single = initial;
  Eigen::Vector3d pos_multi  = initial;
  for (std::size_t i = 0; i < 500; ++i) {
    const TrajectorySetpoints s = single.update(dt, pos_single, target);
    const TrajectorySetpoints m = multi.update(dt, pos_multi, triplet);
    EXPECT_DOUBLE_EQ(s.position.x(), m.position.x()) << "iter " << i;
    EXPECT_DOUBLE_EQ(s.position.y(), m.position.y()) << "iter " << i;
    EXPECT_DOUBLE_EQ(s.position.z(), m.position.z()) << "iter " << i;
    pos_single = s.position;
    pos_multi  = m.position;
  }
}

TEST(WaypointController, MultiWaypointParametersAreReflectedInAccessor) {
  // The constructor must propagate acceptance_radius and trajectory_gain
  // into the underlying generator (so they are honoured by the multi
  // overload). Behavioural divergence between corner-cut on/off needs an
  // external triplet rotation, so we keep the assertion structural here
  // and add an end-to-end test below.
  TrajectoryParameters parameters;
  parameters.max_speed         = 4.0;
  parameters.max_acceleration  = 2.0;
  parameters.max_jerk          = 8.0;
  parameters.acceptance_radius = 1.5;
  parameters.trajectory_gain   = 0.75;

  WaypointTrajectoryController controller(parameters);
  EXPECT_DOUBLE_EQ(controller.parameters().acceptance_radius, 1.5);
  EXPECT_DOUBLE_EQ(controller.parameters().trajectory_gain, 0.75);
}

TEST(WaypointController, MultiWaypointWithExternalRotationCoversFullPath) {
  // End-to-end: with an external waypoint manager rotating the triplet on
  // arrival, the controller's multi update covers a two-segment path.
  TrajectoryParameters parameters;
  parameters.max_speed         = 4.0;
  parameters.max_acceleration  = 2.0;
  parameters.max_jerk          = 8.0;
  parameters.acceptance_radius = 1.0;
  parameters.trajectory_gain   = 1.0;

  WaypointTrajectoryController controller(parameters);
  const Eigen::Vector3d initial(0.0, 0.0, 0.0);
  controller.reset(initial);

  const Eigen::Vector3d wp1(10.0, 0.0, 0.0);
  const Eigen::Vector3d wp2(10.0, 10.0, 0.0);
  std::array<Eigen::Vector3d, 3> triplet = {initial, wp1, wp2};
  bool rotated                           = false;
  const double dt                        = 0.02;

  Eigen::Vector3d position = initial;
  for (std::size_t i = 0; i < 600; ++i) {
    position = controller.update(dt, position, triplet).position;
    if (!rotated && (position - wp1).norm() < 3.0) {
      triplet = {wp1, wp2, wp2};
      rotated = true;
    }
  }
  ASSERT_TRUE(rotated);
  EXPECT_LT((position - wp2).norm(), 0.5);
}

// --- Initial-state injection (reset overload with v, a) -------------------

namespace {
TrajectoryParameters makeStandardParameters() {
  TrajectoryParameters p;
  p.max_speed          = 5.0;
  p.max_acceleration   = 3.0;
  p.max_jerk           = 4.0;
  p.max_tracking_error = 2.0;
  return p;
}
}  // namespace

TEST(WaypointController, ResetWithoutInitialStateMatchesLegacy) {
  // The new reset(Waypoint) with explicit zero velocity / acceleration must
  // produce a trajectory bit-for-bit identical to the legacy reset(p) over
  // many ticks, so existing callers can adopt the new overload without
  // behavioural drift.
  const TrajectoryParameters parameters = makeStandardParameters();
  WaypointTrajectoryController legacy(parameters);
  WaypointTrajectoryController extended(parameters);

  const Eigen::Vector3d initial(0.0, 0.0, 0.0);
  Waypoint extended_state(initial);
  extended_state.velocity     = Eigen::Vector3d::Zero();
  extended_state.acceleration = Eigen::Vector3d::Zero();
  legacy.reset(initial);
  extended.reset(extended_state);

  const Eigen::Vector3d waypoint(10.0, 5.0, 4.0);
  const Eigen::Vector3d ff = Eigen::Vector3d::Zero();
  const double dt          = 0.02;

  Eigen::Vector3d pos_legacy   = initial;
  Eigen::Vector3d pos_extended = initial;
  for (std::size_t i = 0; i < 500; ++i) {
    const TrajectorySetpoints l = legacy.update(dt, pos_legacy, waypoint);
    const TrajectorySetpoints e = extended.update(dt, pos_extended, waypoint);
    EXPECT_DOUBLE_EQ(l.position.x(), e.position.x()) << "iter " << i;
    EXPECT_DOUBLE_EQ(l.position.y(), e.position.y()) << "iter " << i;
    EXPECT_DOUBLE_EQ(l.position.z(), e.position.z()) << "iter " << i;
    EXPECT_DOUBLE_EQ(l.velocity.x(), e.velocity.x()) << "iter " << i;
    EXPECT_DOUBLE_EQ(l.velocity.y(), e.velocity.y()) << "iter " << i;
    EXPECT_DOUBLE_EQ(l.velocity.z(), e.velocity.z()) << "iter " << i;
    pos_legacy   = l.position;
    pos_extended = e.position;
  }
}

TEST(WaypointController, ResetWithNonZeroInitialVelocityHonoursIt) {
  // reset(Waypoint) with a velocity field seeds the integrator with that
  // velocity. The first emitted position must therefore reflect the v * dt
  // displacement up to a quadratic correction from the initial response.
  TrajectoryParameters parameters = makeStandardParameters();
  parameters.max_speed            = 10.0;  // headroom so v_init isn't clipped.
  WaypointTrajectoryController controller(parameters);

  const Eigen::Vector3d initial(0.0, 0.0, 0.0);
  const Eigen::Vector3d v_init(2.0, 0.0, 0.0);
  Waypoint state(initial);
  state.velocity = v_init;
  controller.reset(state);

  const Eigen::Vector3d waypoint(20.0, 0.0, 0.0);
  const double dt                     = 0.02;
  const TrajectorySetpoints setpoints = controller.update(dt, initial, waypoint);

  // The emitted position should sit between (initial + v_init * dt) and that
  // value plus a small quadratic correction (a_max * dt^2 / 2 ~ 6e-4 m here).
  // We allow a generous 5 mm window so the test stays robust to the exact
  // jerk-up phase profile.
  const Eigen::Vector3d expected = initial + v_init * dt;
  EXPECT_NEAR(setpoints.position.x(), expected.x(), 5e-3);
  EXPECT_NEAR(setpoints.position.y(), expected.y(), 1e-9);
  EXPECT_NEAR(setpoints.position.z(), expected.z(), 1e-9);
  // The emitted velocity is the v_init filtered by the post-clip stage; with
  // v_init below max_speed it must remain close to v_init at the first tick.
  EXPECT_NEAR(setpoints.velocity.x(), v_init.x(), 0.2);
  EXPECT_LT(setpoints.velocity.norm(), parameters.max_speed + 1e-6);
}

TEST(WaypointController, ResetWithNonZeroInitialAccelerationDoesNotDiverge) {
  // The reactive integrator does not propagate a seeded initial acceleration
  // into the very first emitted setpoint (T1=T2=T3=0 immediately after
  // reset, so updateTrajectory falls through to the post-trajectory branch
  // and reports zero acceleration). This test confirms the documented
  // behaviour: seeding a non-zero initial acceleration is accepted, the
  // trajectory remains finite and within global bounds across many ticks,
  // and the seeded velocity is still honoured at tick 0. See the doxygen
  // of WaypointTrajectoryController::reset for the full rationale.
  TrajectoryParameters parameters = makeStandardParameters();
  parameters.max_speed            = 10.0;
  WaypointTrajectoryController controller(parameters);

  const Eigen::Vector3d initial(0.0, 0.0, 0.0);
  const Eigen::Vector3d v_init(1.0, 0.0, 0.0);
  const Eigen::Vector3d a_init(0.5, 0.0, 0.0);
  Waypoint state(initial);
  state.velocity     = v_init;
  state.acceleration = a_init;
  controller.reset(state);

  const Eigen::Vector3d waypoint(20.0, 0.0, 0.0);
  const double dt = 0.02;

  Eigen::Vector3d position = initial;
  TrajectorySetpoints setpoints;
  for (std::size_t i = 0; i < 200; ++i) {
    setpoints = controller.update(dt, position, waypoint);
    position  = setpoints.position;
    EXPECT_TRUE(std::isfinite(setpoints.velocity.norm())) << "iter " << i;
    EXPECT_LE(setpoints.velocity.norm(), parameters.max_speed + 1e-6) << "iter " << i;
    EXPECT_LE(setpoints.acceleration.norm(), parameters.max_acceleration + 1e-6) << "iter " << i;
  }
  // The trajectory progresses toward the waypoint despite the unusual seed.
  EXPECT_GT(position.x(), 1.0);
}

TEST(WaypointController, ResetWithVelocityAboveMaxSpeedGetsClippedAtFirstTick) {
  // The reset accepts an out-of-bounds initial velocity verbatim; the
  // post-clip + re-inject inside the next update() must clamp the emitted
  // velocity to max_speed. The actual seeded state may be temporarily
  // infeasible, but no setpoint emitted to the caller may violate the bound.
  TrajectoryParameters parameters = makeStandardParameters();
  parameters.max_speed            = 1.0;
  WaypointTrajectoryController controller(parameters);

  const Eigen::Vector3d initial(0.0, 0.0, 0.0);
  const Eigen::Vector3d v_init(5.0, 0.0, 0.0);  // 5x above max_speed.
  Waypoint state(initial);
  state.velocity = v_init;
  controller.reset(state);

  const TrajectorySetpoints setpoints =
      controller.update(0.02, initial, Eigen::Vector3d(20.0, 0.0, 0.0));
  EXPECT_LE(setpoints.velocity.norm(), parameters.max_speed + 1e-6);
}

}  // namespace
}  // namespace trajectory_generator_jerk_limited
