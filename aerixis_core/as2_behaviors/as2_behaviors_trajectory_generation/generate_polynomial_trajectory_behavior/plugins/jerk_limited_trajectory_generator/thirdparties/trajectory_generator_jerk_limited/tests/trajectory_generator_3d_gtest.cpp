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
 * @file trajectory_generator_3d_gtest.cpp
 *
 * GoogleTest unit tests for JerkLimitedTrajectoryGenerator3D.
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

#include "trajectory_generator_jerk_limited/trajectory_generator_3d.hpp"

namespace trajectory_generator_jerk_limited {
namespace {

struct Parameters {
  double max_speed          = 5.0;
  double max_acceleration   = 3.0;
  double jerk               = 4.0;
  double max_tracking_error = 2.0;
};

JerkLimitedTrajectoryGenerator3D makeGenerator(const Parameters& parameters) {
  JerkLimitedTrajectoryGenerator3D generator;
  generator.setMaxJerk(parameters.jerk);
  generator.setMaxSpeed(parameters.max_speed);
  generator.setMaxAcceleration(parameters.max_acceleration);
  generator.setMaxTrackingError(parameters.max_tracking_error);
  return generator;
}

TEST(TrajectoryGenerator3D, ReachesWaypointWithZeroVelocity) {
  const Parameters params;
  auto generator = makeGenerator(params);
  generator.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  const Eigen::Vector3d waypoint(10.0, 5.0, 4.0);
  const Eigen::Vector3d feedforward_velocity = Eigen::Vector3d::Zero();
  const double dt                            = 0.02;
  const std::size_t num_iterations           = 750;  // 15 s at 50 Hz.

  Eigen::Vector3d vehicle_position = Eigen::Vector3d::Zero();
  TrajectorySetpoints setpoints;
  for (std::size_t i = 0; i < num_iterations; ++i) {
    setpoints        = generator.generate(vehicle_position, waypoint, feedforward_velocity, dt);
    vehicle_position = setpoints.position;
  }

  EXPECT_LT((setpoints.position - waypoint).norm(), 1e-2);
  EXPECT_LT(setpoints.velocity.norm(), 1e-2);
}

TEST(TrajectoryGenerator3D, RespectsGlobalSpeedAccelerationAndJerkBounds) {
  const Parameters params;
  auto generator = makeGenerator(params);
  generator.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  const Eigen::Vector3d waypoint(10.0, 5.0, 4.0);
  const Eigen::Vector3d feedforward_velocity = Eigen::Vector3d::Zero();
  const double dt                            = 0.02;
  const std::size_t num_iterations           = 750;

  Eigen::Vector3d vehicle_position = Eigen::Vector3d::Zero();
  double max_speed_observed        = 0.0;
  double max_acc_observed          = 0.0;
  double max_jerk_observed         = 0.0;
  for (std::size_t i = 0; i < num_iterations; ++i) {
    const TrajectorySetpoints setpoints =
        generator.generate(vehicle_position, waypoint, feedforward_velocity, dt);
    vehicle_position = setpoints.position;

    max_speed_observed = std::max(max_speed_observed, setpoints.velocity.norm());
    max_acc_observed   = std::max(max_acc_observed, setpoints.acceleration.norm());
    max_jerk_observed  = std::max({max_jerk_observed, std::fabs(setpoints.jerk.x()),
                                   std::fabs(setpoints.jerk.y()), std::fabs(setpoints.jerk.z())});
  }

  EXPECT_LE(max_speed_observed, params.max_speed + 1e-6);
  EXPECT_LE(max_acc_observed, params.max_acceleration + 1e-6);
  EXPECT_LE(max_jerk_observed, params.jerk + 1e-9);
}

TEST(TrajectoryGenerator3D, DiagonalTrajectoryRespectsGlobalSpeedBound) {
  Parameters params;
  params.max_speed = 2.0;
  auto generator   = makeGenerator(params);
  generator.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  const Eigen::Vector3d waypoint(30.0, 30.0, 30.0);
  const Eigen::Vector3d feedforward_velocity = Eigen::Vector3d::Zero();
  const double dt                            = 0.02;
  const std::size_t num_iterations           = 1000;

  Eigen::Vector3d vehicle_position = Eigen::Vector3d::Zero();
  double max_speed_observed        = 0.0;
  for (std::size_t i = 0; i < num_iterations; ++i) {
    const TrajectorySetpoints setpoints =
        generator.generate(vehicle_position, waypoint, feedforward_velocity, dt);
    vehicle_position   = setpoints.position;
    max_speed_observed = std::max(max_speed_observed, setpoints.velocity.norm());
  }

  EXPECT_LE(max_speed_observed, params.max_speed + 1e-6);
}

TEST(TrajectoryGenerator3D, ZeroTrackingErrorDisablesTimeStretchGate) {
  Parameters disabled_gate_params;
  disabled_gate_params.max_tracking_error = 0.0;
  auto disabled_gate_generator            = makeGenerator(disabled_gate_params);

  Parameters enabled_gate_params;
  enabled_gate_params.max_tracking_error = 0.1;
  auto enabled_gate_generator            = makeGenerator(enabled_gate_params);

  disabled_gate_generator.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                Eigen::Vector3d::Zero());
  enabled_gate_generator.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                               Eigen::Vector3d::Zero());

  const Eigen::Vector3d lagging_vehicle_position(-100.0, 0.0, 0.0);
  const Eigen::Vector3d waypoint(10.0, 0.0, 0.0);
  const Eigen::Vector3d feedforward_velocity = Eigen::Vector3d::Zero();
  const double dt                            = 0.02;

  // First tick primes the per-axis durations.
  disabled_gate_generator.generate(lagging_vehicle_position, waypoint, feedforward_velocity, dt);
  enabled_gate_generator.generate(lagging_vehicle_position, waypoint, feedforward_velocity, dt);

  // Second tick highlights the gate effect.
  const TrajectorySetpoints no_gate_setpoints = disabled_gate_generator.generate(
      lagging_vehicle_position, waypoint, feedforward_velocity, dt);
  const TrajectorySetpoints gated_setpoints =
      enabled_gate_generator.generate(lagging_vehicle_position, waypoint, feedforward_velocity, dt);

  EXPECT_GT(no_gate_setpoints.velocity.norm(), 1e-6);
  EXPECT_LT(gated_setpoints.velocity.norm(), 1e-9);
}

TEST(TrajectoryGenerator3D, AllDynamicLimitsDisabledThrows) {
  JerkLimitedTrajectoryGenerator3D generator;
  generator.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  EXPECT_THROW(generator.generate(Eigen::Vector3d::Zero(), Eigen::Vector3d(1.0, 0.0, 0.0),
                                  Eigen::Vector3d::Zero(), 0.02),
               std::invalid_argument);
}

// --- Multi-waypoint regression and behaviour ------------------------------

TEST(TrajectoryGenerator3D, MultiWaypointRepeatedTargetMatchesSingleBitForBit) {
  // Driving the multi overload with {wp, wp, wp} should produce exactly the
  // same sequence as the single overload tick by tick. This is the key
  // regression contract: it lets external callers migrate to the multi API
  // for trivial paths without altering numerical behaviour.
  const Parameters params;
  auto single = makeGenerator(params);
  auto multi  = makeGenerator(params);
  // Acceptance radius / trajectory gain set on the multi generator should be
  // ignored when the triplet collapses to a single waypoint.
  multi.setAcceptanceRadius(1.0);
  multi.setTrajectoryGain(1.5);
  single.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  multi.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  const Eigen::Vector3d waypoint(10.0, 5.0, 4.0);
  const std::array<Eigen::Vector3d, 3> triplet = {waypoint, waypoint, waypoint};
  const Eigen::Vector3d ff                     = Eigen::Vector3d::Zero();
  const double dt                              = 0.02;
  const std::size_t num_iterations             = 750;

  Eigen::Vector3d pos_single = Eigen::Vector3d::Zero();
  Eigen::Vector3d pos_multi  = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < num_iterations; ++i) {
    const TrajectorySetpoints s = single.generate(pos_single, waypoint, ff, dt);
    const TrajectorySetpoints m = multi.generate(pos_multi, triplet, ff, dt);
    EXPECT_DOUBLE_EQ(s.position.x(), m.position.x()) << "iter " << i;
    EXPECT_DOUBLE_EQ(s.position.y(), m.position.y()) << "iter " << i;
    EXPECT_DOUBLE_EQ(s.position.z(), m.position.z()) << "iter " << i;
    EXPECT_DOUBLE_EQ(s.velocity.x(), m.velocity.x()) << "iter " << i;
    EXPECT_DOUBLE_EQ(s.velocity.y(), m.velocity.y()) << "iter " << i;
    EXPECT_DOUBLE_EQ(s.velocity.z(), m.velocity.z()) << "iter " << i;
    EXPECT_DOUBLE_EQ(s.acceleration.x(), m.acceleration.x()) << "iter " << i;
    EXPECT_DOUBLE_EQ(s.acceleration.y(), m.acceleration.y()) << "iter " << i;
    EXPECT_DOUBLE_EQ(s.acceleration.z(), m.acceleration.z()) << "iter " << i;
    pos_single = s.position;
    pos_multi  = m.position;
  }
}

TEST(TrajectoryGenerator3D, MultiWaypointZeroAcceptanceRadiusMatchesSingle) {
  // With acceptance_radius == 0 the multi-waypoint path is supposed to
  // degrade to the single-waypoint behaviour even with a non-trivial triplet.
  const Parameters params;
  auto single = makeGenerator(params);
  auto multi  = makeGenerator(params);
  multi.setAcceptanceRadius(0.0);
  single.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  multi.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  const Eigen::Vector3d target(10.0, 0.0, 0.0);
  const std::array<Eigen::Vector3d, 3> triplet = {Eigen::Vector3d::Zero(), target,
                                                  Eigen::Vector3d(10.0, 10.0, 0.0)};
  const Eigen::Vector3d ff                     = Eigen::Vector3d::Zero();
  const double dt                              = 0.02;

  Eigen::Vector3d pos_single = Eigen::Vector3d::Zero();
  Eigen::Vector3d pos_multi  = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < 500; ++i) {
    const TrajectorySetpoints s = single.generate(pos_single, target, ff, dt);
    const TrajectorySetpoints m = multi.generate(pos_multi, triplet, ff, dt);
    EXPECT_DOUBLE_EQ(s.position.x(), m.position.x()) << "iter " << i;
    EXPECT_DOUBLE_EQ(s.position.y(), m.position.y()) << "iter " << i;
    EXPECT_DOUBLE_EQ(s.position.z(), m.position.z()) << "iter " << i;
    pos_single = s.position;
    pos_multi  = m.position;
  }
}

TEST(TrajectoryGenerator3D, MultiWaypointCornerPathDoesNotStopAtTarget) {
  // The hallmark of the multi-waypoint path is that the trajectory does NOT
  // brake to a halt at the intermediate target: the L1 corner-cut and the
  // corner-speed sweep let the vehicle keep momentum across the turn. The
  // single-waypoint reference, by contrast, decelerates to ~zero at the
  // target. We compare them at the same iteration to make the test robust.
  Parameters params;
  params.max_speed        = 4.0;
  params.max_acceleration = 2.0;
  params.jerk             = 8.0;
  auto multi              = makeGenerator(params);
  multi.setAcceptanceRadius(1.0);
  multi.setTrajectoryGain(1.0);
  multi.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  auto single = makeGenerator(params);
  single.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  const std::array<Eigen::Vector3d, 3> triplet = {Eigen::Vector3d(0.0, 0.0, 0.0),
                                                  Eigen::Vector3d(10.0, 0.0, 0.0),
                                                  Eigen::Vector3d(10.0, 10.0, 0.0)};
  const Eigen::Vector3d ff                     = Eigen::Vector3d::Zero();
  const double dt                              = 0.02;

  // Mimic the external waypoint manager: when the multi vehicle approaches
  // the intermediate target, rotate the triplet to {target, next, next} so
  // that the next segment becomes the new active one. PX4 Navigator does
  // this; this library is intentionally manager-less.
  //
  // With the multi path the trajectory continues into the second segment
  // and reaches waypoints[2] before the simulation horizon. The
  // single-waypoint reference, with no manager, parks at waypoints[1].
  // We compare arc lengths.
  std::array<Eigen::Vector3d, 3> active_triplet = triplet;
  Eigen::Vector3d multi_position                = Eigen::Vector3d::Zero();
  Eigen::Vector3d single_position               = Eigen::Vector3d::Zero();
  double multi_arc_length                       = 0.0;
  double single_arc_length                      = 0.0;
  bool rotated                                  = false;
  for (std::size_t i = 0; i < 600; ++i) {
    const TrajectorySetpoints m = multi.generate(multi_position, active_triplet, ff, dt);
    const TrajectorySetpoints s = single.generate(single_position, triplet[1], ff, dt);
    multi_arc_length += (m.position - multi_position).norm();
    single_arc_length += (s.position - single_position).norm();
    multi_position  = m.position;
    single_position = s.position;

    // Trigger triplet rotation when within ~3 m of the intermediate target.
    if (!rotated && (multi_position - triplet[1]).norm() < 3.0) {
      active_triplet = {triplet[1], triplet[2], triplet[2]};
      rotated        = true;
    }
  }

  ASSERT_TRUE(rotated) << "multi never reached the target neighborhood";
  // Multi advances into the second segment after the rotation; single parks.
  // Total path length is ~20 m for multi, ~10 m for single.
  EXPECT_GT(multi_arc_length, single_arc_length + 5.0) << "multi did not progress past target";
  EXPECT_LT((single_position - triplet[1]).norm(), 0.05) << "single did not converge to target";
  // Multi reaches the second waypoint within the horizon.
  EXPECT_LT((multi_position - triplet[2]).norm(), 0.5) << "multi did not reach final waypoint";
}

TEST(TrajectoryGenerator3D, MultiWaypointRespectsGlobal3DBounds) {
  // The post-clip enforces the 3D-norm caps on velocity and acceleration. The
  // multi-waypoint path must respect them just like the single-waypoint one.
  Parameters params;
  params.max_speed        = 3.0;
  params.max_acceleration = 1.5;
  params.jerk             = 6.0;
  auto generator          = makeGenerator(params);
  generator.setAcceptanceRadius(1.0);
  generator.reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  const std::array<Eigen::Vector3d, 3> triplet = {Eigen::Vector3d(0.0, 0.0, 0.0),
                                                  Eigen::Vector3d(8.0, 0.0, 2.0),
                                                  Eigen::Vector3d(8.0, 6.0, 4.0)};
  const Eigen::Vector3d ff                     = Eigen::Vector3d::Zero();
  const double dt                              = 0.02;

  Eigen::Vector3d vehicle_position = Eigen::Vector3d::Zero();
  double max_speed_observed        = 0.0;
  double max_accel_observed        = 0.0;
  double max_jerk_axis_observed    = 0.0;
  for (std::size_t i = 0; i < 600; ++i) {
    const TrajectorySetpoints s = generator.generate(vehicle_position, triplet, ff, dt);
    vehicle_position            = s.position;
    max_speed_observed          = std::max(max_speed_observed, s.velocity.norm());
    max_accel_observed          = std::max(max_accel_observed, s.acceleration.norm());
    const double jx             = std::abs(s.jerk.x());
    const double jy             = std::abs(s.jerk.y());
    const double jz             = std::abs(s.jerk.z());
    max_jerk_axis_observed      = std::max({max_jerk_axis_observed, jx, jy, jz});
  }
  EXPECT_LE(max_speed_observed, params.max_speed + 1e-6);
  EXPECT_LE(max_accel_observed, params.max_acceleration + 1e-6);
  EXPECT_LE(max_jerk_axis_observed, params.jerk + 1e-6);
}

}  // namespace
}  // namespace trajectory_generator_jerk_limited
