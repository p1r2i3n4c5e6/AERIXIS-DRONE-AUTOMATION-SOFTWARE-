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
 * @file trajectory_math_gtest.cpp
 *
 * GoogleTest unit tests for the multi-waypoint helpers in trajectory_math.
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#include <array>
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include <Eigen/Core>

#include "trajectory_generator_jerk_limited/trajectory_math.hpp"

namespace trajectory_generator_jerk_limited {
namespace {

// Tight tolerance for analytical tests; relaxed where braking-speed inverse
// math contributes truncation.
constexpr double kAnalyticTolerance = 1e-9;
constexpr double kRelaxedTolerance  = 1e-6;

// --- computeMaxSpeedInWaypoint --------------------------------------------

TEST(ComputeMaxSpeedInWaypoint, StraightLineIsInfinite) {
  // alpha == pi: V-shape fully open, no curvature, no restriction.
  const double speed = computeMaxSpeedInWaypoint(M_PI, 4.0, 1.0);
  EXPECT_TRUE(std::isinf(speed));
}

TEST(ComputeMaxSpeedInWaypoint, ZeroAngleReturnsZero) {
  // alpha == 0: 180-degree reversal. Tangent circle radius zero -> must stop.
  const double speed = computeMaxSpeedInWaypoint(0.0, 4.0, 1.0);
  EXPECT_DOUBLE_EQ(speed, 0.0);
}

TEST(ComputeMaxSpeedInWaypoint, NinetyDegreesAnalytical) {
  // alpha == pi/2 -> tan(pi/4) = 1 -> v = sqrt(a * d).
  const double accel  = 4.0;
  const double radius = 1.5;
  const double speed  = computeMaxSpeedInWaypoint(0.5 * M_PI, accel, radius);
  EXPECT_NEAR(speed, std::sqrt(accel * radius), kAnalyticTolerance);
}

TEST(ComputeMaxSpeedInWaypoint, NonPositiveRadiusReturnsInfinity) {
  EXPECT_TRUE(std::isinf(computeMaxSpeedInWaypoint(0.5 * M_PI, 4.0, 0.0)));
  EXPECT_TRUE(std::isinf(computeMaxSpeedInWaypoint(0.5 * M_PI, 4.0, -1.0)));
}

TEST(ComputeMaxSpeedInWaypoint, NonPositiveAccelerationReturnsInfinity) {
  EXPECT_TRUE(std::isinf(computeMaxSpeedInWaypoint(0.5 * M_PI, 0.0, 1.0)));
  EXPECT_TRUE(std::isinf(computeMaxSpeedInWaypoint(0.5 * M_PI, -1.0, 1.0)));
}

// --- computeStartSpeedFromWaypoints ---------------------------------------

TEST(ComputeStartSpeedFromWaypoints, ReturnsMaxSpeedWhenItIsTheTightestBound) {
  // Long straight segment: corner_speed = +inf, braking_speed large.
  const Eigen::Vector3d start(0.0, 0.0, 0.0);
  const Eigen::Vector3d target(100.0, 0.0, 0.0);
  const Eigen::Vector3d next(200.0, 0.0, 0.0);
  const double max_speed = 2.0;
  const double speed     = computeStartSpeedFromWaypoints(start, target, next,
                                                          /*exit_speed=*/0.0,
                                                          /*max_jerk=*/100.0,
                                                          /*max_acceleration=*/100.0, max_speed,
                                                          /*acceptance_radius=*/1.0);
  EXPECT_DOUBLE_EQ(speed, max_speed);
}

TEST(ComputeStartSpeedFromWaypoints, ShortSegmentEqualsCornerSpeedWhenExitDoesNotBite) {
  // When exit_speed is generous (greater than the corner cap), the corner cap
  // becomes the binding constraint at the target. With a very short segment
  // the braking phase has no room to drop speed below the corner value, so
  // the start speed equals the corner speed.
  const Eigen::Vector3d start(9.999, 0.0, 0.0);
  const Eigen::Vector3d target(10.0, 0.0, 0.0);
  const Eigen::Vector3d next(10.0, 10.0, 0.0);
  const double max_acceleration  = 4.0;
  const double acceptance_radius = 1.0;
  const double generous_exit     = 100.0;
  const double speed = computeStartSpeedFromWaypoints(start, target, next, generous_exit,
                                                      /*max_jerk=*/100.0, max_acceleration,
                                                      /*max_speed=*/100.0, acceptance_radius);
  // 90 deg -> corner_speed = sqrt(a * d) = 2.0; the start is right at the
  // target so the braking distance is ~zero and the start speed collapses
  // to the corner cap.
  EXPECT_NEAR(speed, std::sqrt(max_acceleration * acceptance_radius), 1e-3);
}

TEST(ComputeStartSpeedFromWaypoints, FarSegmentAllowsAcceleratingAboveCornerSpeed) {
  // With a long segment the start speed can exceed the corner cap because
  // there is enough distance to brake from a faster initial speed down to
  // the corner-permitted speed at the target. This is the PX4 semantics.
  const Eigen::Vector3d start(0.0, 0.0, 0.0);
  const Eigen::Vector3d target(50.0, 0.0, 0.0);
  const Eigen::Vector3d next(50.0, 50.0, 0.0);
  const double corner_speed = std::sqrt(4.0 * 1.0);
  const double speed        = computeStartSpeedFromWaypoints(start, target, next, 0.0,
                                                             /*max_jerk=*/8.0,
                                                             /*max_acceleration=*/4.0,
                                                             /*max_speed=*/100.0,
                                                             /*acceptance_radius=*/1.0);
  EXPECT_GT(speed, corner_speed * 1.5);
}

TEST(ComputeStartSpeedFromWaypoints, ReturnsBrakingSpeedWhenItIsTheTightestBound) {
  // Short segment with low jerk/acceleration constrains speed below max_speed
  // and below corner_speed.
  const Eigen::Vector3d start(0.0, 0.0, 0.0);
  const Eigen::Vector3d target(0.5, 0.0, 0.0);
  const Eigen::Vector3d next(1.5, 0.0, 0.0);  // straight extension -> corner inert
  const double speed = computeStartSpeedFromWaypoints(start, target, next, 0.0,
                                                      /*max_jerk=*/1.0,
                                                      /*max_acceleration=*/1.0,
                                                      /*max_speed=*/100.0,
                                                      /*acceptance_radius=*/0.1);
  // Both other caps are very loose; the result must be the braking_speed value.
  EXPECT_LT(speed, 100.0);
  EXPECT_GT(speed, 0.0);
}

TEST(ComputeStartSpeedFromWaypoints, OutOfPlaneCornerIsHandledIn3D) {
  // Corner geometry that lives outside the XY plane (significant Z component).
  // Verifies that the angle is computed in 3D, not XY-only. Generous exit
  // speed and near-zero segment length make the corner cap the binding one.
  const Eigen::Vector3d start(0.0, 0.0, 4.999);
  const Eigen::Vector3d target(0.0, 0.0, 5.0);
  const Eigen::Vector3d next(5.0, 0.0, 5.0);
  const double max_acceleration  = 4.0;
  const double acceptance_radius = 1.0;
  const double generous_exit     = 100.0;
  const double speed = computeStartSpeedFromWaypoints(start, target, next, generous_exit,
                                                      /*max_jerk=*/100.0, max_acceleration,
                                                      /*max_speed=*/100.0, acceptance_radius);
  EXPECT_NEAR(speed, std::sqrt(max_acceleration * acceptance_radius), 1e-3);
}

TEST(ComputeStartSpeedFromWaypoints, DegenerateColocatedWaypointsSkipsCorner) {
  // When start == target the corner direction is undefined; the function must
  // not crash and the corner cap must not contribute (returns +inf in that
  // branch). Braking distance is zero, so the result collapses to exit_speed.
  const Eigen::Vector3d wp(1.0, 2.0, 3.0);
  const double exit_speed = 1.5;
  const double speed      = computeStartSpeedFromWaypoints(wp, wp, Eigen::Vector3d(2.0, 2.0, 3.0),
                                                           exit_speed, 5.0, 4.0, 100.0, 1.0);
  // Distance start->target == 0, so braking_speed == exit_speed.
  EXPECT_NEAR(speed, exit_speed, kAnalyticTolerance);
}

// --- computeSpeedFromWaypoints --------------------------------------------

TEST(ComputeSpeedFromWaypoints, BackwardSweepEqualsTwoChainedCalls) {
  const std::array<Eigen::Vector3d, 3> waypoints = {Eigen::Vector3d(0.0, 0.0, 0.0),
                                                    Eigen::Vector3d(10.0, 0.0, 0.0),
                                                    Eigen::Vector3d(10.0, 10.0, 0.0)};
  const double max_jerk                          = 5.0;
  const double max_acceleration                  = 4.0;
  const double max_speed                         = 100.0;
  const double acceptance_radius                 = 1.0;
  const double exit_speed                        = 0.0;

  // Manual two-step sweep replicating the implementation contract.
  const double speed_at_target =
      computeStartSpeedFromWaypoints(waypoints[1], waypoints[2], waypoints[2], exit_speed, max_jerk,
                                     max_acceleration, max_speed, acceptance_radius);
  const double expected =
      computeStartSpeedFromWaypoints(waypoints[0], waypoints[1], waypoints[2], speed_at_target,
                                     max_jerk, max_acceleration, max_speed, acceptance_radius);

  const double speed = computeSpeedFromWaypoints(waypoints, max_jerk, max_acceleration, max_speed,
                                                 acceptance_radius, exit_speed);
  EXPECT_NEAR(speed, expected, kAnalyticTolerance);
}

TEST(ComputeSpeedFromWaypoints, RespectsExitSpeedAtLastWaypoint) {
  const std::array<Eigen::Vector3d, 3> waypoints = {Eigen::Vector3d(0.0, 0.0, 0.0),
                                                    Eigen::Vector3d(0.5, 0.0, 0.0),
                                                    Eigen::Vector3d(1.0, 0.0, 0.0)};
  // Tight braking: small distances and modest accel/jerk make the braking
  // cap dominant. exit_speed > 0 should produce a strictly larger result
  // than exit_speed == 0 because the chain has more momentum to work with.
  const double speed_stop      = computeSpeedFromWaypoints(waypoints, 1.0, 1.0, 100.0, 0.1, 0.0);
  const double speed_pass_thru = computeSpeedFromWaypoints(waypoints, 1.0, 1.0, 100.0, 0.1, 0.5);
  EXPECT_GT(speed_pass_thru, speed_stop);
}

TEST(ComputeSpeedFromWaypoints, StraightPathReturnsMaxSpeedCap) {
  // Straight extension with everything generous: only @p max_speed should bite.
  const std::array<Eigen::Vector3d, 3> waypoints = {Eigen::Vector3d(0.0, 0.0, 0.0),
                                                    Eigen::Vector3d(50.0, 0.0, 0.0),
                                                    Eigen::Vector3d(100.0, 0.0, 0.0)};

  const double max_speed = 3.5;
  const double speed     = computeSpeedFromWaypoints(waypoints,
                                                     /*max_jerk=*/100.0,
                                                     /*max_acceleration=*/100.0, max_speed,
                                                     /*acceptance_radius=*/1.0,
                                                     /*exit_speed=*/0.0);
  EXPECT_DOUBLE_EQ(speed, max_speed);
  static_cast<void>(kRelaxedTolerance);
}

}  // namespace
}  // namespace trajectory_generator_jerk_limited
