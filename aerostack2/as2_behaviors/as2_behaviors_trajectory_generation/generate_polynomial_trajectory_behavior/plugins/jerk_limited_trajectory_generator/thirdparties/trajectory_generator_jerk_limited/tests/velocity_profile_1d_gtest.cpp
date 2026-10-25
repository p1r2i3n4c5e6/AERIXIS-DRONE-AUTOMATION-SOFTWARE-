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
 * @file velocity_profile_1d_gtest.cpp
 *
 * GoogleTest unit tests for JerkLimitedVelocityProfile1D.
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "trajectory_generator_jerk_limited/velocity_profile_1d.hpp"

namespace trajectory_generator_jerk_limited {
namespace {

JerkLimitedVelocityProfile1D makeProfile(const double max_jerk,
                                         const double max_acceleration,
                                         const double max_velocity) {
  JerkLimitedVelocityProfile1D profile;
  profile.setMaxJerk(max_jerk);
  profile.setMaxAcceleration(max_acceleration);
  profile.setMaxVelocity(max_velocity);
  profile.reset(0.0, 0.0, 0.0);
  return profile;
}

TEST(VelocityProfile1D, PhaseDurationsAreNonNegative) {
  auto profile = makeProfile(4.0, 3.0, 5.0);
  profile.updateDurations(5.0);

  EXPECT_GE(profile.phase1Duration(), 0.0);
  EXPECT_GE(profile.phase2Duration(), 0.0);
  EXPECT_GE(profile.phase3Duration(), 0.0);
  EXPECT_GT(profile.totalTime(), 0.0);
}

TEST(VelocityProfile1D, ReachesSetpointWithZeroAcceleration) {
  auto profile                   = makeProfile(4.0, 3.0, 5.0);
  const double velocity_setpoint = 5.0;
  profile.updateDurations(velocity_setpoint);

  // Integrate well past the total time.
  const double dt             = 0.01;
  const double total_sim_time = profile.totalTime() + 0.5;
  const int num_steps         = static_cast<int>(total_sim_time / dt);
  for (int i = 0; i < num_steps; ++i) {
    profile.updateTrajectory(dt);
  }

  EXPECT_NEAR(profile.currentVelocity(), velocity_setpoint, 1e-6);
  EXPECT_NEAR(profile.currentAcceleration(), 0.0, 1e-6);
}

TEST(VelocityProfile1D, SynchronizeEqualizesTotalTime) {
  auto profile_fast   = makeProfile(4.0, 3.0, 5.0);
  auto profile_medium = makeProfile(4.0, 3.0, 5.0);
  auto profile_slow   = makeProfile(4.0, 3.0, 5.0);

  profile_fast.updateDurations(1.0);
  profile_medium.updateDurations(2.5);
  profile_slow.updateDurations(5.0);

  std::vector<JerkLimitedVelocityProfile1D*> profiles = {&profile_fast, &profile_medium,
                                                         &profile_slow};
  JerkLimitedVelocityProfile1D::synchronize(profiles);

  const double reference = profile_slow.totalTime();
  EXPECT_NEAR(profile_fast.totalTime(), reference, 1e-9);
  EXPECT_NEAR(profile_medium.totalTime(), reference, 1e-9);
  EXPECT_NEAR(profile_slow.totalTime(), reference, 1e-9);
}

TEST(VelocityProfile1D, JerkRespectsLimitAndIsBoundedByMaxAcceleration) {
  const double max_jerk         = 4.0;
  const double max_acceleration = 3.0;
  auto profile                  = makeProfile(max_jerk, max_acceleration, 5.0);
  profile.updateDurations(5.0);

  const double dt             = 0.01;
  const double total_sim_time = profile.totalTime() + 0.5;
  const int num_steps         = static_cast<int>(total_sim_time / dt);

  double max_observed_jerk         = 0.0;
  double max_observed_acceleration = 0.0;
  for (int i = 0; i < num_steps; ++i) {
    profile.updateTrajectory(dt);
    max_observed_jerk = std::max(max_observed_jerk, std::fabs(profile.currentJerk()));
    max_observed_acceleration =
        std::max(max_observed_acceleration, std::fabs(profile.currentAcceleration()));
  }

  EXPECT_LE(max_observed_jerk, max_jerk + 1e-9);
  EXPECT_LE(max_observed_acceleration, max_acceleration + 1e-6);
}

}  // namespace
}  // namespace trajectory_generator_jerk_limited
