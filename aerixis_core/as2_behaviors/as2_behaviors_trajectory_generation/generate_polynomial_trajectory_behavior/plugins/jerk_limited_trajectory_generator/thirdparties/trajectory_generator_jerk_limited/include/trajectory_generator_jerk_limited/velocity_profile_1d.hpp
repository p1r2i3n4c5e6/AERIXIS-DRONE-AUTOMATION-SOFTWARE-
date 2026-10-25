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
 * @file velocity_profile_1d.hpp
 *
 * Per-axis jerk-limited S-curve velocity profile.
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#ifndef TRAJECTORY_GENERATOR_JERK_LIMITED__VELOCITY_PROFILE_1D_HPP_
#define TRAJECTORY_GENERATOR_JERK_LIMITED__VELOCITY_PROFILE_1D_HPP_

#include <vector>

namespace trajectory_generator_jerk_limited {

/**
 * @brief One-dimensional jerk-limited S-curve velocity profile.
 *
 * Given a target velocity setpoint and jerk / acceleration / velocity bounds,
 * the profile is split into three phases with durations T1 (jerk up-ramp),
 * T2 (constant acceleration) and T3 (jerk down-ramp). On each tick the
 * polynomial is integrated analytically starting from the state captured at
 * the last call to updateDurations(), so errors do not accumulate.
 *
 * A static synchronize() helper stretches shorter profiles to match the
 * longest, which is how straight-line XYZ trajectories are built.
 */
class JerkLimitedVelocityProfile1D {
public:
  /// State snapshot (jerk, acceleration, velocity, position) at one instant.
  struct State {
    double jerk         = 0.0;
    double acceleration = 0.0;
    double velocity     = 0.0;
    double position     = 0.0;
  };

  JerkLimitedVelocityProfile1D() = default;

  /**
   * @brief Construct the profile with an initial kinematic state.
   *
   * @param initial_acceleration  Initial acceleration (m/s^2).
   * @param initial_velocity      Initial velocity (m/s).
   * @param initial_position      Initial position (m).
   */
  JerkLimitedVelocityProfile1D(const double initial_acceleration,
                               const double initial_velocity,
                               const double initial_position);

  /**
   * @brief Reset the current and snapshot states to given values.
   *
   * Clears the accumulated phase time and sets the jerk to zero.
   *
   * @param acceleration  Initial acceleration (m/s^2).
   * @param velocity      Initial velocity (m/s).
   * @param position      Initial position (m).
   */
  void reset(const double acceleration, const double velocity, const double position);

  // --- Limit setters and getters ---------------------------------------------

  void setMaxJerk(const double value) { max_jerk_ = value; }
  void setMaxAcceleration(const double value) { max_acceleration_ = value; }
  void setMaxVelocity(const double value) { max_velocity_ = value; }

  double maxJerk() const { return max_jerk_; }
  double maxAcceleration() const { return max_acceleration_; }
  double maxVelocity() const { return max_velocity_; }

  // --- State getters and setters --------------------------------------------

  double currentJerk() const { return state_.jerk; }
  double currentAcceleration() const { return state_.acceleration; }
  double currentVelocity() const { return state_.velocity; }
  double currentPosition() const { return state_.position; }

  void setCurrentAcceleration(const double value) {
    state_.acceleration = initial_state_.acceleration = value;
  }
  void setCurrentVelocity(const double value) { state_.velocity = initial_state_.velocity = value; }
  void setCurrentPosition(const double value) { state_.position = initial_state_.position = value; }

  // --- Phase durations ------------------------------------------------------

  double phase1Duration() const { return t1_; }
  double phase2Duration() const { return t2_; }
  double phase3Duration() const { return t3_; }
  double totalTime() const { return t1_ + t2_ + t3_; }

  /**
   * @brief Recompute phase durations to drive the current state to @p velocity_setpoint
   *        in minimum total time.
   *
   * @param velocity_setpoint  Target velocity (m/s). Clamped to [-max_velocity, max_velocity]
   *                           only when max_velocity > 0.
   */
  void updateDurations(const double velocity_setpoint);

  /**
   * @brief Integrate the current phase by @p dt, optionally slowed by
   *        @p time_stretch in (0, 1] to wait for a lagging follower.
   *
   * Evaluation starts from the state snapshot captured by the last
   * updateDurations() call and uses an accumulated local time, so repeated
   * calls do not drift numerically.
   *
   * @param dt            Time step (s).
   * @param time_stretch  Multiplicative factor applied to dt when advancing
   *                      the local time (1.0 = nominal speed).
   */
  void updateTrajectory(const double dt, const double time_stretch = 1.0);

  /**
   * @brief Stretch all but the longest profile to a common total time, so
   *        all axes finish simultaneously.
   *
   * @param profiles  Pointers to profiles that must share the longest total time.
   */
  static void synchronize(std::vector<JerkLimitedVelocityProfile1D*>& profiles);

private:
  // Phase-duration math.
  double saturateT1ForAcceleration(const double initial_acceleration,
                                   const double max_jerk_signed,
                                   const double t1,
                                   const double max_acceleration) const;
  double computeT1(const double initial_acceleration,
                   const double delta_velocity,
                   const double max_jerk_signed,
                   const double max_acceleration) const;
  double computeT1GivenTotalTime(const double total_time,
                                 const double initial_acceleration,
                                 const double delta_velocity,
                                 const double max_jerk_signed,
                                 const double max_acceleration) const;
  double computeT2(const double t1,
                   const double t3,
                   const double initial_acceleration,
                   const double delta_velocity,
                   const double max_jerk_signed) const;
  double computeT2GivenTotalTime(const double total_time, const double t1, const double t3) const;
  double computeT3(const double t1,
                   const double initial_acceleration,
                   const double max_jerk_signed) const;

  double velocityAtZeroAcceleration() const;
  int computeDirection() const;

  void updateDurationsMinimizeTotalTime();
  void updateDurationsGivenTotalTime(const double total_time);

  State evaluatePolynomial(const double jerk,
                           const double initial_acceleration,
                           const double initial_velocity,
                           const double initial_position,
                           const double time,
                           const int direction) const;

  // Limits.
  double max_jerk_         = 0.0;
  double max_acceleration_ = 0.0;
  double max_velocity_     = 0.0;

  // Target velocity for the current phase.
  double velocity_setpoint_ = 0.0;

  // Direction of the jerk ramp (+1 / -1 / 0).
  int direction_ = 0;

  // Phase durations.
  double t1_ = 0.0;
  double t2_ = 0.0;
  double t3_ = 0.0;

  // Time elapsed since the last updateDurations() (accumulates dt * time_stretch).
  double local_time_ = 0.0;

  // Current state and snapshot captured at the last updateDurations() call.
  State state_         = {};
  State initial_state_ = {};
};

}  // namespace trajectory_generator_jerk_limited

#endif  // TRAJECTORY_GENERATOR_JERK_LIMITED__VELOCITY_PROFILE_1D_HPP_
