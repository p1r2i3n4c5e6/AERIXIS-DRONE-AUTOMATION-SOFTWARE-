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
 * @file velocity_profile_1d.cpp
 *
 * Per-axis jerk-limited S-curve velocity profile implementation.
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#include "trajectory_generator_jerk_limited/velocity_profile_1d.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

#include "trajectory_generator_jerk_limited/trajectory_math.hpp"

namespace trajectory_generator_jerk_limited {

JerkLimitedVelocityProfile1D::JerkLimitedVelocityProfile1D(const double initial_acceleration,
                                                           const double initial_velocity,
                                                           const double initial_position) {
  reset(initial_acceleration, initial_velocity, initial_position);
}

void JerkLimitedVelocityProfile1D::reset(const double acceleration,
                                         const double velocity,
                                         const double position) {
  state_.jerk         = 0.0;
  state_.acceleration = acceleration;
  state_.velocity     = velocity;
  state_.position     = position;
  initial_state_      = state_;
  local_time_         = 0.0;
  t1_                 = 0.0;
  t2_                 = 0.0;
  t3_                 = 0.0;
  direction_          = 0;
  velocity_setpoint_  = 0.0;
}

double JerkLimitedVelocityProfile1D::saturateT1ForAcceleration(
    const double initial_acceleration,
    const double max_jerk_signed,
    const double t1,
    const double max_acceleration) const {
  const double acceleration_at_t1 = initial_acceleration + max_jerk_signed * t1;
  if (acceleration_at_t1 > max_acceleration) {
    return (max_acceleration - initial_acceleration) / max_jerk_signed;
  }
  if (acceleration_at_t1 < -max_acceleration) {
    return (-max_acceleration - initial_acceleration) / max_jerk_signed;
  }
  return t1;
}

double JerkLimitedVelocityProfile1D::computeT1(const double initial_acceleration,
                                               const double delta_velocity,
                                               const double max_jerk_signed,
                                               const double max_acceleration) const {
  const double delta =
      2.0 * initial_acceleration * initial_acceleration + 4.0 * max_jerk_signed * delta_velocity;
  if (delta < 0.0) {
    return 0.0;
  }
  const double sqrt_delta = std::sqrt(delta);
  const double t1_plus    = (-initial_acceleration + 0.5 * sqrt_delta) / max_jerk_signed;
  const double t1_minus   = (-initial_acceleration - 0.5 * sqrt_delta) / max_jerk_signed;
  const double t3_plus    = initial_acceleration / max_jerk_signed + t1_plus;
  const double t3_minus   = initial_acceleration / max_jerk_signed + t1_minus;

  double t1 = 0.0;
  if (t1_plus >= 0.0 && t3_plus >= 0.0) {
    t1 = t1_plus;
  } else if (t1_minus >= 0.0 && t3_minus >= 0.0) {
    t1 = t1_minus;
  }
  t1 = saturateT1ForAcceleration(initial_acceleration, max_jerk_signed, t1, max_acceleration);
  return std::max(t1, 0.0);
}

double JerkLimitedVelocityProfile1D::computeT1GivenTotalTime(const double total_time,
                                                             const double initial_acceleration,
                                                             const double delta_velocity,
                                                             const double max_jerk_signed,
                                                             const double max_acceleration) const {
  const double a     = -max_jerk_signed;
  const double b     = max_jerk_signed * total_time - initial_acceleration;
  const double delta = total_time * total_time * max_jerk_signed * max_jerk_signed +
                       2.0 * total_time * initial_acceleration * max_jerk_signed -
                       initial_acceleration * initial_acceleration -
                       4.0 * max_jerk_signed * delta_velocity;
  if (delta < 0.0) {
    return 0.0;
  }
  const double sqrt_delta = std::sqrt(delta);
  const double inv_2a     = 1.0 / (2.0 * a);
  const double t1_plus    = std::max((-b + sqrt_delta) * inv_2a, 0.0);
  const double t1_minus   = std::max((-b - sqrt_delta) * inv_2a, 0.0);
  const double t3_plus    = initial_acceleration / max_jerk_signed + t1_plus;
  const double t3_minus   = initial_acceleration / max_jerk_signed + t1_minus;

  double t1 = 0.0;
  if (t1_plus >= 0.0 && t3_plus >= 0.0 && (t1_plus + t3_plus) <= total_time) {
    t1 = t1_plus;
  } else if (t1_minus >= 0.0 && t3_minus >= 0.0 && (t1_minus + t3_minus) <= total_time) {
    t1 = t1_minus;
  }
  t1 = saturateT1ForAcceleration(initial_acceleration, max_jerk_signed, t1, max_acceleration);
  return t1;
}

double JerkLimitedVelocityProfile1D::computeT2(const double t1,
                                               const double t3,
                                               const double initial_acceleration,
                                               const double delta_velocity,
                                               const double max_jerk_signed) const {
  const double denominator = initial_acceleration + max_jerk_signed * t1;
  double t2                = 0.0;
  if (std::fabs(denominator) > kEpsilon) {
    t2 = (-0.5 * t1 * t1 * max_jerk_signed - t1 * t3 * max_jerk_signed - t1 * initial_acceleration +
          0.5 * t3 * t3 * max_jerk_signed - t3 * initial_acceleration + delta_velocity) /
         denominator;
  }
  return std::max(t2, 0.0);
}

double JerkLimitedVelocityProfile1D::computeT2GivenTotalTime(const double total_time,
                                                             const double t1,
                                                             const double t3) const {
  return std::max(total_time - t1 - t3, 0.0);
}

double JerkLimitedVelocityProfile1D::computeT3(const double t1,
                                               const double initial_acceleration,
                                               const double max_jerk_signed) const {
  return std::max(initial_acceleration / max_jerk_signed + t1, 0.0);
}

double JerkLimitedVelocityProfile1D::velocityAtZeroAcceleration() const {
  if (std::fabs(max_jerk_) <= kEpsilon) {
    return state_.velocity;
  }
  if (std::fabs(state_.acceleration) <= kEpsilon) {
    return state_.velocity;
  }
  const double jerk_to_zero_acceleration = -signInt(state_.acceleration) * max_jerk_;
  const double time_to_zero_acceleration = -state_.acceleration / jerk_to_zero_acceleration;
  return state_.velocity + state_.acceleration * time_to_zero_acceleration +
         0.5 * jerk_to_zero_acceleration * time_to_zero_acceleration * time_to_zero_acceleration;
}

int JerkLimitedVelocityProfile1D::computeDirection() const {
  const double velocity_zero_acceleration = velocityAtZeroAcceleration();
  int direction = signInt(velocity_setpoint_ - velocity_zero_acceleration);
  if (direction == 0) {
    direction = signInt(state_.acceleration);
  }
  return direction;
}

void JerkLimitedVelocityProfile1D::updateDurationsMinimizeTotalTime() {
  const double max_jerk_signed = direction_ * max_jerk_;
  const double delta_velocity  = velocity_setpoint_ - state_.velocity;
  if (std::fabs(max_jerk_signed) > kEpsilon) {
    const double effective_max_acceleration = (max_acceleration_ > kEpsilon)
                                                  ? max_acceleration_
                                                  : std::numeric_limits<double>::infinity();
    t1_ =
        computeT1(state_.acceleration, delta_velocity, max_jerk_signed, effective_max_acceleration);
    t3_ = computeT3(t1_, state_.acceleration, max_jerk_signed);
    t2_ = computeT2(t1_, t3_, state_.acceleration, delta_velocity, max_jerk_signed);
  } else {
    t1_ = t2_ = t3_ = 0.0;
  }
}

void JerkLimitedVelocityProfile1D::updateDurationsGivenTotalTime(const double total_time) {
  const double max_jerk_signed = direction_ * max_jerk_;
  const double delta_velocity  = velocity_setpoint_ - state_.velocity;
  if (std::fabs(max_jerk_signed) > kEpsilon) {
    const double effective_max_acceleration = (max_acceleration_ > kEpsilon)
                                                  ? max_acceleration_
                                                  : std::numeric_limits<double>::infinity();
    t1_ = computeT1GivenTotalTime(total_time, state_.acceleration, delta_velocity, max_jerk_signed,
                                  effective_max_acceleration);
    t3_ = computeT3(t1_, state_.acceleration, max_jerk_signed);
    t2_ = computeT2GivenTotalTime(total_time, t1_, t3_);
  } else {
    t1_ = t2_ = t3_ = 0.0;
  }
}

void JerkLimitedVelocityProfile1D::updateDurations(const double velocity_setpoint) {
  if (max_velocity_ > kEpsilon) {
    velocity_setpoint_ = std::clamp(velocity_setpoint, -max_velocity_, max_velocity_);
  } else {
    velocity_setpoint_ = velocity_setpoint;
  }
  local_time_    = 0.0;
  initial_state_ = state_;
  direction_     = computeDirection();
  updateDurationsMinimizeTotalTime();
}

JerkLimitedVelocityProfile1D::State JerkLimitedVelocityProfile1D::evaluatePolynomial(
    const double jerk,
    const double initial_acceleration,
    const double initial_velocity,
    const double initial_position,
    const double time,
    const int direction) const {
  const double jerk_signed = direction * jerk;
  const double t_squared   = time * time;
  const double t_cubed     = t_squared * time;
  State result;
  result.jerk         = jerk_signed;
  result.acceleration = initial_acceleration + jerk_signed * time;
  result.velocity = initial_velocity + initial_acceleration * time + 0.5 * jerk_signed * t_squared;
  result.position = initial_position + initial_velocity * time +
                    0.5 * initial_acceleration * t_squared + (1.0 / 6.0) * jerk_signed * t_cubed;
  return result;
}

void JerkLimitedVelocityProfile1D::updateTrajectory(const double dt, const double time_stretch) {
  const double scaled_dt = std::max(0.0, dt * time_stretch);
  if (max_jerk_ <= kEpsilon) {
    local_time_ += scaled_dt;

    const double velocity_error = velocity_setpoint_ - state_.velocity;
    double delta_velocity       = velocity_error;
    if (max_acceleration_ > kEpsilon && scaled_dt > kEpsilon) {
      const double max_delta_velocity = max_acceleration_ * scaled_dt;
      delta_velocity = std::clamp(velocity_error, -max_delta_velocity, max_delta_velocity);
    }
    const double next_velocity = state_.velocity + delta_velocity;

    state_.jerk = 0.0;
    if (max_acceleration_ > kEpsilon && scaled_dt > kEpsilon) {
      state_.acceleration = delta_velocity / scaled_dt;
    } else {
      state_.acceleration = 0.0;
    }
    state_.velocity = next_velocity;
    state_.position += state_.velocity * scaled_dt;
    return;
  }

  local_time_ += dt * time_stretch;
  double time_remaining = local_time_;

  // Phase 1: jerk ramps acceleration in the commanded direction.
  const double phase1_time = std::min(time_remaining, t1_);
  state_ = evaluatePolynomial(max_jerk_, initial_state_.acceleration, initial_state_.velocity,
                              initial_state_.position, phase1_time, direction_);
  time_remaining -= phase1_time;

  // Phase 2: constant acceleration, zero jerk.
  if (time_remaining > 0.0) {
    const double phase2_time = std::min(time_remaining, t2_);
    state_ = evaluatePolynomial(0.0, state_.acceleration, state_.velocity, state_.position,
                                phase2_time, 0);
    time_remaining -= phase2_time;
  }

  // Phase 3: jerk brings acceleration back to zero.
  if (time_remaining > 0.0) {
    const double phase3_time = std::min(time_remaining, t3_);
    state_ = evaluatePolynomial(max_jerk_, state_.acceleration, state_.velocity, state_.position,
                                phase3_time, -direction_);
    time_remaining -= phase3_time;
  }

  // Post-trajectory: constant velocity, acceleration and jerk both zero.
  if (time_remaining > 0.0) {
    state_ = evaluatePolynomial(0.0, 0.0, state_.velocity, state_.position, time_remaining, 0);
  }
}

void JerkLimitedVelocityProfile1D::synchronize(
    std::vector<JerkLimitedVelocityProfile1D*>& profiles) {
  double target_time        = 0.0;
  std::size_t longest_index = 0;
  for (std::size_t i = 0; i < profiles.size(); ++i) {
    const double total_time = profiles[i]->totalTime();
    if (total_time > target_time) {
      target_time   = total_time;
      longest_index = i;
    }
  }
  if (target_time > kEpsilon) {
    for (std::size_t i = 0; i < profiles.size(); ++i) {
      if (i != longest_index && profiles[i]->totalTime() < target_time) {
        profiles[i]->updateDurationsGivenTotalTime(target_time);
      }
    }
  }
}

}  // namespace trajectory_generator_jerk_limited
