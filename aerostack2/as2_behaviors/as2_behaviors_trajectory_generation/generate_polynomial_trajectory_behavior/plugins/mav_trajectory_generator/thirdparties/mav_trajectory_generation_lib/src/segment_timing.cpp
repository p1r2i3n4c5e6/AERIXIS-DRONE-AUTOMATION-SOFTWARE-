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

#include "segment_timing.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "mav_trajectory_generation/motion_defines.h"

namespace mav_trajectory_generation_cpp {

namespace {
// Floor guarding exactly-coincident vertices (d == 0 -> t == 0 would trip the
// upstream `segment_time > 0` assertion). Negligible vs. real segment times.
constexpr double kMinSegmentTime = 1e-3;  // [s]
// A pinned velocity below this magnitude counts as "at rest".
constexpr double kZeroVelocityTol = 1e-6;  // [m/s]
// Lower bound so a degenerate (zero) limit never divides by zero.
constexpr double kMinLimit = 1e-6;
}  // namespace

std::vector<double> estimateSegmentTimesFromLimits(
    const mav_trajectory_generation::Vertex::Vector& vertices,
    const std::vector<double>& v_max_per_segment,
    const std::vector<double>& a_max_per_segment) {
  using mav_trajectory_generation::derivative_order::POSITION;
  using mav_trajectory_generation::derivative_order::VELOCITY;

  std::vector<double> segment_times;
  if (vertices.size() < 2) {
    return segment_times;
  }
  segment_times.reserve(vertices.size() - 1);

  for (std::size_t i = 0; i + 1 < vertices.size(); ++i) {
    Eigen::VectorXd start, end;
    vertices[i].getConstraint(POSITION, &start);
    vertices[i + 1].getConstraint(POSITION, &end);
    const double distance = (end - start).norm();

    // Per-segment limits supplied by the caller (any slow-down policy lives
    // there); segment i is timed with the i-th limit (sizes = vertices - 1).
    const double v = std::max(kMinLimit, v_max_per_segment[i]);
    const double a = std::max(kMinLimit, a_max_per_segment[i]);

    // Constant-velocity time plus a start/stop ramp (v/a) whenever either
    // endpoint pins zero velocity; the cruise time is not inflated.
    Eigen::VectorXd vel_start, vel_end;
    const bool has_vel_start = vertices[i].getConstraint(VELOCITY, &vel_start);
    const bool has_vel_end = vertices[i + 1].getConstraint(VELOCITY, &vel_end);

    double t;
    if ((has_vel_start && vel_start.norm() < kZeroVelocityTol) ||
        (has_vel_end && vel_end.norm() < kZeroVelocityTol)) {
      t = distance / v + v / a;
    } else {
      t = distance / v;
    }

    segment_times.push_back(std::max(t, kMinSegmentTime));
  }

  return segment_times;
}

}  // namespace mav_trajectory_generation_cpp
