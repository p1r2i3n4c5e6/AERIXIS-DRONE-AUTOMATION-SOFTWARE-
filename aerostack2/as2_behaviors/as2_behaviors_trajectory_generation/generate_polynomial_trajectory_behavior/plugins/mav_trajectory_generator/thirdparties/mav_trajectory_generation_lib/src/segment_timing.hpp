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

#ifndef MAV_TRAJECTORY_GENERATION_CPP_SEGMENT_TIMING_HPP_
#define MAV_TRAJECTORY_GENERATION_CPP_SEGMENT_TIMING_HPP_

#include <vector>

#include "mav_trajectory_generation/vertex.h"

namespace mav_trajectory_generation_cpp {

// Internal facade-level segment-time estimation. The upstream
// estimateSegmentTimes inflates segment durations and collapses them toward
// zero for closely-spaced vertices, so feeding a dense waypoint set to a single
// global min-snap optimisation with those times overshoots badly. This helper
// instead allocates a tight constant-velocity time per segment, with an
// optional start/stop ramp. It depends on the upstream Vertex type and is NOT
// part of the public Pimpl API.

// Per-segment time allocation from caller-supplied kinematic limits. For each
// segment i (vertices[i] -> vertices[i+1]):
//   - distance d = ||p_{i+1} - p_i||
//   - limits (v, a) = (v_max_per_segment[i], a_max_per_segment[i])
//   - t = d / v, plus v / a when either endpoint pins a zero velocity
//     (start/stop ramp)
// A small floor guards exactly-coincident vertices (which would otherwise trip
// the upstream `segment_time > 0` assertion). Any speed policy (cruise vs.
// slow-down) lives in the caller: this routine just times each segment with its
// given limits, so the library carries no waypoint-tag convention.
//
// Parameters:
//   vertices            optimisation vertices (POSITION + optional VELOCITY),
//   v_max_per_segment   cruise speed per segment [m/s], each > 0
//                       (size = vertices.size() - 1),
//   a_max_per_segment   max acceleration per segment [m/s^2], each > 0
//                       (size = vertices.size() - 1).
// Returns one time [s] per segment (size = vertices.size() - 1).
std::vector<double> estimateSegmentTimesFromLimits(
    const mav_trajectory_generation::Vertex::Vector& vertices,
    const std::vector<double>& v_max_per_segment,
    const std::vector<double>& a_max_per_segment);

}  // namespace mav_trajectory_generation_cpp

#endif  // MAV_TRAJECTORY_GENERATION_CPP_SEGMENT_TIMING_HPP_
