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
 * @file corridor_builder.hpp
 * @brief Axis-aligned bounding-box safe flight corridor generator.
 *
 * GCOPTER's setup() requires a Safe Flight Corridor (SFC) expressed as a
 * vector of H-polytopes. In the absence of obstacles we build one polytope
 * per waypoint segment: an axis-aligned bounding box (AABB) that contains the
 * two endpoints plus a user-specified margin on every side. Consecutive AABBs
 * overlap around each intermediate waypoint, which is sufficient for GCOPTER
 * to stitch adjacent pieces into a continuous trajectory.
 */

#ifndef GCOPTER_LIB__CORRIDOR_BUILDER_HPP_
#define GCOPTER_LIB__CORRIDOR_BUILDER_HPP_

#include <Eigen/Eigen>
#include <vector>

namespace gcopter_lib {

/// Polytope in H-representation, matching GCOPTER's expected type
/// `std::vector<Eigen::MatrixX4d>`. Each row is [h0 h1 h2 h3] with
/// h0*x + h1*y + h2*z + h3 <= 0.
using PolyhedraH = std::vector<Eigen::MatrixX4d>;

/**
 * @brief Build an SFC as a chain of AABB polytopes, one per waypoint segment.
 *
 * @param waypoints Ordered positions in a fixed world frame [m]. Size >= 2.
 * @param margin    Half-margin added to each face of every AABB [m]. Must be
 *                  strictly positive; the overlap between consecutive AABBs
 *                  is guaranteed only when margin > 0.
 * @return Vector with `waypoints.size() - 1` polytopes, each a 6x4 matrix.
 * @throws std::invalid_argument if preconditions are violated.
 */
PolyhedraH buildAabbCorridor(const std::vector<Eigen::Vector3d>& waypoints, double margin);

/**
 * @brief Build an SFC as a chain of AABB polytopes with a per-segment margin.
 *
 * Allows tightening the corridor around certain waypoints (e.g. anchor points
 * surrounding a target waypoint) while keeping the rest of the path loose.
 *
 * @param waypoints Ordered positions in a fixed world frame [m]. Size >= 2.
 * @param margins   Half-margin per segment [m]. Size must equal
 *                  `waypoints.size() - 1`. Every entry must be strictly
 *                  positive.
 * @return Vector with `waypoints.size() - 1` polytopes, each a 6x4 matrix.
 * @throws std::invalid_argument if preconditions are violated.
 */
PolyhedraH buildAabbCorridor(const std::vector<Eigen::Vector3d>& waypoints,
                             const std::vector<double>& margins);

}  // namespace gcopter_lib

#endif  // GCOPTER_LIB__CORRIDOR_BUILDER_HPP_
