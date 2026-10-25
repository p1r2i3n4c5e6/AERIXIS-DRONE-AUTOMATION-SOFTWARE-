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

#include "gcopter_lib/corridor_builder.hpp"

#include <algorithm>
#include <stdexcept>

namespace gcopter_lib {

namespace {

// Minimum half-extent applied to each AABB axis to guarantee the polytope is
// strictly non-degenerate. GCOPTER's L-BFGS optimisation occasionally fails to
// converge (cost = INF) when consecutive waypoints share one or more
// coordinates, because the resulting AABB has zero extent on those axes and
// the inflated corridor becomes a thin slab. The epsilon is small enough not
// to relax the corridor in well-formed cases.
constexpr double kAabbAxisEpsilon = 1.0e-3;  // [m]

// Build a single 6x4 H-polytope for an AABB defined by [min, max].
// Row layout matches the convention used by GCOPTER: h0*x + h1*y + h2*z + h3 <= 0.
Eigen::MatrixX4d aabbToHalfSpaces(const Eigen::Vector3d& lower, const Eigen::Vector3d& upper) {
  Eigen::MatrixX4d polytope(6, 4);
  polytope << -1.0, 0.0, 0.0, lower.x(),  //  -x + x_min <= 0  =>  x >= x_min
      1.0, 0.0, 0.0, -upper.x(),          //   x - x_max <= 0  =>  x <= x_max
      0.0, -1.0, 0.0, lower.y(), 0.0, 1.0, 0.0, -upper.y(), 0.0, 0.0, -1.0, lower.z(), 0.0, 0.0,
      1.0, -upper.z();
  return polytope;
}

}  // namespace

PolyhedraH buildAabbCorridor(const std::vector<Eigen::Vector3d>& waypoints, double margin) {
  if (waypoints.size() < 2) {
    throw std::invalid_argument("buildAabbCorridor: need at least two waypoints");
  }
  if (!(margin > 0.0)) {
    throw std::invalid_argument("buildAabbCorridor: margin must be strictly positive");
  }

  const std::vector<double> margins(waypoints.size() - 1, margin);
  return buildAabbCorridor(waypoints, margins);
}

PolyhedraH buildAabbCorridor(const std::vector<Eigen::Vector3d>& waypoints,
                             const std::vector<double>& margins) {
  if (waypoints.size() < 2) {
    throw std::invalid_argument("buildAabbCorridor: need at least two waypoints");
  }
  if (margins.size() != waypoints.size() - 1) {
    throw std::invalid_argument(
        "buildAabbCorridor: margins.size() must equal waypoints.size() - 1");
  }

  PolyhedraH corridor;
  corridor.reserve(waypoints.size() - 1);

  for (std::size_t i = 0; i + 1 < waypoints.size(); ++i) {
    if (!(margins[i] > 0.0)) {
      throw std::invalid_argument("buildAabbCorridor: every margin must be strictly positive");
    }
    const Eigen::Vector3d margin_vec = Eigen::Vector3d::Constant(margins[i]);
    const Eigen::Vector3d& a         = waypoints[i];
    const Eigen::Vector3d& b         = waypoints[i + 1];
    Eigen::Vector3d lower            = a.cwiseMin(b) - margin_vec;
    Eigen::Vector3d upper            = a.cwiseMax(b) + margin_vec;
    // Ensure each AABB axis has strictly positive extent. When consecutive
    // waypoints share a coordinate (e.g. purely vertical motion), the AABB is
    // a thin slab on the affected axis and GCOPTER's solver may diverge.
    // Symmetric inflation by a tiny epsilon keeps the polytope shape but
    // prevents the degenerate geometry without affecting non-degenerate cases.
    for (int k = 0; k < 3; ++k) {
      const double extent = upper[k] - lower[k];
      if (extent < 2.0 * kAabbAxisEpsilon) {
        const double pad = kAabbAxisEpsilon - 0.5 * extent;
        lower[k] -= pad;
        upper[k] += pad;
      }
    }
    corridor.emplace_back(aabbToHalfSpaces(lower, upper));
  }

  return corridor;
}

}  // namespace gcopter_lib
