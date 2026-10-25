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
 * @file types_gtest.cpp
 *
 * Unit tests for the common-interface POD types: Waypoint, EndWaypoint,
 * SegmentPolynomial, Spline (accessors and CSV round-trip).
 */

#include <gtest/gtest.h>

#include <cstdio>
#include <string>

#include "trajectory_generator_jerk_limited/types.hpp"

namespace tg = trajectory_generator_jerk_limited;

namespace {

std::string makeTempPath(const std::string& tag) {
  std::string path = "/tmp/tgjl_types_test_" + tag + ".spline.csv";
  std::remove(path.c_str());
  return path;
}

tg::SegmentPolynomial makeSegment(double duration, int n_poly, double seed) {
  tg::SegmentPolynomial seg;
  seg.duration     = duration;
  seg.coefficients = Eigen::MatrixXd::Zero(3, n_poly);
  for (int d = 0; d < 3; ++d) {
    for (int k = 0; k < n_poly; ++k) {
      seg.coefficients(d, k) = seed + 0.1 * d + 0.01 * k;
    }
  }
  return seg;
}

}  // namespace

TEST(SplineAccessors, EmptySplineHasZeroDuration) {
  tg::Spline sp;
  sp.start_time = 1.5;
  EXPECT_TRUE(sp.segments.empty());
  EXPECT_EQ(sp.duration(), 0.0);
  EXPECT_TRUE(sp.breakpoints().empty());
  EXPECT_TRUE(sp.segmentTimes().empty());
  EXPECT_DOUBLE_EQ(sp.minTime(), 1.5);
  EXPECT_DOUBLE_EQ(sp.maxTime(), 1.5);
}

TEST(SplineAccessors, BreakpointsAndDurationCompose) {
  tg::Spline sp;
  sp.start_time = 0.5;
  sp.segments.push_back(makeSegment(1.0, 4, 1.0));
  sp.segments.push_back(makeSegment(2.0, 4, 2.0));
  sp.segments.push_back(makeSegment(0.5, 4, 3.0));

  EXPECT_DOUBLE_EQ(sp.duration(), 3.5);
  EXPECT_DOUBLE_EQ(sp.minTime(), 0.5);
  EXPECT_DOUBLE_EQ(sp.maxTime(), 4.0);

  const auto times = sp.segmentTimes();
  ASSERT_EQ(times.size(), 3u);
  EXPECT_DOUBLE_EQ(times[0], 1.0);
  EXPECT_DOUBLE_EQ(times[1], 2.0);
  EXPECT_DOUBLE_EQ(times[2], 0.5);

  const auto knots = sp.breakpoints();
  ASSERT_EQ(knots.size(), 4u);
  EXPECT_DOUBLE_EQ(knots[0], 0.5);
  EXPECT_DOUBLE_EQ(knots[1], 1.5);
  EXPECT_DOUBLE_EQ(knots[2], 3.5);
  EXPECT_DOUBLE_EQ(knots[3], 4.0);
}

TEST(SplineCsvRoundtrip, BitExactReload) {
  tg::Spline sp;
  sp.start_time = 0.25;
  sp.segments.push_back(makeSegment(0.7, 6, 1.0));
  sp.segments.push_back(makeSegment(1.3, 6, -0.5));

  const std::string path = makeTempPath("roundtrip");
  ASSERT_TRUE(sp.saveToCsv(path));

  tg::Spline loaded;
  ASSERT_TRUE(loaded.loadFromCsv(path));
  EXPECT_DOUBLE_EQ(loaded.start_time, sp.start_time);
  ASSERT_EQ(loaded.segments.size(), sp.segments.size());
  for (std::size_t i = 0; i < sp.segments.size(); ++i) {
    EXPECT_DOUBLE_EQ(loaded.segments[i].duration, sp.segments[i].duration);
    ASSERT_EQ(loaded.segments[i].coefficients.rows(), sp.segments[i].coefficients.rows());
    ASSERT_EQ(loaded.segments[i].coefficients.cols(), sp.segments[i].coefficients.cols());
    for (int d = 0; d < sp.segments[i].coefficients.rows(); ++d) {
      for (int k = 0; k < sp.segments[i].coefficients.cols(); ++k) {
        EXPECT_DOUBLE_EQ(loaded.segments[i].coefficients(d, k), sp.segments[i].coefficients(d, k));
      }
    }
  }
  std::remove(path.c_str());
}

TEST(SplineCsvRoundtrip, VariablePolynomialOrderIsZeroPaddedOnSave) {
  tg::Spline sp;
  sp.start_time = 0.0;
  sp.segments.push_back(makeSegment(0.3, 2, 1.0));
  sp.segments.push_back(makeSegment(0.5, 4, 2.0));

  const std::string path = makeTempPath("variable_order");
  ASSERT_TRUE(sp.saveToCsv(path));

  tg::Spline loaded;
  ASSERT_TRUE(loaded.loadFromCsv(path));
  ASSERT_EQ(loaded.segments.size(), 2u);

  // Saved shape is rectangular: all segments use the max polynomial order.
  ASSERT_EQ(loaded.segments[0].coefficients.rows(), 3);
  ASSERT_EQ(loaded.segments[1].coefficients.rows(), 3);
  ASSERT_EQ(loaded.segments[0].coefficients.cols(), 4);
  ASSERT_EQ(loaded.segments[1].coefficients.cols(), 4);

  // Existing coefficients are preserved bit-exactly.
  for (int d = 0; d < 3; ++d) {
    for (int k = 0; k < 2; ++k) {
      EXPECT_DOUBLE_EQ(loaded.segments[0].coefficients(d, k), sp.segments[0].coefficients(d, k));
    }
  }
  for (int d = 0; d < 3; ++d) {
    for (int k = 0; k < 4; ++k) {
      EXPECT_DOUBLE_EQ(loaded.segments[1].coefficients(d, k), sp.segments[1].coefficients(d, k));
    }
  }

  // Missing higher-order terms from short segments are serialized as zeros.
  for (int d = 0; d < 3; ++d) {
    EXPECT_DOUBLE_EQ(loaded.segments[0].coefficients(d, 2), 0.0);
    EXPECT_DOUBLE_EQ(loaded.segments[0].coefficients(d, 3), 0.0);
  }

  std::remove(path.c_str());
}

TEST(SplineCsvRoundtrip, RejectsMissingFile) {
  tg::Spline loaded;
  loaded.start_time = 42.0;
  loaded.segments.push_back(makeSegment(1.0, 4, 0.0));
  EXPECT_FALSE(loaded.loadFromCsv("/tmp/this/path/does/not/exist.spline.csv"));
  // Must leave the target unchanged on failure.
  EXPECT_DOUBLE_EQ(loaded.start_time, 42.0);
  EXPECT_EQ(loaded.segments.size(), 1u);
}

TEST(WaypointTypes, EndWaypointPinsZeroDerivatives) {
  tg::EndWaypoint ew(Eigen::Vector3d(1.0, 2.0, 3.0));
  ASSERT_TRUE(ew.velocity.has_value());
  ASSERT_TRUE(ew.acceleration.has_value());
  EXPECT_TRUE(ew.velocity->isZero());
  EXPECT_TRUE(ew.acceleration->isZero());
  EXPECT_EQ(ew.position.x(), 1.0);
  EXPECT_EQ(ew.position.y(), 2.0);
  EXPECT_EQ(ew.position.z(), 3.0);
}

TEST(WaypointTypes, BareWaypointHasNoDerivatives) {
  tg::Waypoint w;
  EXPECT_FALSE(w.velocity.has_value());
  EXPECT_FALSE(w.acceleration.has_value());

  tg::Waypoint w_pos(Eigen::Vector3d(1.0, 0.0, 0.0));
  EXPECT_FALSE(w_pos.velocity.has_value());
  EXPECT_FALSE(w_pos.acceleration.has_value());
  EXPECT_DOUBLE_EQ(w_pos.position.x(), 1.0);
}
