// Hover while moving (2026-11-01): the hover point lies ahead along the velocity so the drone
// brakes smoothly (a search drone at 2 m/s told to hover where it was aborted AIRCRAFT_UNSTABLE).
#include <gtest/gtest.h>

#include <cmath>

#include "as2_motion_controller/controller_handler.hpp"

using controller_handler::hoverStopOffset;

TEST(HoverStop, OffsetIsOneSecondOfTravel)
{
  const auto [dx, dy] = hoverStopOffset(2.0, 0.0, 1.0, 5.0);
  EXPECT_DOUBLE_EQ(dx, 2.0);
  EXPECT_DOUBLE_EQ(dy, 0.0);
  const auto [ex, ey] = hoverStopOffset(-1.2, 1.6, 1.0, 5.0);
  EXPECT_DOUBLE_EQ(ex, -1.2);
  EXPECT_DOUBLE_EQ(ey, 1.6);
}

TEST(HoverStop, CappedKeepingTheDirection)
{
  const auto [dx, dy] = hoverStopOffset(6.0, 8.0, 1.0, 5.0);
  EXPECT_NEAR(std::hypot(dx, dy), 5.0, 1e-9);
  EXPECT_NEAR(dx / dy, 0.75, 1e-9);
}

TEST(HoverStop, NothingWhenStillDisabledOrInvalid)
{
  EXPECT_EQ(hoverStopOffset(0.0, 0.0, 1.0, 5.0), std::make_pair(0.0, 0.0));
  EXPECT_EQ(hoverStopOffset(2.0, 0.0, 0.0, 5.0), std::make_pair(0.0, 0.0));
  EXPECT_EQ(hoverStopOffset(NAN, 0.0, 1.0, 5.0), std::make_pair(0.0, 0.0));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
