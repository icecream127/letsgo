#include <limits>

#include "gtest/gtest.h"
#include "letsgo_path_planner/straight_segment.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace letsgo_path_planner
{
namespace
{

using nav2_costmap_2d::Costmap2D;
using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

geometry_msgs::msg::Point point(double x, double y)
{
  geometry_msgs::msg::Point result;
  result.x = x;
  result.y = y;
  return result;
}

TEST(StraightSegment, ClearDiagonalAndZeroLength)
{
  const Costmap2D costmap(20, 20, 0.05, -0.2, -0.4, FREE_SPACE);
  EXPECT_TRUE(straight_segment_clear(costmap, point(0.1, -0.1), point(0.5, 0.2), 0.25));
  EXPECT_TRUE(straight_segment_clear(costmap, point(0.1, -0.1), point(0.1, -0.1), 0.25));
}

TEST(StraightSegment, RejectsOccupiedCornerAndEdgeContacts)
{
  Costmap2D costmap(12, 12, 1.0, 0.0, 0.0, FREE_SPACE);
  costmap.setCost(1, 2, 40);
  EXPECT_FALSE(straight_segment_clear(costmap, point(1.5, 1.5), point(4.5, 4.5), 0.0));
  EXPECT_FALSE(straight_segment_clear(costmap, point(4.5, 4.5), point(1.5, 1.5), 0.0));
  EXPECT_TRUE(straight_segment_clear(costmap, point(1.5, 1.49), point(4.5, 4.49), 0.0));
  costmap.setCost(4, 3, 40);
  EXPECT_FALSE(straight_segment_clear(costmap, point(2.5, 4.0), point(6.5, 4.0), 0.0));
}

TEST(StraightSegment, RejectsInflatedCostsWhenCrossed)
{
  Costmap2D costmap(12, 12, 1.0, 0.0, 0.0, FREE_SPACE);
  costmap.setCost(5, 5, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_FALSE(straight_segment_clear(costmap, point(2.5, 5.5), point(8.5, 5.5), 0.0));
  EXPECT_TRUE(straight_segment_clear(costmap, point(2.5, 4.9), point(8.5, 4.9), 0.25));
}

TEST(StraightSegment, ChecksContinuousClearanceFromWall)
{
  Costmap2D costmap(12, 12, 1.0, 0.0, 0.0, FREE_SPACE);
  costmap.setCost(5, 5, LETHAL_OBSTACLE);
  EXPECT_FALSE(straight_segment_clear(costmap, point(2.5, 4.6), point(8.5, 4.6), 0.41));
  EXPECT_FALSE(straight_segment_clear(costmap, point(2.5, 4.6), point(8.5, 4.6), 0.40));
  EXPECT_TRUE(straight_segment_clear(costmap, point(2.5, 4.6), point(8.5, 4.6), 0.39));
  EXPECT_TRUE(straight_segment_clear(costmap, point(3.65, 3.65), point(4.4, 4.4), 0.80));
  EXPECT_FALSE(straight_segment_clear(costmap, point(3.65, 3.65), point(4.4, 4.4), 0.85));
}

TEST(StraightSegment, ChecksClearanceFromUnknownSpaceAndMapBoundary)
{
  Costmap2D costmap(12, 12, 1.0, 0.0, 0.0, FREE_SPACE);
  costmap.setCost(5, 2, NO_INFORMATION);
  EXPECT_FALSE(straight_segment_clear(costmap, point(2.5, 1.6), point(8.5, 1.6), 0.40));
  EXPECT_TRUE(straight_segment_clear(costmap, point(2.5, 1.6), point(8.5, 1.6), 0.39));
  EXPECT_FALSE(straight_segment_clear(costmap, point(0.5, 4.5), point(2.5, 4.5), 0.5));
  EXPECT_TRUE(straight_segment_clear(costmap, point(0.51, 4.5), point(2.5, 4.5), 0.5));
  EXPECT_FALSE(straight_segment_clear(costmap, point(0.0, 4.5), point(2.5, 4.5), 0.0));
  EXPECT_FALSE(straight_segment_clear(costmap, point(11.6, 4.5), point(2.5, 4.5), 0.5));
}

TEST(StraightSegment, RejectsInvalidInputs)
{
  const Costmap2D costmap(12, 12, 1.0, 0.0, 0.0, FREE_SPACE);
  const auto start = point(2.5, 2.5);
  const auto end = point(7.5, 7.5);
  const auto invalid = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(straight_segment_clear(costmap, start, end, -0.1));
  EXPECT_FALSE(straight_segment_clear(costmap, start, end, invalid));
  EXPECT_FALSE(straight_segment_clear(costmap, start, point(invalid, 7.5), 0.2));
}

}  // namespace
}  // namespace letsgo_path_planner
