#ifndef LETSGO_PATH_PLANNER__STRAIGHT_SEGMENT_HPP_
#define LETSGO_PATH_PLANNER__STRAIGHT_SEGMENT_HPP_

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace letsgo_path_planner
{

// Check the entire XY segment and its clearance in meters. The caller must hold
// the costmap mutex if the map can be updated concurrently.
bool straight_segment_clear(
  const nav2_costmap_2d::Costmap2D & costmap,
  const geometry_msgs::msg::Point & start,
  const geometry_msgs::msg::Point & goal,
  double clearance);

}  // namespace letsgo_path_planner

#endif  // LETSGO_PATH_PLANNER__STRAIGHT_SEGMENT_HPP_
