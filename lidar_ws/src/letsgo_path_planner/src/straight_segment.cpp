#include "letsgo_path_planner/straight_segment.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "nav2_costmap_2d/cost_values.hpp"

namespace letsgo_path_planner
{
namespace
{

struct Point
{
  double x;
  double y;
};

struct Box
{
  double min_x;
  double min_y;
  double max_x;
  double max_y;
};

bool clip_axis(
  double from, double to, double lower, double upper, double & enter, double & leave)
{
  const double direction = to - from;
  if (direction == 0.0) {
    return from >= lower && from <= upper;
  }
  const double first = (lower - from) / direction;
  const double last = (upper - from) / direction;
  enter = std::max(enter, std::min(first, last));
  leave = std::min(leave, std::max(first, last));
  return enter <= leave;
}

double squared_distance_to_box(const Point & point, const Box & box)
{
  const double dx = std::max({box.min_x - point.x, 0.0, point.x - box.max_x});
  const double dy = std::max({box.min_y - point.y, 0.0, point.y - box.max_y});
  return dx * dx + dy * dy;
}

double squared_distance_to_segment(const Point & point, const Point & from, const Point & to)
{
  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  const double length_squared = dx * dx + dy * dy;
  double position = 0.0;
  if (length_squared > 0.0) {
    const double projection = ((point.x - from.x) * dx + (point.y - from.y) * dy) /
      length_squared;
    position = std::max(0.0, std::min(1.0, projection));
  }
  const double offset_x = point.x - (from.x + position * dx);
  const double offset_y = point.y - (from.y + position * dy);
  return offset_x * offset_x + offset_y * offset_y;
}

double squared_segment_clearance(const Point & from, const Point & to, const Box & box)
{
  double enter = 0.0;
  double leave = 1.0;
  if (clip_axis(from.x, to.x, box.min_x, box.max_x, enter, leave) &&
    clip_axis(from.y, to.y, box.min_y, box.max_y, enter, leave))
  {
    return 0.0;
  }
  // For a disjoint segment and rectangle, a closest pair includes a segment
  // endpoint or a rectangle corner.
  return std::min({
    squared_distance_to_box(from, box),
    squared_distance_to_box(to, box),
    squared_distance_to_segment({box.min_x, box.min_y}, from, to),
    squared_distance_to_segment({box.min_x, box.max_y}, from, to),
    squared_distance_to_segment({box.max_x, box.min_y}, from, to),
    squared_distance_to_segment({box.max_x, box.max_y}, from, to),
  });
}

}  // namespace

bool straight_segment_clear(
  const nav2_costmap_2d::Costmap2D & costmap,
  const geometry_msgs::msg::Point & start,
  const geometry_msgs::msg::Point & goal,
  double clearance)
{
  const double resolution = costmap.getResolution();
  const unsigned int width = costmap.getSizeInCellsX();
  const unsigned int height = costmap.getSizeInCellsY();
  if (!std::isfinite(clearance) || clearance < 0.0 ||
    !std::isfinite(resolution) || resolution <= 0.0 || width == 0 || height == 0)
  {
    return false;
  }

  const double origin_x = costmap.getOriginX();
  const double origin_y = costmap.getOriginY();
  const Point from{(start.x - origin_x) / resolution, (start.y - origin_y) / resolution};
  const Point to{(goal.x - origin_x) / resolution, (goal.y - origin_y) / resolution};
  if (!std::isfinite(from.x) || !std::isfinite(from.y) ||
    !std::isfinite(to.x) || !std::isfinite(to.y))
  {
    return false;
  }

  // Use grid coordinates for the geometry and include contacts at an exact
  // cell edge even when world-to-grid conversion rounds slightly away from it.
  const double scale = std::max({
    1.0, static_cast<double>(width), static_cast<double>(height),
    std::abs(origin_x / resolution), std::abs(origin_y / resolution),
  });
  const double roundoff = 32.0 * std::numeric_limits<double>::epsilon() * scale;
  const double margin = clearance / resolution + roundoff;
  const Box bounds{
    std::min(from.x, to.x) - margin, std::min(from.y, to.y) - margin,
    std::max(from.x, to.x) + margin, std::max(from.y, to.y) + margin,
  };
  if (bounds.min_x <= 0.0 || bounds.min_y <= 0.0 ||
    bounds.max_x >= width || bounds.max_y >= height)
  {
    return false;
  }

  const auto first_x = static_cast<unsigned int>(bounds.min_x);
  const auto last_x = static_cast<unsigned int>(bounds.max_x);
  const auto first_y = static_cast<unsigned int>(bounds.min_y);
  const auto last_y = static_cast<unsigned int>(bounds.max_y);
  for (unsigned int y = first_y; y <= last_y; ++y) {
    for (unsigned int x = first_x; x <= last_x; ++x) {
      const auto cost = costmap.getCost(x, y);
      if (cost == nav2_costmap_2d::FREE_SPACE) {
        continue;
      }
      const Box cell{
        static_cast<double>(x), static_cast<double>(y),
        static_cast<double>(x) + 1.0, static_cast<double>(y) + 1.0,
      };
      const double required =
        (cost == nav2_costmap_2d::LETHAL_OBSTACLE || cost == nav2_costmap_2d::NO_INFORMATION) ?
        margin : roundoff;
      if (squared_segment_clearance(from, to, cell) <= required * required) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace letsgo_path_planner
