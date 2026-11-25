#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "letsgo_path_planner/straight_segment.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_navfn_planner/navfn_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace letsgo_path_planner
{

class StraightNavfnPlanner : public nav2_navfn_planner::NavfnPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    auto node = parent.lock();
    if (!node) {
      throw std::runtime_error("Planner lifecycle node is unavailable");
    }
    const std::string parameter = name + ".shortcut_extra_clearance";
    nav2_util::declare_parameter_if_not_declared(node, parameter, rclcpp::ParameterValue(0.08));
    node->get_parameter(parameter, extra_clearance_);
    if (!std::isfinite(extra_clearance_) || extra_clearance_ < 0.0) {
      throw std::invalid_argument(parameter + " must be a finite, nonnegative distance in meters");
    }
    NavfnPlanner::configure(parent, std::move(name), std::move(tf), costmap_ros);
    shortcut_costmap_ros_ = std::move(costmap_ros);
  }

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override
  {
    auto & costmap = *shortcut_costmap_ros_->getCostmap();
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap.getMutex());
    unsigned int start_x, start_y;
    // NavFn clears its start cell to plan an escape. Only shorten paths whose
    // start cell was already free before that edit.
    const bool free_start = worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y) &&
      costmap.getCost(start_x, start_y) == nav2_costmap_2d::FREE_SPACE;
    auto plan = NavfnPlanner::createPlan(start, goal);
    if (!free_start || plan.poses.size() < 3) {
      return plan;
    }

    const double clearance =
      shortcut_costmap_ros_->getLayeredCostmap()->getCircumscribedRadius() + extra_clearance_;
    const double sample_spacing = costmap.getResolution() / 2.0;

    const auto & poses = plan.poses;
    std::vector<bool> clear_endpoints;
    clear_endpoints.reserve(poses.size());
    for (const auto & pose : poses) {
      const auto & point = pose.pose.position;
      clear_endpoints.push_back(straight_segment_clear(costmap, point, point, clearance));
    }

    nav_msgs::msg::Path result;
    result.header = plan.header;
    result.poses.reserve(poses.size());
    result.poses.push_back(poses.front());
    for (std::size_t anchor = 0; anchor + 1 < poses.size(); ) {
      std::size_t next = anchor + 1;
      const auto & from = poses[anchor].pose.position;
      if (clear_endpoints[anchor]) {
        for (std::size_t candidate = poses.size() - 1; candidate > next; --candidate) {
          if (clear_endpoints[candidate] &&
            straight_segment_clear(costmap, from, poses[candidate].pose.position, clearance))
          {
            next = candidate;
            break;
          }
        }
      }

      if (next > anchor + 1) {
        const auto & to = poses[next].pose.position;
        const double dx = to.x - from.x;
        const double dy = to.y - from.y;
        const auto intervals = static_cast<std::size_t>(
          std::ceil(std::hypot(dx, dy) / sample_spacing));
        const double half_heading = 0.5 * std::atan2(dy, dx);
        for (std::size_t i = 1; i < intervals; ++i) {
          const double fraction = static_cast<double>(i) / intervals;
          auto pose = poses[anchor];
          pose.pose.position.x = from.x + fraction * dx;
          pose.pose.position.y = from.y + fraction * dy;
          pose.pose.position.z = from.z + fraction * (to.z - from.z);
          pose.pose.orientation.x = pose.pose.orientation.y = 0.0;
          pose.pose.orientation.z = std::sin(half_heading);
          pose.pose.orientation.w = std::cos(half_heading);
          result.poses.push_back(std::move(pose));
        }
      }
      // Keep the original anchors, in particular the exact goal and its yaw.
      result.poses.push_back(poses[next]);
      anchor = next;
    }
    return result;
  }

private:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> shortcut_costmap_ros_;
  double extra_clearance_{0.08};
};

}  // namespace letsgo_path_planner

PLUGINLIB_EXPORT_CLASS(letsgo_path_planner::StraightNavfnPlanner, nav2_core::GlobalPlanner)
