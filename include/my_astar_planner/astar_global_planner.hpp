/*
 * A* Global Planner Plugin for Nav2
 * Implements nav2_core::GlobalPlanner interface.
 */

#ifndef MY_ASTAR_PLANNER__ASTAR_GLOBAL_PLANNER_HPP_
#define MY_ASTAR_PLANNER__ASTAR_GLOBAL_PLANNER_HPP_

#include <algorithm>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_astar_planner {

/**
 * Node structure for A* algorithm
 *
 * f(n) = g(n) + h(n)
 * - g_cost: Cost from start to this node
 * - h_cost: Estimated cost from this node to goal (heuristic)
 * - f_cost: Total cost
 */
struct AStarNode {
  int x, y;                          // Grid coordinates
  double g_cost, h_cost, f_cost;     // Costs
  std::shared_ptr<AStarNode> parent; // Parent for path reconstruction

  AStarNode(int _x, int _y, double _g, double _h,
            std::shared_ptr<AStarNode> _parent = nullptr)
      : x(_x), y(_y), g_cost(_g), h_cost(_h), parent(_parent) {
    f_cost = g_cost + h_cost;
  }
};

/**
 * Comparator for priority queue
 * Orders nodes by lowest f_cost
 */
struct CompareNode {
  bool operator()(const std::shared_ptr<AStarNode> &a,
                  const std::shared_ptr<AStarNode> &b) {
    return a->f_cost > b->f_cost;
  }
};

/**
 * Nav2 Global Planner Plugin
 */
class AStarGlobalPlanner : public nav2_core::GlobalPlanner {
public:
  AStarGlobalPlanner() = default;
  ~AStarGlobalPlanner() override = default;

  /**
   * Plugin lifecycle methods
   */
  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  /**
   * Main path planning method
   */
  nav_msgs::msg::Path
  createPlan(const geometry_msgs::msg::PoseStamped &start,
             const geometry_msgs::msg::PoseStamped &goal) override;

private:
  // Nav2 components
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_costmap_2d::Costmap2D *costmap_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("AStarGlobalPlanner")};
  rclcpp::Clock::SharedPtr clock_;

  std::string global_frame_;

  // A* Algorithm Methods
  std::vector<std::pair<int, int>> runAStar(int start_x, int start_y,
                                            int goal_x, int goal_y);
  double heuristic(int x1, int y1, int x2, int y2);
  bool isValid(int x, int y);
  bool isBlocked(unsigned int x, unsigned int y);
};

} // namespace my_astar_planner

#endif // MY_ASTAR_PLANNER__ASTAR_GLOBAL_PLANNER_HPP_
