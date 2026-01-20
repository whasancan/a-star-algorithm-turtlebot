/*
 * A* Global Planner Plugin Implementation
 *
 * Bu dosya astar_global_planner.hpp'de tanımlanan class'ın implementasyonunu
 * içerir.
 *
 * ÖĞRENME NOTLARI:
 * - costmap_->getCost(x, y): Bir hücrenin maliyet değerini döndürür
 * - LETHAL_OBSTACLE (254): Kesinlikle engel
 * - INSCRIBED_INFLATED_OBSTACLE (253): Robot sığmaz
 * - FREE_SPACE (0): Serbest alan
 */

#include "my_astar_planner/astar_global_planner.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace my_astar_planner {

void AStarGlobalPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_ = parent;
  auto node = parent.lock();

  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap(); // Ham costmap'i al
  global_frame_ = costmap_ros_->getGlobalFrameID();

  logger_ = node->get_logger();
  clock_ = node->get_clock();

  RCLCPP_INFO(logger_, "A* Global Planner konfigüre edildi. Frame: %s",
              global_frame_.c_str());
}

void AStarGlobalPlanner::cleanup() {
  RCLCPP_INFO(logger_, "A* Global Planner temizleniyor...");
}

void AStarGlobalPlanner::activate() {
  RCLCPP_INFO(logger_, "A* Global Planner aktif edildi!");
}

void AStarGlobalPlanner::deactivate() {
  RCLCPP_INFO(logger_, "A* Global Planner deaktif edildi.");
}

nav_msgs::msg::Path
AStarGlobalPlanner::createPlan(const geometry_msgs::msg::PoseStamped &start,
                               const geometry_msgs::msg::PoseStamped &goal) {
  nav_msgs::msg::Path path;
  path.header.stamp = clock_->now();
  path.header.frame_id = global_frame_;

  // Dünya koordinatlarından grid koordinatlarına dönüşüm
  unsigned int start_mx, start_my, goal_mx, goal_my;

  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y,
                            start_mx, start_my)) {
    RCLCPP_ERROR(logger_, "Başlangıç noktası harita dışında!");
    return path;
  }

  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx,
                            goal_my)) {
    RCLCPP_ERROR(logger_, "Hedef noktası harita dışında!");
    return path;
  }

  RCLCPP_INFO(logger_, "A* Planlama: (%u,%u) -> (%u,%u)", start_mx, start_my,
              goal_mx, goal_my);

  // Debug: Başlangıç ve hedef noktalarının maliyet değerlerini kontrol et
  unsigned char start_cost = costmap_->getCost(start_mx, start_my);
  unsigned char goal_cost = costmap_->getCost(goal_mx, goal_my);
  RCLCPP_INFO(logger_, "Başlangıç maliyeti: %d, Hedef maliyeti: %d",
              static_cast<int>(start_cost), static_cast<int>(goal_cost));

  // Başlangıç engeldeyse uyar ama yine de dene
  if (start_cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    RCLCPP_WARN(logger_,
                "UYARI: Başlangıç noktası engel bölgesinde! Maliyet: %d",
                static_cast<int>(start_cost));
  }
  if (goal_cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    RCLCPP_WARN(logger_, "UYARI: Hedef noktası engel bölgesinde! Maliyet: %d",
                static_cast<int>(goal_cost));
  }

  // A* algoritmasını çalıştır
  auto grid_path = runAStar(start_mx, start_my, goal_mx, goal_my);

  if (grid_path.empty()) {
    RCLCPP_WARN(logger_, "Yol bulunamadı!");
    return path;
  }

  RCLCPP_INFO(logger_, "Yol bulundu! Uzunluk: %lu waypoint", grid_path.size());

  // Grid koordinatlarını dünya koordinatlarına çevir ve path'e ekle
  for (const auto &point : grid_path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;

    double wx, wy;
    costmap_->mapToWorld(point.first, point.second, wx, wy);

    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;

    path.poses.push_back(pose);
  }

  return path;
}

std::vector<std::pair<int, int>>
AStarGlobalPlanner::runAStar(int start_x, int start_y, int goal_x, int goal_y) {
  // Priority queue - en düşük f_cost önce
  std::priority_queue<std::shared_ptr<AStarNode>,
                      std::vector<std::shared_ptr<AStarNode>>, CompareNode>
      open_list;

  // Track visited cells
  unsigned int size_x = costmap_->getSizeInCellsX();
  unsigned int size_y = costmap_->getSizeInCellsY();
  std::vector<bool> visited(size_x * size_y, false);

  // Start node
  auto start_node = std::make_shared<AStarNode>(
      start_x, start_y, 0.0, heuristic(start_x, start_y, goal_x, goal_y));
  open_list.push(start_node);

  // 8-connected grid directions
  // cost: 1.0 for orthogonal, ~1.41 for diagonal
  int dx[] = {0, 0, 1, -1, 1, 1, -1, -1};
  int dy[] = {1, -1, 0, 0, 1, -1, 1, -1};
  double move_cost[] = {1.0, 1.0, 1.0, 1.0, 1.41, 1.41, 1.41, 1.41};

  while (!open_list.empty()) {
    // Get node with lowest f_cost
    auto current = open_list.top();
    open_list.pop();

    int cx = current->x;
    int cy = current->y;
    unsigned int idx = cy * size_x + cx;

    if (visited[idx])
      continue;
    visited[idx] = true;

    // Check goal reached
    if (cx == goal_x && cy == goal_y) {
      // Reconstruct path
      std::vector<std::pair<int, int>> path;
      auto node = current;
      while (node) {
        path.push_back({node->x, node->y});
        node = node->parent;
      }
      std::reverse(path.begin(), path.end());
      return path;
    }

    // Check neighbors
    for (int i = 0; i < 8; i++) {
      int nx = cx + dx[i];
      int ny = cy + dy[i];

      if (!isValid(nx, ny))
        continue;

      unsigned int n_idx = ny * size_x + nx;
      if (visited[n_idx])
        continue;

      // Check for obstacles using costmap
      if (isBlocked(nx, ny))
        continue;

      // Create neighbor node
      double new_g = current->g_cost + move_cost[i];
      double new_h = heuristic(nx, ny, goal_x, goal_y);

      auto neighbor =
          std::make_shared<AStarNode>(nx, ny, new_g, new_h, current);
      open_list.push(neighbor);
    }
  }

  // Path not found
  return {};
}

double AStarGlobalPlanner::heuristic(int x1, int y1, int x2, int y2) {
  // Euclidean distance
  return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

bool AStarGlobalPlanner::isValid(int x, int y) {
  return x >= 0 && y >= 0 &&
         x < static_cast<int>(costmap_->getSizeInCellsX()) &&
         y < static_cast<int>(costmap_->getSizeInCellsY());
}

bool AStarGlobalPlanner::isBlocked(unsigned int x, unsigned int y) {
  /*
   * Costmap values:
   * - FREE_SPACE (0): Free
   * - 1-252: Inflation gradient
   * - INSCRIBED_INFLATED_OBSTACLE (253): Robot radius center
   * - LETHAL_OBSTACLE (254): Actual obstacle
   * - NO_INFORMATION (255): Unknown
   *
   * We treat LETHAL_OBSTACLE and NO_INFORMATION as blocked.
   * Inflation zones are technically traversable by the global planner,
   * local planner (DWA) will handle obstacle avoidance.
   */
  unsigned char cost = costmap_->getCost(x, y);
  return cost >= nav2_costmap_2d::LETHAL_OBSTACLE;
}

} // namespace my_astar_planner

// Register plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_astar_planner::AStarGlobalPlanner,
                       nav2_core::GlobalPlanner)
