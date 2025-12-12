#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <cmath>
#include <set>
#include <algorithm>

#include "rclcpp/rclcpp.hpp" // c++ için ROS2 temel kütüphanesi
#include "nav_msgs/msg/occupancy_grid.hpp" // navigasyonla ilgili mesajlar harita, yol, odometri
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" // geometri ile ilgili mesajlar poz, hız
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

struct AStarNode { // A* algoritması içiin temel düğüm yapısı
    int x, y;
    double g_cost, h_cost, f_cost;
    std::shared_ptr<AStarNode> parent; // modern C++'da akıllı işaretçi kullanımı, new delete gibi bellek yönetimi yok

    AStarNode(int _x, int _y, double _g, double _h, std::shared_ptr<AStarNode> _parent = nullptr)
        : x(_x), y(_y), g_cost(_g), h_cost(_h), parent(_parent) {
        f_cost = g_cost + h_cost;
    }
};

struct CompareNode {
    bool operator()(const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) {
        return a->f_cost > b->f_cost;
    }
};

class AStarPlanner : public rclcpp::Node {
public:
    AStarPlanner() : Node("astar_planner") {
        RCLCPP_INFO(this->get_logger(), "A* Planner Node Baslatildi!");

        // TF buffer ve listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Map için QoS - transient_local ile latched gibi davranır
        rclcpp::QoS map_qos(10);
        map_qos.transient_local();
        map_qos.reliable();

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", map_qos, std::bind(&AStarPlanner::mapCallback, this, std::placeholders::_1));

        // HER İKİ GOAL TOPIC'İNİ DİNLE
        goal_sub1_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&AStarPlanner::goalCallback, this, std::placeholders::_1));
        
        goal_sub2_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/move_base_simple/goal", 10, std::bind(&AStarPlanner::goalCallback, this, std::placeholders::_1));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/astar_path", 10);

        // ODOM'dan robot pozisyonunu al
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&AStarPlanner::odomCallback, this, std::placeholders::_1));

        // Robot pozisyonu için başlangıç değeri
        odom_x_ = 0.0;
        odom_y_ = 0.0;
        odom_received_ = false;
    }

private:
    nav_msgs::msg::OccupancyGrid current_map_;
    bool map_received_ = false;
    bool odom_received_ = false;
    double odom_x_, odom_y_;  // Odom frame'deki pozisyon
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        current_map_ = *msg;
        if (!map_received_) {
            RCLCPP_INFO(this->get_logger(), "Harita Alındı! Boyut: %dx%d, Origin: (%.2f, %.2f)",
                msg->info.width, msg->info.height, 
                msg->info.origin.position.x, msg->info.origin.position.y);
            map_received_ = true;
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Odom frame'deki pozisyon
        odom_x_ = msg->pose.pose.position.x;
        odom_y_ = msg->pose.pose.position.y;
        if (!odom_received_) {
            RCLCPP_INFO(this->get_logger(), "Odom alındı! Robot (odom frame): (%.2f, %.2f)", odom_x_, odom_y_);
            odom_received_ = true;
        }
    }

    void getRobotPose(double& x, double& y) {
        // Önce TF dene
        try {
            auto transform = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero, 100ms);
            x = transform.transform.translation.x;
            y = transform.transform.translation.y;
            RCLCPP_INFO(this->get_logger(), "Robot pozisyonu (TF): (%.2f, %.2f)", x, y);
            return;
        } catch (tf2::TransformException& ex) {
            // TF yoksa odom kullan + offset ekle
        }
        
        // Odom'dan map'e dönüşüm: map = odom + offset
        // Harita merkezi civarında robot başlangıcı olmalı
        // Offset: haritanın ortası - odom başlangıcı
        double map_center_x = current_map_.info.origin.position.x + 
                              (current_map_.info.width * current_map_.info.resolution / 2.0);
        double map_center_y = current_map_.info.origin.position.y + 
                              (current_map_.info.height * current_map_.info.resolution / 2.0);
        
        // Robot harita merkezinde başladığını varsay
        x = map_center_x + odom_x_;
        y = map_center_y + odom_y_;
        RCLCPP_INFO(this->get_logger(), "Robot pozisyonu (odom+offset): (%.2f, %.2f)", x, y);
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "=== HEDEF ALINDI ===");
        
        if (!map_received_) {
            RCLCPP_ERROR(this->get_logger(), "Harita henüz alınmadı!");
            return;
        }

        double robot_x, robot_y;
        getRobotPose(robot_x, robot_y);

        double goal_x = msg->pose.position.x;
        double goal_y = msg->pose.position.y;

        RCLCPP_INFO(this->get_logger(), "Robot: (%.2f, %.2f) -> Hedef: (%.2f, %.2f)", 
            robot_x, robot_y, goal_x, goal_y);

        int start_gx, start_gy, goal_gx, goal_gy;
        
        if (!worldToGrid(robot_x, robot_y, start_gx, start_gy)) {
            RCLCPP_ERROR(this->get_logger(), "Robot harita dışında!");
            return;
        }
        if (!worldToGrid(goal_x, goal_y, goal_gx, goal_gy)) {
            RCLCPP_ERROR(this->get_logger(), "Hedef harita dışında!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Grid: (%d,%d) -> (%d,%d)", start_gx, start_gy, goal_gx, goal_gy);

        auto path = runAStar(start_gx, start_gy, goal_gx, goal_gy);

        if (path.empty()) {
            RCLCPP_WARN(this->get_logger(), "Yol bulunamadı!");
        } else {
            RCLCPP_INFO(this->get_logger(), "YOL BULUNDU! Uzunluk: %lu", path.size());
            publishPath(path);
        }
    }

    std::vector<std::pair<int, int>> runAStar(int start_x, int start_y, int goal_x, int goal_y) {
        std::priority_queue<std::shared_ptr<AStarNode>, std::vector<std::shared_ptr<AStarNode>>, CompareNode> open_list;
        std::vector<bool> visited(current_map_.info.width * current_map_.info.height, false);

        auto start_node = std::make_shared<AStarNode>(start_x, start_y, 0.0, heuristic(start_x, start_y, goal_x, goal_y));
        open_list.push(start_node);

        // Robot 8 yöne hareket edebiliyor. 
        // Düz hareketler 1 maliyet, çapraz hareketler √2 ≈ 1.41 maliyet. 
        // Çünkü çapraz mesafe daha uzun.
        
        int dx[] = {0, 0, 1, -1, 1, 1, -1, -1};
        int dy[] = {1, -1, 0, 0, 1, -1, 1, -1};
        double cost[] = {1.0, 1.0, 1.0, 1.0, 1.41, 1.41, 1.41, 1.41};

        while (!open_list.empty()) {
            auto curr = open_list.top();
            open_list.pop();

            int idx = curr->y * current_map_.info.width + curr->x;
            if (visited[idx]) continue;
            visited[idx] = true;

            if (curr->x == goal_x && curr->y == goal_y) {
                std::vector<std::pair<int, int>> path;
                while (curr) {
                    path.push_back({curr->x, curr->y});
                    curr = curr->parent;
                }
                // Path'i ters çevir (başlangıçtan hedefe)
                std::reverse(path.begin(), path.end());
                return path;
            }

            for (int i = 0; i < 8; i++) {
                int nx = curr->x + dx[i];
                int ny = curr->y + dy[i];

                if (isValid(nx, ny) && !isBlocked(nx, ny)) {
                    int n_idx = ny * current_map_.info.width + nx;
                    if (!visited[n_idx]) {
                        auto next = std::make_shared<AStarNode>(nx, ny, 
                            curr->g_cost + cost[i], heuristic(nx, ny, goal_x, goal_y), curr);
                        open_list.push(next);
                    }
                }
            }
        }
        return {};
    }

    double heuristic(int x1, int y1, int x2, int y2) {
        return std::sqrt(std::pow(x1-x2, 2) + std::pow(y1-y2, 2));
    }

    bool isValid(int x, int y) {
        return x >= 0 && y >= 0 && x < (int)current_map_.info.width && y < (int)current_map_.info.height;
    }

    // Tek hücre engel kontrolü
    bool isCellBlocked(int x, int y) {
        if (!isValid(x, y)) return true;
        int idx = y * current_map_.info.width + x;
        return current_map_.data[idx] > 50;
    }

    // Inflation ile engel kontrolü - robot yarıçapı kadar güvenlik mesafesi
    bool isBlocked(int x, int y) {
        // Robot yarıçapı ~0.15m, resolution 0.05m -> 3 hücre
        // Güvenlik için 4 hücre kullan
        const int inflation_radius = 3;
        
        for (int dx = -inflation_radius; dx <= inflation_radius; dx++) {
            for (int dy = -inflation_radius; dy <= inflation_radius; dy++) {
                // Daire şeklinde kontrol (kare değil)
                if (dx*dx + dy*dy <= inflation_radius*inflation_radius) {
                    if (isCellBlocked(x + dx, y + dy)) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    bool worldToGrid(double wx, double wy, int& gx, int& gy) {
        double ox = current_map_.info.origin.position.x;
        double oy = current_map_.info.origin.position.y;
        double res = current_map_.info.resolution;
        gx = (int)((wx - ox) / res);
        gy = (int)((wy - oy) / res);
        return isValid(gx, gy);
    }

    void gridToWorld(int gx, int gy, double& wx, double& wy) {
        double ox = current_map_.info.origin.position.x;
        double oy = current_map_.info.origin.position.y;
        double res = current_map_.info.resolution;
        wx = gx * res + ox + res/2.0;
        wy = gy * res + oy + res/2.0;
    }

    void publishPath(const std::vector<std::pair<int, int>>& coords) {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->get_clock()->now();
        path_msg.header.frame_id = "map";

        for (auto& p : coords) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            gridToWorld(p.first, p.second, pose.pose.position.x, pose.pose.position.y);
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }
        path_pub_->publish(path_msg);
        RCLCPP_INFO(this->get_logger(), "Path yayınlandı!");
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub1_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub2_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AStarPlanner>());
    rclcpp::shutdown();
    return 0;
}