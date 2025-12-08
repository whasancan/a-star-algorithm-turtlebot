#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class PathFollower : public rclcpp::Node {
public:
    PathFollower() : Node("path_follower") {
        RCLCPP_INFO(this->get_logger(), "Path Follower Node Başlatıldı!");

        // Subscribers
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/astar_path", 10, std::bind(&PathFollower::pathCallback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&PathFollower::odomCallback, this, std::placeholders::_1));

        // Publishers
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Timer - 20Hz kontrol döngüsü
        timer_ = this->create_wall_timer(50ms, std::bind(&PathFollower::controlLoop, this));

        // Parametreler
        linear_speed_ = 0.08;       // m/s (yavaş ve kontrollü)
        angular_speed_ = 0.4;       // rad/s
        goal_tolerance_ = 0.12;     // metre
        angle_tolerance_ = 0.15;    // radyan (~8 derece)
    }

private:
    std::vector<std::pair<double, double>> path_world_;
    bool path_received_ = false;
    bool odom_received_ = false;
    size_t current_waypoint_ = 0;

    double robot_x_ = 0.0, robot_y_ = 0.0, robot_yaw_ = 0.0;
    double start_x_ = 0.0, start_y_ = 0.0;
    double first_path_x_ = 0.0, first_path_y_ = 0.0;
    bool path_started_ = false;

    double linear_speed_;
    double angular_speed_;
    double goal_tolerance_;
    double angle_tolerance_;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        
        // Quaternion -> Yaw
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        robot_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
        
        odom_received_ = true;
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Boş path alındı!");
            return;
        }
        
        // ÖNCEKİ PATH'İ TAMAMEN SİL
        stopRobot();
        path_world_.clear();
        current_waypoint_ = 0;
        
        // Yeni path'i kaydet
        for (const auto& pose : msg->poses) {
            path_world_.push_back({pose.pose.position.x, pose.pose.position.y});
        }
        
        // Robot ŞUAN Kİ pozisyonunu ve path ilk noktasını kaydet
        start_x_ = robot_x_;
        start_y_ = robot_y_;
        first_path_x_ = path_world_[0].first;
        first_path_y_ = path_world_[0].second;
        
        path_received_ = true;
        path_started_ = true;
        
        RCLCPP_INFO(this->get_logger(), "=== YENİ PATH ===");
        RCLCPP_INFO(this->get_logger(), "Waypoint sayısı: %lu", path_world_.size());
        RCLCPP_INFO(this->get_logger(), "Robot şu an: (%.2f, %.2f)", robot_x_, robot_y_);
        RCLCPP_INFO(this->get_logger(), "Path başlangıç: (%.2f, %.2f)", first_path_x_, first_path_y_);
    }

    void controlLoop() {
        if (!path_received_ || !odom_received_ || path_world_.empty()) {
            return;
        }

        // Path sonuna ulaştık mı?
        if (current_waypoint_ >= path_world_.size()) {
            stopRobot();
            if (path_started_) {
                RCLCPP_INFO(this->get_logger(), "🎯 HEDEFE ULAŞILDI!");
                path_started_ = false;
            }
            path_received_ = false;
            return;
        }

        // Hedef waypoint - path koordinatlarını robot odom'una göre dönüştür
        double path_x = path_world_[current_waypoint_].first;
        double path_y = path_world_[current_waypoint_].second;
        
        // Offset hesapla: robot başlangıcı + (path noktası - path başlangıcı)
        double target_x = start_x_ + (path_x - first_path_x_);
        double target_y = start_y_ + (path_y - first_path_y_);
        
        // Hedefe mesafe ve açı
        double dx = target_x - robot_x_;
        double dy = target_y - robot_y_;
        double distance = std::sqrt(dx * dx + dy * dy);
        double target_yaw = std::atan2(dy, dx);
        double yaw_error = normalizeAngle(target_yaw - robot_yaw_);

        // Waypoint'e ulaştık mı?
        if (distance < goal_tolerance_) {
            current_waypoint_++;
            RCLCPP_INFO(this->get_logger(), "✓ WP %lu/%lu dist:%.2f", 
                current_waypoint_, path_world_.size(), distance);
            return;
        }

        // Kontrol komutu
        geometry_msgs::msg::Twist cmd;

        if (std::abs(yaw_error) > angle_tolerance_) {
            // Önce hedefe dön
            cmd.linear.x = 0.0;
            cmd.angular.z = angular_speed_ * (yaw_error > 0 ? 1.0 : -1.0);
        } else {
            // Düz git + hafif dönüş düzeltmesi
            cmd.linear.x = linear_speed_;
            cmd.angular.z = 1.0 * yaw_error;
        }

        cmd_pub_->publish(cmd);
    }

    void stopRobot() {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_pub_->publish(cmd);
    }

    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathFollower>());
    rclcpp::shutdown();
    return 0;
}
