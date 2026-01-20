#!/bin/bash
# ============================================================
# Docker Entrypoint Script
# ============================================================
# ROS2 environment'ı source eder ve komutu çalıştırır

set -e

# ROS2 ve workspace'i source et
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# TurtleBot3 model
export TURTLEBOT3_MODEL=burger

# Gazebo model path
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models

echo "========================================"
echo "TurtleBot3 A* Planner Container"
echo "TURTLEBOT3_MODEL: $TURTLEBOT3_MODEL"
echo "========================================"
echo ""
echo "Nav2 başlatmak için:"
echo "  ros2 launch my_astar_planner nav2_launch.py"
echo ""
echo "========================================"

exec "$@"
