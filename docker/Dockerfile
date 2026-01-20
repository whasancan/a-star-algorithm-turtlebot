# ============================================================
# TurtleBot3 A* Planner - Docker Image
# ============================================================
# Base image: ROS2 Humble Desktop Full
# Includes Gazebo and RViz for simulation suitable for local development
# with X11 forwarding.
#
# Build:
#   docker build -t turtlebot3_astar .
#
# Run:
#   docker-compose up
# ============================================================

FROM osrf/ros:humble-desktop-full

# System dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3* \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-slam-toolbox \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# TurtleBot3 model
ENV TURTLEBOT3_MODEL=burger

# Create workspace
WORKDIR /ros2_ws
RUN mkdir -p src

# Copy package
COPY . /ros2_ws/src/my_astar_planner/

# Install dependencies and build
RUN . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install

# Entrypoint script
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# ROS2 environment setup
SHELL ["/bin/bash", "-c"]

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
