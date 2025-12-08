cmake_minimum_required(VERSION 3.8)
project(my_astar_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(astar_node src/astar_node.cpp)
add_executable(path_follower src/path_follower.cpp)

ament_target_dependencies(astar_node 
  rclcpp 
  nav_msgs 
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(path_follower 
  rclcpp 
  nav_msgs 
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  astar_node
  path_follower
  DESTINATION lib/${PROJECT_NAME}
)

# Launch ve config dosyalarını yükle
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()