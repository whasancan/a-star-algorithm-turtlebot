cmake_minimum_required(VERSION 3.8)
project(my_astar_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 standardı
set(CMAKE_CXX_STANDARD 17)

# ========================
# BAĞIMLILIKLAR
# ========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Nav2 bağımlılıkları (YENİ)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Tüm bağımlılıklar listesi
set(dependencies
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  rclcpp_lifecycle
)

# ========================
# STANDALONE EXECUTABLE'LAR (Eski sistemin devamı)
# ========================
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

# ========================
# NAV2 PLUGIN (SHARED LIBRARY)
# ========================
# Plugin, shared library olarak derlenir (executable değil!)
add_library(astar_global_planner SHARED
  src/astar_global_planner.cpp
)

# Include dizinini ekle
target_include_directories(astar_global_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(astar_global_planner ${dependencies})

# ========================
# PLUGIN REGISTRATION
# ========================
# Pluginlib'e plugin'i tanıt
pluginlib_export_plugin_description_file(nav2_core astar_plugin.xml)

# ========================
# KURULUM
# ========================
install(TARGETS
  astar_node
  path_follower
  DESTINATION lib/${PROJECT_NAME}
)

# Plugin library kurulumu
install(TARGETS
  astar_global_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Header dosyaları kurulumu
install(DIRECTORY include/
  DESTINATION include
)

# Launch ve config dosyalarını yükle
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Python scripts kurulumu
install(PROGRAMS
  scripts/goal_bridge.py
  scripts/send_goal.sh
  DESTINATION lib/${PROJECT_NAME}
)

# ========================
# EXPORT
# ========================
ament_export_include_directories(include)
ament_export_libraries(astar_global_planner)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()