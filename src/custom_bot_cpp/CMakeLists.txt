# Copyright 2026 Naman Harshwal
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8)
project(custom_bot_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies ──────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# ── 1. arduino_bridge ──────────────────────────────────────────
add_executable(arduino_bridge
  src/arduino_bridge.cpp
)
ament_target_dependencies(arduino_bridge
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ── 2. simple_explorer ─────────────────────────────────────────
add_executable(simple_explorer
  src/simple_explorer.cpp
)
ament_target_dependencies(simple_explorer
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

# ── 3. teleop_key_hold ─────────────────────────────────────────
add_executable(teleop_key_hold
  src/teleop_key_hold.cpp
)
ament_target_dependencies(teleop_key_hold
  rclcpp
  geometry_msgs
)

# ── 4. object_detector_node ───────────────────────────────────
add_executable(object_detector_node
  src/object_detector_node.cpp
)
ament_target_dependencies(object_detector_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  ament_index_cpp
)
target_link_libraries(object_detector_node
  ${OpenCV_LIBS}
)

# ── 5. robot_full_launcher (C++ Hardware Orchestrator) ────────
add_executable(robot_full_launcher
  src/robot_full_launcher.cpp
)
ament_target_dependencies(robot_full_launcher
  ament_index_cpp
)

# ── 6. slam_launcher (C++ SLAM Orchestrator) ──────────────────
add_executable(slam_launcher
  src/slam_launcher.cpp
)
ament_target_dependencies(slam_launcher
  ament_index_cpp
)

# ── 7. autonomous_mapping_launcher (C++ Mapping Master) ───────
add_executable(autonomous_mapping_launcher
  src/autonomous_mapping_launcher.cpp
)
ament_target_dependencies(autonomous_mapping_launcher
  ament_index_cpp
)

# ── 8. full_navigation_launcher (C++ Navigation Master) ───────
add_executable(full_navigation_launcher
  src/full_navigation_launcher.cpp
)
ament_target_dependencies(full_navigation_launcher
  ament_index_cpp
)

# ── 9. waypoint_navigator (C++ Nav2 Action Client) ────────────
add_executable(waypoint_navigator
  src/waypoint_navigator.cpp
)
ament_target_dependencies(waypoint_navigator
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
)

# ── Install Executables ────────────────────────────────────────
install(TARGETS
  arduino_bridge
  simple_explorer
  teleop_key_hold
  object_detector_node
  robot_full_launcher
  slam_launcher
  autonomous_mapping_launcher
  full_navigation_launcher
  waypoint_navigator
  DESTINATION lib/${PROJECT_NAME}
)

# ── Install Directories ────────────────────────────────────────
install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  description
  config
  models
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
