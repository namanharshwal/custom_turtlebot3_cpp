cmake_minimum_required(VERSION 3.8)
project(vl53l0x_sensor_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
)

# 1. Driver Node Executable
add_executable(vl53l0x_node
  src/vl53l0x_node.cpp
)
ament_target_dependencies(vl53l0x_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

# 2. Pure C++ Launcher Executable
add_executable(vl53l0x_launcher
  src/vl53l0x_launcher.cpp
)

install(TARGETS
  vl53l0x_node
  vl53l0x_launcher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
