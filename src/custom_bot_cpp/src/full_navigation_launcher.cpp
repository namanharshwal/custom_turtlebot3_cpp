// Copyright 2026 Naman Harshwal
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// ==============================================================================
// Creator & Maintainer: Naman Harshwal (namanharshwal@gmail.com)
// Owner/Supervisor:     Pradeep Singh Chauhan
// Workspace:            psnh_turtlebot_cpp_ws
// Package:              custom_bot_cpp
// File:                 full_navigation_launcher.cpp
// License:              Apache License 2.0 (c) 2026 Naman Harshwal
//
// Description: Pure C++17 Master Navigation & Waypoint Orchestrator.
// Launches Hardware Stack, Nav2 Stack with saved map, AMCL, and RViz2 with zero Python.
// ==============================================================================

#include "custom_bot_cpp/process_launcher.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

int main(int argc, char ** argv)
{
  auto & launcher = custom_bot_cpp::ProcessLauncher::getInstance();
  launcher.installSignalHandlers();

  std::string pkg_share;
  std::string nav_pkg_share;
  try {
    pkg_share = ament_index_cpp::get_package_share_directory("custom_bot_cpp");
    nav_pkg_share = ament_index_cpp::get_package_share_directory("custom_bot_nav_cpp");
  } catch (...) {
    pkg_share = "/home/ps/psnh_turtlebot_cpp_ws/install/custom_bot_cpp/share/custom_bot_cpp";
    nav_pkg_share = "/home/ps/psnh_turtlebot_cpp_ws/install/custom_bot_nav_cpp/share/custom_bot_nav_cpp";
  }

  std::string urdf_path   = pkg_share + "/description/robot.urdf";
  std::string map_path    = pkg_share + "/maps/room_map.yaml";
  std::string nav2_params = nav_pkg_share + "/config/nav2_params.yaml";
  std::string rviz_config = pkg_share + "/config/nav2.rviz";

  // Check if custom map was passed as CLI argument
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg.rfind("--map=", 0) == 0) {
      map_path = arg.substr(6);
    } else if (arg == "-m" && i + 1 < argc) {
      map_path = argv[++i];
    }
  }

  std::cout << "========================================================\n"
            << "  Custom TurtleBot Burger 2.0: Nav2 Navigation (C++17)\n"
            << "========================================================\n"
            << "[C++ Launcher] Target Map: " << map_path << "\n"
            << "[C++ Launcher] Nav2 Config: " << nav2_params << std::endl;

  // 1. Robot State Publisher
  launcher.spawn("robot_state_publisher", {
    "ros2", "run", "robot_state_publisher", "robot_state_publisher",
    urdf_path,
    "--ros-args", "-p", "use_sim_time:=false"
  });

  // 2. YDLidar Driver
  launcher.spawn("ydlidar_node", {
    "ros2", "run", "ydlidar_ros2_driver", "ydlidar_ros2_driver_node",
    "--ros-args",
    "-p", "port:=/dev/ydlidar",
    "-p", "frame_id:=laser_frame",
    "-p", "baudrate:=115200",
    "-p", "lidar_type:=1",
    "-p", "device_type:=0",
    "-p", "sample_rate:=3",
    "-p", "isSingleChannel:=true",
    "-p", "intensity:=false",
    "-p", "frequency:=10.0",
    "-p", "angle_max:=180.0",
    "-p", "angle_min:=-180.0",
    "-p", "range_max:=8.0",
    "-p", "range_min:=0.15",
    "-p", "resolution_fixed:=true",
    "-p", "auto_reconnect:=true",
    "-p", "reversion:=false"
  });

  // 3. Arduino Bridge (delayed 2s)
  launcher.spawnDelayed("arduino_bridge", {
    "ros2", "run", "custom_bot_cpp", "arduino_bridge",
    "--ros-args",
    "-p", "serial_port:=/dev/ttyAMA0",
    "-p", "baud_rate:=115200",
    "-p", "wheel_diameter:=0.065",
    "-p", "wheel_separation:=0.150",
    "-p", "encoder_ticks_per_rev:=412",
    "-p", "max_speed:=0.5",
    "-p", "publish_tf:=true"
  }, 2);

  // 4. Object Detector Node
  if (access("/dev/video0", F_OK) == 0) {
    launcher.spawnDelayed("object_detector_node", {
      "ros2", "run", "custom_bot_cpp", "object_detector_node",
      "--ros-args",
      "-p", "video_device:=0",
      "-p", "capture_width:=640",
      "-p", "capture_height:=480",
      "-p", "input_size:=320",
      "-p", "conf_threshold:=0.20",
      "-p", "nms_threshold:=0.40",
      "-p", "detection_rate_hz:=2.0",
      "-p", "publish_rate_hz:=10.0",
      "-p", "publish_raw_image:=true",
      "-p", "analogue_gain:=350",
      "-p", "exposure:=1200"
    }, 1);
  }

  // 5. Nav2 Stack Bringup (delayed 3s)
  launcher.spawnDelayed("nav2_bringup", {
    "ros2", "launch", "nav2_bringup", "bringup_launch.py",
    "use_sim_time:=false",
    "map:=" + map_path,
    "params_file:=" + nav2_params
  }, 3);

  // 6. RViz2 (delayed 5s)
  launcher.spawnDelayed("rviz2", {
    "rviz2", "-d", rviz_config
  }, 5);

  std::cout << "[C++ Launcher] Autonomous Navigation stack online. Press Ctrl+C to terminate." << std::endl;
  launcher.waitAll();
  return 0;
}
