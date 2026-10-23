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
// File:                 autonomous_mapping_launcher.cpp
// License:              Apache License 2.0 (c) 2026 Naman Harshwal
//
// Description: Pure C++17 Master Autonomous Mapping Orchestrator.
// Launches Hardware Stack, SLAM Toolbox, RViz2, and Simple Explorer with zero Python.
// ==============================================================================

#include "custom_bot_cpp/process_launcher.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

int main(int argc, char ** argv)
{
  (void)argc;
  (void)argv;

  auto & launcher = custom_bot_cpp::ProcessLauncher::getInstance();
  launcher.installSignalHandlers();

  std::string pkg_share;
  try {
    pkg_share = ament_index_cpp::get_package_share_directory("custom_bot_cpp");
  } catch (...) {
    pkg_share = "/home/ps/psnh_turtlebot_cpp_ws/install/custom_bot_cpp/share/custom_bot_cpp";
  }

  std::string urdf_path   = pkg_share + "/description/robot.urdf";
  std::string slam_params = pkg_share + "/config/slam_toolbox_params.yaml";
  std::string rviz_config = "/home/ps/.rviz2/default.rviz";

  std::cout << "========================================================\n"
            << "  Custom TurtleBot Burger 2.0: Autonomous Mapping (C++17)\n"
            << "========================================================\n";

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

  // 4. Object Detector Node (Camera & YOLOv4-tiny)
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

  // 5. SLAM Toolbox (delayed 3s)
  launcher.spawnDelayed("slam_toolbox", {
    "ros2", "run", "slam_toolbox", "async_slam_toolbox_node",
    "--ros-args",
    "--params-file", slam_params
  }, 3);

  // 6. RViz2 (delayed 3s)
  launcher.spawnDelayed("rviz2", {
    "rviz2", "-d", rviz_config
  }, 3);

  // 7. Simple Explorer (delayed 3s)
  launcher.spawnDelayed("simple_explorer", {
    "ros2", "run", "custom_bot_cpp", "simple_explorer",
    "--ros-args",
    "-p", "linear_speed:=0.05",
    "-p", "angular_speed:=0.25",
    "-p", "obstacle_dist:=0.45",
    "-p", "critical_dist:=0.30",
    "-p", "clear_dist:=0.38",
    "-p", "wall_follow_dist:=0.40",
    "-p", "control_hz:=10.0"
  }, 3);

  std::cout << "[C++ Launcher] Autonomous mapping stack running. Press Ctrl+C to terminate." << std::endl;
  launcher.waitAll();
  return 0;
}
