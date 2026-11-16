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
// Timings match autonomous_mapping.launch.py:
//   t=0s:  robot_state_publisher + ydlidar
//   t=2s:  arduino_bridge
//   t=3s:  object_detector_node
//   t=5s:  slam_toolbox (allows sensor initialization)
//   t=7s:  rviz2
//   t=10s: simple_explorer (allows SLAM to lock static geometry before movement)
// ==============================================================================

#include "custom_bot_cpp/process_launcher.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <unistd.h>
#include <iostream>

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
  std::string rviz_config = pkg_share + "/config/mapping.rviz";

  std::cout << "========================================================\n"
            << "  Custom TurtleBot Burger 2.0: Autonomous Mapping (C++17)\n"
            << "========================================================\n";

  // 1. Robot State Publisher (t=0s)
  launcher.spawn("robot_state_publisher", {
    "ros2", "run", "robot_state_publisher", "robot_state_publisher",
    urdf_path,
    "--ros-args", "-p", "use_sim_time:=false"
  });

  // 2. YDLidar Driver (t=0s)
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
    "-p", "reversion:=false",
    "-p", "inverted:=false",
    "-p", "abnormal_check_count:=4",
    "-p", "invalid_range_is_inf:=true"
  });

  // 3. Arduino Bridge (delayed 2s for UART settle)
  launcher.spawnDelayed("arduino_bridge", {
    "ros2", "run", "custom_bot_cpp", "arduino_bridge",
    "--ros-args",
    "-p", "serial_port:=/dev/ttyAMA0",
    "-p", "baud_rate:=115200",
    "-p", "wheel_diameter:=0.065",
    "-p", "wheel_separation:=0.150",
    "-p", "encoder_ticks_left:=412",
    "-p", "encoder_ticks_right:=440",
    "-p", "max_speed:=0.40",
    "-p", "publish_tf:=true"
  }, 2);

  // 3.5 VL53L0X Time-of-Flight Infrared Distance Sensor (delayed 2s)
  launcher.spawnDelayed("vl53l0x_node", {
    "ros2", "run", "vl53l0x_sensor_cpp", "vl53l0x_node",
    "--ros-args",
    "-p", "i2c_bus:=1",
    "-p", "i2c_address:=41",
    "-p", "frame_id:=tof_link",
    "-p", "min_range:=0.03",
    "-p", "max_range:=0.60",
    "-p", "field_of_view:=0.4363",
    "-p", "publish_rate:=20.0"
  }, 2);

  // 4. Object Detector Node (Camera & YOLOv4-tiny live feedback in RViz2, delayed 3s)
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
      "-p", "detection_rate_hz:=1.5",
      "-p", "publish_rate_hz:=10.0",
      "-p", "publish_raw_image:=true",
      "-p", "analogue_gain:=350",
      "-p", "exposure:=1200"
    }, 3);
  }

  // 5. SLAM Toolbox (delayed 5s to allow gyro calibration and initial scans)
  launcher.spawnDelayed("slam_toolbox", {
    "ros2", "run", "slam_toolbox", "async_slam_toolbox_node",
    "--ros-args",
    "--params-file", slam_params,
    "-p", "use_lifecycle_manager:=false"
  }, 5);

  // 6. RViz2 (delayed 7s)
  launcher.spawnDelayed("rviz2", {
    "bash", "-c",
    "source /opt/ros/humble/setup.bash && source /home/ps/psnh_turtlebot_cpp_ws/install/setup.bash && export DISPLAY=:1 && exec rviz2 -d " + rviz_config
  }, 7);

  // 7. Simple Explorer (delayed 10s to let SLAM establish initial map)
  launcher.spawnDelayed("simple_explorer", {
    "ros2", "run", "custom_bot_cpp", "simple_explorer",
    "--ros-args",
    "-p", "linear_speed:=0.12",
    "-p", "angular_speed:=0.40",
    "-p", "obstacle_dist:=0.40",
    "-p", "critical_dist:=0.25",
    "-p", "clear_dist:=0.48",
    "-p", "wall_follow_dist:=0.38",
    "-p", "control_hz:=10.0"
  }, 10);

  std::cout << "[C++ Launcher] Autonomous mapping stack running. Press Ctrl+C to terminate." << std::endl;
  launcher.waitAll();
  return 0;
}
