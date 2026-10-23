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
// File:                 slam_launcher.cpp
// License:              Apache License 2.0 (c) 2026 Naman Harshwal
//
// Description: Pure C++17 SLAM Launcher for TurtleBot Burger 2.0.
// Launches SLAM Toolbox async node with tuned parameters for real-time mapping.
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

  std::string params_path = pkg_share + "/config/slam_toolbox_params.yaml";

  std::cout << "========================================================\n"
            << "  Custom TurtleBot Burger 2.0: SLAM Launcher (C++17)\n"
            << "========================================================\n"
            << "[C++ Launcher] Target Config: " << params_path << std::endl;

  launcher.spawn("slam_toolbox", {
    "ros2", "run", "slam_toolbox", "async_slam_toolbox_node",
    "--ros-args",
    "--params-file", params_path
  });

  std::cout << "[C++ Launcher] SLAM Toolbox active. Press Ctrl+C to terminate." << std::endl;
  launcher.waitAll();
  return 0;
}
