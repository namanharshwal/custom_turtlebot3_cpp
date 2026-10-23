// ==============================================================================
// Creator & Maintainer: Naman Harshwal (namanharshwal@gmail.com)
// Owner/Supervisor:     Pradeep Singh Chauhan
// Workspace:            psnh_turtlebot_cpp_ws
// Package:              diffdrive_arduino
// File:                 diffbot_launcher.cpp
// License:              Apache License 2.0 (c) 2026 Naman Harshwal
//
// Description: Modern C++17 native process launcher for DiffBot ros2_control stack.
// ==============================================================================

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <fstream>
#include <sstream>
#include <csignal>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

static std::vector<pid_t> child_pids;

void signal_handler(int signum)
{
  std::cout << "\n[diffbot_launcher] Caught signal (" << signum << "). Terminating child processes..." << std::endl;
  for (pid_t pid : child_pids) {
    if (pid > 0) {
      kill(-pid, SIGINT);
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  for (pid_t pid : child_pids) {
    if (pid > 0) {
      kill(-pid, SIGTERM);
    }
  }
  for (pid_t pid : child_pids) {
    if (pid > 0) {
      int status = 0;
      waitpid(pid, &status, 0);
    }
  }
  _exit(0);
}

pid_t spawn(const std::string &name, const std::vector<std::string> &args)
{
  pid_t pid = fork();
  if (pid == 0) {
    setpgid(0, 0);
    std::vector<char *> argv;
    for (const auto &arg : args) {
      argv.push_back(const_cast<char *>(arg.c_str()));
    }
    argv.push_back(nullptr);
    execvp(argv[0], argv.data());
    std::cerr << "Failed to execvp " << name << std::endl;
    _exit(127);
  }
  std::cout << "[diffbot_launcher] Started " << name << " (PID: " << pid << ")" << std::endl;
  child_pids.push_back(pid);
  return pid;
}

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  std::string share_dir;
  try {
    share_dir = ament_index_cpp::get_package_share_directory("diffdrive_arduino");
  } catch (...) {
    share_dir = "/home/ps/psnh_turtlebot_cpp_ws/src/diffdrive_arduino";
  }

  std::string urdf_file = share_dir + "/urdf/diffbot.urdf";
  std::string controllers_file = share_dir + "/config/diffbot_controllers.yaml";
  std::string rviz_file = share_dir + "/rviz/diffbot.rviz";

  std::cout << "========================================================" << std::endl;
  std::cout << " DiffBot ros2_control Native C++17 Launcher             " << std::endl;
  std::cout << " URDF: " << urdf_file << std::endl;
  std::cout << " Controllers: " << controllers_file << std::endl;
  std::cout << "========================================================" << std::endl;

  // 1. Robot State Publisher
  spawn("robot_state_publisher", {
    "ros2", "run", "robot_state_publisher", "robot_state_publisher",
    urdf_file,
    "--ros-args", "-r", "/diff_drive_controller/cmd_vel_unstamped:=/cmd_vel"
  });

  // 2. ros2_control_node
  spawn("ros2_control_node", {
    "ros2", "run", "controller_manager", "ros2_control_node",
    "--ros-args",
    "--params-file", controllers_file
  });

  // Wait 3 seconds for controller_manager to come up
  std::this_thread::sleep_for(std::chrono::seconds(3));

  // 3. Spawner joint_state_broadcaster
  spawn("spawner_jsb", {
    "ros2", "run", "controller_manager", "spawner",
    "joint_state_broadcaster", "--controller-manager", "/controller_manager"
  });

  // 4. Spawner diffbot_base_controller
  spawn("spawner_base", {
    "ros2", "run", "controller_manager", "spawner",
    "diffbot_base_controller", "--controller-manager", "/controller_manager"
  });

  // 5. RViz2
  spawn("rviz2", {"rviz2", "-d", rviz_file});

  for (pid_t pid : child_pids) {
    int status = 0;
    waitpid(pid, &status, 0);
  }
  return 0;
}
