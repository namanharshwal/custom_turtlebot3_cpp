// ==============================================================================
// Creator & Maintainer: Naman Harshwal (namanharshwal@gmail.com)
// Owner/Supervisor:     Pradeep Singh Chauhan
// Workspace:            psnh_turtlebot_cpp_ws
// Package:              diffdrive_arduino
// File:                 view_robot_launcher.cpp
// License:              Apache License 2.0 (c) 2026 Naman Harshwal
//
// Description: Modern C++17 native launcher for DiffBot visual inspection.
// ==============================================================================

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <csignal>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

static std::vector<pid_t> child_pids;

void signal_handler(int signum)
{
  std::cout << "\n[view_robot_launcher] Caught signal (" << signum << "). Terminating child processes..." << std::endl;
  for (pid_t pid : child_pids) {
    if (pid > 0) {
      kill(-pid, SIGINT);
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
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
  std::cout << "[view_robot_launcher] Started " << name << " (PID: " << pid << ")" << std::endl;
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
  std::string rviz_file = share_dir + "/rviz/diffbot_view.rviz";

  std::cout << "========================================================" << std::endl;
  std::cout << " DiffBot Visual Model Inspector (100% Native C++17)     " << std::endl;
  std::cout << " URDF: " << urdf_file << std::endl;
  std::cout << " RViz Config: " << rviz_file << std::endl;
  std::cout << "========================================================" << std::endl;

  // 1. Joint State Publisher GUI
  spawn("joint_state_publisher_gui", {"ros2", "run", "joint_state_publisher_gui", "joint_state_publisher_gui"});

  // 2. Robot State Publisher
  spawn("robot_state_publisher", {
    "ros2", "run", "robot_state_publisher", "robot_state_publisher", urdf_file
  });

  // 3. RViz2
  spawn("rviz2", {"rviz2", "-d", rviz_file});

  for (pid_t pid : child_pids) {
    int status = 0;
    waitpid(pid, &status, 0);
  }
  return 0;
}
