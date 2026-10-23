// ==============================================================================
// Creator & Maintainer: Naman Harshwal (namanharshwal@gmail.com)
// Owner/Supervisor:     Pradeep Singh Chauhan
// Workspace:            psnh_turtlebot_cpp_ws
// Package:              multirobot_map_merge
// File:                 from_map_server_launcher.cpp
// License:              Apache License 2.0 (c) 2026 Naman Harshwal
//
// Description: Modern C++17 native process launcher for static map merge.
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
  std::cout << "\n[from_map_server_launcher] Caught signal (" << signum << "). Terminating child processes..." << std::endl;
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
  std::cout << "[from_map_server_launcher] Started " << name << " (PID: " << pid << ")" << std::endl;
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
    share_dir = ament_index_cpp::get_package_share_directory("multirobot_map_merge");
  } catch (...) {
    share_dir = "/home/ps/psnh_turtlebot_cpp_ws/src/m-explore-ros2/map_merge";
  }

  std::string params_file = share_dir + "/config/params.yaml";
  std::string rviz_file = share_dir + "/launch/map_merge.rviz";

  std::cout << "========================================================" << std::endl;
  std::cout << " Static Map Merge Launcher (100% Native C++17)          " << std::endl;
  std::cout << " Parameters: " << params_file << std::endl;
  std::cout << "========================================================" << std::endl;

  spawn("map_merge_node", {
    "ros2", "run", "multirobot_map_merge", "map_merge",
    "--ros-args", "--params-file", params_file
  });

  spawn("rviz2", {"rviz2", "-d", rviz_file});

  for (pid_t pid : child_pids) {
    int status = 0;
    waitpid(pid, &status, 0);
  }
  return 0;
}
