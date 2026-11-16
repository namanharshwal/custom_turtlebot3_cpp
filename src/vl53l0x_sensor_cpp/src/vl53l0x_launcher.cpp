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
// Package:              vl53l0x_sensor_cpp
// File:                 vl53l0x_launcher.cpp
// License:              Apache License 2.0 (c) 2026 Naman Harshwal
// ==============================================================================

#include <iostream>
#include <vector>
#include <string>
#include <unistd.h>
#include <sys/wait.h>
#include <csignal>

static pid_t child_pid = -1;

void sigHandler(int sig)
{
  (void)sig;
  if (child_pid > 0) {
    kill(child_pid, SIGTERM);
  }
  exit(0);
}

int main(int argc, char ** argv)
{
  (void)argc;
  (void)argv;

  signal(SIGINT, sigHandler);
  signal(SIGTERM, sigHandler);

  std::cout << "========================================================\n"
            << "  VL53L0X Time-of-Flight Distance Sensor Launcher (C++17)\n"
            << "========================================================\n";

  pid_t pid = fork();
  if (pid == 0) {
    std::vector<const char *> args = {
      "ros2", "run", "vl53l0x_sensor_cpp", "vl53l0x_node",
      "--ros-args",
      "-p", "i2c_bus:=1",
      "-p", "i2c_address:=41", // 0x29 decimal is 41
      "-p", "frame_id:=tof_link",
      "-p", "min_range:=0.03",
      "-p", "max_range:=0.60",
      "-p", "field_of_view:=0.4363",
      "-p", "publish_rate:=20.0",
      nullptr
    };
    execvp(args[0], const_cast<char * const *>(args.data()));
    perror("execvp failed");
    exit(1);
  } else if (pid > 0) {
    child_pid = pid;
    int status;
    waitpid(pid, &status, 0);
  }

  return 0;
}
