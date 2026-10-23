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
// File:                 process_launcher.hpp
// License:              Apache License 2.0 (c) 2026 Naman Harshwal
// ==============================================================================

#ifndef CUSTOM_BOT_CPP__PROCESS_LAUNCHER_HPP_
#define CUSTOM_BOT_CPP__PROCESS_LAUNCHER_HPP_

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <atomic>
#include <csignal>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

namespace custom_bot_cpp
{

class ProcessLauncher
{
public:
  static ProcessLauncher & getInstance()
  {
    static ProcessLauncher instance;
    return instance;
  }

  static void signalHandler(int signum)
  {
    std::cout << "\n[C++ ProcessLauncher] Caught signal (" << signum 
              << "). Gracefully shutting down all child processes..." << std::endl;
    getInstance().terminateAll();
  }

  void installSignalHandlers()
  {
    std::signal(SIGINT, ProcessLauncher::signalHandler);
    std::signal(SIGTERM, ProcessLauncher::signalHandler);
  }

  pid_t spawn(const std::string & process_name, const std::vector<std::string> & command_args)
  {
    if (command_args.empty()) return -1;

    pid_t pid = fork();
    if (pid < 0) {
      std::cerr << "[C++ ProcessLauncher] Error: Failed to fork process: " << process_name << std::endl;
      return -1;
    }

    if (pid == 0) {
      // Child process: create new process group
      setpgid(0, 0);

      std::vector<char *> argv;
      for (const auto & arg : command_args) {
        argv.push_back(const_cast<char *>(arg.c_str()));
      }
      argv.push_back(nullptr);

      execvp(argv[0], argv.data());
      std::cerr << "[C++ ProcessLauncher] Failed to execvp: " << process_name << std::endl;
      _exit(127);
    }

    // Parent process
    std::cout << "[C++ ProcessLauncher] Started [" << process_name 
              << "] (PID " << pid << ")" << std::endl;
    pids_.push_back(pid);
    names_.push_back(process_name);
    return pid;
  }

  void spawnDelayed(const std::string & process_name,
                    const std::vector<std::string> & command_args,
                    int delay_seconds)
  {
    std::cout << "[C++ ProcessLauncher] Scheduled [" << process_name 
              << "] with " << delay_seconds << "s delay..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(delay_seconds));
    if (running_) {
      spawn(process_name, command_args);
    }
  }

  void terminateAll()
  {
    running_ = false;
    for (size_t i = 0; i < pids_.size(); ++i) {
      pid_t pid = pids_[i];
      if (pid > 0) {
        std::cout << "[C++ ProcessLauncher] Sending SIGINT to process group for " 
                  << names_[i] << " (PID " << pid << ")..." << std::endl;
        kill(-pid, SIGINT);
      }
    }

    // Give children time to cleanly shut down
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Force kill if any children are still alive
    for (pid_t pid : pids_) {
      if (pid > 0) {
        kill(-pid, SIGTERM);
      }
    }

    waitAll();
  }

  void waitAll()
  {
    for (size_t i = 0; i < pids_.size(); ++i) {
      pid_t pid = pids_[i];
      if (pid > 0) {
        int status = 0;
        waitpid(pid, &status, 0);
      }
    }
    pids_.clear();
    names_.clear();
    std::cout << "[C++ ProcessLauncher] All child processes terminated cleanly." << std::endl;
  }

  bool isRunning() const { return running_; }

private:
  ProcessLauncher() : running_(true) {}
  ~ProcessLauncher() { terminateAll(); }

  std::vector<pid_t> pids_;
  std::vector<std::string> names_;
  std::atomic<bool> running_{true};
};

}  // namespace custom_bot_cpp

#endif  // CUSTOM_BOT_CPP__PROCESS_LAUNCHER_HPP_
