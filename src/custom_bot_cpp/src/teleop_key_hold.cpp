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
// File:                 teleop_key_hold.cpp
// License:              Apache License 2.0 (c) 2026 Naman Harshwal
//
// ==============================================================================
// 🧠 MATHEMATICAL FOUNDATIONS & CONTROL SPECIFICATIONS:
// ==============================================================================
// 1. POSIX NON-BLOCKING RAW MODE:
//    - poll() on STDIN_FILENO with 100 ms timeout (Deadman switch)
//    - Stop latency: T_latency = 100 ms + 50 ms serial bridge = 150 ms
//    - Stopping distance at crawl: s = 0.20 * 0.150 = 30.0 mm
//
// 2. GEOMETRIC VELOCITY SCALING:
//    - Acceleration step: v_{k+1} = min(v_max, v_k * 1.10)
//    - Deceleration step: v_{k+1} = max(v_min, v_k * 0.90)
// ==============================================================================

#include "custom_bot_cpp/teleop_key_hold.hpp"

#include <unistd.h>
#include <poll.h>
#include <iostream>
#include <iomanip>
#include <cmath>

namespace custom_bot_cpp
{

static const char * BANNER =
  "+-------------------------------------------------------+\n"
  "|  custom_bot_cpp Teleop  --  HOLD=move  RELEASE=STOP   |\n"
  "|  Creator: Naman Harshwal (namanharshwal@gmail.com)    |\n"
  "+-------------------------------------------------------+\n"
  "|                                                       |\n"
  "|        u    i    o     q = MAX speed  instantly       |\n"
  "|        j    k    l     z = MIN speed  instantly       |\n"
  "|        m    ,    .     w/x = linear  +/-10%           |\n"
  "|                        e/c = angular +/-10%           |\n"
  "|  k = EMERGENCY STOP                                   |\n"
  "|  CTRL-C = Quit & Halt Robot                           |\n"
  "+-------------------------------------------------------+\n";

TeleopKeyHold::TeleopKeyHold(const rclcpp::NodeOptions & options)
: Node("teleop_key_hold", options)
{
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  initTerminal();
}

TeleopKeyHold::~TeleopKeyHold()
{
  stopRobot();
  restoreTerminal();
}

void TeleopKeyHold::initTerminal()
{
  if (tcgetattr(STDIN_FILENO, &orig_termios_) == 0) {
    termios_saved_ = true;
    struct termios raw = orig_termios_;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN]  = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
  }
}

void TeleopKeyHold::restoreTerminal()
{
  if (termios_saved_) {
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
    termios_saved_ = false;
  }
}

char TeleopKeyHold::getKey(int timeout_ms)
{
  struct pollfd pfd;
  pfd.fd = STDIN_FILENO;
  pfd.events = POLLIN;

  int ret = poll(&pfd, 1, timeout_ms);
  if (ret > 0 && (pfd.revents & POLLIN)) {
    char c = 0;
    if (read(STDIN_FILENO, &c, 1) > 0) {
      if (c == '\x1b') {
        // Discard escape sequence
        char esc[2];
        if (read(STDIN_FILENO, esc, 2) > 0) {
          return 0;
        }
      }
      return c;
    }
  }
  return 0;
}

void TeleopKeyHold::publishTwist(double lin, double ang)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x  = linear_speed_ * lin;
  msg.angular.z = angular_speed_ * ang;
  cmd_pub_->publish(msg);
}

void TeleopKeyHold::stopRobot()
{
  geometry_msgs::msg::Twist msg;
  cmd_pub_->publish(msg);
}

void TeleopKeyHold::printStatus(const std::string & label)
{
  std::cout << "\r  " << std::left << std::setw(28) << label
            << " lin=" << std::fixed << std::setprecision(3) << linear_speed_ << " m/s"
            << " ang=" << std::fixed << std::setprecision(3) << angular_speed_ << " rad/s   "
            << std::flush;
}

void TeleopKeyHold::run()
{
  std::cout << BANNER << std::endl;
  printStatus("Ready -- press a key");
  std::cout << std::endl;

  std::map<char, std::pair<double, double>> key_bindings = {
    {'i', { 1.0,  0.0}},
    {',', {-1.0,  0.0}},
    {'j', { 0.0,  1.0}},
    {'l', { 0.0, -1.0}},
    {'u', { 1.0,  1.0}},
    {'o', { 1.0, -1.0}},
    {'m', {-1.0,  1.0}},
    {'.', {-1.0, -1.0}},
    {'k', { 0.0,  0.0}}
  };

  std::map<char, std::string> labels = {
    {'i', "Forward"},       {',', "Backward"},
    {'j', "Rotate Left"},   {'l', "Rotate Right"},
    {'u', "Diag Fwd-Left"}, {'o', "Diag Fwd-Right"},
    {'m', "Diag Rev-Left"}, {'.', "Diag Rev-Right"},
    {'k', "FORCE STOP"}
  };

  while (rclcpp::ok()) {
    char key = getKey(100);  // 100 ms Deadman timeout

    if (key == '\x03' || key == '\x04') {
      break;
    }

    if (key_bindings.find(key) != key_bindings.end()) {
      auto [lin, ang] = key_bindings[key];
      publishTwist(lin, ang);
      printStatus("[" + std::string(1, key) + "] " + labels[key]);
    } else if (key == 'q') {
      linear_speed_  = LINEAR_MAX;
      angular_speed_ = ANGULAR_MAX;
      printStatus("MAX speed");
    } else if (key == 'z') {
      linear_speed_  = LINEAR_MIN;
      angular_speed_ = ANGULAR_MIN;
      printStatus("MIN speed");
    } else if (key == 'w') {
      linear_speed_ = std::min(LINEAR_MAX, std::round(linear_speed_ * 1.10 * 1000.0) / 1000.0);
      printStatus("Linear +10%");
    } else if (key == 'x') {
      linear_speed_ = std::max(LINEAR_MIN, std::round(linear_speed_ * 0.90 * 1000.0) / 1000.0);
      printStatus("Linear -10%");
    } else if (key == 'e') {
      angular_speed_ = std::min(ANGULAR_MAX, std::round(angular_speed_ * 1.10 * 1000.0) / 1000.0);
      printStatus("Angular +10%");
    } else if (key == 'c') {
      angular_speed_ = std::max(ANGULAR_MIN, std::round(angular_speed_ * 0.90 * 1000.0) / 1000.0);
      printStatus("Angular -10%");
    } else if (key == 0) {
      stopRobot();
    }
  }

  stopRobot();
  std::cout << "\n\n  Robot stopped. Teleop exited cleanly.\n" << std::endl;
}

}  // namespace custom_bot_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<custom_bot_cpp::TeleopKeyHold>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
