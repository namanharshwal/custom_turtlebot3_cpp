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
// File:                 teleop_key_hold.hpp
// License:              Apache License 2.0 (c) 2026 Naman Harshwal
// ==============================================================================

#ifndef CUSTOM_BOT_CPP__TELEOP_KEY_HOLD_HPP_
#define CUSTOM_BOT_CPP__TELEOP_KEY_HOLD_HPP_

#include <termios.h>
#include <memory>
#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace custom_bot_cpp
{

class TeleopKeyHold : public rclcpp::Node
{
public:
  explicit TeleopKeyHold(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~TeleopKeyHold();

  void run();

private:
  void initTerminal();
  void restoreTerminal();
  char getKey(int timeout_ms);
  void publishTwist(double lin, double ang);
  void stopRobot();
  void printStatus(const std::string & label);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  double linear_speed_{0.20};
  double angular_speed_{1.50};

  static constexpr double LINEAR_MAX{0.50};
  static constexpr double LINEAR_MIN{0.10};
  static constexpr double ANGULAR_MAX{2.84};
  static constexpr double ANGULAR_MIN{0.50};

  struct termios orig_termios_;
  bool termios_saved_{false};
};

}  // namespace custom_bot_cpp

#endif  // CUSTOM_BOT_CPP__TELEOP_KEY_HOLD_HPP_
