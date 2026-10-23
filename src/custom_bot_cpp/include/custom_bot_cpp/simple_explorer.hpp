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
// File:                 simple_explorer.hpp
// License:              Apache License 2.0 (c) 2026 Naman Harshwal
// ==============================================================================

#ifndef CUSTOM_BOT_CPP__SIMPLE_EXPLORER_HPP_
#define CUSTOM_BOT_CPP__SIMPLE_EXPLORER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace custom_bot_cpp
{

enum class ExplorerState
{
  FORWARD,
  CURVE,
  ROTATE,
  REVERSE
};

class SimpleExplorer : public rclcpp::Node
{
public:
  explicit SimpleExplorer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~SimpleExplorer() = default;

private:
  void declareParameters();
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void controlLoop();
  double sectorMin(const std::vector<float> & ranges, float angle_min, float angle_inc,
                   double deg_start, double deg_end);

  // Configuration Parameters
  double linear_speed_;
  double angular_speed_;
  double obstacle_dist_;
  double critical_dist_;
  double clear_dist_;
  double wall_follow_dist_;
  double control_hz_;

  // FSM State Variables
  ExplorerState state_{ExplorerState::FORWARD};
  double turn_direction_{1.0};
  double rotate_start_time_{0.0};
  double reverse_start_time_{0.0};
  double last_state_change_{0.0};
  uint64_t log_count_{0};

  // Laser Scan Cache
  sensor_msgs::msg::LaserScan::SharedPtr scan_{nullptr};
  double scan_time_{0.0};
  uint64_t scan_count_{0};

  // ROS 2 Interfaces
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Odometry Stall Monitoring
  double curr_x_{0.0};
  double curr_y_{0.0};
  double last_stall_check_time_{0.0};
  double last_stall_x_{0.0};
  double last_stall_y_{0.0};
  uint32_t stall_count_{0};
};

}  // namespace custom_bot_cpp

#endif  // CUSTOM_BOT_CPP__SIMPLE_EXPLORER_HPP_
