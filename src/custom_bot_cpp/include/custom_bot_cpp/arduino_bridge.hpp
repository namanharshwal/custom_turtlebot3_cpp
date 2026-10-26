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
// File:                 arduino_bridge.hpp
// License:              Apache License 2.0 (c) 2026 Naman Harshwal
// ==============================================================================

#ifndef CUSTOM_BOT_CPP__ARDUINO_BRIDGE_HPP_
#define CUSTOM_BOT_CPP__ARDUINO_BRIDGE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace custom_bot_cpp
{

class ArduinoBridge : public rclcpp::Node
{
public:
  explicit ArduinoBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~ArduinoBridge();

private:
  // ROS 2 Initialization & Configuration
  void declareParameters();
  void initSerial();
  void closeSerial();

  // Callbacks & Threads
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void serialReaderLoop();
  void watchdogKeepaliveLoop();
  void parseSerialLine(const std::string & line);
  void updateOdometry(long raw_right_ticks, long raw_left_ticks,
                      double qw, double qx, double qy, double qz,
                      double ax, double ay, double az,
                      double gx, double gy, double gz);
  void sendMotorPWM(int left_pwm, int right_pwm);

  // Serial device file descriptor & thread management
  int serial_fd_{-1};
  std::thread serial_thread_;
  std::thread keepalive_thread_;
  std::atomic<bool> running_{false};
  std::mutex serial_write_mutex_;
  std::mutex keepalive_mutex_;

  // ROS 2 Communication Interfaces
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Robot Configuration Parameters
  std::string serial_port_;
  int baud_rate_;
  double wheel_diameter_;
  double wheel_radius_;
  double wheel_circumference_;
  double wheel_separation_;
  int ticks_per_rev_left_;
  int ticks_per_rev_right_;
  double meters_per_tick_left_;
  double meters_per_tick_right_;
  double max_speed_;
  bool publish_tf_;
  std::string base_frame_;
  std::string odom_frame_;

  // Motor Control & Anti-Stall Tuning Constants (Identical to proven Python implementation)
  const int MIN_PWM{68};
  const double DEADBAND{0.005};
  const int LEFT_WHEEL_SIGN{1};
  const int RIGHT_WHEEL_SIGN{1};

  // Motor State Variables
  bool was_stopped_{true};
  double kick_until_{0.0};
  int stall_level_{0};
  double last_stall_check_{0.0};
  long last_check_left_ticks_{0};
  long last_check_right_ticks_{0};

  // Watchdog Keepalive State
  double last_cmd_time_{0.0};
  int last_l_pwm_{0};
  int last_r_pwm_{0};
  uint64_t cmd_count_{0};
  uint64_t debug_msg_count_{0};

  // Odometry State
  double x_{0.0};
  double y_{0.0};
  double theta_{0.0};
  double initial_yaw_{0.0};
  double theta_prev_raw_{0.0};
  bool has_initial_yaw_{false};

  long left_ticks_{0};
  long right_ticks_{0};
  long prev_left_ticks_{0};
  long prev_right_ticks_{0};

  double left_wheel_angle_{0.0};
  double right_wheel_angle_{0.0};
};

}  // namespace custom_bot_cpp

#endif  // CUSTOM_BOT_CPP__ARDUINO_BRIDGE_HPP_
