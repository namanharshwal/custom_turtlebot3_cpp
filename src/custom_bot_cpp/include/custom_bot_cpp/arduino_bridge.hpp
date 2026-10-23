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
#include "tf2/LinearMath/Quaternion.h"

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

  // Callbacks
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void serialReaderLoop();
  void parseSerialLine(const std::string & line);
  void updateOdometry(long l_enc, long r_enc, double qw, double qx, double qy, double qz,
                      double ax, double ay, double az, double gx, double gy, double gz);
  void sendMotorPWM(int left_pwm, int right_pwm);
  int velocityToPWM(double vel);
  void sendHeartbeat();

  // Mathematical utility functions
  static inline double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  // Serial device file descriptor & thread management
  int serial_fd_{-1};
  std::thread serial_thread_;
  std::atomic<bool> running_{false};
  std::mutex serial_write_mutex_;

  // ROS 2 Communication Interfaces
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  // Robot Configuration Parameters
  std::string serial_port_;
  int baud_rate_;
  double wheel_diameter_;
  double wheel_radius_;
  double wheel_circumference_;
  double wheel_separation_;
  int cpr_left_;
  int cpr_right_;
  double meters_per_tick_left_;
  double meters_per_tick_right_;
  double max_speed_;
  bool publish_tf_;
  std::string base_frame_;
  std::string odom_frame_;
  int min_pwm_;
  int max_pwm_;

  // Odometry State
  double x_{0.0};
  double y_{0.0};
  double theta_{0.0};
  double theta_prev_raw_{0.0};
  bool first_heading_received_{false};

  long prev_l_enc_{0};
  long prev_r_enc_{0};
  bool first_enc_received_{false};
  double last_odom_time_{0.0};

  double left_wheel_angle_{0.0};
  double right_wheel_angle_{0.0};

  // Layer 1 Anti-Stall State
  double last_stall_check_time_{0.0};
  long stall_check_ticks_{0};
  bool stall_boost_active_{false};
  double stall_boost_end_time_{0.0};
  double last_cmd_vel_linear_{0.0};
  double last_cmd_time_{0.0};
};

}  // namespace custom_bot_cpp

#endif  // CUSTOM_BOT_CPP__ARDUINO_BRIDGE_HPP_
