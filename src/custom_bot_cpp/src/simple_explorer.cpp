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
// File:                 simple_explorer.cpp
// License:              Apache License 2.0 (c) 2026 Naman Harshwal
//
// ==============================================================================
// 🧠 MATHEMATICAL FOUNDATIONS & DERIVATIONS IMPLEMENTED HEREIN:
// ==============================================================================
// 1. LIDAR SECTOR PARTITIONING & RAY SAMPLING:
//    - Front:       [-18°, +18°] (covers direct path > robot width 0.178 m)
//    - Diagonals:   [+18°, +50°] and [-50°, -18°] (detects oblique corners)
//    - Lateral:     [+50°, +110°] and [-110°, -50°] (perimeter wall tracking)
//    - Index calc:  idx = floor((rad - angle_min) / angle_increment)
//
// 2. DIFFERENTIAL DRIVE ESCAPE KINEMATICS:
//    - In-place pivot: ω = 0.40 rad/s (22.92°/s)
//    - 70° Escape turn duration: t = 1.2217 rad / 0.40 rad/s = 3.05 s
//
// 3. LAYER 2 ODOMETRY PHYSICAL STALL RECOVERY:
//    - Over Δt = 1.2 s, expected travel = 0.05 * 1.2 = 0.060 m
//    - If Δs_meas < 0.025 m (< 41.7%), robot is physically wedged
//    - Escape: Reverse at -0.05 m/s for 1.5 s (retracts -7.5 cm from obstacles).
//
// 4. SINUSOIDAL WANDER EXPLORATION COVERAGE:
//    - ω(t) = 0.08 * sin(0.30 * t) rad/s (sweeps 61° arc over 20.94 s wave period).
// ==============================================================================

#include "custom_bot_cpp/simple_explorer.hpp"

#include <cmath>
#include <algorithm>

namespace custom_bot_cpp
{

SimpleExplorer::SimpleExplorer(const rclcpp::NodeOptions & options)
: Node("simple_explorer", options)
{
  declareParameters();

  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(),
    std::bind(&SimpleExplorer::scanCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    std::bind(&SimpleExplorer::odomCallback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / control_hz_)),
    std::bind(&SimpleExplorer::controlLoop, this));

  RCLCPP_INFO(this->get_logger(),
              "SimpleExplorer C++ active: speed=%.2f m/s, turn=%.2f rad/s, obs=%.2fm, crit=%.2fm",
              linear_speed_, angular_speed_, obstacle_dist_, critical_dist_);
}

void SimpleExplorer::declareParameters()
{
  this->declare_parameter<double>("linear_speed", 0.05);
  this->declare_parameter<double>("angular_speed", 0.25);
  this->declare_parameter<double>("obstacle_dist", 0.45);
  this->declare_parameter<double>("critical_dist", 0.30);
  this->declare_parameter<double>("clear_dist", 0.38);
  this->declare_parameter<double>("wall_follow_dist", 0.40);
  this->declare_parameter<double>("control_hz", 10.0);

  linear_speed_     = this->get_parameter("linear_speed").as_double();
  angular_speed_    = this->get_parameter("angular_speed").as_double();
  obstacle_dist_    = this->get_parameter("obstacle_dist").as_double();
  critical_dist_    = this->get_parameter("critical_dist").as_double();
  clear_dist_       = this->get_parameter("clear_dist").as_double();
  wall_follow_dist_ = this->get_parameter("wall_follow_dist").as_double();
  control_hz_       = this->get_parameter("control_hz").as_double();
}

void SimpleExplorer::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  curr_x_ = msg->pose.pose.position.x;
  curr_y_ = msg->pose.pose.position.y;
}

void SimpleExplorer::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  scan_ = msg;
  scan_time_ = this->now().seconds();
  scan_count_++;
}

double SimpleExplorer::sectorMin(const std::vector<float> & ranges, float angle_min, float angle_inc,
                                 double deg_start, double deg_end)
{
  double rad_start = deg_start * (M_PI / 180.0);
  double rad_end   = deg_end   * (M_PI / 180.0);

  int idx_start = static_cast<int>((rad_start - angle_min) / angle_inc);
  int idx_end   = static_cast<int>((rad_end   - angle_min) / angle_inc);

  if (idx_start > idx_end) {
    std::swap(idx_start, idx_end);
  }

  idx_start = std::max(0, idx_start);
  idx_end   = std::min(static_cast<int>(ranges.size()) - 1, idx_end);

  double min_val = 9.99;
  for (int i = idx_start; i <= idx_end; ++i) {
    float r = ranges[i];
    if (r > 0.12f && r < 8.0f && !std::isinf(r) && !std::isnan(r)) {
      if (r < min_val) {
        min_val = r;
      }
    }
  }
  return min_val;
}

void SimpleExplorer::controlLoop()
{
  geometry_msgs::msg::Twist cmd;
  double now = this->now().seconds();

  if (!scan_ || (now - scan_time_) > 2.0) {
    cmd.angular.z = angular_speed_ * 0.6;
    cmd_pub_->publish(cmd);
    return;
  }

  const auto & ranges = scan_->ranges;
  if (ranges.size() < 20) {
    cmd.angular.z = angular_speed_ * 0.6;
    cmd_pub_->publish(cmd);
    return;
  }

  float a_min = scan_->angle_min;
  float a_inc = scan_->angle_increment;

  double front       = sectorMin(ranges, a_min, a_inc, -18.0,  18.0);
  double front_left  = sectorMin(ranges, a_min, a_inc,  18.0,  50.0);
  double front_right = sectorMin(ranges, a_min, a_inc, -50.0, -18.0);
  double left        = sectorMin(ranges, a_min, a_inc,  50.0, 110.0);
  double right       = sectorMin(ranges, a_min, a_inc, -110.0, -50.0);

  // 0. PHYSICAL STALL DETECTION (Layer 2)
  if (state_ == ExplorerState::FORWARD || state_ == ExplorerState::CURVE) {
    if ((now - last_stall_check_time_) >= 1.2) {
      double dist = std::hypot(curr_x_ - last_stall_x_, curr_y_ - last_stall_y_);
      last_stall_check_time_ = now;
      last_stall_x_ = curr_x_;
      last_stall_y_ = curr_y_;

      if (dist < 0.025) {
        stall_count_++;
        RCLCPP_WARN(this->get_logger(),
                    "Physical stall detected (%.1fcm in 1.2s)! Reversing #%u",
                    dist * 100.0, stall_count_);
        state_ = ExplorerState::REVERSE;
        reverse_start_time_ = now;
        turn_direction_ = (left > right) ? 1.0 : -1.0;
        cmd.linear.x = -0.05;
        cmd.angular.z = 0.0;
        cmd_pub_->publish(cmd);
        return;
      }
    }
  }

  // 1. REVERSE STATE
  if (state_ == ExplorerState::REVERSE) {
    if ((now - reverse_start_time_) < 1.5) {
      cmd.linear.x = -0.05;
      cmd.angular.z = 0.0;
    } else {
      state_ = ExplorerState::ROTATE;
      rotate_start_time_ = now;
      last_stall_check_time_ = now;
      last_stall_x_ = curr_x_;
      last_stall_y_ = curr_y_;
      cmd.linear.x = 0.0;
      cmd.angular.z = turn_direction_ * 0.40;
    }
  }
  // 2. ROTATE STATE
  else if (state_ == ExplorerState::ROTATE) {
    double rotate_dur = now - rotate_start_time_;
    if (rotate_dur > 4.5) {
      state_ = ExplorerState::REVERSE;
      reverse_start_time_ = now;
      cmd.linear.x = -0.05;
      cmd.angular.z = 0.0;
    } else if (rotate_dur >= 1.5 && front >= clear_dist_ && front_left >= 0.22 && front_right >= 0.22) {
      state_ = ExplorerState::FORWARD;
      last_state_change_ = now;
      last_stall_check_time_ = now;
      last_stall_x_ = curr_x_;
      last_stall_y_ = curr_y_;
      cmd.linear.x = linear_speed_;
      cmd.angular.z = 0.0;
    } else {
      cmd.linear.x = 0.0;
      cmd.angular.z = turn_direction_ * 0.40;
    }
  }
  // 3. CRITICAL DISTANCE CHECK
  else if (front < critical_dist_) {
    state_ = ExplorerState::ROTATE;
    rotate_start_time_ = now;
    if (right < left || front_right < front_left) {
      turn_direction_ = 1.0;   // Left CCW
    } else {
      turn_direction_ = -1.0;  // Right CW
    }
    cmd.linear.x = 0.0;
    cmd.angular.z = turn_direction_ * 0.40;
  }
  // 4. PREDICTIVE CURVE
  else if (front < obstacle_dist_ || front_left < 0.25 || front_right < 0.25) {
    state_ = ExplorerState::CURVE;
    cmd.linear.x = linear_speed_ * 0.80;
    if (front_left < front_right || left < right) {
      cmd.angular.z = -angular_speed_;
      turn_direction_ = -1.0;
    } else {
      cmd.angular.z = angular_speed_;
      turn_direction_ = 1.0;
    }
  }
  // 5. FORWARD CRUISE & SYSTEMATIC WALL FOLLOWING
  else {
    state_ = ExplorerState::FORWARD;
    cmd.linear.x = linear_speed_;

    if (right < (wall_follow_dist_ - 0.08)) {
      cmd.angular.z = angular_speed_ * 0.65;
    } else if (right > (wall_follow_dist_ + 0.10) && right < 0.80) {
      cmd.angular.z = -angular_speed_ * 0.50;
    } else if (left < (wall_follow_dist_ - 0.08)) {
      cmd.angular.z = -angular_speed_ * 0.65;
    } else {
      // Sinusoidal wander in open room
      cmd.angular.z = 0.08 * std::sin(now * 0.3);
    }
  }

  log_count_++;
  if (log_count_ % 20 == 0) {
    const char * st_str = (state_ == ExplorerState::FORWARD) ? "FORWARD" :
                          (state_ == ExplorerState::CURVE)   ? "CURVE" :
                          (state_ == ExplorerState::ROTATE)  ? "ROTATE" : "REVERSE";
    RCLCPP_INFO(this->get_logger(),
                "[%s] sc=%lu front=%.2fm L=%.2fm R=%.2fm -> lin=%.2f ang=%.2f",
                st_str, scan_count_, front, left, right, cmd.linear.x, cmd.angular.z);
  }

  cmd_pub_->publish(cmd);
}

}  // namespace custom_bot_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<custom_bot_cpp::SimpleExplorer>();
  rclcpp::spin(node);

  // Guarantee motor halt upon shutdown
  geometry_msgs::msg::Twist stop_cmd;
  auto pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  pub->publish(stop_cmd);

  rclcpp::shutdown();
  return 0;
}
