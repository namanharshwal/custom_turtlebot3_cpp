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
#include "sensor_msgs/msg/range.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace custom_bot_cpp
{

enum class ExplorerState
{
  FRONTIER_SEEK,
  WALL_FOLLOW,
  CURVE,
  ROTATE,
  REVERSE,
  RETURN_TO_START
};

struct FrontierPoint
{
  double x{0.0};
  double y{0.0};
};

struct FrontierCluster
{
  double centroid_x{0.0};
  double centroid_y{0.0};
  size_t size{0};
  double distance{0.0};
  double score{0.0};
};

struct BlacklistedTarget
{
  double x{0.0};
  double y{0.0};
  double added_time{0.0};
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
  void tofCallback(const sensor_msgs::msg::Range::SharedPtr msg);
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void controlLoop();
  void updateRobotPose();
  void findFrontiers();
  void publishMarkers();
  double sectorMin(const std::vector<float> & ranges, float angle_min, float angle_inc,
                   double deg_start, double deg_end);
  double findBestClearanceAngle(const std::vector<float> & ranges, float angle_min, float angle_inc);
  bool isPathObstructed(double x0, double y0, double x1, double y1) const;

  // Configuration Parameters
  double linear_speed_{0.12};
  double angular_speed_{0.35};
  double obstacle_dist_{0.40};
  double critical_dist_{0.25};
  double clear_dist_{0.48};
  double wall_follow_dist_{0.38};
  double control_hz_{10.0};

  // FSM State Variables
  ExplorerState state_{ExplorerState::FRONTIER_SEEK};
  double turn_direction_{1.0};
  double rotate_start_time_{0.0};
  double reverse_start_time_{0.0};
  double curve_start_time_{0.0};
  double forward_motion_start_time_{0.0};
  double last_state_change_{0.0};
  uint64_t log_count_{0};

  // Laser Scan Cache
  sensor_msgs::msg::LaserScan::SharedPtr scan_{nullptr};
  double scan_time_{0.0};
  uint64_t scan_count_{0};

  // SLAM Occupancy Grid Map & Frontier Exploration
  nav_msgs::msg::OccupancyGrid::SharedPtr map_{nullptr};
  double map_time_{0.0};
  bool has_map_{false};
  bool has_target_frontier_{false};
  bool all_frontiers_explored_{false};
  FrontierCluster target_frontier_;
  std::vector<FrontierCluster> frontier_clusters_;
  std::vector<BlacklistedTarget> blacklist_;
  double target_lock_time_{0.0};
  double last_target_dist_{999.0};
  double last_frontier_search_time_{0.0};

  // Robot Pose Tracking (Map frame prioritized, Odom fallback)
  double curr_x_{0.0};
  double curr_y_{0.0};
  double curr_yaw_{0.0};
  bool has_pose_{false};

  // Initial Start Pose & Return to Home Navigation
  double initial_x_{0.0};
  double initial_y_{0.0};
  double initial_yaw_{0.0};
  bool has_initial_pose_{false};
  bool reached_start_pos_{false};
  bool reached_initial_heading_{false};
  std::vector<std::pair<double, double>> breadcrumbs_;
  uint32_t no_frontiers_count_{0};
  double exploration_start_time_{0.0};
  double total_distance_traveled_{0.0};
  double prev_pose_x_{0.0};
  double prev_pose_y_{0.0};
  bool first_pose_{true};

  // Odometry Stall Monitoring (Physical Guardian)
  double last_stall_check_time_{0.0};
  double last_stall_x_{0.0};
  double last_stall_y_{0.0};
  uint32_t stall_count_{0};

  // Post-avoidance forward commitment & Rotational stall monitoring
  double commit_forward_until_{0.0};
  double last_rot_stall_check_time_{0.0};
  double last_rot_stall_yaw_{0.0};

  // ToF Range Cache & Moving Median Filter
  double tof_range_{2.00};
  double tof_time_{0.0};
  std::vector<double> tof_history_;

  // TF2 Buffer & Listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ROS 2 Interfaces
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr tof_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace custom_bot_cpp

#endif  // CUSTOM_BOT_CPP__SIMPLE_EXPLORER_HPP_
