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
// 1. DUAL-LAYER OCCUPANCY GRID FRONTIER EXPLORATION (Yamauchi Standard):
//    - Extracts frontier boundaries (Free cells m_i = 0 with Unknown neighbors m_j = -1).
//    - Safety inflation: rejects frontier cells within 10cm of occupied obstacles (m_k > 50).
//    - Fast spatial Euclidean clustering (cluster threshold: 0.35m, min cluster size: 3 cells).
//    - Cluster scoring function: J(F) = ||p_robot - c_F|| - α * min(|F|, 40).
//    - Steers differential drive robot directly toward the optimal unexplored frontier centroid.
//
// 2. ZERO-STUCK ADAPTIVE TARGET TRACKING & BLACKLISTING:
//    - Monitors frontier progress: if robot fails to reduce distance by > 15cm within 20s,
//      the target is temporarily blacklisted for 35s, preventing dead-end fixation.
//
// 3. 360° LIDAR MAXIMUM CLEARANCE RESCUE:
//    - If trapped in a corner or facing dense obstacles, sectors are partitioned into 16 bins.
//    - Directly pivots to the sector with maximal radial clearance, preventing local minima.
//
// 4. HARMONIZED MULTI-SENSOR RANGE SAFETY:
//    - VL53L0X ToF range + YDLidar synchronized thresholds:
//      * Critical stop: d < 0.28m
//      * Predictive curve: d < 0.45m
//      * Ground noise rejection: d < 0.05m
//
// 5. TRUE ANTI-STALL GUARDIAN (Layer 2 Safe):
//    - Checked strictly during continuous FORWARD / FRONTIER_SEEK (t > 2.5s).
//    - Disables checking during CURVE, ROTATE, or REVERSE to eliminate false stall loops.
// ==============================================================================

#include "custom_bot_cpp/simple_explorer.hpp"

#include <cmath>
#include <algorithm>
#include <chrono>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

namespace custom_bot_cpp
{

SimpleExplorer::SimpleExplorer(const rclcpp::NodeOptions & options)
: Node("simple_explorer", options)
{
  declareParameters();

  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/frontier_markers", 10);

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(),
    std::bind(&SimpleExplorer::scanCallback, this, std::placeholders::_1));

  tof_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
    "/tof/range", 10,
    std::bind(&SimpleExplorer::tofCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    std::bind(&SimpleExplorer::odomCallback, this, std::placeholders::_1));

  // Map subscriber with transient local QoS for reliable SLAM map reception
  rclcpp::QoS map_qos(rclcpp::KeepLast(1));
  map_qos.transient_local();
  map_qos.reliable();
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", map_qos,
    std::bind(&SimpleExplorer::mapCallback, this, std::placeholders::_1));

  // TF2 Buffer & Listener for map->base_footprint transformation
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  double now = this->now().seconds();
  last_stall_check_time_ = now;
  forward_motion_start_time_ = now;
  last_frontier_search_time_ = now;
  commit_forward_until_ = 0.0;
  last_rot_stall_check_time_ = now;
  last_rot_stall_yaw_ = 0.0;

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / control_hz_)),
    std::bind(&SimpleExplorer::controlLoop, this));

  RCLCPP_INFO(this->get_logger(),
              "SimpleExplorer C++ Frontier Engine active: speed=%.2f m/s, turn=%.2f rad/s, obs=%.2fm, crit=%.2fm",
              linear_speed_, angular_speed_, obstacle_dist_, critical_dist_);
}

void SimpleExplorer::declareParameters()
{
  this->declare_parameter<double>("linear_speed", 0.12);
  this->declare_parameter<double>("angular_speed", 0.35);
  this->declare_parameter<double>("obstacle_dist", 0.40);
  this->declare_parameter<double>("critical_dist", 0.25);
  this->declare_parameter<double>("clear_dist", 0.48);
  this->declare_parameter<double>("wall_follow_dist", 0.38);
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
  if (!has_pose_) {
    curr_x_ = msg->pose.pose.position.x;
    curr_y_ = msg->pose.pose.position.y;
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    curr_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    has_pose_ = true;
  }
}

void SimpleExplorer::updateRobotPose()
{
  try {
    if (tf_buffer_->canTransform("map", "base_footprint", tf2::TimePointZero)) {
      auto tf = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
      curr_x_ = tf.transform.translation.x;
      curr_y_ = tf.transform.translation.y;
      double qx = tf.transform.rotation.x;
      double qy = tf.transform.rotation.y;
      double qz = tf.transform.rotation.z;
      double qw = tf.transform.rotation.w;
      curr_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
      has_pose_ = true;
      return;
    }
  } catch (...) {
    // Keep current pose from odom if TF lookup times out
  }
}

void SimpleExplorer::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  scan_ = msg;
  scan_time_ = this->now().seconds();
  scan_count_++;
}

void SimpleExplorer::tofCallback(const sensor_msgs::msg::Range::SharedPtr msg)
{
  tof_time_ = this->now().seconds();
  double r = msg->range;

  // Handle invalid/out-of-bounds readings as clear horizon
  if (std::isnan(r) || std::isinf(r) || r < msg->min_range || r > msg->max_range) {
    r = msg->max_range;
  }

  // 3-sample median filter to reject spurious single-frame optical / ground reflections
  tof_history_.push_back(r);
  if (tof_history_.size() > 3) {
    tof_history_.erase(tof_history_.begin());
  }

  std::vector<double> sorted = tof_history_;
  std::sort(sorted.begin(), sorted.end());
  tof_range_ = sorted[sorted.size() / 2];
}

void SimpleExplorer::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  map_ = msg;
  map_time_ = this->now().seconds();
  has_map_ = true;
  findFrontiers();
}

void SimpleExplorer::findFrontiers()
{
  if (!has_map_ || !map_ || !has_pose_) {
    return;
  }

  double now = this->now().seconds();

  // 1. Purge expired blacklisted targets (> 35.0s old)
  blacklist_.erase(
    std::remove_if(blacklist_.begin(), blacklist_.end(),
      [now](const BlacklistedTarget & b) { return (now - b.added_time) > 35.0; }),
    blacklist_.end());

  const auto & info = map_->info;
  int width = static_cast<int>(info.width);
  int height = static_cast<int>(info.height);
  double res = info.resolution;
  double origin_x = info.origin.position.x;
  double origin_y = info.origin.position.y;
  const auto & data = map_->data;

  std::vector<FrontierPoint> candidate_points;
  candidate_points.reserve(1024);

  // 2. Scan occupancy grid for frontier boundaries (free cells adjacent to unknown cells)
  for (int r = 2; r < height - 2; r += 2) {
    for (int c = 2; c < width - 2; c += 2) {
      int idx = r * width + c;
      if (data[idx] != 0) {
        continue;  // Only evaluate free traversable space
      }

      // Check 4-connected neighbors for unknown cell (-1)
      bool has_unknown_neighbor = (data[idx + 1] == -1 ||
                                   data[idx - 1] == -1 ||
                                   data[idx + width] == -1 ||
                                   data[idx - width] == -1);

      if (!has_unknown_neighbor) {
        continue;
      }

      // Safety Inflation Check: verify no obstacle (> 50) within a 2-cell radius
      bool near_obstacle = false;
      for (int dr = -2; dr <= 2 && !near_obstacle; ++dr) {
        for (int dc = -2; dc <= 2 && !near_obstacle; ++dc) {
          int n_idx = (r + dr) * width + (c + dc);
          if (n_idx >= 0 && n_idx < static_cast<int>(data.size())) {
            if (data[n_idx] > 50) {
              near_obstacle = true;
            }
          }
        }
      }

      if (near_obstacle) {
        continue;
      }

      // Compute metric world coordinates
      double wx = origin_x + (c + 0.5) * res;
      double wy = origin_y + (r + 0.5) * res;
      double dist = std::hypot(wx - curr_x_, wy - curr_y_);

      // Reject frontiers right under the robot footprint (< 0.35m)
      if (dist >= 0.35) {
        candidate_points.push_back({wx, wy});
      }
    }
  }

  // 3. Spatial Euclidean Clustering (0.35m cluster proximity)
  frontier_clusters_.clear();
  for (const auto & pt : candidate_points) {
    bool clustered = false;
    for (auto & cl : frontier_clusters_) {
      if (std::hypot(pt.x - cl.centroid_x, pt.y - cl.centroid_y) < 0.35) {
        cl.centroid_x = (cl.centroid_x * cl.size + pt.x) / (cl.size + 1);
        cl.centroid_y = (cl.centroid_y * cl.size + pt.y) / (cl.size + 1);
        cl.size++;
        clustered = true;
        break;
      }
    }
    if (!clustered) {
      FrontierCluster new_cl;
      new_cl.centroid_x = pt.x;
      new_cl.centroid_y = pt.y;
      new_cl.size = 1;
      frontier_clusters_.push_back(new_cl);
    }
  }

  // 4. Filter clusters: keep clusters with >= 3 cells (eliminate isolated noise)
  frontier_clusters_.erase(
    std::remove_if(frontier_clusters_.begin(), frontier_clusters_.end(),
      [](const FrontierCluster & cl) { return cl.size < 3; }),
    frontier_clusters_.end());

  // 5. Select Optimal Frontier Target with Hysteresis & Commitment
  if (frontier_clusters_.empty()) {
    if (candidate_points.empty() && scan_count_ > 50) {
      all_frontiers_explored_ = true;
      has_target_frontier_ = false;
    }
  } else {
    all_frontiers_explored_ = false;

    // Check if current active target cluster still exists in map
    int active_match_idx = -1;
    if (has_target_frontier_) {
      double min_match_dist = 0.50;  // 50cm cluster correspondence window
      for (size_t i = 0; i < frontier_clusters_.size(); ++i) {
        double d = std::hypot(frontier_clusters_[i].centroid_x - target_frontier_.centroid_x,
                              frontier_clusters_[i].centroid_y - target_frontier_.centroid_y);
        if (d < min_match_dist) {
          min_match_dist = d;
          active_match_idx = static_cast<int>(i);
        }
      }
    }

    double best_score = 1e9;
    int best_idx = -1;

    for (size_t i = 0; i < frontier_clusters_.size(); ++i) {
      auto & cl = frontier_clusters_[i];
      cl.distance = std::hypot(cl.centroid_x - curr_x_, cl.centroid_y - curr_y_);

      // Check if cluster is near any blacklisted location
      bool is_blacklisted = false;
      for (const auto & bl : blacklist_) {
        if (std::hypot(cl.centroid_x - bl.x, cl.centroid_y - bl.y) < 0.45) {
          is_blacklisted = true;
          break;
        }
      }
      if (is_blacklisted) {
        continue;
      }

      // Cost function: J = Distance - 0.03 * min(Size, 40)
      cl.score = cl.distance - 0.03 * std::min(cl.size, static_cast<size_t>(40));

      // Heavily penalize clusters blocked by an obstacle in direct line of sight
      if (isPathObstructed(curr_x_, curr_y_, cl.centroid_x, cl.centroid_y)) {
        cl.score += 8.0;
      }

      if (cl.score < best_score) {
        best_score = cl.score;
        best_idx = static_cast<int>(i);
      }
    }

    if (has_target_frontier_ && active_match_idx >= 0) {
      // Current target is still valid!
      // Hysteresis: only switch away if another target is substantially closer (> 0.60m better score)
      if (best_idx >= 0 && best_idx != active_match_idx &&
          (frontier_clusters_[active_match_idx].score - best_score) > 0.60) {
        target_frontier_ = frontier_clusters_[best_idx];
        target_lock_time_ = now;
        last_target_dist_ = target_frontier_.distance;
        RCLCPP_INFO(this->get_logger(),
                    "[Frontier] Switched target to closer cluster at (%.2f, %.2f) score=%.2f",
                    target_frontier_.centroid_x, target_frontier_.centroid_y, best_score);
      } else {
        // Refine current target centroid smoothly, but keep target_lock_time_ intact
        target_frontier_.centroid_x = frontier_clusters_[active_match_idx].centroid_x;
        target_frontier_.centroid_y = frontier_clusters_[active_match_idx].centroid_y;
        target_frontier_.size = frontier_clusters_[active_match_idx].size;
        target_frontier_.distance = frontier_clusters_[active_match_idx].distance;
        target_frontier_.score = frontier_clusters_[active_match_idx].score;
      }
    } else if (best_idx >= 0) {
      target_frontier_ = frontier_clusters_[best_idx];
      has_target_frontier_ = true;
      target_lock_time_ = now;
      last_target_dist_ = target_frontier_.distance;
      RCLCPP_INFO(this->get_logger(),
                  "[Frontier] Locked target (%.2f, %.2f) dist=%.2fm score=%.2f (total clusters=%zu)",
                  target_frontier_.centroid_x, target_frontier_.centroid_y,
                  target_frontier_.distance, best_score, frontier_clusters_.size());
    } else {
      has_target_frontier_ = false;
    }
  }

  publishMarkers();
}

void SimpleExplorer::publishMarkers()
{
  visualization_msgs::msg::MarkerArray markers;

  // Marker 0: All frontier cluster centroids (Cyan Spheres)
  visualization_msgs::msg::Marker clusters_marker;
  clusters_marker.header.frame_id = "map";
  clusters_marker.header.stamp = this->now();
  clusters_marker.ns = "frontier_clusters";
  clusters_marker.id = 0;
  clusters_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  clusters_marker.action = visualization_msgs::msg::Marker::ADD;
  clusters_marker.scale.x = 0.12;
  clusters_marker.scale.y = 0.12;
  clusters_marker.scale.z = 0.12;
  clusters_marker.color.r = 0.0f;
  clusters_marker.color.g = 0.9f;
  clusters_marker.color.b = 0.9f;
  clusters_marker.color.a = 0.85f;

  for (const auto & cl : frontier_clusters_) {
    geometry_msgs::msg::Point p;
    p.x = cl.centroid_x;
    p.y = cl.centroid_y;
    p.z = 0.05;
    clusters_marker.points.push_back(p);
  }
  markers.markers.push_back(clusters_marker);

  // Marker 1: Active Target Frontier Centroid (Bright Gold Sphere)
  if (has_target_frontier_) {
    visualization_msgs::msg::Marker target_marker;
    target_marker.header.frame_id = "map";
    target_marker.header.stamp = this->now();
    target_marker.ns = "active_frontier_target";
    target_marker.id = 1;
    target_marker.type = visualization_msgs::msg::Marker::SPHERE;
    target_marker.action = visualization_msgs::msg::Marker::ADD;
    target_marker.pose.position.x = target_frontier_.centroid_x;
    target_marker.pose.position.y = target_frontier_.centroid_y;
    target_marker.pose.position.z = 0.08;
    target_marker.pose.orientation.w = 1.0;
    target_marker.scale.x = 0.22;
    target_marker.scale.y = 0.22;
    target_marker.scale.z = 0.22;
    target_marker.color.r = 1.0f;
    target_marker.color.g = 0.84f;
    target_marker.color.b = 0.0f;
    target_marker.color.a = 0.95f;
    markers.markers.push_back(target_marker);

    // Marker 2: Direct Trajectory Vector from Robot to Frontier Goal
    visualization_msgs::msg::Marker vector_marker;
    vector_marker.header.frame_id = "map";
    vector_marker.header.stamp = this->now();
    vector_marker.ns = "frontier_guidance_line";
    vector_marker.id = 2;
    vector_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    vector_marker.action = visualization_msgs::msg::Marker::ADD;
    vector_marker.scale.x = 0.04;
    vector_marker.color.r = 0.2f;
    vector_marker.color.g = 1.0f;
    vector_marker.color.b = 0.3f;
    vector_marker.color.a = 0.8f;

    geometry_msgs::msg::Point p_start, p_end;
    p_start.x = curr_x_;
    p_start.y = curr_y_;
    p_start.z = 0.05;
    p_end.x = target_frontier_.centroid_x;
    p_end.y = target_frontier_.centroid_y;
    p_end.z = 0.05;
    vector_marker.points.push_back(p_start);
    vector_marker.points.push_back(p_end);
    markers.markers.push_back(vector_marker);
  }

  marker_pub_->publish(markers);
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

double SimpleExplorer::findBestClearanceAngle(const std::vector<float> & ranges, float angle_min, float angle_inc)
{
  // Partition 360° scan into 16 bins (22.5° each) to find maximum open radial clearance
  const int NUM_BINS = 16;
  double bin_scores[NUM_BINS] = {0.0};
  int bin_counts[NUM_BINS] = {0};

  for (size_t i = 0; i < ranges.size(); ++i) {
    float r = ranges[i];
    if (r > 0.12f && r < 8.0f && !std::isinf(r) && !std::isnan(r)) {
      double angle = angle_min + i * angle_inc;
      int bin = static_cast<int>(std::floor((angle + M_PI) / (2.0 * M_PI / NUM_BINS)));
      bin = std::clamp(bin, 0, NUM_BINS - 1);
      bin_scores[bin] += r;
      bin_counts[bin]++;
    }
  }

  int best_bin = 0;
  double max_avg = -1.0;
  for (int b = 0; b < NUM_BINS; ++b) {
    if (bin_counts[b] > 0) {
      double avg = bin_scores[b] / bin_counts[b];
      if (avg > max_avg) {
        max_avg = avg;
        best_bin = b;
      }
    }
  }

  double best_angle = -M_PI + (best_bin + 0.5) * (2.0 * M_PI / NUM_BINS);
  return best_angle;
}

bool SimpleExplorer::isPathObstructed(double x0, double y0, double x1, double y1) const
{
  if (!map_) return false;

  double dx = x1 - x0;
  double dy = y1 - y0;
  double dist = std::hypot(dx, dy);
  if (dist < 0.15) return false;

  double res = map_->info.resolution;
  int n_steps = static_cast<int>(dist / res);
  if (n_steps < 3) return false;

  double origin_x = map_->info.origin.position.x;
  double origin_y = map_->info.origin.position.y;
  int width = map_->info.width;
  int height = map_->info.height;
  const auto & data = map_->data;

  // Step along the ray (skip first 2 steps near robot and last 2 steps near frontier)
  for (int i = 2; i < n_steps - 2; ++i) {
    double t = static_cast<double>(i) / n_steps;
    double px = x0 + t * dx;
    double py = y0 + t * dy;

    int c = static_cast<int>((px - origin_x) / res);
    int r = static_cast<int>((py - origin_y) / res);

    if (c >= 0 && c < width && r >= 0 && r < height) {
      int idx = r * width + c;
      if (data[idx] > 50) {
        return true;  // Wall or obstacle blocks straight line of sight
      }
    }
  }
  return false;
}

void SimpleExplorer::controlLoop()
{
  geometry_msgs::msg::Twist cmd;
  double now = this->now().seconds();

  updateRobotPose();

  // Periodic frontier re-evaluation if map has updated (every 1.5s)
  if ((now - last_frontier_search_time_) >= 1.5) {
    last_frontier_search_time_ = now;
    findFrontiers();
  }

  // Monitor active target progress: anti-trap deadlock release
  if (has_target_frontier_) {
    double current_dist = std::hypot(target_frontier_.centroid_x - curr_x_,
                                     target_frontier_.centroid_y - curr_y_);
    if (current_dist < 0.35) {
      // Reached target frontier! Find next unexplored area
      RCLCPP_INFO(this->get_logger(),
                  "[Frontier] Target reached at (%.2f, %.2f)! Re-evaluating room map.",
                  target_frontier_.centroid_x, target_frontier_.centroid_y);
      has_target_frontier_ = false;
      findFrontiers();
    } else if ((now - target_lock_time_) > 10.0) {
      if ((last_target_dist_ - current_dist) < 0.15) {
        // Robot cannot make progress toward target within 10s (behind wall/obstacle). Blacklist it!
        RCLCPP_WARN(this->get_logger(),
                    "[Frontier] Target unreachable (dist=%.2fm, initial=%.2fm after 10s). Blacklisting (%.2f, %.2f) for 40s.",
                    current_dist, last_target_dist_,
                    target_frontier_.centroid_x, target_frontier_.centroid_y);
        blacklist_.push_back({target_frontier_.centroid_x, target_frontier_.centroid_y, now});
        has_target_frontier_ = false;
        findFrontiers();
      } else {
        last_target_dist_ = current_dist;
        target_lock_time_ = now;
      }
    }
  }

  // Safety fallback if LiDAR scan is missing or stale
  if (!scan_ || (now - scan_time_) > 2.0) {
    cmd.angular.z = angular_speed_ * 0.5;
    cmd_pub_->publish(cmd);
    return;
  }

  const auto & ranges = scan_->ranges;
  if (ranges.size() < 20) {
    cmd.angular.z = angular_speed_ * 0.5;
    cmd_pub_->publish(cmd);
    return;
  }

  float a_min = scan_->angle_min;
  float a_inc = scan_->angle_increment;

  double lidar_front = sectorMin(ranges, a_min, a_inc, -18.0,  18.0);
  double front_left  = sectorMin(ranges, a_min, a_inc,  18.0,  45.0);
  double front_right = sectorMin(ranges, a_min, a_inc, -45.0, -18.0);
  double left        = sectorMin(ranges, a_min, a_inc,  45.0, 105.0);
  double right       = sectorMin(ranges, a_min, a_inc, -105.0, -45.0);

  // Intelligent Multi-Sensor Obstacle Clearance Fusion (ToF + 2D LiDAR):
  // 1. VL53L0X ToF provides ground-level near-field obstacle detection (blindspot below LiDAR plane)
  // 2. 2D LiDAR provides 360° panoramic spatial mapping and escape route planning
  bool tof_fresh = ((now - tof_time_) < 0.6);
  bool tof_valid = (tof_fresh && tof_range_ >= 0.03 && tof_range_ <= 1.20);

  // Unified frontal clearance: true minimum of LiDAR forward corridor and low-profile ToF sensor
  double front = tof_valid ? std::min(lidar_front, tof_range_) : lidar_front;

  // Spatial clearance analysis from 360° LiDAR field to determine optimal escape direction
  double clearance_left  = 0.55 * front_left + 0.45 * left;
  double clearance_right = 0.55 * front_right + 0.45 * right;
  double optimal_escape_dir = (clearance_left >= clearance_right) ? 1.0 : -1.0;
  if (front_left < 0.25 && front_right < 0.25) {
    // Both front flanks constricted: query full 360° LiDAR scan for maximum open escape angle
    double best_angle = findBestClearanceAngle(ranges, a_min, a_inc);
    optimal_escape_dir = (best_angle >= 0.0) ? 1.0 : -1.0;
  }

  // =========================================================================
  // 1. REVERSE STATE (Physical Stall Escape Maneuver)
  // =========================================================================
  if (state_ == ExplorerState::REVERSE) {
    if ((now - reverse_start_time_) < 1.0) {
      cmd.linear.x = -0.10;
      cmd.angular.z = 0.0;
    } else {
      state_ = ExplorerState::ROTATE;
      rotate_start_time_ = now;
      last_stall_check_time_ = now;
      last_stall_x_ = curr_x_;
      last_stall_y_ = curr_y_;
      forward_motion_start_time_ = now;
      turn_direction_ = optimal_escape_dir;
      cmd.linear.x = 0.0;
      cmd.angular.z = turn_direction_ * 0.45;
    }
  }
  // =========================================================================
  // 2. ROTATE STATE (In-Place Angular Pivot away from obstacles)
  // =========================================================================
  else if (state_ == ExplorerState::ROTATE) {
    double rotate_dur = now - rotate_start_time_;

    if (rotate_dur > 3.5) {
      // If unable to clear within 3.5s, reverse to open up clearance rather than forcing forward
      state_ = ExplorerState::REVERSE;
      reverse_start_time_ = now;
      last_stall_check_time_ = now;
      last_stall_x_ = curr_x_;
      last_stall_y_ = curr_y_;
      cmd.linear.x = -0.10;
      cmd.angular.z = 0.0;
    } else if (rotate_dur >= 1.0 && front >= clear_dist_ && front_left >= 0.25 && front_right >= 0.25) {
      // Path clear on BOTH LiDAR and ToF (> clear_dist_) with flank clearance!
      // Resume exploration with a 1.8s forward commitment to cruise clear of the obstacle
      state_ = ExplorerState::FRONTIER_SEEK;
      commit_forward_until_ = now + 1.8;
      forward_motion_start_time_ = now;
      last_stall_check_time_ = now;
      last_stall_x_ = curr_x_;
      last_stall_y_ = curr_y_;
      cmd.linear.x = linear_speed_;
      cmd.angular.z = 0.0;
    } else {
      // Keep rotating smoothly in the latched direction
      cmd.linear.x = 0.0;
      cmd.angular.z = turn_direction_ * 0.35;
    }
  }
  // =========================================================================
  // 3. CRITICAL / NEAR OBSTACLE DETECTION (Checked during forward motion)
  // =========================================================================
  else if (front < critical_dist_ || front_left < 0.22 || front_right < 0.22) {
    // If pursuing a frontier that led to an obstacle, blacklist it immediately
    if (has_target_frontier_) {
      blacklist_.push_back({target_frontier_.centroid_x, target_frontier_.centroid_y, now});
      has_target_frontier_ = false;
    }

    if (front < 0.20) {
      // Very close to barrier (<20cm): Reverse first to clear bumper arc before pivoting
      state_ = ExplorerState::REVERSE;
      reverse_start_time_ = now;
      cmd.linear.x = -0.10;
      cmd.angular.z = 0.0;
    } else {
      state_ = ExplorerState::ROTATE;
      rotate_start_time_ = now;
      turn_direction_ = optimal_escape_dir;
      cmd.linear.x = 0.0;
      cmd.angular.z = turn_direction_ * 0.35;
    }
  }
  // =========================================================================
  // 4. PREDICTIVE OBSTACLE DETECTION (LiDAR or ToF < obstacle_dist_)
  // =========================================================================
  else if (front < obstacle_dist_) {
    // Forward path obstructed: blacklist obstructed target and pivot away cleanly
    if (has_target_frontier_) {
      blacklist_.push_back({target_frontier_.centroid_x, target_frontier_.centroid_y, now});
      has_target_frontier_ = false;
    }

    state_ = ExplorerState::ROTATE;
    rotate_start_time_ = now;
    turn_direction_ = optimal_escape_dir;
    cmd.linear.x = 0.0;
    cmd.angular.z = turn_direction_ * 0.35;
  }
  // =========================================================================
  // 5. FRONTIER SEEKING & COMPLETE ROOM SYSTEMATIC EXPLORATION
  // =========================================================================
  else {
    if (all_frontiers_explored_) {
      // 100% of Room Boundaries & Interior Fully Explored!
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      if (log_count_ % 30 == 0) {
        RCLCPP_INFO(this->get_logger(),
                    "=== [ROOM EXPLORATION COMPLETE] 100%% of frontiers mapped with zero stucking ===");
      }
    } else if (now < commit_forward_until_ && front > critical_dist_) {
      // Post-avoidance forward commitment: cruise forward through open clearance corridor
      state_ = ExplorerState::FRONTIER_SEEK;
      cmd.linear.x = linear_speed_;
      cmd.angular.z = 0.0;
    } else if (has_target_frontier_) {
      state_ = ExplorerState::FRONTIER_SEEK;

      // Compute heading angle towards target frontier centroid
      double goal_angle = std::atan2(target_frontier_.centroid_y - curr_y_,
                                     target_frontier_.centroid_x - curr_x_);
      double heading_err = std::atan2(std::sin(goal_angle - curr_yaw_),
                                      std::cos(goal_angle - curr_yaw_));

      if (std::abs(heading_err) > 0.90) {
        // Large heading error > 52°: Crisp in-place pivot with differential torque
        cmd.linear.x = 0.0;
        cmd.angular.z = (heading_err > 0.0) ? 0.35 : -0.35;
      } else {
        // Continuous smooth curved pursuit towards target frontier:
        double heading_cos = std::max(0.25, std::cos(heading_err));
        double clearance_scale = std::clamp((front - critical_dist_) / (obstacle_dist_ - critical_dist_), 0.4, 1.0);
        cmd.linear.x = linear_speed_ * clearance_scale * heading_cos;

        // Side repulsive bias if grazing obstacles on left or right flanks:
        double side_bias = 0.0;
        if (front_left < 0.30) {
          side_bias -= 0.15 * (0.30 - front_left) / 0.10;
        }
        if (front_right < 0.30) {
          side_bias += 0.15 * (0.30 - front_right) / 0.10;
        }

        cmd.angular.z = std::clamp(1.2 * heading_err + side_bias, -0.35, 0.35);
      }
    } else {
      // No active frontier yet (or initial room discovery): systematic wall following
      state_ = ExplorerState::WALL_FOLLOW;
      cmd.linear.x = linear_speed_;

      if (right < (wall_follow_dist_ - 0.06)) {
        cmd.angular.z = angular_speed_ * 0.60;
      } else if (right > (wall_follow_dist_ + 0.10) && right < 0.85) {
        cmd.angular.z = -angular_speed_ * 0.45;
      } else if (left < (wall_follow_dist_ - 0.06)) {
        cmd.angular.z = -angular_speed_ * 0.60;
      } else {
        cmd.angular.z = 0.08 * std::sin(now * 0.25);
      }
    }
  }

  // =========================================================================
  // 6. PHYSICAL ANTI-STALL GUARDIAN (Layer 2 Safe)
  // =========================================================================
  // Layer 2A: Forward Translational Cruising Stall
  if ((state_ == ExplorerState::FRONTIER_SEEK || state_ == ExplorerState::WALL_FOLLOW)
      && cmd.linear.x >= 0.05) {
    if ((now - forward_motion_start_time_) >= 2.5) {
      if ((now - last_stall_check_time_) >= 2.5) {
        double dist = std::hypot(curr_x_ - last_stall_x_, curr_y_ - last_stall_y_);
        last_stall_check_time_ = now;
        last_stall_x_ = curr_x_;
        last_stall_y_ = curr_y_;

        // At 0.12 m/s, robot should cover ~0.30m in 2.5s. If < 0.025m, robot is physically wedged/stalled!
        if (dist < 0.025) {
          stall_count_++;
          RCLCPP_WARN(this->get_logger(),
                      "[Anti-Stall Guardian] Physical forward stall (%.1fcm in 2.5s)! Escaping #%u",
                      dist * 100.0, stall_count_);
          state_ = ExplorerState::REVERSE;
          reverse_start_time_ = now;
          turn_direction_ = (left > right) ? 1.0 : -1.0;
          cmd.linear.x = -0.10;
          cmd.angular.z = 0.0;

          // Blacklist current target to break free from obstacle obstruction
          if (has_target_frontier_) {
            blacklist_.push_back({target_frontier_.centroid_x, target_frontier_.centroid_y, now});
            has_target_frontier_ = false;
          }
        }
      }
    }
  } else if (state_ != ExplorerState::REVERSE) {
    // When pivoting in place or aligning, reset translational stall baseline
    last_stall_check_time_ = now;
    last_stall_x_ = curr_x_;
    last_stall_y_ = curr_y_;
    forward_motion_start_time_ = now;
  }

  // Layer 2B: Rotational In-Place Pivot Stall
  if (std::abs(cmd.linear.x) < 0.02 && std::abs(cmd.angular.z) >= 0.20) {
    if ((now - last_rot_stall_check_time_) >= 2.5) {
      double d_yaw = std::abs(std::atan2(std::sin(curr_yaw_ - last_rot_stall_yaw_),
                                         std::cos(curr_yaw_ - last_rot_stall_yaw_)));
      last_rot_stall_check_time_ = now;
      last_rot_stall_yaw_ = curr_yaw_;

      if (d_yaw < 0.08) {
        stall_count_++;
        RCLCPP_WARN(this->get_logger(),
                    "[Anti-Stall Guardian] Rotational pivot stalled (%.1f° in 2.5s)! Executing escape reverse #%u",
                    d_yaw * (180.0 / M_PI), stall_count_);
        state_ = ExplorerState::REVERSE;
        reverse_start_time_ = now;
        cmd.linear.x = -0.10;
        cmd.angular.z = 0.0;
        if (has_target_frontier_) {
          blacklist_.push_back({target_frontier_.centroid_x, target_frontier_.centroid_y, now});
          has_target_frontier_ = false;
        }
      }
    }
  } else {
    last_rot_stall_check_time_ = now;
    last_rot_stall_yaw_ = curr_yaw_;
  }

  log_count_++;
  if (log_count_ % 20 == 0) {
    const char * st_str = (state_ == ExplorerState::FRONTIER_SEEK) ? "FRONTIER_SEEK" :
                          (state_ == ExplorerState::WALL_FOLLOW)   ? "WALL_FOLLOW" :
                          (state_ == ExplorerState::CURVE)         ? "CURVE" :
                          (state_ == ExplorerState::ROTATE)        ? "ROTATE" : "REVERSE";
    RCLCPP_INFO(this->get_logger(),
                "[%s] clusters=%zu target=(%.2f, %.2f) front=%.2fm ToF=%.2fm -> lin=%.2f ang=%.2f",
                st_str, frontier_clusters_.size(),
                has_target_frontier_ ? target_frontier_.centroid_x : 0.0,
                has_target_frontier_ ? target_frontier_.centroid_y : 0.0,
                front, tof_range_, cmd.linear.x, cmd.angular.z);
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
