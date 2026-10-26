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
// File:                 docking_station_visualizer.cpp
// License:              Apache License 2.0 (c) 2026 Naman Harshwal
//
// Description:
// Publishes 3D Visualization Markers for Autonomous Docking Station & Charging Base
// on /docking_station/marker and /charging_station/marker in the global /map frame.
// Visualizes:
//   1. Ground Dock Ramp Base
//   2. Vertical Charging Column
//   3. Copper Dual-Pole Charging Spring Contacts
//   4. High-Visibility Green LED Charging Status Beacon
//   5. Dock Approach Alignment Vector & Visual Guidance Chevrons
//   6. Floating 3D Telemetry Text Banner
// ==============================================================================

#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class DockingStationVisualizer : public rclcpp::Node
{
public:
  DockingStationVisualizer()
  : Node("docking_station_visualizer")
  {
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<double>("dock_x", 0.0);
    this->declare_parameter<double>("dock_y", 0.0);
    this->declare_parameter<double>("dock_yaw", 0.0);

    map_frame_ = this->get_parameter("map_frame").as_string();
    dock_x_    = this->get_parameter("dock_x").as_double();
    dock_y_    = this->get_parameter("dock_y").as_double();
    dock_yaw_  = this->get_parameter("dock_yaw").as_double();

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/docking_station/marker", 10);
    charging_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/charging_station/marker", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/docking_station/pose", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&DockingStationVisualizer::publishMarkers, this));

    RCLCPP_INFO(this->get_logger(),
                "DockingStationVisualizer active: Base at (%.2f, %.2f) in frame [%s]",
                dock_x_, dock_y_, map_frame_.c_str());
  }

private:
  void publishMarkers()
  {
    rclcpp::Time now = this->now();
    visualization_msgs::msg::MarkerArray markers;

    // 1. Base Ground Ramp Plate (Dark Industrial Matte Grey)
    visualization_msgs::msg::Marker base;
    base.header.stamp = now;
    base.header.frame_id = map_frame_;
    base.ns = "docking_hardware";
    base.id = 0;
    base.type = visualization_msgs::msg::Marker::CUBE;
    base.action = visualization_msgs::msg::Marker::ADD;
    base.pose.position.x = dock_x_ + 0.05;
    base.pose.position.y = dock_y_;
    base.pose.position.z = 0.015;
    base.pose.orientation.w = 1.0;
    base.scale.x = 0.35;
    base.scale.y = 0.32;
    base.scale.z = 0.03;
    base.color.r = 0.20f;
    base.color.g = 0.20f;
    base.color.b = 0.22f;
    base.color.a = 0.95f;
    markers.markers.push_back(base);

    // 2. Vertical Charging Column Tower (Signal White)
    visualization_msgs::msg::Marker tower;
    tower.header.stamp = now;
    tower.header.frame_id = map_frame_;
    tower.ns = "docking_hardware";
    tower.id = 1;
    tower.type = visualization_msgs::msg::Marker::CUBE;
    tower.action = visualization_msgs::msg::Marker::ADD;
    tower.pose.position.x = dock_x_ - 0.10;
    tower.pose.position.y = dock_y_;
    tower.pose.position.z = 0.15;
    tower.pose.orientation.w = 1.0;
    tower.scale.x = 0.08;
    tower.scale.y = 0.28;
    tower.scale.z = 0.26;
    tower.color.r = 0.90f;
    tower.color.g = 0.90f;
    tower.color.b = 0.95f;
    tower.color.a = 1.0f;
    markers.markers.push_back(tower);

    // 3. Left Copper Charging Contact (Gold/Copper)
    visualization_msgs::msg::Marker contact_l;
    contact_l.header.stamp = now;
    contact_l.header.frame_id = map_frame_;
    contact_l.ns = "docking_hardware";
    contact_l.id = 2;
    contact_l.type = visualization_msgs::msg::Marker::CYLINDER;
    contact_l.action = visualization_msgs::msg::Marker::ADD;
    contact_l.pose.position.x = dock_x_ - 0.05;
    contact_l.pose.position.y = dock_y_ + 0.06;
    contact_l.pose.position.z = 0.04;
    contact_l.pose.orientation.w = 1.0;
    contact_l.scale.x = 0.025;
    contact_l.scale.y = 0.025;
    contact_l.scale.z = 0.05;
    contact_l.color.r = 0.98f;
    contact_l.color.g = 0.72f;
    contact_l.color.b = 0.10f;
    contact_l.color.a = 1.0f;
    markers.markers.push_back(contact_l);

    // 4. Right Copper Charging Contact (Gold/Copper)
    visualization_msgs::msg::Marker contact_r = contact_l;
    contact_r.id = 3;
    contact_r.pose.position.y = dock_y_ - 0.06;
    markers.markers.push_back(contact_r);

    // 5. LED Status Indicator Beacon (Active Emerald Green)
    visualization_msgs::msg::Marker led;
    led.header.stamp = now;
    led.header.frame_id = map_frame_;
    led.ns = "docking_hardware";
    led.id = 4;
    led.type = visualization_msgs::msg::Marker::SPHERE;
    led.action = visualization_msgs::msg::Marker::ADD;
    led.pose.position.x = dock_x_ - 0.10;
    led.pose.position.y = dock_y_;
    led.pose.position.z = 0.29;
    led.pose.orientation.w = 1.0;
    led.scale.x = 0.04;
    led.scale.y = 0.04;
    led.scale.z = 0.04;
    led.color.r = 0.0f;
    led.color.g = 1.0f;
    led.color.b = 0.35f;
    led.color.a = 1.0f;
    markers.markers.push_back(led);

    // 6. Dock Approach Guidance Arrow
    visualization_msgs::msg::Marker arrow;
    arrow.header.stamp = now;
    arrow.header.frame_id = map_frame_;
    arrow.ns = "docking_guidance";
    arrow.id = 5;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.pose.position.x = dock_x_ + 0.50;
    arrow.pose.position.y = dock_y_;
    arrow.pose.position.z = 0.03;
    arrow.pose.orientation.z = 1.0; // Pointing toward dock (-X)
    arrow.pose.orientation.w = 0.0;
    arrow.scale.x = 0.35;
    arrow.scale.y = 0.05;
    arrow.scale.z = 0.05;
    arrow.color.r = 0.0f;
    arrow.color.g = 0.85f;
    arrow.color.b = 1.0f;
    arrow.color.a = 0.85f;
    markers.markers.push_back(arrow);

    // 7. Text Label Banner
    visualization_msgs::msg::Marker text;
    text.header.stamp = now;
    text.header.frame_id = map_frame_;
    text.ns = "docking_hardware";
    text.id = 6;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = dock_x_ - 0.10;
    text.pose.position.y = dock_y_;
    text.pose.position.z = 0.38;
    text.scale.z = 0.07;
    text.color.r = 1.0f;
    text.color.g = 1.0f;
    text.color.b = 1.0f;
    text.color.a = 1.0f;
    text.text = "CHARGING DOCK (AUTO-ALIGN READY)";
    markers.markers.push_back(text);

    marker_pub_->publish(markers);
    charging_pub_->publish(markers);

    // Publish PoseStamped
    geometry_msgs::msg::PoseStamped dock_pose;
    dock_pose.header.stamp = now;
    dock_pose.header.frame_id = map_frame_;
    dock_pose.pose.position.x = dock_x_;
    dock_pose.pose.position.y = dock_y_;
    dock_pose.pose.position.z = 0.0;
    dock_pose.pose.orientation.w = 1.0;
    pose_pub_->publish(dock_pose);
  }

  std::string map_frame_;
  double dock_x_;
  double dock_y_;
  double dock_yaw_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr charging_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DockingStationVisualizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
