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
// Package:              vl53l0x_sensor_cpp
// File:                 vl53l0x_node.cpp
// License:              Apache License 2.0 (c) 2026 Naman Harshwal
//
// Description:
// Pure C++17 ROS 2 Driver for STMicroelectronics VL53L0X Time-of-Flight (ToF) Distance Sensor.
// Communicates directly over hardware I2C (/dev/i2c-1 @ 0x29) via calibrated Linux I2C registers.
// Publishes standard sensor_msgs/msg/Range on /tof/range for real-time multi-sensor obstacle avoidance.
// ==============================================================================

#include "vl53l0x_sensor_cpp/vl53l0x_driver.hpp"

#include <cmath>
#include <sstream>
#include <iomanip>

namespace vl53l0x_sensor_cpp
{

VL53L0XNode::VL53L0XNode(const rclcpp::NodeOptions & options)
: Node("vl53l0x_node", options)
{
  declareParameters();

  range_pub_ = this->create_publisher<sensor_msgs::msg::Range>("/tof/range", 10);

  initI2C();

  int timer_ms = static_cast<int>(1000.0 / publish_rate_);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(timer_ms),
    std::bind(&VL53L0XNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(),
              "VL53L0X C++17 ToF Driver active on /dev/i2c-%d @ 0x%02X (FoV=%.1f°, Max=%.2fm, Rate=%.1fHz)",
              i2c_bus_, i2c_address_, field_of_view_ * (180.0 / M_PI), max_range_, publish_rate_);
}

VL53L0XNode::~VL53L0XNode()
{
  closeI2C();
}

void VL53L0XNode::declareParameters()
{
  this->declare_parameter<int>("i2c_bus", 1);
  this->declare_parameter<int>("i2c_address", 0x29);
  this->declare_parameter<std::string>("frame_id", "tof_link");
  this->declare_parameter<double>("min_range", 0.03);      // 3 cm minimum
  this->declare_parameter<double>("max_range", 0.45);      // 45 cm calibrated maximum (bounds ToF strictly to bumper collision field)
  this->declare_parameter<double>("field_of_view", 0.4363); // 25° conical cone
  this->declare_parameter<double>("publish_rate", 20.0);   // 20 Hz update rate

  i2c_bus_       = this->get_parameter("i2c_bus").as_int();
  i2c_address_   = this->get_parameter("i2c_address").as_int();
  frame_id_      = this->get_parameter("frame_id").as_string();
  min_range_     = this->get_parameter("min_range").as_double();
  max_range_     = this->get_parameter("max_range").as_double();
  field_of_view_ = this->get_parameter("field_of_view").as_double();
  publish_rate_  = this->get_parameter("publish_rate").as_double();
}

void VL53L0XNode::initI2C()
{
  // Initialize VL53L0X hardware using the proven Linux I2C driver (0 = standard fast mode up to 1.2m)
  int ret = tofInit(i2c_bus_, i2c_address_, 0);
  if (ret == 1) {
    hardware_connected_ = true;
    int model = 0, rev = 0;
    tofGetModel(&model, &rev);
    RCLCPP_INFO(this->get_logger(),
                "VL53L0X hardware verified and calibrated at 0x%02X (Model ID: %d, Rev: %d)",
                i2c_address_, model, rev);
  } else {
    hardware_connected_ = false;
    RCLCPP_WARN(this->get_logger(),
                "Failed to initialize VL53L0X on /dev/i2c-%d @ 0x%02X. Operating in standby mode.",
                i2c_bus_, i2c_address_);
  }
}

void VL53L0XNode::closeI2C()
{
  hardware_connected_ = false;
}

double VL53L0XNode::readDistanceMeters()
{
  if (!hardware_connected_) {
    return max_range_;
  }

  int dist_mm = 0;
  int status = 0;
  tofReadRange(&dist_mm, &status);

  // VL53L0X millimeter distance validation:
  // Accept valid return readings in physical bumper detection zone [30mm, 600mm]
  // (Filter out sentinel error codes 8190/8191 which signify out-of-range horizon)
  int max_mm = static_cast<int>(max_range_ * 1000.0);
  if (dist_mm >= 30 && dist_mm <= max_mm && dist_mm != 8190 && dist_mm != 8191) {
    return static_cast<double>(dist_mm) / 1000.0;
  }

  // Out of range or no optical echo: return max_range
  return max_range_;
}

void VL53L0XNode::timerCallback()
{
  rclcpp::Time now = this->now();
  double dist = readDistanceMeters();

  sensor_msgs::msg::Range msg;
  msg.header.stamp = now;
  msg.header.frame_id = frame_id_;
  msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
  msg.field_of_view = static_cast<float>(field_of_view_);
  msg.min_range = static_cast<float>(min_range_);
  msg.max_range = static_cast<float>(max_range_);
  msg.range = static_cast<float>(std::max(min_range_, std::min(max_range_, dist)));

  range_pub_->publish(msg);
}

}  // namespace vl53l0x_sensor_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vl53l0x_sensor_cpp::VL53L0XNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
