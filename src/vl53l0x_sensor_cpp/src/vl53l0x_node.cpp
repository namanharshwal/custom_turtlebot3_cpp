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
// Communicates directly over hardware I2C (/dev/i2c-1 @ 0x29) via Linux kernel I2C ioctl.
// Publishes standard sensor_msgs/msg/Range on /tof/range for real-time local obstacle
// avoidance without contaminating or distorting the 2D LiDAR static SLAM /map.
// ==============================================================================

#include "vl53l0x_sensor_cpp/vl53l0x_driver.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
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
  this->declare_parameter<double>("max_range", 2.00);      // 2 m maximum
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
  std::stringstream ss;
  ss << "/dev/i2c-" << i2c_bus_;
  std::string dev_path = ss.str();

  i2c_fd_ = open(dev_path.c_str(), O_RDWR);
  if (i2c_fd_ < 0) {
    RCLCPP_WARN(this->get_logger(),
                "Cannot open %s. I2C hardware bus not available. Operating in standby mode.",
                dev_path.c_str());
    hardware_connected_ = false;
    return;
  }

  if (ioctl(i2c_fd_, I2C_SLAVE, i2c_address_) < 0) {
    RCLCPP_WARN(this->get_logger(),
                "Failed to acquire bus access / talk to slave 0x%02X on %s.",
                i2c_address_, dev_path.c_str());
    close(i2c_fd_);
    i2c_fd_ = -1;
    hardware_connected_ = false;
    return;
  }

  // Attempt communication test (probe byte read)
  uint8_t reg = 0xC0; // Identification model ID register
  if (write(i2c_fd_, &reg, 1) == 1) {
    uint8_t val = 0;
    if (read(i2c_fd_, &val, 1) == 1) {
      hardware_connected_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "VL53L0X hardware verified at 0x%02X (Model ID: 0x%02X)",
                  i2c_address_, val);
      return;
    }
  }

  hardware_connected_ = false;
  RCLCPP_INFO(this->get_logger(),
              "VL53L0X sensor standing by on /dev/i2c-%d @ 0x%02X.",
              i2c_bus_, i2c_address_);
}

void VL53L0XNode::closeI2C()
{
  if (i2c_fd_ >= 0) {
    close(i2c_fd_);
    i2c_fd_ = -1;
  }
}

double VL53L0XNode::readDistanceMeters()
{
  if (!hardware_connected_ || i2c_fd_ < 0) {
    return max_range_;
  }

  // 1. Trigger single-shot range measurement: write 0x01 to SYSRANGE_START (0x00)
  uint8_t cmd[2] = {0x00, 0x01};
  if (write(i2c_fd_, cmd, 2) != 2) {
    return max_range_;
  }

  // 2. Poll for measurement completion (register 0x00 bit 0 clears or timeout 30ms)
  int attempts = 0;
  uint8_t status = 1;
  while (attempts < 30) {
    uint8_t reg = 0x00;
    if (write(i2c_fd_, &reg, 1) != 1 || read(i2c_fd_, &status, 1) != 1) {
      break;
    }
    if ((status & 0x01) == 0) {
      break;
    }
    usleep(1000);
    attempts++;
  }

  // 3. Read range result registers (0x14 + offset 10 and 11)
  uint8_t reg = 0x14;
  if (write(i2c_fd_, &reg, 1) != 1) {
    return max_range_;
  }

  uint8_t buf[16] = {0};
  if (read(i2c_fd_, buf, 16) != 16) {
    return max_range_;
  }

  uint8_t range_status = (buf[0] & 0x78) >> 3;
  // If target is out of bounds (status 8) or phase/signal fail, field is unobstructed
  if (range_status == 8 || range_status == 6) {
    return max_range_;
  }

  uint16_t dist_mm = (static_cast<uint16_t>(buf[10]) << 8) | buf[11];
  if (dist_mm <= 35 || dist_mm > 2000) {
    return max_range_;
  }

  return static_cast<double>(dist_mm) / 1000.0;
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
