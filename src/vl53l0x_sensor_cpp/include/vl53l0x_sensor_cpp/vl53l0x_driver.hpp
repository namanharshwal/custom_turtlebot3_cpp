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
// File:                 vl53l0x_driver.hpp
// License:              Apache License 2.0 (c) 2026 Naman Harshwal
// ==============================================================================

#ifndef VL53L0X_SENSOR_CPP__VL53L0X_DRIVER_HPP_
#define VL53L0X_SENSOR_CPP__VL53L0X_DRIVER_HPP_

#include <string>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "vl53l0x_sensor_cpp/tof.h"

namespace vl53l0x_sensor_cpp
{

class VL53L0XNode : public rclcpp::Node
{
public:
  explicit VL53L0XNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~VL53L0XNode();

private:
  void declareParameters();
  void initI2C();
  void closeI2C();
  double readDistanceMeters();
  void timerCallback();

  int i2c_bus_;
  int i2c_address_;
  std::string frame_id_;
  double min_range_;
  double max_range_;
  double field_of_view_;
  double publish_rate_;

  bool hardware_connected_{false};

  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace vl53l0x_sensor_cpp

#endif  // VL53L0X_SENSOR_CPP__VL53L0X_DRIVER_HPP_
