/*
 * ==============================================================================
 * Creator & Maintainer: Naman Harshwal (namanharshwal@gmail.com)
 * Owner/Supervisor:     Pradeep Singh Chauhan
 * Workspace:            psnh_turtlebot_ws
 * License:              Apache License 2.0 (c) 2026 Naman Harshwal
 *
 * Description: Custom TurtleBot Burger 2.0 robotics and autonomous navigation stack.
 * ==============================================================================
 */

/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS 2 Node Client
 *
 *  Copyright 2017 - 2020 EAI TEAM
 *  http://www.eaibot.com
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <math.h>

#define RAD2DEG(x) ((x)*180./M_PI)

static void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan) {
  int count = scan->scan_time / scan->time_increment;
  printf("[YDLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
  printf("[YDLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min),
         RAD2DEG(scan->angle_max));

  for (int i = 0; i < count; i++) {
    float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
    printf("[YDLIDAR INFO]: angle-distance : [%f, %f]\n", degree, scan->ranges[i]);
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_client");

  auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
                        "scan", rclcpp::SensorDataQoS(), scanCb);

  rclcpp::spin(node);

  rclcpp::shutdown();


  return 0;
}
