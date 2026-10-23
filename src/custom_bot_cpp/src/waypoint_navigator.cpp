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
// File:                 waypoint_navigator.cpp
// License:              Apache License 2.0 (c) 2026 Naman Harshwal
//
// Description: Pure C++17 Nav2 Autonomous Waypoint Navigator.
// Implements an action client that commands the robot through arbitrary waypoint sequences.
// ==============================================================================

#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"

namespace custom_bot_cpp
{

class WaypointNavigator : public rclcpp::Node
{
public:
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

  explicit WaypointNavigator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("waypoint_navigator", options)
  {
    client_ = rclcpp_action::create_client<NavigateThroughPoses>(
      this, "navigate_through_poses");

    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&WaypointNavigator::startNavigationTour, this));

    RCLCPP_INFO(this->get_logger(), "C++ Waypoint Navigator initialized.");
  }

  void addWaypoint(double x, double y, double yaw)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;

    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);

    waypoints_.push_back(pose);
  }

  void startNavigationTour()
  {
    timer_->cancel();

    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_WARN(this->get_logger(), "Nav2 action server 'navigate_through_poses' not ready.");
      return;
    }

    if (waypoints_.empty()) {
      // Default patrol tour around mapped room
      addWaypoint(0.50,  0.00, 0.0);
      addWaypoint(0.50,  0.80, 1.57);
      addWaypoint(0.00,  0.80, 3.14);
      addWaypoint(0.00,  0.00, 0.0);
    }

    auto goal_msg = NavigateThroughPoses::Goal();
    goal_msg.poses = waypoints_;

    RCLCPP_INFO(this->get_logger(), "Sending %zu waypoints to Nav2...", waypoints_.size());

    auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this](const GoalHandle::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal was rejected by Nav2 server.");
        } else {
          RCLCPP_INFO(this->get_logger(), "Goal accepted by Nav2. Navigating waypoints...");
        }
      };

    send_goal_options.feedback_callback =
      [this](GoalHandle::SharedPtr,
             const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
          "Navigating: %d poses remaining, distance remaining: %.2fm",
          feedback->number_of_poses_remaining, feedback->distance_remaining);
      };

    send_goal_options.result_callback =
      [this](const GoalHandle::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(this->get_logger(), "Waypoint tour successfully completed!");
        } else {
          RCLCPP_WARN(this->get_logger(), "Waypoint tour ended with code: %d", static_cast<int>(result.code));
        }
      };

    client_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
};

}  // namespace custom_bot_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<custom_bot_cpp::WaypointNavigator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
