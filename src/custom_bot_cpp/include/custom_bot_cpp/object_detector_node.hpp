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
// File:                 object_detector_node.hpp
// License:              Apache License 2.0 (c) 2026 Naman Harshwal
// ==============================================================================

#ifndef CUSTOM_BOT_CPP__OBJECT_DETECTOR_NODE_HPP_
#define CUSTOM_BOT_CPP__OBJECT_DETECTOR_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <thread>
#include <atomic>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/string.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include "cv_bridge/cv_bridge.h"

namespace custom_bot_cpp
{

struct Detection
{
  std::string class_name;
  int class_id;
  float confidence;
  cv::Rect box;
};

class ObjectDetectorNode : public rclcpp::Node
{
public:
  explicit ObjectDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~ObjectDetectorNode();

private:
  void declareParameters();
  void loadNetwork();
  void loadClassNames();
  void configureHardwareSensor(int exposure, int analogue_gain);
  void captureLoop();
  void inferenceLoop();
  void publishDisplayFrame();
  cv::Scalar getClassColor(const std::string & class_name, int class_id);
  void unpackAndNormalize(cv::Mat & frame);

  // Network & Inference
  cv::dnn::Net net_;
  std::vector<std::string> class_names_;
  std::vector<std::string> out_layer_names_;

  // Threading & Synchronization
  std::thread capture_thread_;
  std::thread inference_thread_;
  std::atomic<bool> running_{false};
  std::mutex frame_mutex_;
  cv::Mat latest_frame_;
  bool new_frame_available_{false};

  std::mutex detections_mutex_;
  std::vector<Detection> latest_detections_;

  std::atomic<double> actual_cam_fps_{15.0};
  std::atomic<double> actual_yolo_fps_{2.0};
  std::atomic<double> last_inference_ms_{340.0};

  // ROS 2 Publishers & Timers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr det_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr objects_pub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;

  // Parameters
  int video_device_;
  int capture_width_;
  int capture_height_;
  int input_size_;
  double conf_threshold_;
  double nms_threshold_;
  double detection_rate_hz_;
  double publish_rate_hz_;
  bool publish_raw_image_;
  int exposure_;
  int analogue_gain_;
  std::string flip_mode_;
  std::string weights_path_;
  std::string config_path_;
  std::string names_path_;

  // Color mapping
  std::unordered_map<std::string, cv::Scalar> class_colors_;
};

}  // namespace custom_bot_cpp

#endif  // CUSTOM_BOT_CPP__OBJECT_DETECTOR_NODE_HPP_
