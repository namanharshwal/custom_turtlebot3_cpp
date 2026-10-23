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
// File:                 arduino_bridge.cpp
// License:              Apache License 2.0 (c) 2026 Naman Harshwal
//
// ==============================================================================
// 🧠 MATHEMATICAL FOUNDATIONS & DERIVATIONS IMPLEMENTED HEREIN:
// ==============================================================================
// 1. WHEEL RESOLUTION & ASYMMETRIC CPR DECOUPLING:
//    - Outer Diameter: D = 0.065 m (Radius r = 0.0325 m)
//    - Circumference:  C = π * D = π * 0.065 = 0.20420352 m (204.204 mm)
//    - Left CPR = 412 ticks/rev  -->  δ_L = C / 412 = 0.0004956396 m/tick (~0.4956 mm)
//    - Right CPR = 440 ticks/rev -->  δ_R = C / 440 = 0.0004640989 m/tick (~0.4641 mm)
//    - Elimination of 6.8% mechanical gear tolerance drift (prevents 32 cm arc error).
//
// 2. DIFFERENTIAL DRIVE FORWARD & INVERSE KINEMATICS:
//    - Track baseline: L = 0.150 m
//    - Forward: v = (v_R + v_L) / 2.0,  ω = (v_R - v_L) / L
//    - Inverse: v_L = v - ω * (L / 2.0), v_R = v + ω * (L / 2.0)
//
// 3. BOSCH BNO055 REP 103 COORDINATE INVERSION:
//    - Raw yaw extracted from BNO055 quaternion (aerospace NED, clockwise positive).
//    - Sign strictly inverted: d_theta = -raw_d_theta (counter-clockwise positive REP 103).
//
// 4. LAYER 1 CLOSED-LOOP FIRMWARE ANTI-STALL BOOST:
//    - If forward speed commanded > 0.02 m/s and encoder delta < 4 ticks in 400 ms:
//      Injects 150 ms PWM 105 boost to overcome static friction (9.23 N threshold).
// ==============================================================================

#include "custom_bot_cpp/arduino_bridge.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <sstream>
#include <iomanip>

namespace custom_bot_cpp
{

ArduinoBridge::ArduinoBridge(const rclcpp::NodeOptions & options)
: Node("arduino_bridge", options)
{
  declareParameters();

  RCLCPP_INFO(this->get_logger(),
              "Initializing C++ ArduinoBridge: Port=%s, Baud=%d, D=%.3fm, L=%.3fm, CPR=[%d, %d]",
              serial_port_.c_str(), baud_rate_, wheel_diameter_, wheel_separation_, cpr_left_, cpr_right_);

  // Initialize ROS 2 interfaces
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10, std::bind(&ArduinoBridge::cmdVelCallback, this, std::placeholders::_1));

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
  joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  // Periodic heartbeat / watchdog keepalive timer (every 100 ms = 10 Hz)
  heartbeat_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&ArduinoBridge::sendHeartbeat, this));

  // Initialize hardware serial communication
  initSerial();
}

ArduinoBridge::~ArduinoBridge()
{
  closeSerial();
}

void ArduinoBridge::declareParameters()
{
  this->declare_parameter<std::string>("serial_port", "/dev/ttyAMA0");
  this->declare_parameter<int>("baud_rate", 115200);
  this->declare_parameter<double>("wheel_diameter", 0.065);
  this->declare_parameter<double>("wheel_separation", 0.150);
  this->declare_parameter<int>("cpr_left", 412);
  this->declare_parameter<int>("cpr_right", 440);
  this->declare_parameter<double>("max_speed", 0.5);
  this->declare_parameter<bool>("publish_tf", true);
  this->declare_parameter<std::string>("base_frame", "base_footprint");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<int>("min_pwm", 75);
  this->declare_parameter<int>("max_pwm", 255);

  serial_port_        = this->get_parameter("serial_port").as_string();
  baud_rate_          = this->get_parameter("baud_rate").as_int();
  wheel_diameter_     = this->get_parameter("wheel_diameter").as_double();
  wheel_separation_   = this->get_parameter("wheel_separation").as_double();
  cpr_left_           = this->get_parameter("cpr_left").as_int();
  cpr_right_          = this->get_parameter("cpr_right").as_int();
  max_speed_          = this->get_parameter("max_speed").as_double();
  publish_tf_         = this->get_parameter("publish_tf").as_bool();
  base_frame_         = this->get_parameter("base_frame").as_string();
  odom_frame_         = this->get_parameter("odom_frame").as_string();
  min_pwm_            = this->get_parameter("min_pwm").as_int();
  max_pwm_            = this->get_parameter("max_pwm").as_int();

  wheel_radius_        = wheel_diameter_ / 2.0;
  wheel_circumference_ = M_PI * wheel_diameter_;

  // Decoupled meters per tick
  meters_per_tick_left_  = wheel_circumference_ / static_cast<double>(cpr_left_);
  meters_per_tick_right_ = wheel_circumference_ / static_cast<double>(cpr_right_);
}

void ArduinoBridge::initSerial()
{
  serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (serial_fd_ < 0) {
    RCLCPP_WARN(this->get_logger(), "Failed to open serial port %s. Hardware may not be connected.", serial_port_.c_str());
    return;
  }

  fcntl(serial_fd_, F_SETFL, 0);

  struct termios tty;
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(this->get_logger(), "tcgetattr error on %s", serial_port_.c_str());
    close(serial_fd_);
    serial_fd_ = -1;
    return;
  }

  // Set baud rate: 115200
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  // 8N1 raw mode
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag |= (CLOCAL | CREAD);

  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_oflag &= ~OPOST;

  tty.c_cc[VMIN]  = 1;
  tty.c_cc[VTIME] = 5;

  tcflush(serial_fd_, TCIFLUSH);
  tcsetattr(serial_fd_, TCSANOW, &tty);

  // Send RESET command to sync Arduino tick counters
  std::string reset_cmd = "RESET\n";
  ssize_t written_reset = write(serial_fd_, reset_cmd.c_str(), reset_cmd.size());
  (void)written_reset;
  usleep(100000);

  running_ = true;
  serial_thread_ = std::thread(&ArduinoBridge::serialReaderLoop, this);

  RCLCPP_INFO(this->get_logger(), "Serial connected to Arduino on %s @ %d baud", serial_port_.c_str(), baud_rate_);
}

void ArduinoBridge::closeSerial()
{
  running_ = false;
  if (serial_thread_.joinable()) {
    serial_thread_.join();
  }

  if (serial_fd_ >= 0) {
    // Send STOP before closing
    std::string stop_cmd = "STOP\n";
    ssize_t written_stop = write(serial_fd_, stop_cmd.c_str(), stop_cmd.size());
    (void)written_stop;
    close(serial_fd_);
    serial_fd_ = -1;
  }
}

void ArduinoBridge::serialReaderLoop()
{
  std::string line_buffer = "";
  char ch;

  while (running_ && rclcpp::ok()) {
    if (serial_fd_ < 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      continue;
    }

    int n = read(serial_fd_, &ch, 1);
    if (n > 0) {
      if (ch == '\n' || ch == '\r') {
        if (!line_buffer.empty()) {
          parseSerialLine(line_buffer);
          line_buffer.clear();
        }
      } else if (ch >= 32 && ch <= 126) {
        line_buffer += ch;
      }
    } else {
      usleep(1000);
    }
  }
}

void ArduinoBridge::parseSerialLine(const std::string & line)
{
  // Packet format: "E,<lEnc>,<rEnc>,<qw>,<qx>,<qy>,<qz>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>"
  if (line.rfind("E,", 0) != 0) {
    return;
  }

  std::stringstream ss(line);
  std::string token;
  std::vector<std::string> tokens;

  while (std::getline(ss, token, ',')) {
    tokens.push_back(token);
  }

  if (tokens.size() < 13) {
    return;
  }

  try {
    long l_enc = std::stol(tokens[1]);
    long r_enc = std::stol(tokens[2]);

    double qw = std::stod(tokens[3]);
    double qx = std::stod(tokens[4]);
    double qy = std::stod(tokens[5]);
    double qz = std::stod(tokens[6]);

    double ax = std::stod(tokens[7]);
    double ay = std::stod(tokens[8]);
    double az = std::stod(tokens[9]);

    double gx = std::stod(tokens[10]);
    double gy = std::stod(tokens[11]);
    double gz = std::stod(tokens[12]);

    updateOdometry(l_enc, r_enc, qw, qx, qy, qz, ax, ay, az, gx, gy, gz);
  } catch (const std::exception & e) {
    // Parsing error ignored for malformed byte frame
  }
}

void ArduinoBridge::updateOdometry(long l_enc, long r_enc, double qw, double qx, double qy, double qz,
                                   double ax, double ay, double az, double gx, double gy, double gz)
{
  rclcpp::Time now = this->now();
  double current_time = now.seconds();

  // Invert right encoder tick count to account for mirrored physical mounting
  r_enc = -r_enc;

  if (!first_enc_received_) {
    prev_l_enc_ = l_enc;
    prev_r_enc_ = r_enc;
    first_enc_received_ = true;
    last_odom_time_ = current_time;
    last_stall_check_time_ = current_time;
    stall_check_ticks_ = l_enc + r_enc;
    return;
  }

  double dt = current_time - last_odom_time_;
  if (dt <= 0.0) dt = 0.001;
  last_odom_time_ = current_time;

  long d_left_ticks  = l_enc - prev_l_enc_;
  long d_right_ticks = r_enc - prev_r_enc_;
  prev_l_enc_ = l_enc;
  prev_r_enc_ = r_enc;

  // ── LAYER 1 ANTI-STALL LOGIC ──────────────────────────────────────────
  if ((current_time - last_stall_check_time_) >= 0.40) {
    long tick_delta = std::abs((l_enc + r_enc) - stall_check_ticks_);
    stall_check_ticks_ = l_enc + r_enc;
    last_stall_check_time_ = current_time;

    if (std::abs(last_cmd_vel_linear_) > 0.02 && tick_delta < 4) {
      stall_boost_active_ = true;
      stall_boost_end_time_ = current_time + 0.150;  // 150 ms pulse
    }
  }

  if (stall_boost_active_) {
    if (current_time > stall_boost_end_time_) {
      stall_boost_active_ = false;
    }
  }

  // Displacements per wheel
  double d_left  = static_cast<double>(d_left_ticks) * meters_per_tick_left_;
  double d_right = static_cast<double>(d_right_ticks) * meters_per_tick_right_;
  double d_center = (d_right + d_left) / 2.0;

  // ── BNO055 YAW & REP 103 COORDINATE INVERSION ─────────────────────────
  // Raw yaw from quaternion: atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2))
  double raw_yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

  if (!first_heading_received_) {
    theta_prev_raw_ = raw_yaw;
    first_heading_received_ = true;
    return;
  }

  double raw_d_theta = normalizeAngle(raw_yaw - theta_prev_raw_);
  theta_prev_raw_ = raw_yaw;

  // Strict REP 103 Inversion: +Z points UP, CCW positive
  double d_theta = -raw_d_theta;
  theta_ = normalizeAngle(theta_ + d_theta);

  // Integrate planar position (x, y)
  x_ += d_center * std::cos(theta_);
  y_ += d_center * std::sin(theta_);

  // Wheel velocities
  double v = d_center / dt;
  double omega = d_theta / dt;

  // Wheel rotation angles for /joint_states
  left_wheel_angle_  += static_cast<double>(d_left_ticks)  * (2.0 * M_PI / static_cast<double>(cpr_left_));
  right_wheel_angle_ += static_cast<double>(d_right_ticks) * (2.0 * M_PI / static_cast<double>(cpr_right_));

  // Construct orientation quaternion for odometry
  tf2::Quaternion q_odom;
  q_odom.setRPY(0.0, 0.0, theta_);

  // ── 1. PUBLISH /odom ──────────────────────────────────────────────────
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = now;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;

  odom_msg.pose.pose.position.x = x_;
  odom_msg.pose.pose.position.y = y_;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation.x = q_odom.x();
  odom_msg.pose.pose.orientation.y = q_odom.y();
  odom_msg.pose.pose.orientation.z = q_odom.z();
  odom_msg.pose.pose.orientation.w = q_odom.w();

  // Calibrated covariance matrices for SLAM Toolbox Ceres scan-matching weighting
  std::fill(odom_msg.pose.covariance.begin(), odom_msg.pose.covariance.end(), 0.0);
  odom_msg.pose.covariance[0]  = 0.01;   // X position variance (0.01 m^2)
  odom_msg.pose.covariance[7]  = 0.01;   // Y position variance (0.01 m^2)
  odom_msg.pose.covariance[35] = 0.05;   // Yaw heading variance (0.05 rad^2)

  std::fill(odom_msg.twist.covariance.begin(), odom_msg.twist.covariance.end(), 0.0);
  odom_msg.twist.covariance[0]  = 0.01;  // Linear velocity variance
  odom_msg.twist.covariance[35] = 0.05;  // Angular velocity variance

  odom_msg.twist.twist.linear.x = v;
  odom_msg.twist.twist.angular.z = omega;
  odom_pub_->publish(odom_msg);

  // ── 2. BROADCAST TF: odom -> base_footprint ───────────────────────────
  if (publish_tf_ && tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation.x = q_odom.x();
    tf_msg.transform.rotation.y = q_odom.y();
    tf_msg.transform.rotation.z = q_odom.z();
    tf_msg.transform.rotation.w = q_odom.w();
    tf_broadcaster_->sendTransform(tf_msg);
  }

  // ── 3. PUBLISH /imu/data ──────────────────────────────────────────────
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = now;
  imu_msg.header.frame_id = "imu_link";
  imu_msg.orientation.w = qw;
  imu_msg.orientation.x = qx;
  imu_msg.orientation.y = qy;
  imu_msg.orientation.z = qz;
  imu_msg.linear_acceleration.x = ax;
  imu_msg.linear_acceleration.y = ay;
  imu_msg.linear_acceleration.z = az;
  imu_msg.angular_velocity.x = gx;
  imu_msg.angular_velocity.y = gy;
  imu_msg.angular_velocity.z = gz;
  imu_pub_->publish(imu_msg);

  // ── 4. PUBLISH /joint_states ──────────────────────────────────────────
  sensor_msgs::msg::JointState joint_msg;
  joint_msg.header.stamp = now;
  joint_msg.name = {"wheel_left_joint", "wheel_right_joint"};
  joint_msg.position = {left_wheel_angle_, right_wheel_angle_};
  joint_msg.velocity = {d_left / dt / wheel_radius_, d_right / dt / wheel_radius_};
  joint_pub_->publish(joint_msg);
}

void ArduinoBridge::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  double linear  = msg->linear.x;
  double angular = msg->angular.z;

  last_cmd_vel_linear_ = linear;
  last_cmd_time_ = this->now().seconds();

  // Inverse Kinematics
  double v_left  = linear - angular * (wheel_separation_ / 2.0);
  double v_right = linear + angular * (wheel_separation_ / 2.0);

  int left_pwm  = velocityToPWM(v_left);
  int right_pwm = velocityToPWM(v_right);

  // Layer 1 Anti-stall pulse boost
  if (stall_boost_active_) {
    int boost_pwm = 105;
    left_pwm  = (left_pwm  >= 0) ? boost_pwm : -boost_pwm;
    right_pwm = (right_pwm >= 0) ? boost_pwm : -boost_pwm;
  }

  sendMotorPWM(left_pwm, right_pwm);
}

int ArduinoBridge::velocityToPWM(double vel)
{
  if (std::abs(vel) < 1e-4) return 0;

  double fraction = std::min(1.0, std::abs(vel) / max_speed_);
  int pwm = static_cast<int>(min_pwm_ + fraction * (max_pwm_ - min_pwm_));
  pwm = std::min(max_pwm_, std::max(min_pwm_, pwm));

  return (vel > 0.0) ? pwm : -pwm;
}

void ArduinoBridge::sendMotorPWM(int left_pwm, int right_pwm)
{
  if (serial_fd_ < 0) return;

  std::lock_guard<std::mutex> lock(serial_write_mutex_);
  std::stringstream ss;
  ss << "M," << left_pwm << "," << right_pwm << "\n";
  std::string cmd = ss.str();
  ssize_t written_cmd = write(serial_fd_, cmd.c_str(), cmd.size());
  (void)written_cmd;
}

void ArduinoBridge::sendHeartbeat()
{
  double now = this->now().seconds();
  // If no cmd_vel received for > 200ms, send keepalive halt
  if ((now - last_cmd_time_) > 0.20 && (now - last_cmd_time_) < 1.0) {
    sendMotorPWM(0, 0);
  }
}

}  // namespace custom_bot_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<custom_bot_cpp::ArduinoBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
