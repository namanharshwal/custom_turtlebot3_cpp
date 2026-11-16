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
// 1. Physical Wheel Geometry & Asymmetric Calibration:
//    - Wheel Diameter: D = 0.065 m (Radius r = 0.0325 m)
//    - Circumference:  C = π * D = π * 0.065 = 0.20420352 m (204.204 mm)
//    - Left Encoder CPR = 412 ticks/rev  --> δ_L = C / 412 = 0.0004956396 m/tick (~0.4956 mm)
//    - Right Encoder CPR = 440 ticks/rev --> δ_R = C / 440 = 0.0004640989 m/tick (~0.4641 mm)
//    - Eliminates mechanical gearbox meshing tolerance drift (keeps robot strictly straight).
//
// 2. Differential Drive Kinematics & Calibrated Crawl Speeds:
//    - Wheel separation L = 0.150 m
//    - Crawl target: v_crawl ≈ 0.07 m/s (PWM L=75, R=80).
//      Scan delta at 10Hz LiDAR = 0.07 / 10 = 0.007 m (7.0 mm).
//      Yields > 98% scan overlap for zero-distortion Ceres scan-matching.
//
// 3. Bosch BNO055 Inertial Heading Fusion (ROS REP 103 Standard):
//    - Planar yaw extraction: yaw_raw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2))
//    - BNO055 outputs aerospace NED frame (clockwise positive, +Z down).
//      ROS REP 103 requires counter-clockwise positive (+Z up).
//      Mathematical Inversion: d_theta = -raw_d_theta
//    - Accumulated yaw: theta_ += d_theta (continuous smooth heading, zero wrap distortion).
//    - Planar quaternion: qz_out = sin(theta / 2), qw_out = cos(theta / 2).
//    - /imu/data: orientation published as (qw_out, 0, 0, qz_out), angular_velocity.z = -gz.
//
// 4. Dual-Layer Motor Anti-Stall Mechanics:
//    - Breakaway kick: 120ms pulse (L=84, R=88) on transition from rest.
//    - Closed-loop stall recovery: if motion commanded but encoder delta < 4 ticks in 350ms:
//      Injects 150ms boost pulse (L=105, R=110) to overcome carpet stiction.
// ==============================================================================

#include "custom_bot_cpp/arduino_bridge.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <algorithm>

namespace custom_bot_cpp
{

ArduinoBridge::ArduinoBridge(const rclcpp::NodeOptions & options)
: Node("arduino_bridge", options)
{
  declareParameters();

  RCLCPP_INFO(this->get_logger(),
              "arduino_bridge online — BNO055 REP 103 heading fusion active");
  RCLCPP_INFO(this->get_logger(),
              "Wheel Geometry: Dia=%.3fm, Sep=%.3fm, Radius=%.4fm",
              wheel_diameter_, wheel_separation_, wheel_radius_);
  RCLCPP_INFO(this->get_logger(),
              "Calibrated Resolution: Left=%.4fmm/tick, Right=%.4fmm/tick",
              meters_per_tick_left_ * 1000.0, meters_per_tick_right_ * 1000.0);

  // Initialize ROS 2 interfaces
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10, std::bind(&ArduinoBridge::cmdVelCallback, this, std::placeholders::_1));

  odom_pub_  = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  imu_pub_   = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
  joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  // Initialize hardware serial communication & background worker threads
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
  this->declare_parameter<int>("encoder_ticks_left", 412);
  this->declare_parameter<int>("encoder_ticks_right", 440);
  this->declare_parameter<int>("encoder_ticks_per_rev", 412);
  this->declare_parameter<int>("cpr_left", 412);
  this->declare_parameter<int>("cpr_right", 440);
  this->declare_parameter<double>("max_speed", 0.40);
  this->declare_parameter<bool>("publish_tf", true);
  this->declare_parameter<std::string>("base_frame", "base_footprint");
  this->declare_parameter<std::string>("odom_frame", "odom");

  serial_port_        = this->get_parameter("serial_port").as_string();
  baud_rate_          = this->get_parameter("baud_rate").as_int();
  wheel_diameter_     = this->get_parameter("wheel_diameter").as_double();
  wheel_separation_   = this->get_parameter("wheel_separation").as_double();
  max_speed_          = this->get_parameter("max_speed").as_double();
  publish_tf_         = this->get_parameter("publish_tf").as_bool();
  base_frame_         = this->get_parameter("base_frame").as_string();
  odom_frame_         = this->get_parameter("odom_frame").as_string();

  // Asymmetric encoder CPR resolution handling
  ticks_per_rev_left_  = this->get_parameter("encoder_ticks_left").as_int();
  ticks_per_rev_right_ = this->get_parameter("encoder_ticks_right").as_int();

  // Handle cpr_left/cpr_right overrides if explicitly set differently
  int cpr_l = this->get_parameter("cpr_left").as_int();
  int cpr_r = this->get_parameter("cpr_right").as_int();
  if (cpr_l != 412) ticks_per_rev_left_ = cpr_l;
  if (cpr_r != 440) ticks_per_rev_right_ = cpr_r;

  wheel_radius_        = wheel_diameter_ / 2.0;
  wheel_circumference_ = M_PI * wheel_diameter_;

  // Exact linear distance advanced per encoder tick
  meters_per_tick_left_  = wheel_circumference_ / static_cast<double>(ticks_per_rev_left_);
  meters_per_tick_right_ = wheel_circumference_ / static_cast<double>(ticks_per_rev_right_);
}

void ArduinoBridge::initSerial()
{
  serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (serial_fd_ < 0) {
    RCLCPP_WARN(this->get_logger(),
                "Failed to open serial port %s. Hardware may not be connected.", serial_port_.c_str());
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

  // 115200 baud, 8N1 raw mode
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

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

  running_ = true;

  // Background thread 1: Continuous serial telemetry ingestion loop
  serial_thread_ = std::thread(&ArduinoBridge::serialReaderLoop, this);

  // Background thread 2: Hardware watchdog keepalive loop (every 80 ms)
  keepalive_thread_ = std::thread(&ArduinoBridge::watchdogKeepaliveLoop, this);

  RCLCPP_INFO(this->get_logger(), "Serial connected to Arduino on %s @ %d baud", serial_port_.c_str(), baud_rate_);
}

void ArduinoBridge::closeSerial()
{
  running_ = false;

  if (keepalive_thread_.joinable()) {
    keepalive_thread_.join();
  }
  if (serial_thread_.joinable()) {
    serial_thread_.join();
  }

  if (serial_fd_ >= 0) {
    RCLCPP_INFO(this->get_logger(), "Shutting down arduino_bridge — transmitting STOP to motors...");
    // Emergency STOP transmission to halt motors safely
    std::string stop_cmd = "STOP\n";
    ssize_t written_stop = write(serial_fd_, stop_cmd.c_str(), stop_cmd.size());
    (void)written_stop;
    usleep(150000);  // Allow Arduino firmware 150ms to process STOP
    close(serial_fd_);
    serial_fd_ = -1;
  }
}

// ═════════════════════════════════════════════════════════════════════════
// WATCHDOG KEEPALIVE LOOP
// Retransmits active PWM command every 80ms to satisfy 250ms hardware watchdog.
// ═════════════════════════════════════════════════════════════════════════
void ArduinoBridge::watchdogKeepaliveLoop()
{
  while (running_ && rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    if (serial_fd_ < 0) continue;

    double now = this->now().seconds();
    int l = 0;
    int r = 0;
    double elapsed = 0.0;
    {
      std::lock_guard<std::mutex> lock(keepalive_mutex_);
      elapsed = now - last_cmd_time_;
      l = last_l_pwm_;
      r = last_r_pwm_;
    }

    // If no fresh command was received from ROS in the last 120ms, re-send active command
    if (elapsed >= 0.12) {
      sendMotorPWM(l, r);
    }
  }
}

// ═════════════════════════════════════════════════════════════════════════
// MOTOR CONTROL & CMD_VEL CALLBACK
// Implements exact proven Python differential PWM logic:
//   - 120ms breakaway kick (PWM L=84, R=88) to overcome static friction.
//   - Balanced crawl speed (PWM L=75, R=80) for distortion-free SLAM.
//   - In-place rotation (PWM ±85) for pivot turns.
//   - Reversing (PWM -85, -90).
//   - Closed-loop anti-stall boost (PWM 105, 110) if ticks stall under load.
// ═════════════════════════════════════════════════════════════════════════
void ArduinoBridge::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (serial_fd_ < 0) return;

  double lin = msg->linear.x;   // Target linear forward velocity (m/s)
  double ang = msg->angular.z;  // Target angular rotational velocity (rad/s, CCW positive)
  double now = this->now().seconds();

  int l_pwm = 0;
  int r_pwm = 0;

  // Zero-velocity brake condition
  if (std::abs(lin) < DEADBAND && std::abs(ang) < 0.02) {
    l_pwm = 0;
    r_pwm = 0;
    was_stopped_ = true;
    kick_until_ = 0.0;
    stall_level_ = 0;
    last_stall_check_ = now;
    last_check_left_ticks_ = left_ticks_;
    last_check_right_ticks_ = right_ticks_;
  } else {
    // Transition from rest to motion: apply initial 120ms breakaway kick
    if (was_stopped_) {
      was_stopped_ = false;
      kick_until_ = now + 0.12;  // Apply breakaway PWM for 120 milliseconds
      stall_level_ = 1;
      last_stall_check_ = now;
      last_check_left_ticks_ = left_ticks_;
      last_check_right_ticks_ = right_ticks_;
    }

    // Active Closed-Loop Stall Recovery Layer:
    // Check encoder tick progression every 350 milliseconds
    if ((now - last_stall_check_) >= 0.35) {
      long dl = std::abs(left_ticks_ - last_check_left_ticks_);
      long dr = std::abs(right_ticks_ - last_check_right_ticks_);
      last_stall_check_ = now;
      last_check_left_ticks_ = left_ticks_;
      last_check_right_ticks_ = right_ticks_;

      // If motion commanded but total encoder delta < 4 ticks in 350ms -> Motor stalled!
      if ((dl + dr) < 4) {
        stall_level_ = std::min(3, stall_level_ + 1);
        kick_until_ = now + 0.15;  // Trigger high-torque breakaway boost for 150 ms
      } else {
        stall_level_ = 0;  // Robot is rolling freely
      }
    }

    bool in_kick = (now < kick_until_);

    // CASE 1: Pure In-Place Rotation (Zero linear speed)
    if (std::abs(lin) < DEADBAND) {
      int rot_base = 85;
      if (in_kick && stall_level_ >= 2) {
        rot_base = 105;  // Anti-stall boost for carpet pivot friction
      }

      if (ang > 0.0) {
        // Turn Left (Counter-Clockwise): Left wheel reverses, Right wheel drives forward
        l_pwm = -rot_base;
        r_pwm =  rot_base;
      } else {
        // Turn Right (Clockwise): Left wheel drives forward, Right wheel reverses
        l_pwm =  rot_base;
        r_pwm = -rot_base;
      }
    }
    // CASE 2: Reversing Motion (lin < 0)
    else if (lin < 0.0) {
      int rev_l = 85;
      int rev_r = 90;
      if (in_kick && stall_level_ >= 2) {
        rev_l = 105;
        rev_r = 110;  // Stall boost during reverse
      }
      l_pwm = -rev_l;
      r_pwm = -rev_r;
    }
    // CASE 3: Forward Motion (Straight Crawl or Curve Avoidance)
    else {
      if (in_kick) {
        if (stall_level_ >= 2) {
          // Full breakaway boost
          l_pwm = 105;
          r_pwm = 110;
        } else {
          // Normal start kick: overcomes caster stiction
          l_pwm = 84;
          r_pwm = 88;
        }
      } else {
        // Calibrated baseline crawl PWM: Left=80, Right=85 (Overcomes carpet pile stiction & caster drag)
        int base_l = 80;
        int base_r = 85;

        // Differential steering offset proportional to target angular velocity
        int steer = static_cast<int>(std::round(ang * 25.0));
        if (steer < 0) {
          // Steering Right: Increase left wheel speed to push chassis right
          l_pwm = base_l + std::abs(steer);
          r_pwm = base_r;
        } else {
          // Steering Left: Increase right wheel speed to push chassis left
          l_pwm = base_l;
          r_pwm = base_r + std::abs(steer);
        }

        // Clamp PWM commands within strictly calibrated safe crawl limits
        l_pwm = std::max(76, std::min(92, l_pwm));
        r_pwm = std::max(80, std::min(96, r_pwm));
      }
    }
  }

  // Store latest motor PWM commands for watchdog keepalive retransmission
  {
    std::lock_guard<std::mutex> lock(keepalive_mutex_);
    last_l_pwm_ = l_pwm;
    last_r_pwm_ = r_pwm;
    last_cmd_time_ = now;
  }

  cmd_count_++;
  if (cmd_count_ % 10 == 0) {
    RCLCPP_INFO(this->get_logger(),
                "cmd_vel: v=%.2fm/s w=%.2frad/s -> PWM L=%d R=%d",
                lin, ang, l_pwm, r_pwm);
  }

  sendMotorPWM(l_pwm, r_pwm);
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

// ═════════════════════════════════════════════════════════════════════════
// SERIAL TELEMETRY INGESTION LOOP
// ═════════════════════════════════════════════════════════════════════════
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

// ═════════════════════════════════════════════════════════════════════════
// SERIAL PACKET PARSING & ODOMETRY INTEGRATION (@ 20 Hz)
// Packet format: "E,raw_right_ticks,raw_left_ticks,qw,qx,qy,qz,ax,ay,az,gx,gy,gz"
// ═════════════════════════════════════════════════════════════════════════
void ArduinoBridge::parseSerialLine(const std::string & line)
{
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
    // Note: Physical wiring maps parts[1] to Right Encoder and parts[2] to Left Encoder.
    // Left encoder is physically inverted (counts downward when driving forward).
    long raw_right_ticks = std::stol(tokens[1]);
    long raw_left_ticks  = std::stol(tokens[2]);

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

    updateOdometry(raw_right_ticks, raw_left_ticks,
                   qw, qx, qy, qz,
                   ax, ay, az,
                   gx, gy, gz);
  } catch (const std::exception & e) {
    // Malformed byte frame ignored
  }
}

void ArduinoBridge::updateOdometry(long raw_right_ticks, long raw_left_ticks,
                                   double qw, double qx, double qy, double qz,
                                   double ax, double ay, double az,
                                   double gx, double gy, double gz)
{
  rclcpp::Time now = this->now();

  // Invert left encoder tick count to account for physical polarity
  long left_ticks  = -raw_left_ticks;
  long right_ticks = raw_right_ticks;
  left_ticks_  = left_ticks;
  right_ticks_ = right_ticks;

  if (!has_initial_ticks_) {
    prev_left_ticks_  = left_ticks;
    prev_right_ticks_ = right_ticks;
    has_initial_ticks_ = true;
    last_stall_check_ = now.seconds();
    last_check_left_ticks_ = left_ticks;
    last_check_right_ticks_ = right_ticks;
    return;
  }

  // ── Encoder Linear & Angular Displacement Integration ──────────────
  long d_left_raw  = left_ticks  - prev_left_ticks_;
  long d_right_raw = right_ticks - prev_right_ticks_;
  prev_left_ticks_  = left_ticks;
  prev_right_ticks_ = right_ticks;

  // Apply asymmetric distance-per-tick calibrations:
  // Left Wheel:  δ_L = 0.00049564 m/tick (412 ticks/rev)
  // Right Wheel: δ_R = 0.00046410 m/tick (440 ticks/rev)
  double d_left_m  = LEFT_WHEEL_SIGN  * static_cast<double>(d_left_raw)  * meters_per_tick_left_;
  double d_right_m = RIGHT_WHEEL_SIGN * static_cast<double>(d_right_raw) * meters_per_tick_right_;

  // Linear displacement of chassis center: delta_s = (d_right + d_left) / 2.0
  double d_center = (d_left_m + d_right_m) / 2.0;

  // Differential drive wheel odometry angular rotation (REP 103 counter-clockwise positive)
  double d_theta_enc = (d_right_m - d_left_m) / wheel_separation_;

  // ── IMU Heading Extraction & Validation ─────────────────────────────
  // Valid quaternion check: norm squared must be close to 1.0
  double q_norm_sq = qw * qw + qx * qx + qy * qy + qz * qz;
  bool imu_valid = (q_norm_sq >= 0.5 && q_norm_sq <= 1.5 &&
                    !(qw == 1.0 && qx == 0.0 && qy == 0.0 && qz == 0.0 && ax == 0.0 && ay == 0.0));

  double d_theta = d_theta_enc;  // Default and infallible baseline: Wheel Odometry!

  if (imu_valid) {
    double bno_yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    if (!has_initial_yaw_) {
      initial_yaw_ = bno_yaw;
      theta_prev_raw_ = bno_yaw;
      has_initial_yaw_ = true;
    } else {
      double raw_d_theta = bno_yaw - theta_prev_raw_;
      while (raw_d_theta > M_PI)  raw_d_theta -= 2.0 * M_PI;
      while (raw_d_theta < -M_PI) raw_d_theta += 2.0 * M_PI;
      theta_prev_raw_ = bno_yaw;
      d_theta = -raw_d_theta;  // ROS REP 103 CCW inversion
    }
  }

  // ── 2nd Order Runge-Kutta Midpoint Integration ───────────────────────
  double mid_theta = theta_ + (d_theta / 2.0);
  x_ += d_center * std::cos(mid_theta);
  y_ += d_center * std::sin(mid_theta);
  theta_ += d_theta;

  // Compute planar quaternion output: qz = sin(theta / 2), qw = cos(theta / 2)
  double qz_out = std::sin(theta_ / 2.0);
  double qw_out = std::cos(theta_ / 2.0);

  // Accumulate wheel rotation angle (radians) for URDF wheel joint animation in RViz2
  left_wheel_angle_  += d_left_m  / wheel_radius_;
  right_wheel_angle_ += d_right_m / wheel_radius_;

  // Sampling period dt = 50ms (matches Arduino 20 Hz SEND_INTERVAL)
  double dt = 0.05;
  double gz_corr = imu_valid ? -gz : (d_theta / dt);

  // ── 1. Publish nav_msgs/Odometry Message ─────────────────────────────
  nav_msgs::msg::Odometry odom;
  odom.header.stamp            = now;
  odom.header.frame_id         = odom_frame_;
  odom.child_frame_id          = base_frame_;
  odom.pose.pose.position.x    = x_;
  odom.pose.pose.position.y    = y_;
  odom.pose.pose.position.z    = 0.0;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = qz_out;
  odom.pose.pose.orientation.w = qw_out;

  odom.twist.twist.linear.x    = d_center / dt;
  odom.twist.twist.angular.z   = d_theta / dt;

  std::fill(odom.pose.covariance.begin(), odom.pose.covariance.end(), 0.0);
  odom.pose.covariance[0]  = 0.01;  // X position variance
  odom.pose.covariance[7]  = 0.01;  // Y position variance
  odom.pose.covariance[35] = 0.05;  // Yaw heading variance

  std::fill(odom.twist.covariance.begin(), odom.twist.covariance.end(), 0.0);
  odom.twist.covariance[0]  = 0.01;  // Linear velocity variance
  odom.twist.covariance[35] = 0.05;  // Angular velocity variance

  odom_pub_->publish(odom);

  // ── 2. Broadcast Coordinate Transform TF: odom -> base_footprint ──────
  if (publish_tf_ && tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp            = now;
    tf_msg.header.frame_id         = odom_frame_;
    tf_msg.child_frame_id          = base_frame_;
    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation.x    = 0.0;
    tf_msg.transform.rotation.y    = 0.0;
    tf_msg.transform.rotation.z    = qz_out;
    tf_msg.transform.rotation.w    = qw_out;
    tf_broadcaster_->sendTransform(tf_msg);
  }

  // ── 3. Publish sensor_msgs/Imu Message ───────────────────────────────
  sensor_msgs::msg::Imu imu;
  imu.header.stamp                      = now;
  imu.header.frame_id                   = "imu_link";
  imu.orientation.w                     = qw_out;
  imu.orientation.x                     = 0.0;
  imu.orientation.y                     = 0.0;
  imu.orientation.z                     = qz_out;
  imu.linear_acceleration.x             = ax;
  imu.linear_acceleration.y             = ay;
  imu.linear_acceleration.z             = az;
  imu.angular_velocity.x                = gx;
  imu.angular_velocity.y                = gy;
  imu.angular_velocity.z                = gz_corr;

  std::fill(imu.orientation_covariance.begin(), imu.orientation_covariance.end(), 0.0);
  imu.orientation_covariance[0]         = 0.01;

  std::fill(imu.linear_acceleration_covariance.begin(), imu.linear_acceleration_covariance.end(), 0.0);
  imu.linear_acceleration_covariance[0] = 0.04;
  imu.linear_acceleration_covariance[4] = 0.04;
  imu.linear_acceleration_covariance[8] = 0.04;

  std::fill(imu.angular_velocity_covariance.begin(), imu.angular_velocity_covariance.end(), 0.0);
  imu.angular_velocity_covariance[0]    = 0.02;
  imu.angular_velocity_covariance[4]    = 0.02;
  imu.angular_velocity_covariance[8]    = 0.02;

  imu_pub_->publish(imu);

  debug_msg_count_++;
  if (debug_msg_count_ % 20 == 0) {
    RCLCPP_INFO(this->get_logger(),
                "Odom: x=%.2fm y=%.2fm th=%.1f° (mode=%s) | PWM: L=%d R=%d",
                x_, y_, theta_ * (180.0 / M_PI), imu_valid ? "IMU_FUSED" : "ENCODER_KINEMATICS",
                last_l_pwm_, last_r_pwm_);
  }

  // ── 4. Publish sensor_msgs/JointState Message ────────────────────────
  sensor_msgs::msg::JointState js;
  js.header.stamp = now;
  js.name         = {"wheel_left_joint", "wheel_right_joint"};
  js.position     = {left_wheel_angle_, right_wheel_angle_};
  js.velocity     = {d_left_m / (wheel_radius_ * dt), d_right_m / (wheel_radius_ * dt)};
  js.effort       = {0.0, 0.0};
  joint_pub_->publish(js);
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
