#!/usr/bin/env python3
# Copyright 2026 Naman Harshwal
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# ==============================================================================
# Creator & Maintainer: Naman Harshwal (namanharshwal@gmail.com)
# Owner/Supervisor:     Pradeep Singh Chauhan
# Workspace:            psnh_turtlebot_ws
# License:              Apache License 2.0 (c) 2026 Naman Harshwal
# 
# Description: Main hardware bringup launch file for the entire robot.
# Launches the robot state publisher, YDLidar driver, Arduino bridge, and Camera.
# Nodes launched: robot_state_publisher, ydlidar_ros2_driver_node, arduino_bridge, v4l2_camera.
# ==============================================================================

"""
robot_full.launch.py  —  v3.4  FINAL  custom_bot
=================================================
Hardware confirmed working:
  ✅ YDLidar X2          /dev/ydlidar    → /scan
  ✅ Arduino UNO         /dev/ttyAMA0    → /odom  /imu/data  /joint_states
  ✅ MPU6050 IMU         I2C on Arduino  → gyro-Z heading fusion
  ✅ N25 Hall Encoders   D2/D3 INT pins  → wheel TF synced to real rotation
  ✅ RPi Camera Module v1 ov5647 CSI     → /camera/image_raw

v3.4 fix vs v3.3:
  ✅ CRITICAL: frame_id "base_scan" → "laser_frame"
     This was the root cause of all SLAM/RViz failures.
  ✅ range_min raised 0.12 → 0.15 to suppress self-return noise

Launch sequence:
  t=0s   robot_state_publisher  (URDF TF tree)
  t=0s   ydlidar_node           (laser scan, frame_id=laser_frame)
  t=2s   arduino_bridge         (odom + IMU + joint_states)
  t=3s   camera_node            (live camera feed)

Usage:
  Terminal 1: ros2 launch custom_bot robot_full.launch.py
  Terminal 2: ros2 launch custom_bot slam.launch.py   (after gyro calib msg)
  Terminal 3: ros2 run custom_bot teleop_key_hold.py

RViz:
  Fixed Frame → odom  (switch to map after SLAM starts)
  Add → RobotModel     → /robot_description
  Add → LaserScan      → /scan
  Add → Odometry       → /odom
  Add → Map            → /map              (only after slam.launch.py)
  Add → Camera         → /camera/image_raw (Overlay Alpha 0.5)

TF chain (fully verified):
  map → odom → base_footprint → base_link → laser_frame          ✅
                                           → imu_link             ✅
                                           → camera_link
                                             → camera_optical_frame ✅
                                           → wheel_left_link  (dynamic) ✅
                                           → wheel_right_link (dynamic) ✅
"""

import os
import xacro

from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# ── Camera hardware check ──────────────────────────────────────
# True if CSI/USB camera device /dev/video0 is present
CAMERA_AVAILABLE = os.path.exists('/dev/video0')


def generate_launch_description():

    pkg_path   = get_package_share_directory("custom_bot_cpp")
    xacro_file = os.path.join(pkg_path, "description", "robot.urdf.xacro")
    robot_desc = xacro.process_file(xacro_file).toxml()

    nodes = []

    # ════════════════════════════════════════════════════════════
    # 1. ROBOT STATE PUBLISHER
    #
    #    Reads robot.urdf.xacro → publishes static TFs for ALL
    #    fixed joints automatically:
    #      base_footprint → base_link
    #      base_link      → laser_frame       (lidar — v3.4 name)
    #      base_link      → imu_link
    #      base_link      → camera_link
    #      camera_link    → camera_optical_frame
    #      base_link      → caster_back_link
    #
    #    Also listens to /joint_states and publishes DYNAMIC TFs:
    #      base_link → wheel_left_link
    #      base_link → wheel_right_link
    #    /joint_states source = arduino_bridge.py (real encoder data)
    #
    #    ⚠️  DO NOT run joint_state_publisher alongside this.
    #        It will overwrite encoder data with zeros and freeze
    #        the wheel meshes in RViz permanently.
    # ════════════════════════════════════════════════════════════
    nodes.append(
        Node(
            package    = "robot_state_publisher",
            executable = "robot_state_publisher",
            name       = "robot_state_publisher",
            output     = "screen",
            parameters = [{
                "robot_description": robot_desc,
                "use_sim_time":      False,
            }],
        )
    )

    # ════════════════════════════════════════════════════════════
    # 2. YDLIDAR X2
    #
    #    Port     : /dev/ydlidar  (udev symlink)
    #    Publishes: /scan  frame_id: laser_frame  @ 7 Hz
    #
    #    v3.4 FIX: frame_id "base_scan" → "laser_frame"
    #    MUST match the link name in robot.urdf.xacro v4.
    #    Mismatch was root cause of:
    #      "Filter dropping message: frame base_scan" in SLAM
    #      "Frame [map] does not exist" in RViz
    #
    #    ℹ️  "Fail to get baseplate device information" → NORMAL for X2
    #    ℹ️  "Real point count > fixed point count"     → NORMAL for X2
    # ════════════════════════════════════════════════════════════
    nodes.append(
        Node(
            package    = "ydlidar_ros2_driver",
            executable = "ydlidar_ros2_driver_node",
            name       = "ydlidar_node",
            output     = "screen",
            parameters = [{
                "port":                 "/dev/ydlidar",
                "frame_id":             "laser_frame",
                "baudrate":             115200,
                "lidar_type":           1,
                "device_type":          0,
                "sample_rate":          3,
                "isSingleChannel":      True,
                "intensity":            False,
                "frequency":            10.0,
                "angle_max":            180.0,
                "angle_min":           -180.0,
                "range_max":            8.0,
                "range_min":            0.15,
                "resolution_fixed":     True,
                "auto_reconnect":       True,
                "reversion":            False,
                "inverted":             False,
                "abnormal_check_count": 4,
                "invalid_range_is_inf": True,
            }],
        )
    )

    # ════════════════════════════════════════════════════════════
    # 3. ARDUINO BRIDGE  (2s delay — serial port needs boot time)
    #
    #    Port       : /dev/ttyAMA0  (Pi GPIO UART)
    #    Subscribes : /cmd_vel  → sends "M,<l_pwm>,<r_pwm>\n"
    #    Publishes  : /odom         (IMU gyro-Z + encoder odometry)
    #                 /imu/data     (raw MPU6050 accel + gyro)
    #                 /joint_states (wheel angles from encoder ticks)
    #    TF         : odom → base_footprint  (broadcast live)
    #
    #    ⚠️  GYRO CALIBRATION: Keep robot COMPLETELY STILL for ~4s.
    #        Wait for: "✅ Gyro calibrated — bias = X.XXXXXX rad/s"
    #        before launching slam.launch.py or driving the robot.
    #
    #    CALIBRATE encoder_ticks_per_rev if /odom drifts:
    #      N25 common values:
    #        1:20 gear = 440    1:34 gear = 748    1:48 gear = 1056
    #
    #    CALIBRATE wheel_separation if turns are inaccurate:
    #      actual > 360° → increase wheel_separation value
    #      actual < 360° → decrease wheel_separation value
    # ════════════════════════════════════════════════════════════
    nodes.append(
        TimerAction(
            period  = 2.0,
            actions = [
                Node(
                    package    = "custom_bot_cpp",
                    executable = "arduino_bridge",
                    name       = "arduino_bridge",
                    output     = "screen",
                    parameters = [{
                        "serial_port":           "/dev/ttyAMA0",
                        "baud_rate":              115200,
                        "wheel_diameter":         0.065,
                        "wheel_separation":       0.150,
                        "encoder_ticks_per_rev":  412,
                        "max_speed":              0.5,
                        "publish_tf":             True,
                    }],
                ),
            ],
        )
    )

    # ════════════════════════════════════════════════════════════
    # 3.5 ROBOT LOCALIZATION (EKF)
    #
    #    Fuses /odom (wheel encoders) and /imu/data (gyro-Z)
    #    Publishes: TF odom → base_footprint and /odometry/filtered
    # ════════════════════════════════════════════════════════════
    # ekf_config_path = os.path.join(pkg_path, "config", "ekf.yaml")
    # nodes.append(
    #     Node(
    #         package    = "robot_localization",
    #         executable = "ekf_node",
    #         name       = "ekf_filter_node",
    #         output     = "screen",
    #         parameters = [ekf_config_path],
    #     )
    # )

    # ════════════════════════════════════════════════════════════
    # 4. RASPBERRY PI CAMERA MODULE v1  —  ov5647  (3s delay)
    #
    #    Driver  : camera_ros  (libcamera native)
    #    Publishes: /camera/image_raw    (RGB888  640×480)
    #               /camera/camera_info
    #    frame_id : camera_optical_frame  (matches URDF)
    #
    #    /boot/firmware/config.txt MUST contain:
    #      dtoverlay=ov5647        ← NO ",media-controller=0" suffix
    #      camera_auto_detect=0
    #      gpu_mem=128
    #
    #    Install: sudo apt install ros-humble-camera-ros
    #
    #    RViz: Add → By topic → /camera/image_raw → Camera
    #          Overlay Alpha: 0.5
    #
    #    ✅ CRASH ISOLATION: camera crash does NOT affect other nodes.
    # ════════════════════════════════════════════════════════════
    if CAMERA_AVAILABLE:

        camera_node = Node(
            package    = "custom_bot_cpp",
            executable = "object_detector_node",
            name       = "object_detector_node",
            output     = "screen",
            parameters = [{
                "video_device": 0,
                "capture_width": 640,
                "capture_height": 480,
                "input_size": 320,
                "conf_threshold": 0.20,
                "nms_threshold": 0.40,
                "detection_rate_hz": 2.0,
                "publish_rate_hz": 10.0,
                "publish_raw_image": True,
                "analogue_gain": 350,
                "exposure": 1200,
            }],
        )

        camera_crash_guard = RegisterEventHandler(
            OnProcessExit(
                target_action = camera_node,
                on_exit       = [
                    LogInfo(msg=(
                        "\n"
                        "╔══════════════════════════════════════════════════════════════╗\n"
                        "║  ⚠️  object_detector_node exited — robot continues           ║\n"
                        "║  LiDAR, odometry, and SLAM are completely unaffected.        ║\n"
                        "╚══════════════════════════════════════════════════════════════╝"
                    ))
                ],
            )
        )

        nodes.append(TimerAction(period=3.0, actions=[camera_node]))
        nodes.append(camera_crash_guard)

    else:
        nodes.append(
            LogInfo(msg=(
                "\n"
                "╔══════════════════════════════════════════════════════════════╗\n"
                "║  ⚠️  camera_ros not installed — launching without camera     ║\n"
                "║  Install: sudo apt install ros-humble-camera-ros             ║\n"
                "╚══════════════════════════════════════════════════════════════╝"
            ))
        )

    return LaunchDescription(nodes)