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
# Package:              custom_bot
# Launch File:          robot_hw.launch.py
# License:              Apache License 2.0 (c) 2026 Naman Harshwal
# 
# Description:
# Hardware abstraction launcher bringing up:
# 1. robot_state_publisher: parses robot.urdf.xacro and broadcasts static/joint TF frames
# 2. ydlidar_ros2_driver_node: 10 Hz 2D planar LiDAR on /dev/ydlidar (frame_id: laser_frame)
# 3. arduino_bridge: UART interface to Arduino UNO on /dev/ttyAMA0 @ 115200 baud
# ==============================================================================

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    pkg = get_package_share_directory('custom_bot_cpp')
    xacro_file = os.path.join(pkg, 'description', 'robot.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False,
        }]
    )

    lidar = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_node',
        output='screen',
        parameters=[{
            'port':                 '/dev/ydlidar',
            'frame_id':             'laser_frame',
            'baudrate':             115200,
            'lidar_type':           1,
            'device_type':          0,
            'sample_rate':          3,
            'isSingleChannel':      True,
            'intensity':            False,
            'frequency':            5.0,
            'angle_max':            180.0,
            'angle_min':           -180.0,
            'range_max':            8.0,
            'range_min':            0.15,
            'resolution_fixed':     True,
            'auto_reconnect':       True,
            'reversion':            False,
            'inverted':             False,
            'abnormal_check_count': 4,
            'invalid_range_is_inf': False,
        }]
    )

    arduino = Node(
        package='custom_bot_cpp',
        executable='arduino_bridge',
        name='arduino_bridge',
        output='screen',
        parameters=[{
            'serial_port':           '/dev/ttyAMA0',
            'baud_rate':              115200,
            'wheel_diameter':         0.065,
            'wheel_separation':       0.150,
            'encoder_ticks_per_rev':  440,
            'max_speed':              0.5,
        }]
    )

    return LaunchDescription([robot_state_publisher, lidar, arduino])