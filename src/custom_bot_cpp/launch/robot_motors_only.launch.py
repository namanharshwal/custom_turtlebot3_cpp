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
# Launch File:          robot_motors_only.launch.py
# License:              Apache License 2.0 (c) 2026 Naman Harshwal
# 
# Description:
# Lightweight hardware diagnostics launcher bringing up solely the Arduino serial
# bridge and static base/IMU transforms for motor testing and encoder calibration.
# ==============================================================================


from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        
        # Arduino Bridge Node ONLY
        Node(
            package='custom_bot_cpp',
            executable='arduino_bridge',
            name='arduino_bridge',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyAMA0',
                'baud_rate': 115200,
                'wheel_diameter': 0.065,
                'wheel_separation': 0.15,
                'encoder_slots': 20,
                'max_speed': 0.5
            }]
        ),
        
        # Static TF: base_footprint → base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
        ),
        
        # Static TF: base_link → imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu_tf',
            arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link']
        )
    ])