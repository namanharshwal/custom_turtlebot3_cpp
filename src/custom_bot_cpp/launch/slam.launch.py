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
# Launch File:          slam.launch.py
# License:              Apache License 2.0 (c) 2026 Naman Harshwal
# 
# Description:
# Asynchronous SLAM Toolbox launcher for real-time 2D spatial mapping.
# Subscribes to /scan (10 Hz laser range scans) and /odom (differential wheel odometry
# fused with Bosch BNO055 heading), generating the global /map occupancy grid and
# broadcasting the dynamic map -> odom transform via Ceres non-linear scan matching.
# ==============================================================================

"""
slam.launch.py  -- run AFTER robot_full.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('custom_bot_cpp'),
        'config',
        'slam_toolbox_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                config,
                {'use_lifecycle_manager': False},
            ],
        )
    ])