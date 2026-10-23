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
# autonomous_mapping.launch.py — Complete Autonomous Mapping with RViz2
#
# Launches:
#   1. mapping.launch.py  → robot_full (LiDAR, IMU, Camera, RSP) + SLAM Toolbox
#   2. RViz2              → Live visualization with optimized display config
#   3. simple_explorer    → Continuous reactive exploration (wall-follow + avoid)
# ==============================================================================

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_custom_bot = get_package_share_directory('custom_bot_cpp')
    rviz_config_path = '/home/ps/.rviz2/default.rviz'

    # 1. Hardware + SLAM (LiDAR, encoders, IMU, camera, SLAM Toolbox)
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_custom_bot, 'launch', 'mapping.launch.py')
        )
    )

    # 2. RViz2 (delayed 7s to let SLAM and robot_state_publisher start)
    rviz_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_path],
                output='screen'
            )
        ]
    )

    # 3. Simple Explorer (delayed 10s to let SLAM establish initial /map)
    explorer_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='custom_bot_cpp',
                executable='simple_explorer',
                name='simple_explorer',
                output='screen',
                parameters=[{
                    'linear_speed': 0.05,       # m/s — slowest crawl for clean distortion-free mapping
                    'angular_speed': 0.25,      # rad/s — gentle turn rate to prevent scan smearing
                    'obstacle_dist': 0.45,      # m — start curve avoidance (safe distance from walls)
                    'critical_dist': 0.30,      # m — in-place rotate threshold (never get closer than 30cm)
                    'clear_dist': 0.38,         # m — resume forward drive threshold
                    'wall_follow_dist': 0.40,   # m — wall-following distance
                    'control_hz': 10.0,         # Hz — cmd_vel publish rate
                }]
            )
        ]
    )

    return LaunchDescription([
        mapping_launch,
        rviz_node,
        explorer_node,
    ])
