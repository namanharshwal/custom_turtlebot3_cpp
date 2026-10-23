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
# Description: SLAM (Simultaneous Localization and Mapping) launch file.
# Launches slam_toolbox in asynchronous mode to generate a map from LiDAR (/scan) and Odometry (/odom).
# Nodes launched: slam_toolbox
# ==============================================================================

"""
mapping.launch.py
=================================================
An all-in-one launch file that starts:
1. The hardware stack (robot_full.launch.py)
2. The SLAM toolbox for mapping (delayed by 5s for gyro calibration)
3. RViz2 for visualization (delayed by 6s)

NOTE: Teleop MUST be run in a separate terminal.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_custom_bot = get_package_share_directory('custom_bot_cpp')

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_custom_bot, 'launch', 'robot_full.launch.py')
        )
    )

    slam_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_custom_bot, 'launch', 'slam.launch.py')
                )
            )
        ]
    )

    # Launch RViz2 delayed by 6s to allow SLAM to spin up
    # rviz_node = TimerAction(
    #     period=6.0,
    #     actions=[
    #         Node(
    #             package='rviz2',
    #             executable='rviz2',
    #             name='rviz2',
    #             output='screen'
    #         )
    #     ]
    # )

    return LaunchDescription([
        hardware_launch,
        slam_launch
        # rviz_node
    ])