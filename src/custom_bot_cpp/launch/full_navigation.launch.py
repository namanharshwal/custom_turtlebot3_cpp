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
# Workspace:            psnh_turtlebot_cpp_ws
# License:              Apache License 2.0 (c) 2026 Naman Harshwal
# 
# Description: Autonomous Navigation (Nav2) launch file.
# Launches hardware stack, Nav2 stack with saved map, AMCL, and RViz2.
# ==============================================================================

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_custom_bot = get_package_share_directory('custom_bot_cpp')
    pkg_custom_bot_nav = get_package_share_directory('custom_bot_nav_cpp')

    default_map = os.path.join(pkg_custom_bot, 'maps', 'room_map.yaml')
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map yaml file to load'
    )
    map_config = LaunchConfiguration('map')

    # 1. Hardware Launch
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_custom_bot, 'launch', 'robot_full.launch.py')
        )
    )

    # 2. Navigation Launch (delayed by 5s)
    nav_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_custom_bot_nav, 'launch', 'nav2.launch.py')
                ),
                launch_arguments={'map': map_config}.items()
            )
        ]
    )

    # 3. RViz2 (delayed by 8s)
    rviz_config = os.path.join(pkg_custom_bot, 'config', 'nav2.rviz')
    if not os.path.exists(rviz_config):
        nav2_bringup_dir = get_package_share_directory('nav2_bringup')
        rviz_config = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    rviz_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config]
            )
        ]
    )

    return LaunchDescription([
        map_arg,
        hardware_launch,
        nav_launch,
        rviz_node
    ])
