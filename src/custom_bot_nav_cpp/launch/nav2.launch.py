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
# Package:              custom_bot_nav
# Launch File:          nav2.launch.py
# License:              Apache License 2.0 (c) 2026 Naman Harshwal
# 
# Description:
# Autonomous Navigation (Nav2) launch file.
# Integrates Nav2 stack bringup with costmaps, DWB local planner, NavFn global planner,
# and behavior trees for waypoint-directed autonomous navigation.
# ==============================================================================

"""
nav2.launch.py
Nav2 autonomous navigation — run AFTER robot_full.launch.py + map saved
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    nav2_dir = get_package_share_directory('nav2_bringup')
    nav_pkg_dir = get_package_share_directory('custom_bot_nav_cpp')

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    default_map = os.path.join(nav_pkg_dir, 'maps', 'room_map.yaml')

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map yaml file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav_pkg_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use')

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'map': map_file,
            'params_file': params_file,
        }.items()
    )

    return LaunchDescription([
        declare_map_cmd,
        declare_params_file_cmd,
        bringup_cmd
    ])