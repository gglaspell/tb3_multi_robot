#!/usr/bin/env python3
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

"""
follow_tb3.launch.py

Launches the full tb1-leader / tb3-follower scenario:

  1. Nav2 for tb1  → burger_nav2_params.yaml       (normal NavigateToPose)
  2. Nav2 for tb3  → burger_nav2_params_tb3.yaml    (FollowDynamicPoint BT)
  3. tb3_follow_tb1 node                            (publishes trailing goal)
  4. Two RViz instances (one per robot)

Usage
-----
  ros2 launch tb3_multi_robot follow_tb3.launch.py

Optional overrides
------------------
  map:=<path>              – path to map.yaml (default: package map/map.yaml)
  use_sim_time:=true/false – (default true)
  follow_distance:=0.5     – metres tb3 trails behind tb1 (default 0.5)
  publish_rate:=2.0        – Hz goal is republished (default 2.0)
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from multi_robot_scripts.utils import generate_rviz_config


def _nav2_bringup(robot_name: str, params_path: str, map_path, use_sim_time) -> IncludeLaunchDescription:
    """Return a namespaced Nav2 bringup action for a single robot."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py',
            )
        ),
        launch_arguments={
            'map':          map_path,
            'use_sim_time': use_sim_time,
            'params_file':  params_path,
            'use_namespace': 'true',
            'namespace':     robot_name,
        }.items(),
    )


def generate_launch_description():
    # ── Paths ─────────────────────────────────────────────────────────────────
    pkg_dir  = get_package_share_directory('tb3_multi_robot')
    tb3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')

    default_map    = os.path.join(pkg_dir, 'map',    'map.yaml')
    params_tb1     = os.path.join(pkg_dir, 'params', f'{tb3_model}_nav2_params.yaml')
    params_tb3     = os.path.join(pkg_dir, 'params', f'{tb3_model}_nav2_params_tb3.yaml')
    rviz_template  = os.path.join(pkg_dir, 'rviz',   'tb3_navigation2.rviz')

    # ── Launch arguments ──────────────────────────────────────────────────────
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map.yaml',
    ))
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock',
    ))
    ld.add_action(DeclareLaunchArgument(
        'follow_distance',
        default_value='0.5',
        description='Metres tb3 trails behind tb1',
    ))
    ld.add_action(DeclareLaunchArgument(
        'publish_rate',
        default_value='2.0',
        description='Hz at which the follow-goal is republished',
    ))
    ld.add_action(DeclareLaunchArgument(
        'heading_history_size',
        default_value='5',
        description='Number of recent tb1 poses used to estimate heading',
    ))
    ld.add_action(DeclareLaunchArgument(
        'deadband_distance',
        default_value='0.1',
        description='Minimum goal displacement (m) before republishing',
    ))

    map_path     = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── tb1: normal Nav2 bringup ──────────────────────────────────────────────
    ld.add_action(LogInfo(msg='[follow_tb3] Launching Nav2 for tb1 (NavigateToPose mode)'))
    ld.add_action(_nav2_bringup('tb1', params_tb1, map_path, use_sim_time))

    rviz_tb1 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace='/tb1',
        arguments=['-d', generate_rviz_config('tb1', rviz_template)],
        parameters=[{'use_sim_time': use_sim_time, 'log_level': 'warn'}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen',
    )
    ld.add_action(rviz_tb1)

    # ── tb3: follow-mode Nav2 bringup ─────────────────────────────────────────
    ld.add_action(LogInfo(msg='[follow_tb3] Launching Nav2 for tb3 (FollowDynamicPoint mode)'))
    ld.add_action(_nav2_bringup('tb3', params_tb3, map_path, use_sim_time))

    rviz_tb3 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace='/tb3',
        arguments=['-d', generate_rviz_config('tb3', rviz_template)],
        parameters=[{'use_sim_time': use_sim_time, 'log_level': 'warn'}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        output='screen',
    )
    ld.add_action(rviz_tb3)

    # ── tb3_follow_tb1 node ───────────────────────────────────────────────────
    ld.add_action(LogInfo(msg='[follow_tb3] Starting tb3_follow_tb1 node'))
    follow_node = Node(
        package='tb3_multi_robot',
        executable='tb3_follow_tb1',
        name='tb3_follow_tb1',
        output='screen',
        parameters=[{
            'use_sim_time':         use_sim_time,
            'follow_distance':      LaunchConfiguration('follow_distance'),
            'publish_rate':         LaunchConfiguration('publish_rate'),
            'heading_history_size': LaunchConfiguration('heading_history_size'),
            'deadband_distance':    LaunchConfiguration('deadband_distance'),
        }],
    )
    ld.add_action(follow_node)

    return ld
