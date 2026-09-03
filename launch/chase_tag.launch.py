#!/usr/bin/env python3

"""Launch the Gazebo arena, three Nav2 stacks, and chase/tag game."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)


def generate_launch_description():
    package_dir = get_package_share_directory('tb3_multi_robot')
    launch_dir = os.path.join(package_dir, 'launch')
    robot_config = os.path.join(
        package_dir, 'config', 'chase_tag_robots.yaml'
    )

    world_name = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_config_arg = LaunchConfiguration('robot_config')
    map_path = LaunchConfiguration('map')

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            launch_dir, 'tb3_world.launch.py'
        )),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': LaunchConfiguration('gui'),
            'clock_rate': LaunchConfiguration('clock_rate'),
            'world': world_name,
            'robot_config': robot_config_arg,
            # Keep tb1's exact odometry as its Nav2 source. Every robot still
            # exposes independent ground truth for game-rule evaluation.
            'mapping_use_ground_truth_odom': 'true',
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            launch_dir, 'chase_tag_nav.launch.py'
        )),
        launch_arguments={
            'robot_config': robot_config_arg,
            'map': map_path,
            'use_sim_time': use_sim_time,
            'autostart': LaunchConfiguration('autostart'),
            'log_level': LaunchConfiguration('log_level'),
            'rviz': LaunchConfiguration('rviz'),
            'rviz_render_threads': LaunchConfiguration(
                'rviz_render_threads'
            ),
            'red_linear_speed': LaunchConfiguration('red_linear_speed'),
            'blue_linear_speed': LaunchConfiguration('blue_linear_speed'),
            'control_rate': LaunchConfiguration('control_rate'),
            'goal_update_rate': LaunchConfiguration('goal_update_rate'),
            'goal_deadband': LaunchConfiguration('goal_deadband'),
            'tag_distance': LaunchConfiguration('tag_distance'),
            'tag_pause': LaunchConfiguration('tag_pause'),
            'reset_distance': LaunchConfiguration('reset_distance'),
            'score_to_win': LaunchConfiguration('score_to_win'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('ros_domain_id', default_value='74'),
        SetEnvironmentVariable(
            'ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')
        ),
        DeclareLaunchArgument(
            'gz_partition',
            default_value=[
                TextSubstitution(text='tb3_chase_tag_'),
                LaunchConfiguration('ros_domain_id'),
            ],
            description=(
                'Gazebo Transport partition; unique ROS domains are isolated '
                'from stale or concurrent simulator discovery'
            ),
        ),
        SetEnvironmentVariable(
            'GZ_PARTITION', LaunchConfiguration('gz_partition')
        ),
        DeclareLaunchArgument(
            'world',
            default_value='open_arena',
            description=(
                'Scenario: open_arena, corridor, obstacle_course, or '
                'tb3_world'
            ),
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('clock_rate', default_value='250.0'),
        DeclareLaunchArgument(
            'robot_config',
            default_value=robot_config,
            description='Robot poses, colors, and team assignments',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                package_dir, 'map', [world_name, '.yaml']
            ]),
            description='Shared occupancy map used by all three robots',
        ),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'rviz_render_threads',
            default_value=EnvironmentVariable(
                'LP_NUM_THREADS', default_value='2'
            ),
        ),
        DeclareLaunchArgument('red_linear_speed', default_value='0.22'),
        DeclareLaunchArgument('blue_linear_speed', default_value='0.16'),
        DeclareLaunchArgument('control_rate', default_value='10.0'),
        DeclareLaunchArgument('goal_update_rate', default_value='1.0'),
        DeclareLaunchArgument('goal_deadband', default_value='0.15'),
        DeclareLaunchArgument('tag_distance', default_value='0.35'),
        DeclareLaunchArgument('tag_pause', default_value='1.5'),
        DeclareLaunchArgument('reset_distance', default_value='1.0'),
        DeclareLaunchArgument(
            'score_to_win',
            default_value='0',
            description='Tags needed to stop; 0 keeps playing rounds',
        ),
        world,
        # Gazebo and all three bridges should exist before Nav2 discovery.
        TimerAction(period=5.0, actions=[navigation]),
    ])
