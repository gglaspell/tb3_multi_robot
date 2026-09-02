#!/usr/bin/env python3

"""Launch the Gazebo world and the complete tb1/tb3 follow scenario."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_dir = get_package_share_directory('tb3_multi_robot')
    launch_dir = os.path.join(package_dir, 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    clock_rate = LaunchConfiguration('clock_rate')
    rviz = LaunchConfiguration('rviz')
    follow_distance = LaunchConfiguration('follow_distance')
    publish_rate = LaunchConfiguration('publish_rate')
    stationary_threshold = LaunchConfiguration('stationary_threshold')
    stationary_angular_threshold = LaunchConfiguration(
        'stationary_angular_threshold'
    )
    use_composition = LaunchConfiguration('use_composition')
    log_level = LaunchConfiguration('log_level')

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'tb3_world.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': gui,
            'clock_rate': clock_rate,
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'follow_tb3.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': rviz,
            'follow_distance': follow_distance,
            'publish_rate': publish_rate,
            'stationary_threshold': stationary_threshold,
            'stationary_angular_threshold': stationary_angular_threshold,
            'use_composition': use_composition,
            'log_level': log_level,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo simulation clock',
        ),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Start the Gazebo graphical client',
        ),
        DeclareLaunchArgument(
            'clock_rate', default_value='250.0',
            description='Maximum ROS /clock publication rate in Hz',
        ),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start one RViz instance for each robot',
        ),
        DeclareLaunchArgument(
            'follow_distance', default_value='0.5',
            description='Metres tb3 trails behind tb1',
        ),
        DeclareLaunchArgument(
            'publish_rate', default_value='2.0',
            description='Hz at which follower goals are evaluated',
        ),
        DeclareLaunchArgument(
            'stationary_threshold', default_value='0.05',
            description='Leader translation threshold used to detect motion',
        ),
        DeclareLaunchArgument(
            'stationary_angular_threshold', default_value='0.1',
            description='Leader rotation threshold used to detect motion',
        ),
        DeclareLaunchArgument(
            'use_composition', default_value='True',
            description='Run each Nav2 stack in a component container',
        ),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='Nav2 logging level',
        ),
        world,
        # Give Gazebo time to advertise its create service before Nav2 starts.
        TimerAction(period=5.0, actions=[navigation]),
    ])
