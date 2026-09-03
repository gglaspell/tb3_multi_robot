#!/usr/bin/env python3

"""Launch the Gazebo world and the complete tb1/tb3 follow scenario."""

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
)


def generate_launch_description():
    package_dir = get_package_share_directory('tb3_multi_robot')
    launch_dir = os.path.join(package_dir, 'launch')

    ros_domain_id = LaunchConfiguration('ros_domain_id')
    world_name = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    clock_rate = LaunchConfiguration('clock_rate')
    rviz = LaunchConfiguration('rviz')
    rviz_render_threads = LaunchConfiguration('rviz_render_threads')
    map_path = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    auto_map = LaunchConfiguration('auto_map')
    follow_distance = LaunchConfiguration('follow_distance')
    publish_rate = LaunchConfiguration('publish_rate')
    heading_history_size = LaunchConfiguration('heading_history_size')
    deadband_distance = LaunchConfiguration('deadband_distance')
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
            'software_rendering': LaunchConfiguration('software_rendering'),
            'clock_rate': clock_rate,
            'world': world_name,
            'mapping_use_ground_truth_odom': LaunchConfiguration(
                'mapping_use_ground_truth_odom'
            ),
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'follow_tb3.launch.py')
        ),
        launch_arguments={
            'map': map_path,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'auto_map': auto_map,
            'mapping_use_ground_truth_odom': LaunchConfiguration(
                'mapping_use_ground_truth_odom'
            ),
            'use_ground_truth_pose': LaunchConfiguration(
                'use_ground_truth_pose'
            ),
            'auto_mapper_startup_delay': LaunchConfiguration(
                'auto_mapper_startup_delay'
            ),
            'auto_mapper_min_free_neighbors': LaunchConfiguration(
                'auto_mapper_min_free_neighbors'
            ),
            'map_output_path': LaunchConfiguration('map_output_path'),
            'rviz': rviz,
            'rviz_render_threads': rviz_render_threads,
            'follow_distance': follow_distance,
            'publish_rate': publish_rate,
            'heading_history_size': heading_history_size,
            'deadband_distance': deadband_distance,
            'stationary_threshold': stationary_threshold,
            'stationary_angular_threshold': stationary_angular_threshold,
            'use_composition': use_composition,
            'log_level': log_level,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'ros_domain_id', default_value='73',
            description='ROS domain used by every process in this scenario',
        ),
        SetEnvironmentVariable('ROS_DOMAIN_ID', ros_domain_id),
        DeclareLaunchArgument(
            'world', default_value='tb3_world',
            description=(
                'Scenario name: tb3_world, open_arena, corridor, or '
                'obstacle_course'
            ),
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo simulation clock',
        ),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Start the Gazebo graphical client',
        ),
        DeclareLaunchArgument(
            'software_rendering',
            default_value=EnvironmentVariable(
                'TB3_SOFTWARE_RENDERING', default_value='false'
            ),
            description='Force Mesa software rendering instead of GPU use',
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
            'rviz_render_threads',
            default_value=EnvironmentVariable(
                'LP_NUM_THREADS', default_value='8'
            ),
            description='Mesa software-rendering threads per RViz process',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                package_dir,
                'map',
                [world_name, '.yaml'],
            ]),
            description='Map YAML; defaults to the map matching world',
        ),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically activate both Nav2 stacks',
        ),
        DeclareLaunchArgument(
            'auto_map', default_value='true',
            description='Have tb1 build the map and explore its frontiers',
        ),
        DeclareLaunchArgument(
            'mapping_use_ground_truth_odom', default_value='true',
            description=(
                'Use stable Gazebo odometry for tb1 SLAM while retaining '
                'wheel odometry on /tb1/wheel_odom'
            ),
        ),
        DeclareLaunchArgument(
            'use_ground_truth_pose', default_value='true',
            description=(
                'Use slip-free Gazebo pose for cross-map follower accuracy'
            ),
        ),
        DeclareLaunchArgument(
            'auto_mapper_startup_delay', default_value='8.0',
            description='Seconds of SLAM warmup before frontier exploration',
        ),
        DeclareLaunchArgument(
            'auto_mapper_min_free_neighbors', default_value='2',
            description=(
                'Free neighbors required to qualify an unknown frontier cell'
            ),
        ),
        DeclareLaunchArgument(
            'map_output_path', default_value='/tmp/tb1_map',
            description='Base path used for automatic map snapshots',
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
            'heading_history_size', default_value='5',
            description='Number of recent leader poses used to detect motion',
        ),
        DeclareLaunchArgument(
            'deadband_distance', default_value='0.2',
            description=(
                'Minimum follower-goal displacement before updating; the '
                'mapping default limits needless Nav2 preemption'
            ),
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
