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
#
# Authors: Arshad Mehmood

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node

from multi_robot_scripts.utils import (
    create_namespaced_bridge_yaml,
    load_sdf_with_namespace,
)

import yaml


def _robot_actions(context, tb3_multi_dir):
    """Create robot processes after resolving the selected config file."""
    robot_config_path = LaunchConfiguration('robot_config').perform(context)
    with open(robot_config_path, 'r') as config_file:
        config = yaml.safe_load(config_file)
    robots = [
        robot for robot in config['robots'] if robot.get('enabled', True)
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')
    mapping_use_ground_truth_odom = LaunchConfiguration(
        'mapping_use_ground_truth_odom'
    )
    tb3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    model_dir = f'turtlebot3_{tb3_model}'
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    urdf_path = os.path.join(
        tb3_multi_dir, 'urdf', f'turtlebot3_{tb3_model}.urdf'
    )
    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    actions = []
    for robot in robots:
        namespace = robot['name']
        sdf_path = os.path.join(
            tb3_multi_dir, 'models', model_dir, 'model.sdf'
        )
        patched_sdf = load_sdf_with_namespace(
            sdf_path, namespace, color=robot.get('color')
        )
        actions.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            remappings=remappings,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
                # TF is isolated by topic namespace; frame IDs stay local.
                'frame_prefix': '',
            }],
        ))
        actions.append(Node(
            package='ros_gz_sim',
            executable='create',
            namespace=namespace,
            arguments=[
                '-name', f'{namespace}_{tb3_model}',
                '-string', patched_sdf,
                '-x', str(robot['x_pose']),
                '-y', str(robot['y_pose']),
                '-z', str(robot.get('z_pose', 0.01)),
                '-Y', str(robot.get('yaw_pose', 0.0)),
            ],
            output='screen',
        ))

        bridge_template = os.path.join(
            tb3_multi_dir, 'params', f'{tb3_model}_bridge.yaml'
        )
        if namespace == 'tb1':
            wheel_bridge = create_namespaced_bridge_yaml(
                bridge_template, namespace, use_ground_truth_odom=False,
                filter_scans=True,
            )
            truth_bridge = create_namespaced_bridge_yaml(
                bridge_template, namespace, use_ground_truth_odom=True,
                filter_scans=True,
            )
            actions.extend([
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name=f'{namespace}_bridge',
                    arguments=[
                        '--ros-args', '-p', f'config_file:={truth_bridge}'
                    ],
                    output='screen',
                    condition=IfCondition(mapping_use_ground_truth_odom),
                ),
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name=f'{namespace}_bridge',
                    arguments=[
                        '--ros-args', '-p', f'config_file:={wheel_bridge}'
                    ],
                    output='screen',
                    condition=UnlessCondition(mapping_use_ground_truth_odom),
                ),
                # Gazebo's 2D OdometryPublisher can retain a small contact
                # roll/pitch.  Flatten it before the 2D SLAM/Nav2 stack uses
                # the primary odom and odom -> base_footprint transform.
                Node(
                    package='tb3_multi_robot',
                    executable='planar_odom',
                    name='planar_odom',
                    namespace=namespace,
                    remappings=[('/tf', f'/{namespace}/tf')],
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'input_topic': 'ground_truth_odom',
                        'odom_topic': 'odom',
                    }],
                    condition=IfCondition(mapping_use_ground_truth_odom),
                ),
            ])
            # The bridge preserves the LaserScan acquisition timestamp, but a
            # scan can reach ROS well after current TF.  Publish it on the
            # normal topic only when its historical transform is available.
            actions.append(Node(
                package='tb3_multi_robot',
                executable='scan_tf_gate',
                name='scan_tf_gate',
                namespace=namespace,
                remappings=[
                    ('/tf', f'/{namespace}/tf'),
                    ('/tf_static', f'/{namespace}/tf_static'),
                ],
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'raw_scan_topic': 'scan_raw',
                    'scan_topic': 'scan',
                    'max_scan_age_sec': 0.35,
                    'transform_timeout_sec': 0.35,
                    'tf_buffer_duration_sec': 30.0,
                }],
            ))
        else:
            namespaced_bridge = create_namespaced_bridge_yaml(
                bridge_template, namespace
            )
            actions.append(Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name=f'{namespace}_bridge',
                arguments=[
                    '--ros-args', '-p', f'config_file:={namespaced_bridge}'
                ],
                output='screen',
            ))

        if tb3_model != 'burger':
            actions.append(Node(
                package='ros_gz_image',
                executable='image_bridge',
                namespace=namespace,
                arguments=[f'/{namespace}/camera/image_raw'],
                output='screen',
            ))

    # Keep Docker readiness aligned with whichever robot set was selected.
    actions.append(Node(
        package='tb3_multi_robot',
        executable='simulation_health_monitor',
        name='simulation_health_monitor',
        output='screen',
        respawn=True,
        respawn_delay=1.0,
        parameters=[{
            'use_sim_time': False,
            'robot_names': [robot['name'] for robot in robots],
            'odom_timeout': 2.0,
            'heartbeat_period': 0.5,
            'ready_file': '/tmp/tb3_multi_robot.ready',
        }],
    ))
    return actions


def generate_launch_description():
    # Paths
    tb3_multi_dir = get_package_share_directory('tb3_multi_robot')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    # Simulation config
    gui = LaunchConfiguration('gui')
    software_rendering = LaunchConfiguration('software_rendering')
    clock_rate = LaunchConfiguration('clock_rate')
    world_name = LaunchConfiguration('world')
    world_path = PathJoinSubstitution([
        tb3_multi_dir,
        'worlds',
        [world_name, '.world'],
    ])

    # Launch Gazebo server and client
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            ros_gz_sim_dir, 'launch', 'gz_sim.launch.py'
        )),
        launch_arguments={
            'gz_args': ['-r -s -v2 ', world_path],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=IfCondition(gui),
    )
    gzserver_headless_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            ros_gz_sim_dir, 'launch', 'gz_sim.launch.py'
        )),
        launch_arguments={
            'gz_args': ['-r -s --headless-rendering -v2 ', world_path],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=UnlessCondition(gui),
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            ros_gz_sim_dir, 'launch', 'gz_sim.launch.py'
        )),
        launch_arguments={'gz_args': '-g -v2', 'on_exit_shutdown': 'true'}.items(),
        condition=IfCondition(gui),
    )

    # Main LaunchDescription
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use the Gazebo simulation clock',
    ))
    ld.add_action(DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start the Gazebo graphical client',
    ))
    ld.add_action(DeclareLaunchArgument(
        'software_rendering',
        default_value=EnvironmentVariable(
            'TB3_SOFTWARE_RENDERING', default_value='false'
        ),
        description='Force Mesa software rendering instead of GPU rendering',
    ))
    ld.add_action(DeclareLaunchArgument(
        'world',
        default_value='tb3_world',
        description=(
            'Scenario name: tb3_world, open_arena, corridor, or '
            'obstacle_course'
        ),
    ))
    ld.add_action(DeclareLaunchArgument(
        'clock_rate',
        default_value='250.0',
        description=(
            'Maximum ROS /clock publication rate in Hz. Gazebo physics and '
            'the /clock_raw stream remain at their native 1 kHz rate.'
        ),
    ))
    ld.add_action(DeclareLaunchArgument(
        'mapping_use_ground_truth_odom',
        default_value='true',
        description=(
            'Use Gazebo world-pose odometry for tb1 navigation and SLAM; '
            'wheel-integrated odometry remains on /tb1/wheel_odom'
        ),
    ))
    ld.add_action(DeclareLaunchArgument(
        'robot_config',
        default_value=os.path.join(
            tb3_multi_dir, 'config', 'robots.yaml'
        ),
        description='Robot spawn/team configuration YAML',
    ))

    # Set model search paths before Gazebo starts. The local package contains
    # the robot/world SDF files, while turtlebot3_gazebo supplies common meshes.
    ld.add_action(AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(tb3_multi_dir, 'models'),
    ))
    ld.add_action(AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(turtlebot3_gazebo_dir, 'models'),
    ))
    # Keep the Mesa path available for hosts without GPU passthrough, but do
    # not force it for ordinary headless GPU rendering.
    ld.add_action(SetEnvironmentVariable(
        'LIBGL_ALWAYS_SOFTWARE', '1',
        condition=IfCondition(software_rendering),
    ))
    ld.add_action(gzserver_cmd)
    ld.add_action(gzserver_headless_cmd)
    ld.add_action(gzclient_cmd)

    ld.add_action(OpaqueFunction(
        function=_robot_actions,
        kwargs={'tb3_multi_dir': tb3_multi_dir},
    ))

    # In a multi-robot setup using Gazebo Sim (Harmonic or later), each robot typically
    # requires a separate ROS-Gazebo bridge to relay topics such as sensor data, odometry,
    # and control commands between Gazebo and ROS 2.
    # However, some topics like `/clock` are *global* and should be published only once
    # to avoid conflicts or duplication. If multiple bridges publish `/clock`, it may lead
    # to inconsistent simulation time behavior across nodes or unnecessary topic traffic.
    # Therefore, the `/clock` topic is handled separately:
    # - It is excluded from the per-robot bridge configuration files (YAMLs).
    # - A dedicated, single bridge instance is launched to publish `/clock` from Gazebo to ROS 2.
    # This ensures consistent simulation time across the entire ROS 2 system while supporting
    # multiple robot instances with their own bridges.

    # Preserve the native 1 kHz Gazebo clock for high-resolution diagnostics,
    # then fan out exact clock samples at a configurable rate appropriate for
    # Nav2's 10-20 Hz control loops. This avoids delivering every physics tick
    # to every use_sim_time node without changing physics integration accuracy.
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        remappings=[('/clock', '/clock_raw')],
    )
    ld.add_action(clock_bridge)

    clock_throttle = Node(
        package='topic_tools',
        executable='throttle',
        name='clock_throttle',
        output='screen',
        arguments=['messages', '/clock_raw', clock_rate, '/clock'],
        parameters=[{
            'use_sim_time': False,
            'use_wall_clock': True,
        }],
    )
    ld.add_action(clock_throttle)

    return ld
