#!/usr/bin/env python3

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

# http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch the full tb1-leader / tb3-follower scenario.

1. Nav2 for tb1 → burger_nav2_params.yaml (normal NavigateToPose)
2. Nav2 for tb3 → burger_nav2_params_tb3.yaml (FollowDynamicPoint BT)
3. tb3_follow_tb1 node (publishes trailing goal)
4. Two RViz instances (one per robot)

Usage
-----
ros2 launch tb3_multi_robot follow_tb3.launch.py

Optional overrides
------------------
map:=            – path to map.yaml (default: package map/map.yaml)
use_sim_time:=true/false – (default true)
follow_distance:=0.5 – metres tb3 trails behind tb1 (default 0.5)
publish_rate:=2.0    – Hz goal is republished (default 2.0)
stationary_threshold:=0.05 – translational motion threshold in metres
stationary_angular_threshold:=0.1 – rotational motion threshold in radians
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node, PushROSNamespace, SetRemap
from launch_ros.parameter_descriptions import ParameterValue

from multi_robot_scripts.utils import (
    generate_mapping_nav2_params,
    generate_mapping_slam_params,
)


def _nav2_bringup(robot_name: str, params_path: str, map_path,
                  use_sim_time, autostart, use_composition,
                  log_level, slam='False',
                  use_localization='True') -> IncludeLaunchDescription:
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
            'map': map_path,
            'use_sim_time': use_sim_time,
            'params_file': params_path,
            'use_namespace': 'true',
            'namespace': robot_name,
            'autostart': autostart,
            'use_composition': use_composition,
            'log_level': log_level,
            # Jazzy's bringup launch evaluates these values as Python
            # expressions, so use Python Boolean spelling here.
            'slam': slam,
            'use_localization': use_localization,
        }.items(),
    )


def generate_launch_description():
    # ── Paths ─────────────────────────────────────────────────────────────────
    pkg_dir = get_package_share_directory('tb3_multi_robot')
    tb3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')

    default_map = os.path.join(pkg_dir, 'map', 'map.yaml')
    params_tb1 = os.path.join(pkg_dir, 'params', f'{tb3_model}_nav2_params.yaml')
    params_tb3 = os.path.join(pkg_dir, 'params', f'{tb3_model}_nav2_params_tb3.yaml')
    rviz_tb1_config = os.path.join(
        pkg_dir, 'rviz', 'tb1_navigation2.rviz'
    )
    rviz_tb3_config = os.path.join(
        pkg_dir, 'rviz', 'tb3_navigation2.rviz'
    )
    slam_config = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'config',
        'mapper_params_online_sync.yaml',
    )
    params_tb1_mapping = generate_mapping_nav2_params(
        'tb1', params_tb1, slam_config
    )
    params_tb1_slam_wheel = generate_mapping_slam_params(
        'tb1', slam_config, use_scan_matching=True
    )
    params_tb1_slam_truth = generate_mapping_slam_params(
        'tb1', slam_config, use_scan_matching=False
    )

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
        'autostart',
        default_value='true',
        description='Automatically activate each Nav2 stack',
    ))
    ld.add_action(DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='Run each Nav2 stack in a component container',
    ))
    ld.add_action(DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start one RViz instance for each robot',
    ))
    ld.add_action(DeclareLaunchArgument(
            'rviz_render_threads',
            default_value=EnvironmentVariable(
                'LP_NUM_THREADS', default_value='8'
            ),
        description='Mesa software-rendering threads per RViz process',
    ))
    ld.add_action(DeclareLaunchArgument(
        'auto_map',
        default_value='false',
        description='Use tb1 SLAM and frontier exploration instead of AMCL',
    ))
    ld.add_action(DeclareLaunchArgument(
        'mapping_use_ground_truth_odom',
        default_value='false',
        description=(
            'Disable SLAM pose correction when Gazebo truth odometry drives '
            'the leader; follow_sim.launch.py enables this by default'
        ),
    ))
    ld.add_action(DeclareLaunchArgument(
        'use_ground_truth_pose',
        default_value='true',
        description=(
            'Use slip-free Gazebo pose for accurate cross-map following'
        ),
    ))
    ld.add_action(DeclareLaunchArgument(
        'auto_mapper_startup_delay',
        default_value='8.0',
        description='Seconds of SLAM warmup before frontier exploration',
    ))
    ld.add_action(DeclareLaunchArgument(
        'auto_mapper_min_free_neighbors',
        default_value='2',
        description=(
            'Free neighbors required to qualify an unknown frontier cell'
        ),
    ))
    ld.add_action(DeclareLaunchArgument(
        'map_output_path',
        default_value='/tmp/tb1_map',
        description='Base path used for automatic map snapshots',
    ))
    ld.add_action(DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Nav2 logging level',
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
    ld.add_action(DeclareLaunchArgument(
        'stationary_threshold',
        default_value='0.05',
        description='Leader translation threshold (m) used to detect motion',
    ))
    ld.add_action(DeclareLaunchArgument(
        'stationary_angular_threshold',
        default_value='0.1',
        description='Leader rotation threshold (rad) used to detect motion',
    ))

    map_path = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    rviz = LaunchConfiguration('rviz')
    rviz_render_threads = LaunchConfiguration('rviz_render_threads')
    auto_map = LaunchConfiguration('auto_map')
    mapping_use_ground_truth_odom = LaunchConfiguration(
        'mapping_use_ground_truth_odom'
    )
    use_ground_truth_pose = LaunchConfiguration('use_ground_truth_pose')
    localization_python_bool = PythonExpression([
        "'False' if '", auto_map, "'.lower() == 'true' else 'True'",
    ])
    log_level = LaunchConfiguration('log_level')

    # ── tb1: normal Nav2 bringup ──────────────────────────────────────────────
    ld.add_action(LogInfo(msg='[follow_tb3] Launching Nav2 for tb1 (NavigateToPose mode)'))
    ld.add_action(_nav2_bringup(
        'tb1', params_tb1_mapping, map_path, use_sim_time,
        autostart, use_composition, log_level,
        use_localization=localization_python_bool,
    ))

    # Nav2 Jazzy forwards the combined parameter file to a namespaced SLAM
    # node without applying its namespace rewrite. Launch SLAM Toolbox and its
    # map saver explicitly so the tuned, fully-qualified profile is honored.
    def mapping_actions(slam_params):
        return [
            PushROSNamespace('tb1'),
            SetRemap(src='/scan', dst='scan'),
            SetRemap(src='/tf', dst='tf'),
            SetRemap(src='/tf_static', dst='tf_static'),
            SetRemap(src='/map', dst='map'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('slam_toolbox'),
                    'launch',
                    'online_sync_launch.py',
                )),
                launch_arguments={
                    'autostart': autostart,
                    'slam_params_file': slam_params,
                    'use_sim_time': use_sim_time,
                }.items(),
            ),
            Node(
                package='nav2_map_server',
                executable='map_saver_server',
                name='map_saver',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_slam',
                output='screen',
                parameters=[{
                    'autostart': autostart,
                    'node_names': ['map_saver'],
                    'use_sim_time': use_sim_time,
                }],
            ),
        ]

    mapping_stack = GroupAction(
        condition=IfCondition(auto_map),
        actions=[
            GroupAction(
                condition=IfCondition(mapping_use_ground_truth_odom),
                actions=mapping_actions(params_tb1_slam_truth),
            ),
            GroupAction(
                condition=UnlessCondition(mapping_use_ground_truth_odom),
                actions=mapping_actions(params_tb1_slam_wheel),
            ),
        ],
    )
    ld.add_action(mapping_stack)

    rviz_tb1 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace='/tb1',
        arguments=[
            '-d',
            rviz_tb1_config,
        ],
        parameters=[{'use_sim_time': use_sim_time, 'log_level': 'warn'}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        additional_env={'LP_NUM_THREADS': rviz_render_threads},
        output='screen',
        condition=IfCondition(rviz),
    )
    ld.add_action(rviz_tb1)

    map_pose_node = Node(
        package='tb3_multi_robot',
        executable='tf_pose_publisher',
        name='map_pose_publisher',
        namespace='/tb1',
        output='screen',
        condition=IfCondition(auto_map),
        remappings=[
            ('/tf', '/tb1/tf'),
            ('/tf_static', '/tb1/tf_static'),
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_frame': 'map',
            'source_frame': 'base_link',
            'pose_topic': 'map_pose',
            'publish_rate': 5.0,
        }],
    )
    ld.add_action(map_pose_node)

    auto_mapper_node = Node(
        package='auto_mapper',
        executable='auto_mapper',
        name='auto_mapper',
        namespace='/tb1',
        output='screen',
        condition=IfCondition(auto_map),
        remappings=[
            ('/navigate_to_pose', '/tb1/navigate_to_pose'),
            ('/map_server/save_map', '/tb1/map_saver/save_map'),
            ('/frontiers', '/tb1/frontiers'),
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_topic': '/tb1/map',
            'odom_topic': '',
            'pose_topic': '/tb1/map_pose',
            'map_path': LaunchConfiguration('map_output_path'),
            'startup_delay_sec': LaunchConfiguration(
                'auto_mapper_startup_delay'
            ),
            # A four-neighbor threshold fragments the curved boundary of a
            # laser-built map into sub-threshold islands. Two rejects isolated
            # speckle while preserving continuous, navigable scan frontiers.
            'min_free_threshold': ParameterValue(
                LaunchConfiguration('auto_mapper_min_free_neighbors'),
                value_type=int,
            ),
        }],
    )
    ld.add_action(auto_mapper_node)

    # ── tb3: follow-mode Nav2 bringup ─────────────────────────────────────────
    ld.add_action(LogInfo(msg='[follow_tb3] Launching Nav2 for tb3 (FollowDynamicPoint mode)'))
    ld.add_action(_nav2_bringup(
        'tb3', params_tb3, map_path, use_sim_time,
        autostart, use_composition, log_level,
    ))

    rviz_tb3 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace='/tb3',
        arguments=[
            '-d',
            rviz_tb3_config,
        ],
        parameters=[{'use_sim_time': use_sim_time, 'log_level': 'warn'}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        additional_env={'LP_NUM_THREADS': rviz_render_threads},
        output='screen',
        condition=IfCondition(rviz),
    )
    ld.add_action(rviz_tb3)

    # ── tb3_follow_tb1 node ───────────────────────────────────────────────────
    ld.add_action(LogInfo(msg='[follow_tb3] Starting tb3_follow_tb1 node'))
    follow_node = Node(
        package='tb3_multi_robot',
        executable='tb3_follow_tb1',
        name='tb3_follow_tb1',
        output='screen',
        remappings=[
            ('/tf', '/tb1/tf'),
            ('/tf_static', '/tb1/tf_static'),
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
            'follow_distance': LaunchConfiguration('follow_distance'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'heading_history_size': LaunchConfiguration('heading_history_size'),
            'deadband_distance': LaunchConfiguration('deadband_distance'),
            'stationary_threshold': LaunchConfiguration(
                'stationary_threshold'
            ),
            'stationary_angular_threshold': LaunchConfiguration(
                'stationary_angular_threshold'
            ),
            'use_ground_truth_pose': ParameterValue(
                use_ground_truth_pose, value_type=bool
            ),
        }],
    )
    ld.add_action(follow_node)

    return ld
