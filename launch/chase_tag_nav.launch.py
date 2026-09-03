#!/usr/bin/env python3

"""Launch three namespaced Nav2 stacks and the chase/tag coordinator."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration

from launch_ros.actions import LoadComposableNodes, Node, SetParameter
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.parameter_descriptions import ParameterValue

from multi_robot_scripts.utils import (
    generate_chase_nav2_params,
    generate_rviz_config,
)

from nav2_common.launch import RewrittenYaml

import yaml


def _nav2_bringup(
    robot_name,
    params_path,
    map_path,
    use_sim_time,
    autostart,
    log_level,
):
    """Launch the installed Nav2 servers without the bringup meta-package.

    ROS 2 Lyrical publishes the Nav2 server packages but not nav2_bringup.
    Put each robot's servers in its own isolated component container. This
    keeps the three stacks independent while avoiding a separate DDS
    participant for every server process.
    """
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_path,
            root_key=robot_name,
            param_rewrites={'use_sim_time': use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )
    sim_time = ParameterValue(use_sim_time, value_type=bool)
    autostart_value = ParameterValue(autostart, value_type=bool)
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    def nav2_component(package, plugin, name, extra_remappings=None,
                       extra_parameters=None):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name=name,
            namespace=f'/{robot_name}',
            parameters=[
                configured_params,
                {'use_sim_time': sim_time},
            ] + (extra_parameters or []),
            remappings=remappings + (extra_remappings or []),
            extra_arguments=[{'use_intra_process_comms': True}],
        )

    localization_nodes = ['map_server', 'amcl']
    navigation_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
        'waypoint_follower',
    ]

    components = [
        nav2_component(
            'nav2_map_server',
            'nav2_map_server::MapServer',
            'map_server',
            extra_parameters=[{'yaml_filename': map_path}],
        ),
        nav2_component(
            'nav2_amcl', 'nav2_amcl::AmclNode', 'amcl'
        ),
        nav2_component(
            'nav2_controller',
            'nav2_controller::ControllerServer',
            'controller_server',
            extra_remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),
        nav2_component(
            'nav2_planner',
            'nav2_planner::PlannerServer',
            'planner_server',
        ),
        nav2_component(
            'nav2_behaviors',
            'behavior_server::BehaviorServer',
            'behavior_server',
            extra_remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),
        nav2_component(
            'nav2_velocity_smoother',
            'nav2_velocity_smoother::VelocitySmoother',
            'velocity_smoother',
            extra_remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),
        nav2_component(
            'nav2_collision_monitor',
            'nav2_collision_monitor::CollisionMonitor',
            'collision_monitor',
        ),
        nav2_component(
            'nav2_bt_navigator',
            'nav2_bt_navigator::BtNavigator',
            'bt_navigator',
        ),
        nav2_component(
            'nav2_waypoint_follower',
            'nav2_waypoint_follower::WaypointFollower',
            'waypoint_follower',
        ),
        ComposableNode(
            package='nav2_lifecycle_manager',
            plugin='nav2_lifecycle_manager::LifecycleManager',
            name='lifecycle_manager_localization',
            namespace=f'/{robot_name}',
            parameters=[{
                'use_sim_time': sim_time,
                'autostart': autostart_value,
                'node_names': localization_nodes,
            }],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
        ComposableNode(
            package='nav2_lifecycle_manager',
            plugin='nav2_lifecycle_manager::LifecycleManager',
            name='lifecycle_manager_navigation',
            namespace=f'/{robot_name}',
            parameters=[{
                'use_sim_time': sim_time,
                'autostart': autostart_value,
                'node_names': navigation_nodes,
            }],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ]

    container_name = f'{robot_name}_nav2_container'
    return GroupAction(actions=[
        SetParameter(name='use_sim_time', value=sim_time),
        Node(
            package='rclcpp_components',
            executable='component_container',
            name=container_name,
            namespace=f'/{robot_name}',
            output='screen',
            remappings=remappings,
            parameters=[
                configured_params,
                {'use_sim_time': sim_time},
            ],
            arguments=[
                '--executor-type', 'single-threaded', '--isolated',
                '--ros-args', '--log-level', log_level,
            ],
        ),
        LoadComposableNodes(
            target_container=f'/{robot_name}/{container_name}',
            composable_node_descriptions=components,
        ),
    ])


def _game_actions(context, package_dir):
    robot_config = LaunchConfiguration('robot_config').perform(context)
    with open(robot_config, 'r') as config_file:
        robots = [
            robot for robot in yaml.safe_load(config_file)['robots']
            if robot.get('enabled', True)
        ]

    red_robots = [
        robot['name'] for robot in robots if robot.get('team') == 'red'
    ]
    blue_robots = [
        robot['name'] for robot in robots if robot.get('team') == 'blue'
    ]
    if not red_robots or len(blue_robots) != 1:
        raise ValueError(
            'Chase/tag robot config requires at least one red and one blue'
        )
    blue_robot = blue_robots[0]

    model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    parameter_candidates = [
        os.path.join(package_dir, 'params', f'{model}_nav2_params.yaml'),
        os.path.join(package_dir, 'params', f'{model}.yaml'),
    ]
    base_params = next(
        (path for path in parameter_candidates if os.path.exists(path)), None
    )
    if base_params is None:
        raise ValueError(f'No Nav2 parameters available for model {model}')

    dynamic_tree = os.path.join(
        package_dir, 'behavior_trees', 'follow_dynamic_point.xml'
    )
    red_speed = float(
        LaunchConfiguration('red_linear_speed').perform(context)
    )
    blue_speed = float(
        LaunchConfiguration('blue_linear_speed').perform(context)
    )
    map_path = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    rviz = LaunchConfiguration('rviz')
    rviz_template = os.path.join(
        package_dir, 'rviz', 'tb3_navigation2.rviz'
    )

    actions = []
    for robot in robots:
        name = robot['name']
        speed = red_speed if name in red_robots else blue_speed
        params = generate_chase_nav2_params(
            name,
            base_params,
            (
                robot['x_pose'],
                robot['y_pose'],
                robot.get('yaw_pose', 0.0),
            ),
            speed,
            behavior_tree_path=dynamic_tree,
        )
        actions.append(LogInfo(
            msg=f'[chase_tag] Nav2 {name}: team={robot.get("team")} '
                f'max_speed={speed:.2f}m/s'
        ))
        actions.append(_nav2_bringup(
            name,
            params,
            map_path,
            use_sim_time,
            autostart,
            log_level,
        ))
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=f'/{name}',
            arguments=[
                '-d', generate_rviz_config(
                    name, rviz_template, map_topic='/map'
                ),
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            additional_env={
                'LP_NUM_THREADS': LaunchConfiguration('rviz_render_threads')
            },
            output='screen',
            condition=IfCondition(rviz),
        ))

    actions.append(Node(
        package='tb3_multi_robot',
        executable='chase_tag',
        name='chase_tag',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'red_robots': red_robots,
            'blue_robot': blue_robot,
            'control_rate': float(
                LaunchConfiguration('control_rate').perform(context)
            ),
            'goal_update_rate': float(
                LaunchConfiguration('goal_update_rate').perform(context)
            ),
            'goal_deadband': float(
                LaunchConfiguration('goal_deadband').perform(context)
            ),
            'red_linear_speed': red_speed,
            'tag_distance': float(
                LaunchConfiguration('tag_distance').perform(context)
            ),
            'tag_pause': float(
                LaunchConfiguration('tag_pause').perform(context)
            ),
            'reset_distance': float(
                LaunchConfiguration('reset_distance').perform(context)
            ),
            'score_to_win': int(
                LaunchConfiguration('score_to_win').perform(context)
            ),
        }],
    ))
    return actions


def generate_launch_description():
    package_dir = get_package_share_directory('tb3_multi_robot')
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_config',
            default_value=os.path.join(
                package_dir, 'config', 'chase_tag_robots.yaml'
            ),
            description='Robot poses, colors, and team assignments',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                package_dir, 'map', 'open_arena.yaml'
            ),
            description='Shared occupancy map used by all Nav2 stacks',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
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
            description='Tags needed to stop the game; 0 runs indefinitely',
        ),
        OpaqueFunction(
            function=_game_actions,
            kwargs={'package_dir': package_dir},
        ),
    ])
