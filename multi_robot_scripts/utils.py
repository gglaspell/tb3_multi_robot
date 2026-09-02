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

"""Runtime configuration helpers for the multi-robot simulation."""

import os
import tempfile

import yaml


def create_namespaced_bridge_yaml(
    base_yaml_path, namespace, use_ground_truth_odom=False
):
    """Create a namespaced bridge YAML with selectable primary odometry."""
    with open(base_yaml_path, 'r') as f:
        bridges = yaml.safe_load(f)

    # Keep wheel-integrated odometry available for realism and diagnostics. In
    # the simulation-accuracy profile, Gazebo's independent world-pose
    # OdometryPublisher becomes the primary odom/TF source used by Nav2 and
    # SLAM, while the original stream moves to wheel_odom.
    configured_bridges = []
    for bridge in bridges:
        if use_ground_truth_odom and bridge['ros_topic_name'] == 'tf':
            continue
        if use_ground_truth_odom and bridge['ros_topic_name'] == 'odom':
            bridge['ros_topic_name'] = 'wheel_odom'
        configured_bridges.append(bridge)

    ground_truth_odom = {
        'ros_topic_name': 'ground_truth_odom',
        'gz_topic_name': 'ground_truth_odom',
        'ros_type_name': 'nav_msgs/msg/Odometry',
        'gz_type_name': 'gz.msgs.Odometry',
        'direction': 'GZ_TO_ROS',
    }
    configured_bridges.append(ground_truth_odom)

    if use_ground_truth_odom:
        configured_bridges.extend([
            {
                **ground_truth_odom,
                'ros_topic_name': 'odom',
            },
            {
                'ros_topic_name': 'tf',
                'gz_topic_name': 'ground_truth_tf',
                'ros_type_name': 'tf2_msgs/msg/TFMessage',
                'gz_type_name': 'gz.msgs.Pose_V',
                'direction': 'GZ_TO_ROS',
            },
        ])

    if namespace and not namespace.endswith('/'):
        namespace_with_slash = namespace + '/'
    else:
        namespace_with_slash = namespace

    namespaced_bridges = []
    for bridge in configured_bridges:
        if bridge['ros_topic_name'] not in ['clock']:
            bridge['ros_topic_name'] = (
                f"{namespace_with_slash}{bridge['ros_topic_name']}"
            )
        if bridge['gz_topic_name'] not in ['clock']:
            bridge['gz_topic_name'] = (
                f"{namespace_with_slash}{bridge['gz_topic_name']}"
            )
        namespaced_bridges.append(bridge)

    profile = 'truth' if use_ground_truth_odom else 'wheel'
    output_path = os.path.join(
        tempfile.gettempdir(), f"{namespace.strip('/')}_{profile}_bridge.yaml"
    )
    with open(output_path, 'w') as f:
        yaml.dump(namespaced_bridges, f)

    return output_path


def load_sdf_with_namespace(model_path, namespace):
    """Patch SDF file to inject robot namespace into all relevant topic tags."""
    with open(model_path, 'r') as f:
        sdf_text = f.read()

    topic_map = {
        '<tf_topic>/tf</tf_topic>': f'<tf_topic>{namespace}/tf</tf_topic>',
        '<topic>cmd_vel</topic>': f'<topic>{namespace}/cmd_vel</topic>',
        '<odom_topic>odom</odom_topic>': (
            f'<odom_topic>{namespace}/odom</odom_topic>'
        ),
        '<topic>joint_states</topic>': (
            f'<topic>{namespace}/joint_states</topic>'
        ),
        '<topic>imu</topic>': f'<topic>{namespace}/imu</topic>',
        '<topic>scan</topic>': f'<topic>{namespace}/scan</topic>',
        '<topic>camera/image_raw</topic>': (
            f'<topic>{namespace}/camera/image_raw</topic>'
        ),
        '<camera_info_topic>camera/camera_info</camera_info_topic>': (
            f'<camera_info_topic>{namespace}/camera/camera_info</camera_info_topic>'
        ),
    }

    for original, replacement in topic_map.items():
        sdf_text = sdf_text.replace(original, replacement)

    ground_truth_plugin = f"""
    <plugin filename="gz-sim-odometry-publisher-system"
            name="gz::sim::systems::OdometryPublisher">
      <odom_frame>odom</odom_frame>
      <robot_base_frame>base_footprint</robot_base_frame>
      <odom_publish_frequency>20</odom_publish_frequency>
      <odom_topic>{namespace}/ground_truth_odom</odom_topic>
      <odom_covariance_topic>{namespace}/ground_truth_odom_cov</odom_covariance_topic>
      <tf_topic>{namespace}/ground_truth_tf</tf_topic>
      <dimensions>2</dimensions>
    </plugin>
"""
    model_end = sdf_text.rfind('</model>')
    if model_end < 0:
        raise ValueError(f'No closing model tag in {model_path}')
    sdf_text = (
        sdf_text[:model_end]
        + ground_truth_plugin
        + sdf_text[model_end:]
    )

    return sdf_text


def generate_rviz_config(robot_name, base_config_path, map_topic='/map'):
    """Generate a namespaced RViz configuration from the shared template."""
    # Read the base RViz config
    with open(base_config_path, 'r') as f:
        config = f.read()

    # Replace placeholders
    config = config.replace('<ROBOT_NAME>', robot_name)
    config = config.replace('<MAP_TOPIC>', map_topic)

    # Use system temp directory
    temp_dir = tempfile.gettempdir()
    output_config_path = os.path.join(
        temp_dir, f'{robot_name}_rviz_config.rviz'
    )

    with open(output_config_path, 'w') as f:
        f.write(config)

    return output_config_path


def _mapping_slam_parameters(
    robot_name, slam_config_path, use_scan_matching=True
):
    """Return tuned SLAM Toolbox parameters for one robot."""
    with open(slam_config_path, 'r') as slam_file:
        slam_config = yaml.safe_load(slam_file)

    slam_parameters = slam_config['slam_toolbox']['ros__parameters']
    slam_parameters.update({
        'base_frame': 'base_footprint',
        'debug_logging': False,
        'enable_interactive_mode': False,
        'map_frame': 'map',
        'map_update_interval': 1.0,
        'max_laser_range': 3.5,
        # The 10 Hz stream commonly arrives a fraction under 0.1 s apart.
        # A 0.1 s gate therefore discarded alternating scans; 0.05 s accepts
        # every sensor update while still rejecting accidental duplicates.
        'minimum_time_interval': 0.05,
        'minimum_travel_distance': 0.05,
        'minimum_travel_heading': 0.05,
        'odom_frame': 'odom',
        'scan_topic': f'/{robot_name}/scan',
        'transform_publish_period': 0.02,
        'use_map_saver': True,
        'use_scan_matching': use_scan_matching,
    })
    if not use_scan_matching:
        # Exact simulation odometry is already a stronger pose source. Loop
        # closure would otherwise be free to bend that trajectory again.
        slam_parameters['do_loop_closing'] = False
    return slam_parameters


def generate_mapping_slam_params(
    robot_name, slam_config_path, use_scan_matching=True
):
    """Write a SLAM-only file matching the node's fully-qualified name."""
    profile = 'wheel' if use_scan_matching else 'truth'
    output_path = os.path.join(
        tempfile.gettempdir(), f'{robot_name}_{profile}_slam_params.yaml'
    )
    config = {
        f'/{robot_name}/slam_toolbox': {
            'ros__parameters': _mapping_slam_parameters(
                robot_name, slam_config_path, use_scan_matching
            )
        }
    }
    with open(output_path, 'w') as config_file:
        yaml.safe_dump(config, config_file, sort_keys=False)

    return output_path


def generate_mapping_nav2_params(
    robot_name, base_config_path, slam_config_path
):
    """Merge mapping-related settings into a robot's Nav2 parameter file."""
    with open(base_config_path, 'r') as config_file:
        config = yaml.safe_load(config_file)

    slam_parameters = _mapping_slam_parameters(
        robot_name, slam_config_path
    )
    config['slam_toolbox'] = {'ros__parameters': slam_parameters}

    static_layer = config['global_costmap']['global_costmap'][
        'ros__parameters'
    ]['static_layer']
    static_layer['map_topic'] = f'/{robot_name}/map'
    # The Burger lidar sees 360 degrees, so terminal yaw does not reveal any
    # additional space. Accept any arrival heading to avoid spending tens of
    # seconds rotating beside a frontier after reaching its position.
    config['controller_server']['ros__parameters']['goal_checker'][
        'yaw_goal_tolerance'
    ] = 3.14159
    config['amcl']['ros__parameters']['map_topic'] = f'/{robot_name}/map'
    config['map_server']['ros__parameters']['topic_name'] = (
        f'/{robot_name}/map'
    )

    output_path = os.path.join(
        tempfile.gettempdir(), f'{robot_name}_mapping_nav2_params.yaml'
    )
    with open(output_path, 'w') as config_file:
        yaml.safe_dump(config, config_file, sort_keys=False)

    return output_path
