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


def create_namespaced_bridge_yaml(base_yaml_path, namespace):
    """Create a temporary namespaced bridge YAML for ros_gz_bridge."""
    with open(base_yaml_path, 'r') as f:
        bridges = yaml.safe_load(f)

    # Gazebo's OdometryPublisher reads the simulated world pose instead of
    # integrating wheel motion. Keep this separate from the realistic odometry
    # used by SLAM so it is available only for simulation truth/validation.
    bridges.append({
        'ros_topic_name': 'ground_truth_odom',
        'gz_topic_name': 'ground_truth_odom',
        'ros_type_name': 'nav_msgs/msg/Odometry',
        'gz_type_name': 'gz.msgs.Odometry',
        'direction': 'GZ_TO_ROS',
    })

    if namespace and not namespace.endswith('/'):
        namespace_with_slash = namespace + '/'
    else:
        namespace_with_slash = namespace

    namespaced_bridges = []
    for bridge in bridges:
        if bridge['ros_topic_name'] not in ['clock']:
            bridge['ros_topic_name'] = (
                f"{namespace_with_slash}{bridge['ros_topic_name']}"
            )
        if bridge['gz_topic_name'] not in ['clock']:
            bridge['gz_topic_name'] = (
                f"{namespace_with_slash}{bridge['gz_topic_name']}"
            )
        namespaced_bridges.append(bridge)

    output_path = f"/tmp/{namespace.strip('/')}_bridge.yaml"
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
      <odom_frame>ground_truth_odom</odom_frame>
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


def generate_mapping_nav2_params(
    robot_name, base_config_path, slam_config_path
):
    """Merge SLAM Toolbox settings into a robot's Nav2 parameter file."""
    with open(base_config_path, 'r') as config_file:
        config = yaml.safe_load(config_file)
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
        'minimum_time_interval': 0.2,
        'minimum_travel_distance': 0.1,
        'minimum_travel_heading': 0.1,
        'odom_frame': 'odom',
        'scan_topic': f'/{robot_name}/scan',
        'transform_publish_period': 0.02,
        'use_map_saver': True,
    })
    config['slam_toolbox'] = {'ros__parameters': slam_parameters}

    static_layer = config['global_costmap']['global_costmap'][
        'ros__parameters'
    ]['static_layer']
    static_layer['map_topic'] = f'/{robot_name}/map'
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
