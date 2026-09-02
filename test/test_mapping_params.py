"""Regression tests for the generated namespaced mapping profile."""

from pathlib import Path

from multi_robot_scripts import utils

import pytest
import yaml


@pytest.fixture
def mapping_inputs(tmp_path: Path) -> tuple[Path, Path]:
    """Write the minimal Nav2 and SLAM inputs used by the generator."""
    nav2_path = tmp_path / 'nav2.yaml'
    nav2_path.write_text(
        yaml.safe_dump({
            'amcl': {'ros__parameters': {}},
            'controller_server': {
                'ros__parameters': {
                    'goal_checker': {'yaw_goal_tolerance': 0.25}
                }
            },
            'global_costmap': {
                'global_costmap': {
                    'ros__parameters': {'static_layer': {}}
                }
            },
            'map_server': {'ros__parameters': {}},
        }),
        encoding='utf-8',
    )
    slam_path = tmp_path / 'slam.yaml'
    slam_path.write_text(
        yaml.safe_dump({
            'slam_toolbox': {
                'ros__parameters': {
                    'map_update_interval': 10.0,
                    'max_laser_range': 25.0,
                    'minimum_time_interval': 0.5,
                    'minimum_travel_distance': 0.5,
                    'minimum_travel_heading': 0.5,
                }
            }
        }),
        encoding='utf-8',
    )
    return nav2_path, slam_path


def test_mapping_profile_is_tuned_and_namespaced(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    mapping_inputs: tuple[Path, Path],
) -> None:
    monkeypatch.setattr(utils.tempfile, 'gettempdir', lambda: str(tmp_path))
    nav2_path, slam_path = mapping_inputs

    nav2_output = utils.generate_mapping_nav2_params(
        'tb1', str(nav2_path), str(slam_path)
    )
    slam_output = utils.generate_mapping_slam_params(
        'tb1', str(slam_path)
    )
    truth_slam_output = utils.generate_mapping_slam_params(
        'tb1', str(slam_path), use_scan_matching=False
    )
    config = yaml.safe_load(Path(nav2_output).read_text(encoding='utf-8'))
    slam_config = yaml.safe_load(
        Path(slam_output).read_text(encoding='utf-8')
    )
    truth_slam_config = yaml.safe_load(
        Path(truth_slam_output).read_text(encoding='utf-8')
    )

    detected = config['slam_toolbox']['ros__parameters']
    namespaced = slam_config['/tb1/slam_toolbox']['ros__parameters']
    assert namespaced == detected
    assert namespaced['map_update_interval'] == pytest.approx(1.0)
    assert namespaced['max_laser_range'] == pytest.approx(3.5)
    assert namespaced['minimum_time_interval'] == pytest.approx(0.05)
    assert namespaced['minimum_travel_distance'] == pytest.approx(0.05)
    assert namespaced['minimum_travel_heading'] == pytest.approx(0.05)
    assert namespaced['scan_topic'] == '/tb1/scan'
    assert namespaced['use_scan_matching'] is True
    truth_namespaced = truth_slam_config[
        '/tb1/slam_toolbox'
    ]['ros__parameters']
    assert truth_namespaced['use_scan_matching'] is False
    assert truth_namespaced['do_loop_closing'] is False
    assert config['global_costmap']['global_costmap']['ros__parameters'][
        'static_layer'
    ]['map_topic'] == '/tb1/map'
    assert config['controller_server']['ros__parameters']['goal_checker'][
        'yaw_goal_tolerance'
    ] == pytest.approx(3.14159)


def _write_bridge_input(path: Path) -> Path:
    path.write_text(
        yaml.safe_dump([
            {
                'ros_topic_name': 'odom',
                'gz_topic_name': 'odom',
                'ros_type_name': 'nav_msgs/msg/Odometry',
                'gz_type_name': 'gz.msgs.Odometry',
                'direction': 'GZ_TO_ROS',
            },
            {
                'ros_topic_name': 'tf',
                'gz_topic_name': 'tf',
                'ros_type_name': 'tf2_msgs/msg/TFMessage',
                'gz_type_name': 'gz.msgs.Pose_V',
                'direction': 'GZ_TO_ROS',
            },
        ]),
        encoding='utf-8',
    )
    return path


def test_truth_bridge_retains_wheel_odom_and_replaces_primary_tf(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(utils.tempfile, 'gettempdir', lambda: str(tmp_path))
    bridge_input = _write_bridge_input(tmp_path / 'bridge.yaml')

    output = utils.create_namespaced_bridge_yaml(
        str(bridge_input), 'tb1', use_ground_truth_odom=True
    )
    bridges = yaml.safe_load(Path(output).read_text(encoding='utf-8'))
    topics = {
        bridge['ros_topic_name']: bridge['gz_topic_name']
        for bridge in bridges
    }

    assert topics['tb1/wheel_odom'] == 'tb1/odom'
    assert topics['tb1/odom'] == 'tb1/ground_truth_odom'
    assert topics['tb1/ground_truth_odom'] == 'tb1/ground_truth_odom'
    assert topics['tb1/tf'] == 'tb1/ground_truth_tf'


def test_wheel_bridge_keeps_original_odom_and_tf(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(utils.tempfile, 'gettempdir', lambda: str(tmp_path))
    bridge_input = _write_bridge_input(tmp_path / 'bridge.yaml')

    output = utils.create_namespaced_bridge_yaml(
        str(bridge_input), 'tb1', use_ground_truth_odom=False
    )
    bridges = yaml.safe_load(Path(output).read_text(encoding='utf-8'))
    topics = {
        bridge['ros_topic_name']: bridge['gz_topic_name']
        for bridge in bridges
    }

    assert topics['tb1/odom'] == 'tb1/odom'
    assert topics['tb1/tf'] == 'tb1/tf'
    assert topics['tb1/ground_truth_odom'] == 'tb1/ground_truth_odom'
    assert 'tb1/wheel_odom' not in topics
