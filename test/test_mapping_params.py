"""Regression tests for the generated namespaced mapping profile."""

from pathlib import Path
import math

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
    assert namespaced['transform_timeout'] == pytest.approx(0.35)
    assert namespaced['tf_buffer_duration'] == pytest.approx(30.0)
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
            {
                'ros_topic_name': 'scan',
                'gz_topic_name': 'scan',
                'ros_type_name': 'sensor_msgs/msg/LaserScan',
                'gz_type_name': 'gz.msgs.LaserScan',
                'direction': 'GZ_TO_ROS',
            },
        ]),
        encoding='utf-8',
    )
    return path


def test_truth_bridge_retains_wheel_odom_without_bypassing_planar_adapter(
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
    assert topics['tb1/ground_truth_odom'] == 'tb1/ground_truth_odom'
    assert 'tb1/odom' not in topics
    assert 'tb1/tf' not in topics


def test_planar_quaternion_discards_roll_and_pitch_without_changing_yaw() -> None:
    # 0.7 degrees of pitch is representative of the contact-induced tilt
    # measured in Gazebo's supposedly 2D ground-truth odometry stream.
    roll = math.radians(0.3)
    pitch = math.radians(-0.7)
    yaw = math.radians(-98.969)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    source = (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )

    qx, qy, qz, qw = utils.planar_quaternion(*source)

    assert (qx, qy) == pytest.approx((0.0, 0.0))
    assert utils.yaw_from_quaternion(qx, qy, qz, qw) == pytest.approx(yaw)


def test_truth_mapping_launch_uses_the_planar_odom_adapter() -> None:
    launch = (
        Path(__file__).resolve().parents[1] / 'launch' / 'tb3_world.launch.py'
    ).read_text(encoding='utf-8')

    assert "executable='planar_odom'" in launch
    assert "'input_topic': 'ground_truth_odom'" in launch
    assert "'odom_topic': 'odom'" in launch
    assert "('/tf', f'/{namespace}/tf')" in launch


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


def test_mapping_scan_bridge_routes_tb1_through_the_timestamp_gate(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(utils.tempfile, 'gettempdir', lambda: str(tmp_path))
    bridge_input = _write_bridge_input(tmp_path / 'bridge.yaml')

    output = utils.create_namespaced_bridge_yaml(
        str(bridge_input), 'tb1', filter_scans=True
    )
    bridges = yaml.safe_load(Path(output).read_text(encoding='utf-8'))
    topics = {
        bridge['ros_topic_name']: bridge['gz_topic_name']
        for bridge in bridges
    }

    assert topics['tb1/scan_raw'] == 'tb1/scan'
    assert 'tb1/scan' not in topics


def test_burger_wheel_odometry_rate_matches_mapping_pose_budget() -> None:
    model_path = (
        Path(__file__).resolve().parents[1]
        / 'models'
        / 'turtlebot3_burger'
        / 'model.sdf'
    )

    model = model_path.read_text(encoding='utf-8')

    assert '<odom_publisher_frequency>20</odom_publisher_frequency>' in model


def test_follow_rviz_profiles_are_tracked_and_render_at_30_fps() -> None:
    rviz_dir = Path(__file__).resolve().parents[1] / 'rviz'
    expected_maps = {
        'tb1_navigation2.rviz': '/tb1/map',
        'tb3_navigation2.rviz': '/map',
    }

    for filename, map_topic in expected_maps.items():
        profile = (rviz_dir / filename).read_text(encoding='utf-8')
        assert '<ROBOT_NAME>' not in profile
        assert '<MAP_TOPIC>' not in profile
        assert 'Frame Rate: 30' in profile
        assert f'Value: {map_topic}' in profile

    template = (rviz_dir / 'navigation2_template.rviz').read_text(
        encoding='utf-8'
    )
    assert '<ROBOT_NAME>' in template
    assert '<MAP_TOPIC>' in template
    assert 'Frame Rate: 30' in template


def test_rviz_software_rendering_defaults_to_eight_threads() -> None:
    package_root = Path(__file__).resolve().parents[1]
    compose = (package_root / 'docker' / 'docker-compose.yaml').read_text(
        encoding='utf-8'
    )
    assert compose.count('TB3_RVIZ_RENDER_THREADS:-8') == 6
    assert compose.count('gpus: all') == 4
    assert compose.count('NVIDIA_DRIVER_CAPABILITIES:') == 4

    for launch_name in (
        'follow_sim.launch.py',
        'follow_tb3.launch.py',
        'chase_tag.launch.py',
        'chase_tag_nav.launch.py',
    ):
        launch = (package_root / 'launch' / launch_name).read_text(
            encoding='utf-8'
        )
        assert "'LP_NUM_THREADS', default_value='8'" in launch

    world_launch = (package_root / 'launch' / 'tb3_world.launch.py').read_text(
        encoding='utf-8'
    )
    assert "'TB3_SOFTWARE_RENDERING', default_value='false'" in world_launch
    assert "condition=IfCondition(software_rendering)" in world_launch
