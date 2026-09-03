"""Unit tests for chase/tag pursuit, evasion, and Nav2 profiles."""

import math
from pathlib import Path

from multi_robot_scripts.chase_tag import (
    ChaseTag,
    GoalState,
    Pose2D,
    compute_escape_goal,
    compute_escape_heading,
    compute_pursuit_goal,
    nearest_tagger,
    occupancy_point_is_free,
)
from multi_robot_scripts.utils import (
    generate_chase_nav2_params,
    load_sdf_with_namespace,
)

from nav_msgs.msg import OccupancyGrid

import pytest

import yaml


def test_two_pursuers_flank_opposite_sides_of_moving_blue() -> None:
    red = Pose2D(-1.0, 0.0, 0.0)
    blue = Pose2D(1.0, 0.0, 0.0, vx=0.15, vy=0.0)

    lower_goal = compute_pursuit_goal(
        red, blue, -1.0, pursuer_speed=0.22, tag_distance=0.35
    )
    upper_goal = compute_pursuit_goal(
        red, blue, 1.0, pursuer_speed=0.22, tag_distance=0.35
    )

    assert lower_goal[0] > blue.x
    assert upper_goal[0] > blue.x
    assert lower_goal[1] < blue.y
    assert upper_goal[1] > blue.y


def test_pursuit_offsets_collapse_at_tag_distance() -> None:
    red = Pose2D(0.0, 0.0, 0.0)
    blue = Pose2D(0.35, 0.0, 0.0, vx=0.2, vy=0.0)

    goal = compute_pursuit_goal(
        red, blue, 1.0, pursuer_speed=0.22, tag_distance=0.35
    )

    assert goal[:2] == pytest.approx((blue.x, blue.y))


def test_blue_flees_symmetric_red_pair() -> None:
    blue = Pose2D(1.0, 0.0, 0.0)
    red = [Pose2D(-1.0, -0.5, 0.0), Pose2D(-1.0, 0.5, 0.0)]

    heading = compute_escape_heading(
        blue,
        red,
        (-5.5, 5.5, -3.5, 3.5),
        wall_margin=0.8,
        wander_heading=0.0,
        wander_weight=0.0,
    )

    assert heading == pytest.approx(0.0)


def test_wall_repulsion_turns_blue_back_into_arena() -> None:
    blue = Pose2D(5.35, 0.0, 0.0)
    heading = compute_escape_heading(
        blue,
        [Pose2D(0.0, 0.0, 0.0)],
        (-5.5, 5.5, -3.5, 3.5),
        wall_margin=0.8,
        wander_heading=0.0,
        wander_weight=0.0,
    )

    assert math.cos(heading) < 0.0


def test_escape_goal_stays_inside_navigation_bounds() -> None:
    goal = compute_escape_goal(
        Pose2D(5.4, 3.4, 0.0),
        [Pose2D(4.0, 2.0, 0.0)],
        (-5.5, 5.5, -3.5, 3.5),
        wall_margin=0.8,
        wander_heading=0.0,
        goal_distance=2.0,
    )

    assert -5.25 <= goal[0] <= 5.25
    assert -3.25 <= goal[1] <= 3.25


def test_nearest_red_gets_tag_credit() -> None:
    blue = Pose2D(0.0, 0.0, 0.0)
    tag = nearest_tagger([
        ('tb1', Pose2D(0.3, 0.0, 0.0)),
        ('tb2', Pose2D(0.2, 0.0, 0.0)),
    ], blue, tag_distance=0.35)

    assert tag is not None
    assert tag[0] == 'tb2'
    assert tag[1] == pytest.approx(0.2)


def test_active_navigation_receives_goal_update_without_replacement() -> None:
    class NavigationClient:

        def __init__(self) -> None:
            self.send_count = 0

        def server_is_ready(self) -> bool:
            return True

        def send_goal_async(self, _goal):
            self.send_count += 1

    class GoalPublisher:

        def __init__(self) -> None:
            self.messages = []

        def publish(self, message) -> None:
            self.messages.append(message)

    goal = (1.0, 0.0, 0.0)
    client = NavigationClient()
    publisher = GoalPublisher()
    coordinator = object.__new__(ChaseTag)
    coordinator._goal_states = {
        'tb1': GoalState(
            queued=goal,
            active_sequence=4,
            handle=object(),
        )
    }
    coordinator._navigation_clients = {'tb1': client}
    coordinator._goal_update_publishers = {'tb1': publisher}
    coordinator._goal_message = lambda value: value

    coordinator._dispatch_goal('tb1')

    assert client.send_count == 0
    assert publisher.messages == [goal]
    assert coordinator._goal_states['tb1'].queued is None
    assert coordinator._goal_states['tb1'].last_requested == goal


def test_escape_waypoint_rejects_occupied_and_unknown_map_cells() -> None:
    grid = OccupancyGrid()
    grid.info.resolution = 1.0
    grid.info.width = 5
    grid.info.height = 5
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * 25
    grid.data[2 * 5 + 3] = 100
    grid.data[3 * 5 + 2] = -1

    assert occupancy_point_is_free(grid, 1.5, 1.5)
    assert not occupancy_point_is_free(grid, 3.5, 2.5)
    assert not occupancy_point_is_free(grid, 2.5, 3.5)
    assert not occupancy_point_is_free(grid, -0.5, 1.5)


def test_chase_nav2_profile_has_pose_namespace_and_speed(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        'multi_robot_scripts.utils.tempfile.gettempdir', lambda: str(tmp_path)
    )
    package_dir = Path(__file__).parents[1]
    output = generate_chase_nav2_params(
        'tb2',
        str(package_dir / 'params' / 'burger_nav2_params_tb3.yaml'),
        (-1.5, 0.5, 0.25),
        0.22,
        behavior_tree_path='/tmp/chase_tree.xml',
    )
    config = yaml.safe_load(Path(output).read_text(encoding='utf-8'))

    amcl = config['amcl']['ros__parameters']
    assert amcl['initial_pose']['x'] == pytest.approx(-1.5)
    assert amcl['initial_pose']['y'] == pytest.approx(0.5)
    assert amcl['initial_pose']['yaw'] == pytest.approx(0.25)
    assert amcl['scan_topic'] == '/tb2/scan'
    controller = config['controller_server']['ros__parameters']['FollowPath']
    assert controller['plugin'].endswith('RegulatedPurePursuitController')
    assert controller['max_linear_vel'] == pytest.approx(0.22)
    assert config['bt_navigator']['ros__parameters'][
        'default_nav_to_pose_bt_xml'
    ] == '/tmp/chase_tree.xml'
    assert '<robot_namespace>' not in Path(output).read_text(encoding='utf-8')


def test_spawn_sdf_is_namespaced_and_team_colored(tmp_path: Path) -> None:
    model = tmp_path / 'model.sdf'
    model.write_text(
        '<sdf><model><visual name="base_visual"><material>'
        '<ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse>'
        '</material></visual><topic>cmd_vel</topic></model></sdf>',
        encoding='utf-8',
    )

    result = load_sdf_with_namespace(
        str(model), 'tb2', color=[0.85, 0.05, 0.05, 1.0]
    )

    assert '<topic>tb2/cmd_vel</topic>' in result
    assert '<ambient>0.85 0.05 0.05 1</ambient>' in result
    assert '<diffuse>0.85 0.05 0.05 1</diffuse>' in result
    assert '<odom_topic>tb2/ground_truth_odom</odom_topic>' in result
