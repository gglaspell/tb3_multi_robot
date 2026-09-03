#!/usr/bin/env python3

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

"""Coordinate an autonomous two-red-versus-one-blue Nav2 tag game.

Each robot owns a namespaced Nav2 stack and local costmaps. This node shares
team pose information, sends flanking intercept goals to red, sends escape
waypoints to blue, and owns the tag/score/round state machine.
"""

import json
import math
from dataclasses import dataclass
from typing import Any, Iterable, Sequence

from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped

from nav2_msgs.action import NavigateToPose

from nav_msgs.msg import OccupancyGrid, Odometry

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from std_msgs.msg import String

from tf_transformations import euler_from_quaternion, quaternion_from_euler


@dataclass(frozen=True)
class Pose2D:
    """Planar robot pose and world-frame translational velocity."""

    x: float
    y: float
    yaw: float
    vx: float = 0.0
    vy: float = 0.0


@dataclass
class GoalState:
    """Bounded asynchronous state for one Nav2 action client."""

    last_requested: tuple[float, float, float] | None = None
    queued: tuple[float, float, float] | None = None
    pending: tuple[float, float, float] | None = None
    send_in_flight: bool = False
    sequence: int = 0
    active_sequence: int | None = None
    cancel_in_flight: bool = False
    handle: Any = None


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp *value* to the inclusive range [lower, upper]."""
    return max(lower, min(value, upper))


def normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def distance(first: Pose2D, second: Pose2D) -> float:
    """Return planar centre-to-centre distance between two poses."""
    return math.hypot(first.x - second.x, first.y - second.y)


def nearest_tagger(
    red_poses: Iterable[tuple[str, Pose2D]],
    blue_pose: Pose2D,
    tag_distance: float,
) -> tuple[str, float] | None:
    """Return the closest red within tag range, if one exists."""
    distances = [
        (name, distance(pose, blue_pose)) for name, pose in red_poses
    ]
    if not distances:
        return None
    name, separation = min(distances, key=lambda item: item[1])
    if separation <= tag_distance:
        return name, separation
    return None


def compute_pursuit_goal(
    pursuer: Pose2D,
    evader: Pose2D,
    flank_side: float,
    pursuer_speed: float,
    tag_distance: float,
    max_lead_time: float = 1.0,
    flank_distance: float = 0.45,
    flank_fade_distance: float = 1.25,
) -> tuple[float, float, float]:
    """Predict an evader intercept and offset it for two-sided pursuit.

    ``flank_side`` should be negative for one chaser and positive for the
    other. Lead and flank offsets collapse near tag range, ensuring the
    robots converge instead of orbiting blue.
    """
    separation = distance(pursuer, evader)
    closing_distance = max(0.0, separation - tag_distance)
    lead_time = min(max_lead_time, closing_distance / pursuer_speed)
    predicted_x = evader.x + evader.vx * lead_time
    predicted_y = evader.y + evader.vy * lead_time

    velocity_norm = math.hypot(evader.vx, evader.vy)
    if velocity_norm > 1e-3:
        direction_x = evader.vx / velocity_norm
        direction_y = evader.vy / velocity_norm
    else:
        direction_x = math.cos(evader.yaw)
        direction_y = math.sin(evader.yaw)

    fade_span = max(1e-6, flank_fade_distance - tag_distance)
    flank_scale = clamp(closing_distance / fade_span, 0.0, 1.0)
    offset = flank_side * flank_distance * flank_scale
    goal_x = predicted_x - direction_y * offset
    goal_y = predicted_y + direction_x * offset
    goal_yaw = math.atan2(evader.y - pursuer.y, evader.x - pursuer.x)
    return goal_x, goal_y, goal_yaw


def compute_escape_heading(
    evader: Pose2D,
    pursuers: Sequence[Pose2D],
    arena_bounds: tuple[float, float, float, float],
    wall_margin: float,
    wander_heading: float,
    wall_weight: float = 2.5,
    wander_weight: float = 0.12,
) -> float:
    """Choose a heading away from pursuers while repelling arena walls."""
    force_x = wander_weight * math.cos(wander_heading)
    force_y = wander_weight * math.sin(wander_heading)

    for pursuer in pursuers:
        away_x = evader.x - pursuer.x
        away_y = evader.y - pursuer.y
        separation = max(math.hypot(away_x, away_y), 0.05)
        inverse_square = 1.0 / (separation * separation)
        force_x += away_x / separation * inverse_square
        force_y += away_y / separation * inverse_square

    min_x, max_x, min_y, max_y = arena_bounds

    def wall_repulsion(clearance: float) -> float:
        if clearance >= wall_margin:
            return 0.0
        normalized = (wall_margin - max(clearance, 0.0)) / wall_margin
        return wall_weight * normalized * normalized

    force_x += wall_repulsion(evader.x - min_x)
    force_x -= wall_repulsion(max_x - evader.x)
    force_y += wall_repulsion(evader.y - min_y)
    force_y -= wall_repulsion(max_y - evader.y)

    if math.hypot(force_x, force_y) < 1e-6:
        return normalize_angle(evader.yaw)
    return math.atan2(force_y, force_x)


def compute_escape_goal(
    evader: Pose2D,
    pursuers: Sequence[Pose2D],
    arena_bounds: tuple[float, float, float, float],
    wall_margin: float,
    wander_heading: float,
    goal_distance: float = 1.5,
    boundary_clearance: float = 0.25,
) -> tuple[float, float, float]:
    """Return a bounded waypoint in the preferred escape direction."""
    heading = compute_escape_heading(
        evader,
        pursuers,
        arena_bounds,
        wall_margin,
        wander_heading,
    )
    min_x, max_x, min_y, max_y = arena_bounds
    return (
        clamp(
            evader.x + goal_distance * math.cos(heading),
            min_x + boundary_clearance,
            max_x - boundary_clearance,
        ),
        clamp(
            evader.y + goal_distance * math.sin(heading),
            min_y + boundary_clearance,
            max_y - boundary_clearance,
        ),
        heading,
    )


def compute_escape_candidates(
    evader: Pose2D,
    pursuers: Sequence[Pose2D],
    arena_bounds: tuple[float, float, float, float],
    wall_margin: float,
    wander_heading: float,
    goal_distance: float = 1.5,
) -> list[tuple[float, float, float]]:
    """Return preferred and alternate escape goals for map validation."""
    preferred = compute_escape_heading(
        evader,
        pursuers,
        arena_bounds,
        wall_margin,
        wander_heading,
    )
    min_x, max_x, min_y, max_y = arena_bounds
    angle_offsets = (
        0.0,
        math.pi / 4.0,
        -math.pi / 4.0,
        math.pi / 2.0,
        -math.pi / 2.0,
        3.0 * math.pi / 4.0,
        -3.0 * math.pi / 4.0,
        math.pi,
    )
    candidates = []
    for candidate_distance in (
        goal_distance,
        goal_distance * 0.7,
        goal_distance * 0.4,
    ):
        for offset in angle_offsets:
            heading = normalize_angle(preferred + offset)
            candidates.append((
                clamp(
                    evader.x + candidate_distance * math.cos(heading),
                    min_x + 0.25,
                    max_x - 0.25,
                ),
                clamp(
                    evader.y + candidate_distance * math.sin(heading),
                    min_y + 0.25,
                    max_y - 0.25,
                ),
                heading,
            ))
    return candidates


def occupancy_point_is_free(
    grid: OccupancyGrid,
    x: float,
    y: float,
    clearance: float = 0.2,
    occupied_threshold: int = 50,
) -> bool:
    """Check that a map-frame point and circular clearance are known free."""
    resolution = grid.info.resolution
    if resolution <= 0.0 or grid.info.width == 0 or grid.info.height == 0:
        return False
    origin = grid.info.origin
    orientation = origin.orientation
    origin_yaw = euler_from_quaternion((
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    ))[2]
    dx = x - origin.position.x
    dy = y - origin.position.y
    cos_yaw = math.cos(origin_yaw)
    sin_yaw = math.sin(origin_yaw)
    local_x = cos_yaw * dx + sin_yaw * dy
    local_y = -sin_yaw * dx + cos_yaw * dy
    column = math.floor(local_x / resolution)
    row = math.floor(local_y / resolution)
    radius_cells = math.ceil(clearance / resolution)

    for row_offset in range(-radius_cells, radius_cells + 1):
        for column_offset in range(-radius_cells, radius_cells + 1):
            if math.hypot(row_offset, column_offset) * resolution > clearance:
                continue
            checked_row = row + row_offset
            checked_column = column + column_offset
            if not (
                0 <= checked_row < grid.info.height
                and 0 <= checked_column < grid.info.width
            ):
                return False
            value = grid.data[
                checked_row * grid.info.width + checked_column
            ]
            if value < 0 or value >= occupied_threshold:
                return False
    return True


class ChaseTag(Node):
    """Coordinate Nav2 pursuit, evasion, scoring, and round resets."""

    def __init__(self) -> None:
        super().__init__('chase_tag')

        self.declare_parameter('red_robots', ['tb1', 'tb2'])
        self.declare_parameter('blue_robot', 'tb3')
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('goal_update_rate', 1.0)
        self.declare_parameter('goal_deadband', 0.15)
        self.declare_parameter('red_linear_speed', 0.22)
        self.declare_parameter('tag_distance', 0.35)
        self.declare_parameter('tag_pause', 1.5)
        self.declare_parameter('reset_distance', 1.0)
        self.declare_parameter('release_timeout', 8.0)
        self.declare_parameter('pose_timeout', 1.0)
        self.declare_parameter('prediction_time', 1.0)
        self.declare_parameter('flank_distance', 0.45)
        self.declare_parameter('escape_goal_distance', 1.5)
        self.declare_parameter('score_to_win', 0)
        self.declare_parameter('arena_min_x', -5.5)
        self.declare_parameter('arena_max_x', 5.5)
        self.declare_parameter('arena_min_y', -3.5)
        self.declare_parameter('arena_max_y', 3.5)
        self.declare_parameter('wall_margin', 0.8)

        self.red_robots = [
            str(name).strip('/')
            for name in self.get_parameter('red_robots').value
        ]
        self.blue_robot = str(
            self.get_parameter('blue_robot').value
        ).strip('/')
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.goal_update_rate = float(
            self.get_parameter('goal_update_rate').value
        )
        self.goal_deadband = float(
            self.get_parameter('goal_deadband').value
        )
        self.red_speed = float(
            self.get_parameter('red_linear_speed').value
        )
        self.tag_distance = float(
            self.get_parameter('tag_distance').value
        )
        self.tag_pause = float(self.get_parameter('tag_pause').value)
        self.reset_distance = float(
            self.get_parameter('reset_distance').value
        )
        self.release_timeout = float(
            self.get_parameter('release_timeout').value
        )
        self.pose_timeout = float(
            self.get_parameter('pose_timeout').value
        )
        self.prediction_time = float(
            self.get_parameter('prediction_time').value
        )
        self.flank_distance = float(
            self.get_parameter('flank_distance').value
        )
        self.escape_goal_distance = float(
            self.get_parameter('escape_goal_distance').value
        )
        self.score_to_win = int(
            self.get_parameter('score_to_win').value
        )
        self.arena_bounds = (
            float(self.get_parameter('arena_min_x').value),
            float(self.get_parameter('arena_max_x').value),
            float(self.get_parameter('arena_min_y').value),
            float(self.get_parameter('arena_max_y').value),
        )
        self.wall_margin = float(
            self.get_parameter('wall_margin').value
        )
        self._validate_parameters()

        self.robot_names = [*self.red_robots, self.blue_robot]
        self._poses: dict[str, Pose2D] = {}
        self._pose_times: dict[str, float] = {}
        self._phase = 'waiting'
        self._phase_deadline = 0.0
        self._score = 0
        self._last_tagger: str | None = None
        self._nearest_distance: float | None = None
        self._shared_map: OccupancyGrid | None = None

        self._navigation_clients = {
            name: ActionClient(
                self, NavigateToPose, f'/{name}/navigate_to_pose'
            )
            for name in self.robot_names
        }
        self._goal_update_publishers = {
            name: self.create_publisher(
                PoseStamped, f'/{name}/goal_update', 10
            )
            for name in self.robot_names
        }
        self._goal_states = {
            name: GoalState() for name in self.robot_names
        }

        status_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._status_publisher = self.create_publisher(
            String, '/chase_tag/status', status_qos
        )
        self._event_publisher = self.create_publisher(
            String, '/chase_tag/events', 10
        )

        self._game_subscriptions = []
        for name in self.robot_names:
            self._game_subscriptions.append(self.create_subscription(
                Odometry,
                f'/{name}/ground_truth_odom',
                lambda message, robot=name: self._odom_callback(
                    robot, message
                ),
                qos_profile_sensor_data,
            ))
        self._game_subscriptions.append(self.create_subscription(
            OccupancyGrid,
            '/map',
            self._map_callback,
            status_qos,
        ))

        self._control_timer = self.create_timer(
            1.0 / self.control_rate, self._control_tick
        )
        self._goal_timer = self.create_timer(
            1.0 / self.goal_update_rate, self._update_navigation_goals
        )
        self._status_timer = self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            'Chase/tag coordinator ready: red=%s blue=%s '
            'tag_distance=%.2fm'
            % (self.red_robots, self.blue_robot, self.tag_distance)
        )

    def _validate_parameters(self) -> None:
        if not self.red_robots:
            raise ValueError('red_robots must contain at least one robot')
        if len(set(self.red_robots)) != len(self.red_robots):
            raise ValueError('red_robots must not contain duplicates')
        if self.blue_robot in self.red_robots:
            raise ValueError('blue_robot cannot also be a red robot')
        if self.control_rate <= 0.0 or self.goal_update_rate <= 0.0:
            raise ValueError('control and goal update rates must be positive')
        if self.red_speed <= 0.0:
            raise ValueError('red_linear_speed must be positive')
        if self.tag_distance <= 0.0:
            raise ValueError('tag_distance must be positive')
        if self.reset_distance <= self.tag_distance:
            raise ValueError('reset_distance must exceed tag_distance')
        min_x, max_x, min_y, max_y = self.arena_bounds
        if min_x >= max_x or min_y >= max_y:
            raise ValueError('arena bounds must have positive area')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _odom_callback(self, robot: str, message: Odometry) -> None:
        orientation = message.pose.pose.orientation
        yaw = euler_from_quaternion((
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ))[2]
        body_vx = message.twist.twist.linear.x
        body_vy = message.twist.twist.linear.y
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        self._poses[robot] = Pose2D(
            x=message.pose.pose.position.x,
            y=message.pose.pose.position.y,
            yaw=yaw,
            vx=cos_yaw * body_vx - sin_yaw * body_vy,
            vy=sin_yaw * body_vx + cos_yaw * body_vy,
        )
        self._pose_times[robot] = self._now()

    def _map_callback(self, message: OccupancyGrid) -> None:
        self._shared_map = message

    def _poses_are_fresh(self, now: float) -> bool:
        return all(
            name in self._poses
            and now - self._pose_times.get(name, -math.inf)
            <= self.pose_timeout
            for name in self.robot_names
        )

    def _set_phase(self, phase: str, event: str | None = None) -> None:
        changed = phase != self._phase
        self._phase = phase
        if event is not None:
            message = String()
            message.data = event
            self._event_publisher.publish(message)
            self.get_logger().info(event)
        elif changed:
            self.get_logger().info(f'Game phase: {phase}')
        if changed or event is not None:
            self._publish_status()

    def _control_tick(self) -> None:
        now = self._now()
        if not self._poses_are_fresh(now):
            if self._phase not in ('waiting', 'game_over'):
                self._set_phase('waiting', 'Waiting for fresh robot poses')
                self._cancel_all_goals()
            return

        if self._phase == 'waiting':
            self._set_phase('active', 'Round started')

        blue_pose = self._poses[self.blue_robot]
        red_pose_items = [
            (name, self._poses[name]) for name in self.red_robots
        ]
        separations = [
            (name, distance(pose, blue_pose))
            for name, pose in red_pose_items
        ]
        _, self._nearest_distance = min(
            separations, key=lambda item: item[1]
        )

        if self._phase == 'active':
            tag = nearest_tagger(
                red_pose_items, blue_pose, self.tag_distance
            )
            if tag is None:
                return
            tagger, separation = tag
            self._score += 1
            self._last_tagger = tagger
            if self.score_to_win > 0 and self._score >= self.score_to_win:
                self._set_phase(
                    'game_over',
                    f'{tagger} tagged {self.blue_robot} at '
                    f'{separation:.2f}m; red wins {self._score}',
                )
            else:
                self._phase_deadline = now + self.tag_pause
                self._set_phase(
                    'paused',
                    f'{tagger} tagged {self.blue_robot} at '
                    f'{separation:.2f}m; score={self._score}',
                )
            self._cancel_all_goals()
            return

        if self._phase == 'paused' and now >= self._phase_deadline:
            self._phase_deadline = now + self.release_timeout
            self._set_phase(
                'release', f'{self.blue_robot} has a head start'
            )
            return

        if self._phase == 'release' and (
            self._nearest_distance >= self.reset_distance
            or now >= self._phase_deadline
        ):
            self._set_phase('active', 'Red released; chase resumed')

    def _update_navigation_goals(self) -> None:
        if not self._poses_are_fresh(self._now()):
            return
        if self._phase not in ('active', 'release'):
            return

        blue_pose = self._poses[self.blue_robot]
        red_pose_items = [
            (name, self._poses[name]) for name in self.red_robots
        ]
        if self._phase == 'active':
            flank_sides = (-1.0, 1.0)
            for index, (name, red_pose) in enumerate(red_pose_items):
                goal = compute_pursuit_goal(
                    red_pose,
                    blue_pose,
                    flank_side=flank_sides[index % len(flank_sides)],
                    pursuer_speed=self.red_speed,
                    tag_distance=self.tag_distance,
                    max_lead_time=self.prediction_time,
                    flank_distance=self.flank_distance,
                )
                goal = self._bound_goal(goal)
                if (
                    self._shared_map is not None
                    and not occupancy_point_is_free(
                        self._shared_map, goal[0], goal[1]
                    )
                ):
                    goal = (blue_pose.x, blue_pose.y, goal[2])
                self._queue_goal(name, goal)

        blue_goal = self._select_escape_goal(
            blue_pose, [pose for _, pose in red_pose_items]
        )
        self._queue_goal(self.blue_robot, blue_goal)

        for name in self.robot_names:
            self._dispatch_goal(name)

    def _bound_goal(
        self, goal: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        min_x, max_x, min_y, max_y = self.arena_bounds
        return (
            clamp(goal[0], min_x + 0.2, max_x - 0.2),
            clamp(goal[1], min_y + 0.2, max_y - 0.2),
            normalize_angle(goal[2]),
        )

    def _select_escape_goal(
        self, blue_pose: Pose2D, red_poses: Sequence[Pose2D]
    ) -> tuple[float, float, float]:
        candidates = compute_escape_candidates(
            blue_pose,
            red_poses,
            self.arena_bounds,
            self.wall_margin,
            wander_heading=0.37 * self._now(),
            goal_distance=self.escape_goal_distance,
        )
        if self._shared_map is None:
            return candidates[0]
        for candidate in candidates:
            if occupancy_point_is_free(
                self._shared_map, candidate[0], candidate[1]
            ):
                return candidate
        # Staying put is safer than asking Nav2 to plan through occupied space.
        return blue_pose.x, blue_pose.y, blue_pose.yaw

    def _queue_goal(
        self, robot: str, goal: tuple[float, float, float]
    ) -> None:
        state = self._goal_states[robot]
        reference = state.queued or state.pending or state.last_requested
        if reference is not None and math.hypot(
            goal[0] - reference[0], goal[1] - reference[1]
        ) < self.goal_deadband:
            return
        # Coalesce updates while Nav2 acknowledges the previous send request.
        state.queued = goal

    def _dispatch_goal(self, robot: str) -> None:
        state = self._goal_states[robot]
        client = self._navigation_clients[robot]
        if state.send_in_flight or state.queued is None:
            return
        if state.handle is not None:
            if state.cancel_in_flight:
                return
            goal = state.queued
            state.queued = None
            state.last_requested = goal
            self._goal_update_publishers[robot].publish(
                self._goal_message(goal)
            )
            return
        if not client.server_is_ready():
            self.get_logger().warning(
                f'/{robot}/navigate_to_pose unavailable; goal queued',
                throttle_duration_sec=5.0,
            )
            return

        goal = state.queued
        state.queued = None
        state.pending = goal
        state.send_in_flight = True
        state.sequence += 1
        sequence = state.sequence

        action_goal = NavigateToPose.Goal()
        action_goal.pose = self._goal_message(goal)

        future = client.send_goal_async(action_goal)
        future.add_done_callback(
            lambda completed, name=robot, sent_goal=goal, sent_seq=sequence:
            self._goal_response(completed, name, sent_goal, sent_seq)
        )

    def _goal_message(
        self, goal: tuple[float, float, float]
    ) -> PoseStamped:
        """Build a map-frame pose for an action or live goal update."""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = goal[0]
        pose.pose.position.y = goal[1]
        quaternion = quaternion_from_euler(0.0, 0.0, goal[2])
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        return pose

    def _goal_response(
        self,
        future,
        robot: str,
        goal: tuple[float, float, float],
        sequence: int,
    ) -> None:
        state = self._goal_states[robot]
        state.send_in_flight = False
        state.pending = None
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f'Failed to send {robot} navigation goal: {exc}'
            )
            self._dispatch_goal(robot)
            return

        if not handle.accepted:
            self.get_logger().warning(f'{robot} navigation goal rejected')
            self._dispatch_goal(robot)
            return

        state.handle = handle
        state.active_sequence = sequence
        state.last_requested = goal
        handle.get_result_async().add_done_callback(
            lambda completed, name=robot, sent_seq=sequence:
            self._goal_result(completed, name, sent_seq)
        )
        if not self._robot_should_move(robot):
            self._cancel_robot_goal(robot)
            return

        self._dispatch_goal(robot)

    def _goal_result(self, future, robot: str, sequence: int) -> None:
        state = self._goal_states[robot]
        try:
            status = future.result().status
        except Exception as exc:  # noqa: BLE001
            self.get_logger().debug(
                f'{robot} navigation result unavailable: {exc}'
            )
            status = None
        if sequence == state.active_sequence:
            state.handle = None
            state.active_sequence = None
            state.cancel_in_flight = False
            if status != GoalStatus.STATUS_SUCCEEDED:
                # Permit a fresh attempt when a planner/controller aborts.
                state.last_requested = None
            self._dispatch_goal(robot)

    def _robot_should_move(self, robot: str) -> bool:
        return self._phase == 'active' or (
            self._phase == 'release' and robot == self.blue_robot
        )

    def _cancel_robot_goal(self, robot: str) -> None:
        state = self._goal_states[robot]
        state.queued = None
        state.last_requested = None
        self._request_goal_cancel(robot)

    def _request_goal_cancel(self, robot: str) -> None:
        state = self._goal_states[robot]
        if state.handle is None or state.cancel_in_flight:
            return
        sequence = state.active_sequence
        state.cancel_in_flight = True
        future = state.handle.cancel_goal_async()
        future.add_done_callback(
            lambda completed, name=robot, sent_seq=sequence:
            self._cancel_response(completed, name, sent_seq)
        )

    def _cancel_response(self, future, robot: str, sequence: int) -> None:
        state = self._goal_states[robot]
        if sequence != state.active_sequence:
            return
        state.cancel_in_flight = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().debug(
                f'Failed to cancel {robot} navigation goal: {exc}'
            )
            return
        if not response.goals_canceling:
            self.get_logger().debug(
                f'{robot} navigation goal was no longer cancellable'
            )

    def _cancel_all_goals(self) -> None:
        for robot in self.robot_names:
            self._cancel_robot_goal(robot)

    def _publish_status(self) -> None:
        message = String()
        message.data = json.dumps({
            'phase': self._phase,
            'score': self._score,
            'red_robots': self.red_robots,
            'blue_robot': self.blue_robot,
            'last_tagger': self._last_tagger,
            'nearest_distance': (
                round(self._nearest_distance, 3)
                if self._nearest_distance is not None else None
            ),
            'shared_map_ready': self._shared_map is not None,
        }, separators=(',', ':'))
        self._status_publisher.publish(message)

    def stop(self) -> None:
        """Cancel outstanding Nav2 goals when the game coordinator exits."""
        self._cancel_all_goals()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ChaseTag()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
