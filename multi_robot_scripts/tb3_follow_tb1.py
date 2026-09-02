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

"""
tb3_follow_tb1.py

Subscribes to tb1's AMCL pose (map-frame, drift-corrected) and sends
NavigateToPose action goals to tb3's Nav2 stack so tb3 trails behind tb1.

Fixes vs. original:
  1. Subscribes to /tb1/amcl_pose (PoseWithCovarianceStamped, map frame)
     instead of /tb1/odom (odom frame — drifts relative to map).
  2. Uses a NavigateToPose action client to /tb3/navigate_to_pose
     instead of publishing to /tb3/goal_pose (unreliable topic path,
     no feedback, ignored when bt_navigator is inactive).
  3. Goals are computed and sent entirely in the map frame.
  4. Coalesces goal updates and uses Nav2's native same-tree preemption instead
     of racing asynchronous cancel and replacement requests.
  5. Uses tb1's localized body heading so turns in place update the trailing
     point as accurately as translational motion.
  6. Samples tb1's live map-to-base transform at the goal evaluation rate,
     retaining AMCL pose messages as a drift-corrected fallback.

Parameters
----------
follow_distance : float (default 0.5)
    Metres to trail behind tb1.
publish_rate : float (default 2.0)
    Hz, how often to evaluate and send a new goal.
heading_history_size : int (default 5)
    Recent poses used to detect motion.
deadband_distance : float (default 0.1)
    Skip a replacement when the goal moved less than this distance in metres.
stationary_threshold : float (default 0.05)
    Treat tb1 as stopped below this translational distance in metres.
stationary_angular_threshold : float (default 0.1)
    Treat tb1 as stopped below this rotation in radians.
"""

import math
from collections import deque

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from nav2_msgs.action import NavigateToPose

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros import Buffer, TransformException, TransformListener

from tf_transformations import euler_from_quaternion, quaternion_from_euler


class Tb3FollowTb1(Node):

    def __init__(self):
        super().__init__('tb3_follow_tb1')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('follow_distance',      0.5)
        self.declare_parameter('publish_rate',         2.0)
        self.declare_parameter('heading_history_size', 5)
        self.declare_parameter('deadband_distance',    0.1)
        self.declare_parameter('stationary_threshold', 0.05)
        self.declare_parameter('stationary_angular_threshold', 0.1)

        self.follow_distance = self.get_parameter('follow_distance').value
        self.publish_rate = self.get_parameter('publish_rate').value
        history_size = self.get_parameter(
            'heading_history_size'
        ).value
        self.deadband_distance = self.get_parameter(
            'deadband_distance'
        ).value
        self.stationary_threshold = self.get_parameter(
            'stationary_threshold'
        ).value
        self.stationary_angular_threshold = self.get_parameter(
            'stationary_angular_threshold'
        ).value

        # ── State ─────────────────────────────────────────────────────
        # Ring buffer of (x, y, yaw) tuples in the map frame
        self._pose_history: deque = deque(maxlen=history_size)
        self._last_goal: tuple | None = None
        self._queued_goal: tuple | None = None
        self._pending_send_goal: tuple | None = None
        self._send_in_flight = False
        self._goal_sequence = 0
        self._active_goal_sequence: int | None = None
        self._goal_handle = None
        self._latest_amcl_pose: tuple | None = None

        # Receive only tb1's namespaced TF stream (remapped by the launch
        # description). It combines AMCL's map correction with live odometry,
        # avoiding the quantization imposed by AMCL's update_min_d threshold.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=False
        )

        # ── Subscriber: tb1 AMCL pose (map frame, drift-corrected) ────
        # FIX 1: was /tb1/odom (odom frame) — now /tb1/amcl_pose (map frame)
        self._amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/tb1/amcl_pose',
            self._amcl_callback,
            10
        )

        # ── Action client: tb3 NavigateToPose ─────────────────────────
        # FIX 2: was a topic publisher to /tb3/goal_pose — now an action
        # client to /tb3/navigate_to_pose for reliable goal delivery.
        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            '/tb3/navigate_to_pose'
        )

        period = 1.0 / self.publish_rate
        self._timer = self.create_timer(period, self._publish_goal)

        self.get_logger().info(
            f'tb3_follow_tb1 started — '
            f'follow_distance={self.follow_distance}m  '
            f'publish_rate={self.publish_rate}Hz  '
            f'history_size={history_size}'
        )

    # ── AMCL callback (map-frame pose) ────────────────────────────────

    def _amcl_callback(self, msg: PoseWithCovarianceStamped) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        yaw = euler_from_quaternion((
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ))[2]
        self._latest_amcl_pose = (x, y, yaw)

    def _current_leader_pose(self) -> tuple | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                'map', 'base_link', Time()
            )
        except TransformException:
            return self._latest_amcl_pose

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = euler_from_quaternion((
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        ))[2]
        return (translation.x, translation.y, yaw)

    @staticmethod
    def _angle_difference(first: float, second: float) -> float:
        return math.atan2(
            math.sin(first - second), math.cos(first - second)
        )

    # ── Timer callback ────────────────────────────────────────────────

    def _publish_goal(self) -> None:
        leader_pose = self._current_leader_pose()
        if leader_pose is None:
            return

        self._pose_history.append(leader_pose)
        if len(self._pose_history) < 2:
            return

        p_now = self._pose_history[-1]    # most recent tb1 map pose
        p_past = self._pose_history[0]    # oldest pose in the window

        dx = p_now[0] - p_past[0]
        dy = p_now[1] - p_past[1]
        dist_moved = math.hypot(dx, dy)
        angle_moved = abs(self._angle_difference(p_now[2], p_past[2]))

        if (
            dist_moved < self.stationary_threshold
            and angle_moved < self.stationary_angular_threshold
        ):
            # An accepted action remains active until completion;
            # leave it alone while the leader is stationary.
            self._dispatch_latest_goal()
            return

        # A differential-drive robot's localized body orientation is a more
        # accurate trailing direction than a chord through noisy position
        # samples, and it continues to work while tb1 turns in place.
        heading = p_now[2]

        # Trailing point: follow_distance metres behind tb1
        gx = p_now[0] - self.follow_distance * math.cos(heading)
        gy = p_now[1] - self.follow_distance * math.sin(heading)

        # Compare with the newest requested goal, including one whose action
        # response is still pending, so callbacks cannot create duplicate work.
        reference_goal = (
            self._queued_goal
            or self._pending_send_goal
            or self._last_goal
        )
        if reference_goal is not None:
            delta = math.hypot(
                gx - reference_goal[0], gy - reference_goal[1]
            )
            if delta < self.deadband_distance:
                self._dispatch_latest_goal()
                return

        # Keep only the freshest target while a previous send request is being
        # acknowledged. This bounds work even if DDS or Nav2 is briefly slow.
        self._queued_goal = (gx, gy, heading)
        self._dispatch_latest_goal()

    # ── Action goal sender ────────────────────────────────────────────

    def _dispatch_latest_goal(self) -> None:
        if self._send_in_flight or self._queued_goal is None:
            return

        if not self._nav_client.server_is_ready():
            self.get_logger().warning(
                '/tb3/navigate_to_pose action server unavailable; goal queued',
                throttle_duration_sec=5.0,
            )
            return

        goal = self._queued_goal
        self._queued_goal = None
        self._pending_send_goal = goal
        self._send_in_flight = True
        self._goal_sequence += 1
        sequence = self._goal_sequence
        x, y, yaw = goal

        # Build the goal pose (FIX 3: computed in map frame from amcl_pose)
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        # NavigateToPose natively accepts same-behavior-tree preemption and
        # updates its blackboard goal. Avoiding an explicit cancel preserves
        # the running BT and removes a cancellation race between result
        # callbacks from the old and new goals.
        send_future = self._nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(
            lambda future: self._goal_response_callback(
                future, goal, sequence
            )
        )

        self.get_logger().debug(
            f'Goal → x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f}°'
        )

    def _goal_response_callback(self, future, goal, sequence: int) -> None:
        self._send_in_flight = False
        self._pending_send_goal = None

        try:
            goal_handle = future.result()
        # ROS futures surface transport errors through this callback.
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Failed to send follower goal: {exc}')
            self._dispatch_latest_goal()
            return

        if not goal_handle.accepted:
            self.get_logger().warning(
                'Goal rejected by /tb3/navigate_to_pose'
            )
            self._dispatch_latest_goal()
            return

        self._goal_handle = goal_handle
        self._active_goal_sequence = sequence
        self._last_goal = goal
        goal_handle.get_result_async().add_done_callback(
            lambda result_future: self._goal_result_callback(
                result_future, sequence
            )
        )
        self._dispatch_latest_goal()

    def _goal_result_callback(self, future, sequence: int) -> None:
        try:
            status = future.result().status
        # ROS futures surface transport errors through this callback.
        except Exception as exc:  # noqa: BLE001
            self.get_logger().debug(f'Goal result unavailable: {exc}')
            status = 'unknown'

        # A result from a preempted goal must not clear the handle belonging to
        # the newer active goal.
        if sequence == self._active_goal_sequence:
            self._goal_handle = None
            self._active_goal_sequence = None
        self.get_logger().debug(f'Goal completed with status {status}')


def main(args=None):
    rclpy.init(args=args)
    node = Tb3FollowTb1()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
