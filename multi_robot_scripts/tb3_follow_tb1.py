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

Parameters
----------
follow_distance      : float (default 0.5)  – metres to trail behind tb1
publish_rate         : float (default 2.0)  – Hz, how often to send a new goal
heading_history_size : int   (default 5)    – past poses used to estimate direction
deadband_distance    : float (default 0.1)  – skip if goal moved less than this (m)
stationary_threshold : float (default 0.05) – treat tb1 as stopped below this (m)
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from tf_transformations import quaternion_from_euler


class Tb3FollowTb1(Node):

    def __init__(self):
        super().__init__('tb3_follow_tb1')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('follow_distance',      0.5)
        self.declare_parameter('publish_rate',         2.0)
        self.declare_parameter('heading_history_size', 5)
        self.declare_parameter('deadband_distance',    0.1)
        self.declare_parameter('stationary_threshold', 0.05)

        self.follow_distance      = self.get_parameter('follow_distance').value
        self.publish_rate         = self.get_parameter('publish_rate').value
        history_size              = self.get_parameter('heading_history_size').value
        self.deadband_distance    = self.get_parameter('deadband_distance').value
        self.stationary_threshold = self.get_parameter('stationary_threshold').value

        # ── State ─────────────────────────────────────────────────────
        # Ring buffer of (x, y) tuples in the map frame
        self._pose_history: deque = deque(maxlen=history_size)
        self._last_goal: tuple | None = None   # (x, y) of last sent goal
        self._last_heading: float = 0.0
        self._goal_handle = None               # active action goal handle
        self._goal_in_flight: bool = False

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
        self._pose_history.append((x, y))

    # ── Timer callback ────────────────────────────────────────────────

    def _publish_goal(self) -> None:
        if len(self._pose_history) < 2:
            return

        p_now  = self._pose_history[-1]   # most recent tb1 map pose
        p_past = self._pose_history[0]    # oldest pose in the window

        dx = p_now[0] - p_past[0]
        dy = p_now[1] - p_past[1]
        dist_moved = math.hypot(dx, dy)

        if dist_moved < self.stationary_threshold:
            # tb1 stationary — re-send the last valid goal to keep Nav2 alive
            if self._last_goal is None:
                return
            gx, gy = self._last_goal
            self._send_goal(gx, gy, self._last_heading)
            return

        # Heading tb1 is travelling along
        heading = math.atan2(dy, dx)

        # Trailing point: follow_distance metres behind tb1
        gx = p_now[0] - self.follow_distance * math.cos(heading)
        gy = p_now[1] - self.follow_distance * math.sin(heading)

        # Deadband: skip if the goal hasn't moved enough to warrant a replan
        if self._last_goal is not None:
            delta = math.hypot(gx - self._last_goal[0], gy - self._last_goal[1])
            if delta < self.deadband_distance:
                return

        self._last_goal    = (gx, gy)
        self._last_heading = heading
        self._send_goal(gx, gy, heading)

    # ── Action goal sender ────────────────────────────────────────────

    def _send_goal(self, x: float, y: float, yaw: float) -> None:
        # Wait briefly for the action server to become available
        if not self._nav_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn(
                '/tb3/navigate_to_pose action server not available — skipping',
                throttle_duration_sec=5.0
            )
            return

        # Cancel the previous in-flight goal before sending a new one
        if self._goal_in_flight and self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_in_flight = False

        # Build the goal pose (FIX 3: computed in map frame from amcl_pose)
        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
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

        send_future = self._nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_callback)

        self.get_logger().debug(
            f'Goal → x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f}°'
        )

    def _goal_response_callback(self, future) -> None:
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().warn('Goal rejected by /tb3/navigate_to_pose')
            self._goal_in_flight = False
            return
        self._goal_in_flight = True
        self._goal_handle.get_result_async().add_done_callback(
            self._goal_result_callback
        )

    def _goal_result_callback(self, future) -> None:
        self._goal_in_flight = False
        self.get_logger().debug('Goal completed')


def main(args=None):
    rclpy.init(args=args)
    node = Tb3FollowTb1()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
