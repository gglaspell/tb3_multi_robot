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

"""
tb3_follow_tb1.py

Subscribes to tb1's odometry and continuously publishes a trailing goal
pose to tb3's /goal_pose topic.

The goal is computed as a point `follow_distance` metres behind tb1
along the direction tb1 has been travelling, estimated from a short
pose history. tb3's Nav2 stack (using the follow_dynamic_point BT)
replans to this goal as it moves.

Parameters
----------
follow_distance       : float  (default 0.5)  – metres to trail behind tb1
publish_rate          : float  (default 2.0)  – Hz, how often to push a new goal
heading_history_size  : int    (default 5)    – number of past poses used to
                                                estimate tb1's travel direction
deadband_distance     : float  (default 0.1)  – metres; skip publish if the new
                                                goal hasn't moved this far from
                                                the last published goal
stationary_threshold  : float  (default 0.05) – metres; if tb1 moves less than
                                                this between oldest and newest
                                                history pose, treat it as stopped
                                                and hold the last valid goal
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class Tb3FollowTb1(Node):

    def __init__(self):
        super().__init__('tb3_follow_tb1')

        # ── Parameters ────────────────────────────────────────────────────────
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

        # ── State ─────────────────────────────────────────────────────────────
        # Ring buffer of (x, y) tuples — most recent appended to the right
        self._pose_history: deque = deque(maxlen=history_size)
        self._last_goal: tuple | None = None   # (x, y) of the last published goal

        # ── ROS interfaces ────────────────────────────────────────────────────
        self._odom_sub = self.create_subscription(
            Odometry,
            '/tb1/odom',
            self._odom_callback,
            10
        )

        self._goal_pub = self.create_publisher(
            PoseStamped,
            '/tb3/goal_pose',
            10
        )

        period = 1.0 / self.publish_rate
        self._timer = self.create_timer(period, self._publish_goal)

        self.get_logger().info(
            f'tb3_follow_tb1 started — '
            f'follow_distance={self.follow_distance}m  '
            f'publish_rate={self.publish_rate}Hz  '
            f'history_size={history_size}'
        )

    # ── Odometry callback ─────────────────────────────────────────────────────

    def _odom_callback(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self._pose_history.append((x, y))

    # ── Timer callback ────────────────────────────────────────────────────────

    def _publish_goal(self) -> None:
        if len(self._pose_history) < 2:
            # Not enough data yet
            return

        p_now  = self._pose_history[-1]   # most recent tb1 pose
        p_past = self._pose_history[0]    # oldest pose in the history window

        dx = p_now[0] - p_past[0]
        dy = p_now[1] - p_past[1]
        dist_moved = math.hypot(dx, dy)

        if dist_moved < self.stationary_threshold:
            # tb1 is effectively stationary — hold the last valid goal
            if self._last_goal is None:
                return
            gx, gy = self._last_goal
            # Still publish at the held goal so Nav2 doesn't time out
            self._send_goal(gx, gy, self._last_heading if hasattr(self, '_last_heading') else 0.0)
            return

        # Travel direction: the heading tb1 is moving along
        heading = math.atan2(dy, dx)

        # Trailing goal: step back along tb1's direction of travel
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

    # ── Publisher helper ──────────────────────────────────────────────────────

    def _send_goal(self, x: float, y: float, yaw: float) -> None:
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, yaw)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self._goal_pub.publish(msg)
        self.get_logger().debug(
            f'Goal → x={x:.3f}  y={y:.3f}  yaw={math.degrees(yaw):.1f}°'
        )


def main(args=None):
    rclpy.init(args=args)
    node = Tb3FollowTb1()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
