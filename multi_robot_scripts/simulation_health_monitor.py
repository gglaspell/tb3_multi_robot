#!/usr/bin/env python3

"""Maintain a lightweight readiness heartbeat for the simulated robots."""

from functools import partial
from pathlib import Path
import time

from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class SimulationHealthMonitor(Node):
    """Track odometry freshness with one persistent DDS participant."""

    def __init__(self) -> None:
        super().__init__('simulation_health_monitor')

        self.declare_parameter('robot_names', ['tb1', 'tb3'])
        self.declare_parameter('odom_timeout', 2.0)
        self.declare_parameter('heartbeat_period', 0.5)
        self.declare_parameter(
            'ready_file', '/tmp/tb3_multi_robot.ready'
        )

        robot_names_parameter = self.get_parameter('robot_names')
        self._robot_names = list(
            robot_names_parameter.get_parameter_value().string_array_value
        )
        self._odom_timeout = float(
            self.get_parameter('odom_timeout').value
        )
        heartbeat_period = float(
            self.get_parameter('heartbeat_period').value
        )
        self._ready_file = Path(
            self.get_parameter('ready_file').value
        )

        if not self._robot_names:
            raise ValueError('robot_names must contain at least one robot')
        if self._odom_timeout <= 0.0:
            raise ValueError('odom_timeout must be positive')
        if heartbeat_period <= 0.0:
            raise ValueError('heartbeat_period must be positive')

        self._last_odom = dict.fromkeys(self._robot_names)
        self._healthy = False
        self._subscriptions = [
            self.create_subscription(
                Odometry,
                f'/{name}/odom',
                partial(self._odom_callback, name),
                qos_profile_sensor_data,
            )
            for name in self._robot_names
        ]
        self._timer = self.create_timer(
            heartbeat_period, self._update_health
        )

        self.clear_ready_file()
        self.get_logger().info(
            'Monitoring odometry freshness for: '
            + ', '.join(self._robot_names)
        )

    def _odom_callback(self, robot_name: str, _msg: Odometry) -> None:
        self._last_odom[robot_name] = time.monotonic()

    def _update_health(self) -> None:
        now = time.monotonic()
        healthy = all(
            last_seen is not None and now - last_seen <= self._odom_timeout
            for last_seen in self._last_odom.values()
        )

        if healthy:
            # Refreshing the mtime lets the Docker check detect both stale
            # odometry and a monitor process that has stopped running.
            self._ready_file.touch()
        else:
            self.clear_ready_file()

        if healthy != self._healthy:
            self._healthy = healthy
            message = 'All robot odometry streams are fresh'
            if healthy:
                self.get_logger().info(message)
            else:
                stale = [
                    name for name, last_seen in self._last_odom.items()
                    if last_seen is None
                    or now - last_seen > self._odom_timeout
                ]
                self.get_logger().warning(
                    'Stale odometry streams: ' + ', '.join(stale)
                )

    def clear_ready_file(self) -> None:
        self._ready_file.unlink(missing_ok=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimulationHealthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.clear_ready_file()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
