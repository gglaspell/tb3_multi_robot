#!/usr/bin/env python3

"""Pass only laser scans that have a transform at their acquisition time."""

from collections import deque
from dataclasses import dataclass

from sensor_msgs.msg import LaserScan

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from tf2_ros import Buffer, TransformException, TransformListener


@dataclass
class GateStatistics:
    """Counts and timing bounds accumulated between status reports."""

    accepted: int = 0
    rejected_age: int = 0
    rejected_transform: int = 0
    minimum_age_sec: float = float('inf')
    maximum_age_sec: float = float('-inf')
    minimum_latest_offset_sec: float = float('inf')
    maximum_latest_offset_sec: float = float('-inf')

    def record_age(self, age_sec: float) -> None:
        self.minimum_age_sec = min(self.minimum_age_sec, age_sec)
        self.maximum_age_sec = max(self.maximum_age_sec, age_sec)

    def record_latest_offset(self, offset_sec: float) -> None:
        self.minimum_latest_offset_sec = min(
            self.minimum_latest_offset_sec, offset_sec
        )
        self.maximum_latest_offset_sec = max(
            self.maximum_latest_offset_sec, offset_sec
        )


class ScanTfGate(Node):
    """Keep delayed scans from being associated with a latest-pose yaw."""

    def __init__(self) -> None:
        super().__init__('scan_tf_gate')

        self.declare_parameter('raw_scan_topic', 'scan_raw')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('source_frame', 'base_footprint')
        self.declare_parameter('max_scan_age_sec', 0.35)
        self.declare_parameter('transform_timeout_sec', 0.35)
        self.declare_parameter('tf_buffer_duration_sec', 30.0)
        self.declare_parameter('statistics_period_sec', 5.0)

        raw_scan_topic = self.get_parameter('raw_scan_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        self._target_frame = self.get_parameter('target_frame').value
        self._source_frame = self.get_parameter('source_frame').value
        self._max_scan_age_sec = float(
            self.get_parameter('max_scan_age_sec').value
        )
        self._transform_timeout = Duration(seconds=float(
            self.get_parameter('transform_timeout_sec').value
        ))
        tf_buffer_duration = float(
            self.get_parameter('tf_buffer_duration_sec').value
        )
        statistics_period = float(
            self.get_parameter('statistics_period_sec').value
        )

        if self._max_scan_age_sec <= 0.0:
            raise ValueError('max_scan_age_sec must be positive')
        if self._transform_timeout.nanoseconds <= 0:
            raise ValueError('transform_timeout_sec must be positive')
        if tf_buffer_duration <= 0.0:
            raise ValueError('tf_buffer_duration_sec must be positive')
        if statistics_period <= 0.0:
            raise ValueError('statistics_period_sec must be positive')

        self._tf_buffer = Buffer(
            cache_time=Duration(seconds=tf_buffer_duration), node=self
        )
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=False
        )
        self._publisher = self.create_publisher(
            LaserScan, scan_topic, qos_profile_sensor_data
        )
        self._subscription = self.create_subscription(
            LaserScan, raw_scan_topic, self._scan_callback,
            qos_profile_sensor_data,
        )
        self._pending_scans = deque()
        self._statistics = GateStatistics()
        self._retry_timer = self.create_timer(0.01, self._retry_pending_scans)
        self._statistics_timer = self.create_timer(
            statistics_period, self._report_statistics
        )

        self.get_logger().info(
            f'Gating {raw_scan_topic} -> {scan_topic} with '
            f'{self._target_frame} -> {self._source_frame} at scan stamps; '
            f'maximum receipt age {self._max_scan_age_sec:.3f} s and TF '
            f'wait {self._transform_timeout.nanoseconds / 1e9:.3f} s'
        )

    @staticmethod
    def _scan_time(scan: LaserScan) -> Time:
        return Time.from_msg(scan.header.stamp)

    def _scan_age_sec(self, scan: LaserScan) -> float:
        return (
            self.get_clock().now().nanoseconds
            - self._scan_time(scan).nanoseconds
        ) / 1e9

    def _scan_callback(self, scan: LaserScan) -> None:
        # A use_sim_time node starts at zero until its first /clock sample.
        # Do not mistake an already-running simulator's scan stamp for a scan
        # that is millions of seconds in the future during that short window.
        if self.get_clock().now().nanoseconds == 0:
            self.get_logger().warning(
                'Waiting for the first /clock sample before validating scans',
                throttle_duration_sec=5.0,
            )
            return

        age_sec = self._scan_age_sec(scan)
        self._statistics.record_age(age_sec)
        if age_sec > self._max_scan_age_sec:
            self._reject_age(age_sec)
            return
        if not self._publish_if_transform_available(scan):
            self._pending_scans.append((scan, self.get_clock().now()))

    def _retry_pending_scans(self) -> None:
        if not self._pending_scans:
            return

        pending_scans = self._pending_scans
        self._pending_scans = deque()
        now = self.get_clock().now()
        for scan, queued_at in pending_scans:
            age_sec = self._scan_age_sec(scan)
            waited_sec = (now.nanoseconds - queued_at.nanoseconds) / 1e9
            if age_sec > self._max_scan_age_sec:
                self._reject_age(age_sec)
            elif waited_sec > self._transform_timeout.nanoseconds / 1e9:
                self._statistics.rejected_transform += 1
                self.get_logger().warning(
                    f'Dropped scan after waiting {waited_sec:.3f} s for '
                    f'{self._target_frame} -> {self._source_frame} at its '
                    'timestamp',
                    throttle_duration_sec=5.0,
                )
            elif not self._publish_if_transform_available(scan):
                self._pending_scans.append((scan, queued_at))

    def _reject_age(self, age_sec: float) -> None:
        self._statistics.rejected_age += 1
        self.get_logger().warning(
            f'Dropped scan {age_sec:.3f} s old; maximum is '
            f'{self._max_scan_age_sec:.3f} s',
            throttle_duration_sec=5.0,
        )

    def _publish_if_transform_available(self, scan: LaserScan) -> bool:
        scan_time = self._scan_time(scan)
        try:
            # This is deliberately the scan timestamp, not Time(). A latest
            # lookup would rotate the scan by yaw accumulated in transport.
            self._tf_buffer.lookup_transform(
                self._target_frame, self._source_frame, scan_time
            )
            latest = self._tf_buffer.lookup_transform(
                self._target_frame, self._source_frame, Time()
            )
        except TransformException:
            return False

        latest_time = Time.from_msg(latest.header.stamp)
        latest_offset_sec = (
            latest_time.nanoseconds - scan_time.nanoseconds
        ) / 1e9
        self._statistics.record_latest_offset(latest_offset_sec)
        self._statistics.accepted += 1
        self._publisher.publish(scan)
        return True

    def _report_statistics(self) -> None:
        statistics = self._statistics
        total = (
            statistics.accepted
            + statistics.rejected_age
            + statistics.rejected_transform
        )
        if total == 0:
            return

        latest_offset = 'n/a'
        if statistics.accepted:
            latest_offset = (
                f'{statistics.minimum_latest_offset_sec * 1e3:.1f}..'
                f'{statistics.maximum_latest_offset_sec * 1e3:.1f} ms'
            )
        self.get_logger().info(
            f'scan timing: accepted={statistics.accepted} '
            f'rejected_age={statistics.rejected_age} '
            f'rejected_tf={statistics.rejected_transform} receipt_age='
            f'{statistics.minimum_age_sec * 1e3:.1f}..'
            f'{statistics.maximum_age_sec * 1e3:.1f} ms '
            f'latest_tf_minus_scan={latest_offset}'
        )
        self._statistics = GateStatistics()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanTfGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
