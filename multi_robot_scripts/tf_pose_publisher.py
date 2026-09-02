#!/usr/bin/env python3

"""Publish a TF-derived pose for consumers that require a pose topic."""

from geometry_msgs.msg import PoseStamped

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros import Buffer, TransformException, TransformListener


class TfPosePublisher(Node):
    """Sample a transform and publish it as a PoseStamped message."""

    def __init__(self) -> None:
        super().__init__('tf_pose_publisher')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('source_frame', 'base_link')
        self.declare_parameter('pose_topic', 'map_pose')
        self.declare_parameter('publish_rate', 5.0)

        self._target_frame = self.get_parameter('target_frame').value
        self._source_frame = self.get_parameter('source_frame').value
        pose_topic = self.get_parameter('pose_topic').value
        publish_rate = float(self.get_parameter('publish_rate').value)
        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be positive')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=False
        )
        self._publisher = self.create_publisher(PoseStamped, pose_topic, 10)
        self._timer = self.create_timer(1.0 / publish_rate, self._publish_pose)

    def _publish_pose(self) -> None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame, self._source_frame, Time()
            )
        except TransformException as exc:
            self.get_logger().warning(
                f'Waiting for {self._target_frame} -> '
                f'{self._source_frame}: {exc}',
                throttle_duration_sec=5.0,
            )
            return

        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        self._publisher.publish(pose)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TfPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
