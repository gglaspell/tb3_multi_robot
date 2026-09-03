#!/usr/bin/env python3

"""Publish yaw-only odometry and TF for the 2D TurtleBot mapping profile."""

from nav_msgs.msg import Odometry

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster

from multi_robot_scripts.utils import planar_quaternion


class PlanarOdom(Node):
    """Flatten Gazebo's contact-induced roll/pitch before Nav2 and SLAM use it."""

    def __init__(self) -> None:
        super().__init__('planar_odom')

        self.declare_parameter('input_topic', 'ground_truth_odom')
        self.declare_parameter('odom_topic', 'odom')

        input_topic = self.get_parameter('input_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self._publisher = self.create_publisher(Odometry, odom_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._subscription = self.create_subscription(
            Odometry,
            input_topic,
            self._odometry_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'Flattening {input_topic} into yaw-only {odom_topic} and TF'
        )

    def _odometry_callback(self, source: Odometry) -> None:
        if not source.header.frame_id or not source.child_frame_id:
            self.get_logger().warning(
                'Ignoring odometry with an empty parent or child frame',
                throttle_duration_sec=5.0,
            )
            return

        source_orientation = source.pose.pose.orientation
        qx, qy, qz, qw = planar_quaternion(
            source_orientation.x,
            source_orientation.y,
            source_orientation.z,
            source_orientation.w,
        )

        odometry = Odometry()
        odometry.header = source.header
        odometry.child_frame_id = source.child_frame_id
        odometry.pose.pose.position.x = source.pose.pose.position.x
        odometry.pose.pose.position.y = source.pose.pose.position.y
        odometry.pose.pose.position.z = 0.0
        odometry.pose.pose.orientation.x = qx
        odometry.pose.pose.orientation.y = qy
        odometry.pose.pose.orientation.z = qz
        odometry.pose.pose.orientation.w = qw
        odometry.pose.covariance = source.pose.covariance
        odometry.twist = source.twist
        self._publisher.publish(odometry)

        transform = TransformStamped()
        transform.header = source.header
        transform.child_frame_id = source.child_frame_id
        transform.transform.translation.x = source.pose.pose.position.x
        transform.transform.translation.y = source.pose.pose.position.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlanarOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
