#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class InitialPoseSender(Node):
    """Publish /initialpose a few times for AMCL bootstrap."""

    def __init__(self):
        super().__init__('initial_pose_sender')

        self.declare_parameter('topic', '/initialpose')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('repeat_count', 10)
        self.declare_parameter('interval_sec', 0.5)
        self.declare_parameter('cov_xy', 0.25)
        self.declare_parameter('cov_yaw', 0.07)

        topic = self.get_parameter('topic').value
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.x = float(self.get_parameter('x').value)
        self.y = float(self.get_parameter('y').value)
        self.yaw = float(self.get_parameter('yaw').value)
        self.repeat_count = max(1, int(self.get_parameter('repeat_count').value))
        interval_sec = float(self.get_parameter('interval_sec').value)
        self.cov_xy = float(self.get_parameter('cov_xy').value)
        self.cov_yaw = float(self.get_parameter('cov_yaw').value)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, topic, qos)

        self.sent_count = 0
        self.timer = self.create_timer(max(0.1, interval_sec), self._publish_once)
        self.get_logger().info(
            f'Publishing initial pose to {topic} in frame {self.frame_id}: '
            f'x={self.x:.3f}, y={self.y:.3f}, yaw={self.yaw:.3f} rad'
        )

    def _build_msg(self) -> PoseWithCovarianceStamped:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(self.yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(self.yaw * 0.5)
        msg.pose.covariance[0] = self.cov_xy
        msg.pose.covariance[7] = self.cov_xy
        msg.pose.covariance[35] = self.cov_yaw
        return msg

    def _publish_once(self):
        self.publisher.publish(self._build_msg())
        self.sent_count += 1
        if self.sent_count >= self.repeat_count:
            self.get_logger().info('Initial pose bootstrap complete.')
            self.timer.cancel()
            self.destroy_node()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
