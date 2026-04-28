"""
Simple cmd_vel relay for UMMA robot.

Architecture:
  teleop / nav2 ──► /cmd_vel_raw ──► [this node] ──► /cmd_vel ──► motor controller

Only Ctrl+C shutdown handling remains:
  - pass /cmd_vel_raw through to /cmd_vel
  - publish zero velocity once during shutdown
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist


class EmergencyStopNode(Node):
    """Relay /cmd_vel_raw to /cmd_vel and stop once on shutdown."""

    def __init__(self):
        super().__init__('emergency_stop_node')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._cmd_sub = self.create_subscription(
            Twist, 'cmd_vel_raw', self._cmd_vel_raw_callback, qos
        )
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        self.get_logger().info(
            'Emergency stop node ready | Listen on /cmd_vel_raw -> publish to /cmd_vel'
        )

    def _cmd_vel_raw_callback(self, msg: Twist):
        """Pass through velocity commands unchanged."""
        self._cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
