"""
Differential drive odometry node.
Subscribes to /joint_states from ZLAC8015D motor controller,
computes odometry and publishes nav_msgs/Odometry + TF (odom -> base_footprint).
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster


def quaternion_from_yaw(yaw: float) -> Quaternion:
    """Create a Quaternion message from a yaw angle."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class DiffDriveOdometryNode(Node):
    """Differential drive odometry from wheel encoders."""

    def __init__(self):
        super().__init__('diff_drive_odometry_node')

        # Parameters (must match motor controller config)
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('wheel_base', 0.5)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Previous encoder positions (radians)
        self.prev_left_pos = None
        self.prev_right_pos = None
        self.prev_time = None

        # QoS
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscriber
        self.joint_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, qos
        )

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            f'Odometry node started (wheel_radius={self.wheel_radius}, '
            f'wheel_base={self.wheel_base})'
        )

    def joint_state_callback(self, msg: JointState):
        """Process joint states and compute odometry."""
        # Find left / right wheel indices
        try:
            left_idx = msg.name.index('left_wheel_joint')
            right_idx = msg.name.index('right_wheel_joint')
        except ValueError:
            return

        if len(msg.position) <= max(left_idx, right_idx):
            return

        current_time = self.get_clock().now()
        left_pos = msg.position[left_idx]
        right_pos = msg.position[right_idx]

        if self.prev_left_pos is None:
            # First message — just store
            self.prev_left_pos = left_pos
            self.prev_right_pos = right_pos
            self.prev_time = current_time
            return

        # Delta angles (radians)
        dl = left_pos - self.prev_left_pos
        dr = right_pos - self.prev_right_pos
        dt = (current_time - self.prev_time).nanoseconds * 1e-9

        if dt <= 0.0:
            return

        # Arc distances
        left_dist = dl * self.wheel_radius
        right_dist = dr * self.wheel_radius

        # Differential drive kinematics
        linear_dist = (right_dist + left_dist) / 2.0
        angular_dist = (right_dist - left_dist) / self.wheel_base

        # Update pose
        if abs(angular_dist) < 1e-6:
            # Straight line
            self.x += linear_dist * math.cos(self.theta)
            self.y += linear_dist * math.sin(self.theta)
        else:
            # Arc motion
            radius = linear_dist / angular_dist
            self.x += radius * (math.sin(self.theta + angular_dist) - math.sin(self.theta))
            self.y -= radius * (math.cos(self.theta + angular_dist) - math.cos(self.theta))

        self.theta += angular_dist
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Velocities
        vx = linear_dist / dt
        vth = angular_dist / dt

        # Store for next iteration
        self.prev_left_pos = left_pos
        self.prev_right_pos = right_pos
        self.prev_time = current_time

        # Build and publish Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = quaternion_from_yaw(self.theta)

        # Covariance (x, y, z, roll, pitch, yaw)
        odom_msg.pose.covariance[0] = 0.01   # x
        odom_msg.pose.covariance[7] = 0.01   # y
        odom_msg.pose.covariance[35] = 0.03  # yaw

        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.angular.z = vth

        odom_msg.twist.covariance[0] = 0.01
        odom_msg.twist.covariance[35] = 0.03

        self.odom_pub.publish(odom_msg)

        # Publish TF: odom -> base_footprint
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = quaternion_from_yaw(self.theta)
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
