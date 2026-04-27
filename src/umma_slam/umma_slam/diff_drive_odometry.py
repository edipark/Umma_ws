"""
Differential-drive odometry node.

Subscribes to /joint_states (published by the ZLAC8015D motor controller),
computes the pose of the robot's **geometric center** (base_footprint),
and publishes nav_msgs/Odometry + the TF odom -> base_footprint.

좌표계 규약 (중요):
    base_footprint : 로봇 기하학적 중심(지면 투영점). 라이다 바로 아래.
    axle midpoint  : 좌/우 구동바퀴 축의 중점.
    wheel_offset_x : base_footprint -> axle midpoint 의 x(전방) 거리.
                     바퀴가 앞쪽에 쏠려 있으면 양수.

이 노드는 내부적으로 **axle midpoint 의 pose** 를 먼저 적분한 뒤,
wheel_offset_x 만큼 뒤로 평행이동(body frame)하여 base_footprint pose 를
계산해 publish 합니다. 이렇게 하면 SLAM (라이다 = base_footprint 위)에서
회전 시에 odom 기준 laser 위치가 정확히 보정됩니다.
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


def normalize_angle(a: float) -> float:
    """Wrap angle into [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class DiffDriveOdometryNode(Node):
    """Differential-drive odometry from wheel encoders.

    Integrates the axle-midpoint pose using a mid-point rule, then shifts
    by -wheel_offset_x in body frame to get the base_footprint pose.
    """

    def __init__(self):
        super().__init__('diff_drive_odometry_node')

        # ---- Parameters (전부 robot_params.yaml 에서 주입하는 것을 권장) ----
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('wheel_base', 0.5)
        # base_footprint -> axle midpoint 의 x(전방) 오프셋.
        # 바퀴가 앞쪽에 쏠려 있으면 양수. 0 이면 종래 동작과 동일.
        self.declare_parameter('wheel_offset_x', 0.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.wheel_offset_x = float(self.get_parameter('wheel_offset_x').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        if self.wheel_base <= 0.0 or self.wheel_radius <= 0.0:
            self.get_logger().error(
                f'Invalid wheel_base={self.wheel_base} or '
                f'wheel_radius={self.wheel_radius}. Odometry will be disabled.'
            )

        # ---- Internal state: pose of the AXLE MIDPOINT ----
        self.axle_x = 0.0
        self.axle_y = 0.0
        self.axle_theta = 0.0

        # Previous encoder angles [rad]
        self.prev_left_pos = None
        self.prev_right_pos = None
        self.prev_time = None

        # ---- QoS / pub / sub ----
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.joint_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, qos
        )
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            f'Odometry node started | wheel_radius={self.wheel_radius}, '
            f'wheel_base={self.wheel_base}, wheel_offset_x={self.wheel_offset_x}'
        )

    # ------------------------------------------------------------------

    def joint_state_callback(self, msg: JointState):
        """Process joint states and compute odometry."""
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

        # First sample — just store and wait for the next one
        if self.prev_left_pos is None:
            self.prev_left_pos = left_pos
            self.prev_right_pos = right_pos
            self.prev_time = current_time
            return

        dl = left_pos - self.prev_left_pos
        dr = right_pos - self.prev_right_pos
        dt = (current_time - self.prev_time).nanoseconds * 1e-9

        # store for next iteration regardless of dt
        self.prev_left_pos = left_pos
        self.prev_right_pos = right_pos
        self.prev_time = current_time

        if dt <= 0.0:
            return

        # Arc lengths travelled by each wheel [m]
        left_dist = dl * self.wheel_radius
        right_dist = dr * self.wheel_radius

        # Differential-drive kinematics (axle-midpoint displacement in body frame)
        linear_dist = 0.5 * (right_dist + left_dist)
        angular_dist = (right_dist - left_dist) / self.wheel_base

        # ---- Mid-point (2nd-order) integration of axle-midpoint pose ----
        # Works seamlessly for both straight and arc motion; no branching,
        # no division by angular_dist, no numerical blow-up.
        theta_mid = self.axle_theta + 0.5 * angular_dist
        self.axle_x += linear_dist * math.cos(theta_mid)
        self.axle_y += linear_dist * math.sin(theta_mid)
        self.axle_theta = normalize_angle(self.axle_theta + angular_dist)

        # ---- Transform axle midpoint -> base_footprint ----
        # base_footprint is a fixed point on the rigid body, located at
        #   (-wheel_offset_x, 0) relative to the axle midpoint in body frame.
        cos_t = math.cos(self.axle_theta)
        sin_t = math.sin(self.axle_theta)
        base_x = self.axle_x - self.wheel_offset_x * cos_t
        base_y = self.axle_y - self.wheel_offset_x * sin_t
        base_theta = self.axle_theta  # 자세는 동일 (강체)

        # ---- Body-frame twist AT base_footprint (child_frame) ----
        # v_axle_body = (v_fwd, 0), omega = omega
        # v_base_body = v_axle_body + omega x r  (r = base - axle = (-offset, 0))
        #             = (v_fwd, -wheel_offset_x * omega)
        vx_fwd = linear_dist / dt
        vth = angular_dist / dt
        vx_body = vx_fwd
        vy_body = -self.wheel_offset_x * vth

        # ---- Publish Odometry ----
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = base_x
        odom_msg.pose.pose.position.y = base_y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = quaternion_from_yaw(base_theta)

        # Covariance (x, y, z, roll, pitch, yaw)
        odom_msg.pose.covariance[0] = 0.01   # x
        odom_msg.pose.covariance[7] = 0.01   # y
        odom_msg.pose.covariance[35] = 0.03  # yaw

        odom_msg.twist.twist.linear.x = vx_body
        odom_msg.twist.twist.linear.y = vy_body
        odom_msg.twist.twist.angular.z = vth

        odom_msg.twist.covariance[0] = 0.01
        odom_msg.twist.covariance[7] = 0.01
        odom_msg.twist.covariance[35] = 0.03

        self.odom_pub.publish(odom_msg)

        # ---- Publish TF: odom -> base_footprint ----
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = base_x
            t.transform.translation.y = base_y
            t.transform.translation.z = 0.0
            t.transform.rotation = quaternion_from_yaw(base_theta)
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
