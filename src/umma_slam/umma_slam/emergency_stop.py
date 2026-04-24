"""
Emergency stop node for UMMA robot.

Architecture (passthrough):
  teleop ──► /cmd_vel_raw ──► [this node] ──► /cmd_vel ──► motor controller

Features:
  - Passthrough with optional blocking when E-stop is active
  - Watchdog (deadman switch): auto-stop if no /cmd_vel_raw within `watchdog_timeout` seconds
  - Service /estop/activate  (std_srvs/Trigger) — engage E-stop
  - Service /estop/release   (std_srvs/Trigger) — release E-stop
  - Topic   /estop/state     (std_msgs/Bool)     — current E-stop state (true = stopped)

Usage:
  Run teleop in a separate terminal, remapped to /cmd_vel_raw:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard \\
      --ros-args -r cmd_vel:=cmd_vel_raw

  Activate E-stop:
    ros2 service call /estop/activate std_srvs/srv/Trigger '{}'

  Release E-stop:
    ros2 service call /estop/release std_srvs/srv/Trigger '{}'
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import threading


class EmergencyStopNode(Node):
    """Command velocity passthrough with watchdog and emergency stop."""

    def __init__(self):
        super().__init__('emergency_stop_node')

        self.declare_parameter('watchdog_timeout', 1.0)   # seconds
        self.declare_parameter('zero_publish_rate', 10.0) # Hz (while stopped)

        self._watchdog_timeout = self.get_parameter('watchdog_timeout').value
        self._zero_rate = self.get_parameter('zero_publish_rate').value

        self._lock = threading.Lock()
        self._estop_active = False
        self._last_cmd_time = None  # None means never received

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscriber: raw velocity from teleop
        self._cmd_sub = self.create_subscription(
            Twist, 'cmd_vel_raw', self._cmd_vel_raw_callback, qos
        )

        # Publisher: gated velocity to motor controller
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        # Publisher: E-stop state
        self._state_pub = self.create_publisher(Bool, 'estop/state', 10)

        # Services
        self.create_service(Trigger, 'estop/activate', self._activate_callback)
        self.create_service(Trigger, 'estop/release', self._release_callback)

        # Watchdog + zero-publish timer (runs at zero_publish_rate)
        period = 1.0 / self._zero_rate
        self._timer = self.create_timer(period, self._timer_callback)

        self.get_logger().info(
            f'Emergency stop node ready | watchdog={self._watchdog_timeout}s | '
            f'Listen on /cmd_vel_raw → publish to /cmd_vel'
        )
        self.get_logger().warn(
            'E-stop ACTIVE until first /cmd_vel_raw is received.'
        )

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _cmd_vel_raw_callback(self, msg: Twist):
        """Pass through to /cmd_vel only when E-stop is not active."""
        now = self.get_clock().now()
        with self._lock:
            self._last_cmd_time = now
            if self._estop_active:
                # Silently drop — keep publishing zeros via timer
                return
        self._cmd_pub.publish(msg)

    def _timer_callback(self):
        """Watchdog check + publish zero when stopped."""
        now = self.get_clock().now()
        with self._lock:
            # Watchdog: trigger E-stop if no cmd_vel_raw received in time
            if self._last_cmd_time is not None and not self._estop_active:
                elapsed = (now - self._last_cmd_time).nanoseconds * 1e-9
                if elapsed > self._watchdog_timeout:
                    self._estop_active = True
                    self.get_logger().warn(
                        f'Watchdog triggered: no /cmd_vel_raw for {elapsed:.1f}s → E-stop ACTIVE'
                    )

            stopped = self._estop_active

        # Publish zero if stopped
        if stopped:
            self._cmd_pub.publish(Twist())

        # Publish state
        state_msg = Bool()
        state_msg.data = stopped
        self._state_pub.publish(state_msg)

    def _activate_callback(self, request, response):
        with self._lock:
            self._estop_active = True
        self._cmd_pub.publish(Twist())  # Immediate zero
        self.get_logger().warn('E-stop ACTIVATED via service')
        response.success = True
        response.message = 'Emergency stop activated'
        return response

    def _release_callback(self, request, response):
        with self._lock:
            if self._last_cmd_time is None:
                response.success = False
                response.message = 'Cannot release: no teleop input detected yet'
                self.get_logger().warn(response.message)
                return response
            self._estop_active = False
            self._last_cmd_time = self.get_clock().now()  # Reset watchdog clock
        self.get_logger().info('E-stop RELEASED via service')
        response.success = True
        response.message = 'Emergency stop released'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure motors are zeroed on shutdown
        node._cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
