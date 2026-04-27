"""
ZLAC8015D ROS2 control node.
Provides position, velocity, and torque control interfaces.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, Bool, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, SetBool
import time
from typing import Optional

from .zlac8015d_driver import ZLAC8015DDriver, DriverState, OperationMode
from .canopen_interface import CANopenInterface, MockCANopenInterface, SocketCANopenInterface


class ZLAC8015DControlNode(Node):
    """ZLAC8015D ROS2 control node."""
    
    def __init__(self):
        super().__init__('zlac8015d_control_node')
        
        # Declare parameters
        self.declare_parameter('node_id', 1)
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('use_mock', False)  # Use mock interface for testing
        self.declare_parameter('publish_rate', 50.0)  # State publish rate (Hz)
        self.declare_parameter('encoder_resolution', 1024)  # Encoder resolution
        # wheel_radius / wheel_base 는 실제 값으로 robot_params.yaml 에서 주입됨.
        # 아래 값은 robot_params.yaml 이 로드되지 않을 때의 안전 기본값.
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('wheel_base', 0.5)
        self.declare_parameter('reverse_left_motor', False)  # Reverse left motor direction
        self.declare_parameter('reverse_right_motor', True)  # Reverse right motor direction
        
        # Read parameters
        node_id = self.get_parameter('node_id').value
        can_interface = self.get_parameter('can_interface').value
        use_mock = self.get_parameter('use_mock').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.encoder_resolution = self.get_parameter('encoder_resolution').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.reverse_left_motor = self.get_parameter('reverse_left_motor').value
        self.reverse_right_motor = self.get_parameter('reverse_right_motor').value
        
        # Create CANopen interface
        if use_mock:
            self.get_logger().info('Using mock CANopen interface')
            can_interface_obj = MockCANopenInterface(node_id, can_interface)
        else:
            # Use real CANopen interface
            self.get_logger().info(f'Using real CANopen interface: {can_interface}')
            can_interface_obj = SocketCANopenInterface(node_id, can_interface)
            # Connect to CAN bus
            if not can_interface_obj.connect():
                self.get_logger().error(f'Failed to connect to CAN interface {can_interface}')
                raise RuntimeError(f'CAN interface connection failed: {can_interface}')
        
        # Create driver instance
        self.driver = ZLAC8015DDriver(can_interface_obj)
        
        # QoS configuration
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        # Create subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            qos_profile
        )
        
        self.position_cmd_sub = self.create_subscription(
            JointState,
            'joint_position_command',
            self.position_cmd_callback,
            10
        )
        
        self.velocity_cmd_sub = self.create_subscription(
            JointState,
            'joint_velocity_command',
            self.velocity_cmd_callback,
            10
        )
        
        # Create publishers
        self.joint_state_pub = self.create_publisher(
            JointState,
            'joint_states',
            qos_profile
        )
        
        self.status_pub = self.create_publisher(
            Int32,
            'driver_status',
            qos_profile
        )
        
        self.fault_pub = self.create_publisher(
            Int32,
            'fault_code',
            qos_profile
        )
        
        # Create services
        self.init_srv = self.create_service(
            Trigger,
            'initialize',
            self.initialize_service
        )
        
        self.stop_srv = self.create_service(
            Trigger,
            'stop',
            self.stop_service
        )
        
        self.clear_fault_srv = self.create_service(
            Trigger,
            'clear_fault',
            self.clear_fault_service
        )
        
        self.enable_srv = self.create_service(
            SetBool,
            'enable',
            self.enable_service
        )
        
        # State variables
        self.is_initialized = False
        self.is_enabled = False
        self.current_mode = OperationMode.NO_MODE
        
        # Create timer
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'ZLAC8015D control node started (node_id: {node_id}, CAN interface: {can_interface})')
    
    def timer_callback(self):
        """Timer callback: publish status information."""
        if not self.is_initialized:
            return
        
        # Update driver state
        self.driver.update_state()
        
        # Publish joint states
        self.publish_joint_state()
        
        # Publish driver state
        status_msg = Int32()
        status_msg.data = int(self.driver.current_state)
        self.status_pub.publish(status_msg)
        
        # Publish fault code
        if self.driver.fault_code != 0:
            fault_msg = Int32()
            fault_msg.data = self.driver.fault_code
            self.fault_pub.publish(fault_msg)
    
    def publish_joint_state(self):
        """Publish joint states."""
        # Read position and velocity (can be None if CAN read failed)
        left_pos, right_pos = self.driver.get_actual_position()
        left_vel, right_vel = self.driver.get_actual_velocity()

        # None 체크를 부호 반전보다 먼저 수행해야 함 (None에 unary minus 적용 시 TypeError)
        if left_pos is None or right_pos is None:
            return

        if self.reverse_left_motor:
            left_pos = -left_pos
            if left_vel is not None:
                left_vel = -left_vel
        if self.reverse_right_motor:
            right_pos = -right_pos
            if right_vel is not None:
                right_vel = -right_vel

        # Convert to angle (assuming encoder feedback is in counts)
        # Adjust based on actual encoder configuration
        left_angle = (left_pos / self.encoder_resolution) * 2.0 * math.pi
        right_angle = (right_pos / self.encoder_resolution) * 2.0 * math.pi

        # Convert to angular velocity (rad/s)
        left_velocity = (left_vel / 10.0 / 60.0) * 2.0 * math.pi if left_vel is not None else 0.0
        right_velocity = (right_vel / 10.0 / 60.0) * 2.0 * math.pi if right_vel is not None else 0.0
        
        # Build JointState message
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [left_angle, right_angle]
        joint_state.velocity = [left_velocity, right_velocity]
        joint_state.effort = [0.0, 0.0]  # Torque can be added if available
        
        self.joint_state_pub.publish(joint_state)
    
    def cmd_vel_callback(self, msg: Twist):
        """Velocity command callback (differential drive mode)."""
        if not self.is_initialized or not self.is_enabled:
            return
        
        # Differential drive model: convert linear/angular velocity to wheel velocity
        linear = msg.linear.x
        angular = msg.angular.z
        
        left_velocity = (linear - angular * self.wheel_base / 2.0) / self.wheel_radius
        right_velocity = (linear + angular * self.wheel_base / 2.0) / self.wheel_radius
        
        # Convert to r/min
        left_rpm = left_velocity * 60.0 / (2.0 * math.pi)
        right_rpm = right_velocity * 60.0 / (2.0 * math.pi)
        
        # Apply motor direction inversion
        if self.reverse_left_motor:
            left_rpm = -left_rpm
        if self.reverse_right_motor:
            right_rpm = -right_rpm
        
        # Limit velocity range
        left_rpm = max(-1000, min(1000, int(left_rpm)))
        right_rpm = max(-1000, min(1000, int(right_rpm)))
        
        # Switch to velocity mode if needed
        if self.current_mode != OperationMode.PROFILE_VELOCITY:
            if self.driver.enable_velocity_mode(sync_control=False):
                self.current_mode = OperationMode.PROFILE_VELOCITY
                self.get_logger().info('Switched to velocity mode')
        
        # Set target velocity
        self.driver.set_target_velocity(left_rpm, right_rpm, sync=False)
    
    def position_cmd_callback(self, msg: JointState):
        """Position command callback."""
        if not self.is_initialized or not self.is_enabled:
            return
        
        if len(msg.position) < 2:
            self.get_logger().warn('Position command requires at least 2 joints')
            return
        
        # Convert to counts (adjust based on encoder configuration)
        left_pos = int(msg.position[0] * self.encoder_resolution / (2.0 * math.pi))
        right_pos = int(msg.position[1] * self.encoder_resolution / (2.0 * math.pi))
        
        # Switch to position mode
        if self.current_mode != OperationMode.PROFILE_POSITION:
            if self.driver.enable_position_mode():
                self.current_mode = OperationMode.PROFILE_POSITION
                self.get_logger().info('Switched to position mode')
        
        # Set target position (absolute by default)
        absolute = True
        if len(msg.name) > 0 and 'relative' in str(msg.name).lower():
            absolute = False
        
        self.driver.set_target_position(left_pos, right_pos, absolute=absolute)
    
    def velocity_cmd_callback(self, msg: JointState):
        """Velocity command callback (direct wheel joint velocity)."""
        if not self.is_initialized or not self.is_enabled:
            return
        
        if len(msg.velocity) < 2:
            self.get_logger().warn('Velocity command requires at least 2 joints')
            return
        
        # Convert to r/min
        left_rpm = int(msg.velocity[0] * 60.0 / (2.0 * math.pi))
        right_rpm = int(msg.velocity[1] * 60.0 / (2.0 * math.pi))
        
        # Apply motor direction inversion
        if self.reverse_left_motor:
            left_rpm = -left_rpm
        if self.reverse_right_motor:
            right_rpm = -right_rpm
        
        # Limit velocity range
        left_rpm = max(-1000, min(1000, left_rpm))
        right_rpm = max(-1000, min(1000, right_rpm))
        
        # Switch to velocity mode
        if self.current_mode != OperationMode.PROFILE_VELOCITY:
            if self.driver.enable_velocity_mode(sync_control=False):
                self.current_mode = OperationMode.PROFILE_VELOCITY
                self.get_logger().info('Switched to velocity mode')
        
        # Set target velocity
        self.driver.set_target_velocity(left_rpm, right_rpm, sync=False)
    
    def initialize_service(self, request, response):
        """Initialize service."""
        self.get_logger().info('Running driver initialization...')

        if self.driver.initialize():
            self.is_initialized = True
            response.success = True
            response.message = 'Driver initialization succeeded'
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = 'Driver initialization failed'
            self.get_logger().error(response.message)
            # 진단 가이드 — 실제 설정된 CAN 인터페이스 이름을 동적으로 출력
            ifname = getattr(self.driver.can, 'can_interface', 'can0')
            nid = getattr(self.driver.can, 'node_id', '?')
            self.get_logger().error(
                f'Probable causes:\n'
                f'  1) {ifname} is DOWN. Bring it up:\n'
                f'       sudo ip link set {ifname} type can bitrate 500000\n'
                f'       sudo ip link set up {ifname}\n'
                f'     Check state:   ip -details link show {ifname}   (should say "UP RUNNING")\n'
                f'     Sniff traffic: candump {ifname}                 (ZLAC heartbeat should appear)\n'
                f'  2) Wrong node_id (current={nid}). ZLAC8015D default is 1.\n'
                f'  3) Wrong bitrate. Driver uses 500 kbit/s; {ifname} must match.\n'
                f'  4) Power/wiring: 24 V supply, CAN_H/CAN_L, 120 Ω termination.\n'
                f'  5) To test SLAM stack without hardware, launch with use_mock:=true'
            )

        return response
    
    def stop_service(self, request, response):
        """Stop service."""
        self.get_logger().info('Stopping motors...')
        self.driver.stop()
        self.is_enabled = False
        response.success = True
        response.message = 'Motors stopped'
        return response
    
    def clear_fault_service(self, request, response):
        """Clear fault service."""
        self.get_logger().info('Clearing fault...')
        
        if self.driver.clear_fault():
            response.success = True
            response.message = 'Fault cleared'
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = 'Failed to clear fault'
            self.get_logger().error(response.message)
        
        return response
    
    def enable_service(self, request, response):
        """Enable/disable service."""
        if request.data:
            if not self.is_initialized:
                response.success = False
                response.message = 'Initialize the driver first'
                return response
            
            self.is_enabled = True
            response.success = True
            response.message = 'Driver enabled'
            self.get_logger().info(response.message)
        else:
            self.driver.stop()
            self.is_enabled = False
            response.success = True
            response.message = 'Driver disabled'
            self.get_logger().info(response.message)
        
        return response


def main(args=None):
    rclpy.init(args=args)
    
    node = ZLAC8015DControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.driver.stop()
        # Disconnect CAN
        if hasattr(node.driver, 'can') and hasattr(node.driver.can, 'disconnect'):
            node.driver.can.disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
