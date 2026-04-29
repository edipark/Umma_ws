#!/usr/bin/env python3

import math
import os
from typing import Dict

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


class TargetGoalDispatcher(Node):
    """Dispatch named targets from YAML to nav2 NavigateToPose action."""

    def __init__(self):
        super().__init__('target_goal_dispatcher')

        self.declare_parameter('targets_file', '')
        self.declare_parameter('target_name', '')
        self.declare_parameter('default_frame_id', 'map')
        self.declare_parameter('input_topic', '/navigation_target')

        self._targets_file = self.get_parameter('targets_file').value
        self._target_name = self.get_parameter('target_name').value
        self._default_frame_id = self.get_parameter('default_frame_id').value
        self._input_topic = self.get_parameter('input_topic').value

        self._frame_id = self._default_frame_id
        self._targets: Dict[str, Dict] = {}
        self._active_goal = False

        self._load_targets(self._targets_file)

        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.create_subscription(String, self._input_topic, self._target_callback, 10)

        if self._target_name:
            self._startup_timer = self.create_timer(1.0, self._dispatch_initial_target)

        self.get_logger().info(
            f"Target dispatcher ready | topic={self._input_topic} | targets={len(self._targets)}"
        )

    def _load_targets(self, path: str):
        if not path:
            self.get_logger().warn('targets_file is empty; no named targets loaded.')
            return

        if not os.path.isfile(path):
            self.get_logger().error(f'targets_file not found: {path}')
            return

        try:
            with open(path, 'r', encoding='utf-8') as file:
                data = yaml.safe_load(file) or {}
        except Exception as exc:
            self.get_logger().error(f'Failed to read targets file: {exc}')
            return

        self._frame_id = data.get('frame_id', self._default_frame_id)
        self._targets = data.get('targets', {})
        if not isinstance(self._targets, dict):
            self.get_logger().error("Invalid targets format: expected mapping under key 'targets'")
            self._targets = {}

    def _dispatch_initial_target(self):
        self._startup_timer.cancel()
        self._send_target_by_name(self._target_name)

    def _target_callback(self, msg: String):
        name = msg.data.strip()
        if not name:
            self.get_logger().warn('Received empty target name; ignoring.')
            return
        self._send_target_by_name(name)

    def _send_target_by_name(self, name: str):
        if self._active_goal:
            self.get_logger().warn(
                f"Goal already active; ignoring new target '{name}'."
            )
            return

        target = self._targets.get(name)
        if target is None:
            known = ', '.join(sorted(self._targets.keys()))
            self.get_logger().error(f"Unknown target '{name}'. Known targets: [{known}]")
            return

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_pose action server is not available.')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._to_pose_stamped(target)
        self._active_goal = True

        send_future = self._nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_callback)
        self.get_logger().info(f"Sent target '{name}' to nav2.")

    def _to_pose_stamped(self, target: Dict) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = target.get('frame_id', self._frame_id)
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(target['x'])
        pose.pose.position.y = float(target['y'])
        pose.pose.position.z = 0.0

        yaw = float(target.get('yaw', 0.0))
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        return pose

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._active_goal = False
            self.get_logger().error('nav2 rejected the goal.')
            return

        self.get_logger().info('nav2 accepted the goal.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        self._active_goal = False
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Target reached successfully.')
        else:
            self.get_logger().warn(f'Navigation finished with status code: {status}')


def main(args=None):
    rclpy.init(args=args)
    node = TargetGoalDispatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
