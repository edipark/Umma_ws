"""
Visualization-only launch: RViz2 subscriber for a remote hardware node.

Run this on a separate PC that shares the same ROS_DOMAIN_ID (and, for
DDS multicast, the same subnet) as the robot.  The robot side should
already be publishing /scan, /map, /tf, /tf_static, and /robot_description.

Usage
-----
  # Ensure ROS_DOMAIN_ID matches the robot:
  export ROS_DOMAIN_ID=<same as robot>

  # If using Cyclone or FastDDS with unicast, also set RMW_IMPLEMENTATION and
  # a peer XML pointing at the robot IP.

  ros2 launch umma_slam viz.launch.py
  ros2 launch umma_slam viz.launch.py rviz_config:=/path/to/custom.rviz
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('umma_slam')
    default_rviz_config = os.path.join(pkg_share, 'config', 'slam.rviz')

    # ── Arguments ──
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path to the RViz2 config file',
    )

    # ── RViz2 ──
    # All data (tf, scan, map, robot_description) is received over the
    # network from the robot; no local nodes are required.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen',
    )

    return LaunchDescription([
        rviz_config_arg,
        rviz_node,
    ])
