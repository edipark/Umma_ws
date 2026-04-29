"""
Visualization-only launch (remote PC): RViz2 with nav2 goal panel.

로봇 측에서 navigation.launch.py 가 이미 실행 중이어야 합니다.
이 런치는 별도 PC에서 RViz2만 띄워 /map, /tf, /scan 등을 구독하고
2D Nav Goal / Initial Pose 설정 기능을 사용할 수 있게 합니다.

사전 조건
---------
  export ROS_DOMAIN_ID=<로봇과 동일한 값>
  # 다른 서브넷이면 DDS peer 설정도 필요 (Cyclone: CYCLONEDDS_URI 등)

Usage
-----
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
    default_rviz_config = os.path.join(pkg_share, 'config', 'navigation.rviz')

    # ── Arguments ──
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path to the RViz2 config file (default: navigation.rviz)',
    )

    # ── RViz2 ──
    # /map, /tf, /tf_static, /scan, /robot_description 등은 로봇 측에서
    # 네트워크를 통해 수신됩니다.
    # Nav2 Goal / Initial Pose 패널은 /navigate_to_pose, /initialpose 토픽을
    # 로봇 측 nav2로 직접 전달합니다 — 별도 노드 불필요.
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
