"""
SLAM launch: bringup + slam_toolbox in online async mode.
Produces a live map while driving.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('umma_slam')
    bringup_launch = os.path.join(pkg_share, 'launch', 'bringup.launch.py')
    slam_config = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')

    # ── Arguments ──
    use_mock_arg = DeclareLaunchArgument(
        'use_mock', default_value='false',
        description='Use mock CAN interface for testing'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2 for visualization (false for headless)'
    )

    # ── Bringup (motor + lidar + odom + robot description) ──
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments={'use_mock': LaunchConfiguration('use_mock')}.items(),
    )

    # ── SLAM Toolbox (online async) ──
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_config],
        output='screen',
    )

    # ── Emergency stop (passthrough + watchdog) ──
    # teleop 은 별도 터미널에서 /cmd_vel_raw 로 publish:
    #   ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    #     --ros-args -r cmd_vel:=cmd_vel_raw
    estop_node = Node(
        package='umma_slam',
        executable='emergency_stop',
        name='emergency_stop_node',
        parameters=[{
            'watchdog_timeout': 1.0,   # 1초 입력 없으면 자동 정지
            'zero_publish_rate': 10.0,
        }],
        output='screen',
    )

    # ── RViz2 (선택적 시각화 — 헤드리스 운용 시 rviz:=false) ──
    rviz_config = os.path.join(pkg_share, 'config', 'slam.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # ── Ctrl+C → 모든 노드 종료 전에 모터 정지 보장 ──
    # OnShutdown 은 Ctrl+C(SIGINT) 수신 직후, 자식 프로세스 SIGINT 전파 전에 실행됨.
    # 1) /estop/activate  → emergency_stop 노드가 /cmd_vel 에 zero 퍼블리시
    # 2) /stop            → 모터 컨트롤러가 직접 드라이버 정지
    shutdown_handler = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call',
                        '/estop/activate',
                        'std_srvs/srv/Trigger', '{}',
                    ],
                    output='screen',
                ),
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call',
                        '/stop',
                        'std_srvs/srv/Trigger', '{}',
                    ],
                    output='screen',
                ),
            ]
        )
    )

    return LaunchDescription([
        use_mock_arg,
        rviz_arg,
        bringup,
        estop_node,
        slam_toolbox_node,
        rviz_node,
        shutdown_handler,
    ])
