"""
Navigation launch: localization + nav2 + E-stop + hardware bringup.
Use this after building a map with slam.launch.py.

  ros2 launch umma_slam navigation.launch.py map_file:=/home/mingun/maps/my_map
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('umma_slam')
    bringup_launch = os.path.join(pkg_share, 'launch', 'bringup.launch.py')
    loc_config = os.path.join(pkg_share, 'config', 'localization.yaml')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'navigation.rviz')

    # ── Arguments ──
    use_mock_arg = DeclareLaunchArgument(
        'use_mock', default_value='false',
        description='Use mock CAN interface'
    )
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Full path to serialized map file (without extension)'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2 (false for headless)'
    )

    # ── Hardware bringup (motor + lidar + odom + URDF) ──
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments={'use_mock': LaunchConfiguration('use_mock')}.items(),
    )

    # ── Emergency stop (nav2 → /cmd_vel_raw → E-stop → /cmd_vel → motor) ──
    estop_node = Node(
        package='umma_slam',
        executable='emergency_stop',
        name='emergency_stop_node',
        output='screen',
    )

    # ── Localization (slam_toolbox: loads saved map, publishes map→odom TF) ──
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            loc_config,
            {'map_file_name': LaunchConfiguration('map_file')},
        ],
        output='screen',
    )

    # ── nav2 (path planning + obstacle avoidance → /cmd_vel_raw) ──
    nav2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params,
            # nav2의 cmd_vel 출력을 /cmd_vel_raw 로 리맵: E-stop 안전장치 유지
            'cmd_vel_topic': '/cmd_vel_raw',
        }.items(),
    )

    # ── RViz2 (목적지 지정용 — 2D Nav Goal 버튼 사용) ──
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        use_mock_arg,
        map_file_arg,
        rviz_arg,
        bringup,
        estop_node,
        slam_toolbox_node,
        nav2_node,
        rviz_node,
    ])
