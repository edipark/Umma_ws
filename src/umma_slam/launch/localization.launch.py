"""
Localization launch: loads a saved map and runs slam_toolbox in localization mode.
Use this after you have built a map with slam.launch.py.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('umma_slam')
    bringup_launch = os.path.join(pkg_share, 'launch', 'bringup.launch.py')
    loc_config = os.path.join(pkg_share, 'config', 'localization.yaml')

    # ── Arguments ──
    use_mock_arg = DeclareLaunchArgument(
        'use_mock', default_value='false',
        description='Use mock CAN interface'
    )
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Full path to the serialized map file (without extension)'
    )

    # ── Bringup ──
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments={'use_mock': LaunchConfiguration('use_mock')}.items(),
    )

    # ── SLAM Toolbox (localization mode) ──
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

    return LaunchDescription([
        use_mock_arg,
        map_file_arg,
        bringup,
        slam_toolbox_node,
    ])
