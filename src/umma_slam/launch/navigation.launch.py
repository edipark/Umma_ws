"""
Navigation launch: selectable localization + nav2 + E-stop + hardware bringup.
Use this after building a map with slam.launch.py.

  # slam_toolbox localization (.posegraph/.data)
  ros2 launch umma_slam navigation.launch.py \
    localization_mode:=slam_toolbox \
    map_file:=/home/mingun/maps/my_map

  # nav2 map_server + amcl (.yaml + .pgm)
  ros2 launch umma_slam navigation.launch.py \
    localization_mode:=nav2_map_server \
    map_yaml:=/home/mingun/maps/my_map.yaml
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
from launch.substitutions import LaunchConfiguration, PythonExpression
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
        description='Base path for slam_toolbox map (.posegraph/.data, without extension)'
    )
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value='',
        description='Full path to map yaml file for nav2 map_server mode'
    )
    localization_mode_arg = DeclareLaunchArgument(
        'localization_mode',
        default_value='slam_toolbox',
        description='Localization backend: slam_toolbox | nav2_map_server'
    )
    auto_initial_pose_arg = DeclareLaunchArgument(
        'auto_initial_pose',
        default_value='true',
        description='Auto publish /initialpose in nav2_map_server mode'
    )
    initial_pose_x_arg = DeclareLaunchArgument(
        'initial_pose_x',
        default_value='0.0',
        description='Initial pose x (map frame) for nav2_map_server mode'
    )
    initial_pose_y_arg = DeclareLaunchArgument(
        'initial_pose_y',
        default_value='0.0',
        description='Initial pose y (map frame) for nav2_map_server mode'
    )
    initial_pose_yaw_arg = DeclareLaunchArgument(
        'initial_pose_yaw',
        default_value='0.0',
        description='Initial pose yaw (rad) for nav2_map_server mode'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2 (false for headless)'
    )

    mode_is_slam_toolbox = IfCondition(
        PythonExpression([
            "'",
            LaunchConfiguration('localization_mode'),
            "' == 'slam_toolbox'",
        ])
    )
    mode_is_nav2_map_server = IfCondition(
        PythonExpression([
            "'",
            LaunchConfiguration('localization_mode'),
            "' == 'nav2_map_server'",
        ])
    )
    auto_initial_pose_enabled = IfCondition(
        PythonExpression([
            "'",
            LaunchConfiguration('localization_mode'),
            "' == 'nav2_map_server' and '",
            LaunchConfiguration('auto_initial_pose'),
            "' == 'true'",
        ])
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
        condition=mode_is_slam_toolbox,
    )

    # ── nav2 (slam_toolbox provides /map and map->odom TF) ──
    nav2_node_with_slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params,
        }.items(),
        condition=mode_is_slam_toolbox,
    )

    # ── nav2 (map_server + amcl + navigation with yaml/pgm map) ──
    nav2_node_with_map_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'bringup_launch.py'
            )
        ),
        launch_arguments={
            'slam': 'False',
            'map': LaunchConfiguration('map_yaml'),
            'use_sim_time': 'false',
            'params_file': nav2_params,
            'use_composition': 'False',
        }.items(),
        condition=mode_is_nav2_map_server,
    )

    initial_pose_sender_node = Node(
        package='umma_slam',
        executable='initial_pose_sender',
        name='initial_pose_sender',
        output='screen',
        parameters=[{
            'topic': '/initialpose',
            'frame_id': 'map',
            'x': LaunchConfiguration('initial_pose_x'),
            'y': LaunchConfiguration('initial_pose_y'),
            'yaw': LaunchConfiguration('initial_pose_yaw'),
            'repeat_count': 12,
            'interval_sec': 0.5,
            'cov_xy': 0.25,
            'cov_yaw': 0.07,
        }],
        condition=auto_initial_pose_enabled,
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
        map_yaml_arg,
        localization_mode_arg,
        auto_initial_pose_arg,
        initial_pose_x_arg,
        initial_pose_y_arg,
        initial_pose_yaw_arg,
        rviz_arg,
        bringup,
        estop_node,
        slam_toolbox_node,
        nav2_node_with_slam_toolbox,
        nav2_node_with_map_server,
        initial_pose_sender_node,
        rviz_node,
    ])
