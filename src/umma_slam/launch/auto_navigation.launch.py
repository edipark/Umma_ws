"""
Auto navigation launch:
navigation stack + named-target dispatcher.

Example:
  # slam_toolbox localization (.posegraph/.data)
  ros2 launch umma_slam auto_navigation.launch.py \
    localization_mode:=slam_toolbox \
    map_file:=/home/mingun/maps/my_map \
    target_name:=charging_station

  # nav2 map_server + amcl (.yaml + .pgm)
  ros2 launch umma_slam auto_navigation.launch.py \
    localization_mode:=nav2_map_server \
    map_yaml:=/home/mingun/maps/my_map.yaml \
    target_name:=charging_station
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
    navigation_launch = os.path.join(pkg_share, 'launch', 'navigation.launch.py')
    default_targets = os.path.join(pkg_share, 'config', 'navigation_targets.yaml')

    use_mock_arg = DeclareLaunchArgument(
        'use_mock', default_value='false',
        description='Use mock CAN interface'
    )
    map_file_arg = DeclareLaunchArgument(
        'map_file', default_value='',
        description='Base path for slam_toolbox map (.posegraph/.data, without extension)'
    )
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml', default_value='',
        description='Full path to map yaml file for nav2 map_server mode'
    )
    localization_mode_arg = DeclareLaunchArgument(
        'localization_mode', default_value='slam_toolbox',
        description='Localization backend: slam_toolbox | nav2_map_server'
    )
    auto_initial_pose_arg = DeclareLaunchArgument(
        'auto_initial_pose', default_value='true',
        description='Auto publish /initialpose in nav2_map_server mode'
    )
    initial_pose_x_arg = DeclareLaunchArgument(
        'initial_pose_x', default_value='0.0',
        description='Initial pose x (map frame) for nav2_map_server mode'
    )
    initial_pose_y_arg = DeclareLaunchArgument(
        'initial_pose_y', default_value='0.0',
        description='Initial pose y (map frame) for nav2_map_server mode'
    )
    initial_pose_yaw_arg = DeclareLaunchArgument(
        'initial_pose_yaw', default_value='0.0',
        description='Initial pose yaw (rad) for nav2_map_server mode'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz2 or headless mode'
    )
    targets_file_arg = DeclareLaunchArgument(
        'targets_file', default_value=default_targets,
        description='YAML file path for named targets'
    )
    target_name_arg = DeclareLaunchArgument(
        'target_name', default_value='',
        description='Target name in targets YAML to send at startup'
    )
    input_topic_arg = DeclareLaunchArgument(
        'input_topic', default_value='/navigation_target',
        description='Topic that receives target name as std_msgs/String'
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch),
        launch_arguments={
            'use_mock': LaunchConfiguration('use_mock'),
            'map_file': LaunchConfiguration('map_file'),
            'map_yaml': LaunchConfiguration('map_yaml'),
            'localization_mode': LaunchConfiguration('localization_mode'),
            'auto_initial_pose': LaunchConfiguration('auto_initial_pose'),
            'initial_pose_x': LaunchConfiguration('initial_pose_x'),
            'initial_pose_y': LaunchConfiguration('initial_pose_y'),
            'initial_pose_yaw': LaunchConfiguration('initial_pose_yaw'),
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )

    target_dispatcher_node = Node(
        package='umma_slam',
        executable='target_goal_dispatcher',
        name='target_goal_dispatcher',
        parameters=[{
            'targets_file': LaunchConfiguration('targets_file'),
            'target_name': LaunchConfiguration('target_name'),
            'input_topic': LaunchConfiguration('input_topic'),
            'default_frame_id': 'map',
        }],
        output='screen',
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
        targets_file_arg,
        target_name_arg,
        input_topic_arg,
        navigation,
        target_dispatcher_node,
    ])
