"""
Bringup launch: starts motor controller + LiDAR + odometry + robot_state_publisher.
This is the base layer — run this before SLAM.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('umma_slam')
    motor_pkg_share = get_package_share_directory('zlac8015d_control')

    # ── Arguments ──
    use_mock_arg = DeclareLaunchArgument(
        'use_mock', default_value='false',
        description='Use mock CAN interface for testing'
    )
    motor_config_arg = DeclareLaunchArgument(
        'motor_config', default_value='default.yaml',
        description='Motor controller config file name'
    )
    lidar_params_arg = DeclareLaunchArgument(
        'lidar_params',
        default_value=os.path.join(pkg_share, 'config', 'lidar.yaml'),
        description='LiDAR parameters file'
    )

    # ── Robot description (URDF via xacro) ──
    xacro_file = os.path.join(pkg_share, 'urdf', 'umma_robot.urdf.xacro')
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file]),
            'use_sim_time': False,
        }],
        output='screen',
    )

    # ── Motor controller ──
    motor_node = Node(
        package='zlac8015d_control',
        executable='zlac8015d_control_node',
        name='zlac8015d_control_node',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('zlac8015d_control'), 'config',
                LaunchConfiguration('motor_config')
            ]),
            {'use_mock': LaunchConfiguration('use_mock')},
        ],
        output='screen',
    )

    # ── Odometry ──
    odom_config = os.path.join(pkg_share, 'config', 'odometry.yaml')
    odometry_node = Node(
        package='umma_slam',
        executable='diff_drive_odometry',
        name='diff_drive_odometry_node',
        parameters=[odom_config],
        output='screen',
    )

    # ── YDLidar ──
    lidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        parameters=[LaunchConfiguration('lidar_params')],
        output='screen',
    )

    # ── Motor auto-initialization (실제 로봇 운용 시 자동으로 initialize/enable 서비스 호출) ──
    # 모터 노드가 올라온 직후(3 s 대기) /initialize → /enable 순서로 서비스 호출
    motor_init_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/initialize',
            'std_srvs/srv/Trigger',
            '{}',
        ],
        output='screen',
    )
    motor_enable_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/enable',
            'std_srvs/srv/SetBool',
            '{"data": true}',
        ],
        output='screen',
    )
    # /initialize 완료 후 1 s 대기하고 /enable 호출
    motor_enable_after_init = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=motor_init_cmd,
            on_start=[
                TimerAction(period=1.0, actions=[motor_enable_cmd])
            ],
        )
    )
    # 모터 노드 기동 후 3 s 대기하고 /initialize 호출
    motor_auto_init = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=motor_node,
            on_start=[
                TimerAction(period=3.0, actions=[motor_init_cmd])
            ],
        )
    )

    return LaunchDescription([
        use_mock_arg,
        motor_config_arg,
        lidar_params_arg,
        robot_state_publisher,
        motor_node,
        motor_enable_after_init,
        motor_auto_init,
        odometry_node,
        lidar_node,
    ])
