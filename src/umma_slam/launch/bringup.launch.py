"""
Bringup launch: motor controller + LiDAR + odometry + robot_state_publisher.
This is the base layer — run this before SLAM.

robot_params.yaml 한 파일에서 모든 기구 파라미터(wheel_radius, wheel_base,
wheel_offset_x, lidar_x/y/z) 를 주입합니다:
    - URDF (xacro 인자)
    - diff_drive_odometry_node
    - zlac8015d_control_node
바퀴/라이다 위치 변경은 robot_params.yaml 한 곳만 수정하면 됩니다.
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _load_robot_params(path: str) -> dict:
    """robot_params.yaml (/**/ros__parameters:) 를 평탄화된 dict 로 로드."""
    with open(path, 'r') as f:
        data = yaml.safe_load(f) or {}
    # 우선순위: '/**' 와일드카드 > 파일 전체
    node_ns = data.get('/**', data)
    if isinstance(node_ns, dict) and 'ros__parameters' in node_ns:
        return node_ns['ros__parameters']
    return node_ns if isinstance(node_ns, dict) else {}


def generate_launch_description():
    pkg_share = get_package_share_directory('umma_slam')

    # ── Arguments ────────────────────────────────────────────────
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

    # ── Shared robot params (SINGLE SOURCE OF TRUTH) ────────────
    robot_params_file = os.path.join(pkg_share, 'config', 'robot_params.yaml')
    robot_params = _load_robot_params(robot_params_file)

    # ── Robot description (URDF via xacro, 공통 파라미터 주입) ──
    xacro_file = os.path.join(pkg_share, 'urdf', 'umma_robot.urdf.xacro')
    xacro_cmd = [
        'xacro ', xacro_file,
        ' wheel_radius:=',   str(robot_params.get('wheel_radius', 0.1)),
        ' wheel_base:=',     str(robot_params.get('wheel_base', 0.5)),
        ' wheel_offset_x:=', str(robot_params.get('wheel_offset_x', 0.0)),
        ' lidar_x:=',        str(robot_params.get('lidar_x', 0.0)),
        ' lidar_y:=',        str(robot_params.get('lidar_y', 0.0)),
        ' lidar_z:=',        str(robot_params.get('lidar_z', 0.17)),
        ' lidar_roll:=',     str(robot_params.get('lidar_roll', 0.0)),
        ' lidar_pitch:=',    str(robot_params.get('lidar_pitch', 0.0)),
        ' lidar_yaw:=',      str(robot_params.get('lidar_yaw', 0.0)),
    ]
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            # Command substitution 결과는 기본적으로 YAML 로 파싱되기 때문에
            # 반드시 ParameterValue(..., value_type=str) 로 감싸야 합니다.
            'robot_description': ParameterValue(
                Command(xacro_cmd), value_type=str
            ),
            'use_sim_time': False,
        }],
        output='screen',
    )

    # ── Motor controller ───────────────────────────────────────
    # 파라미터 로드 순서: robot_params(공통) → motor_config(모터 고유) → use_mock
    # (뒤에 오는 것이 우선 적용됨)
    motor_node = Node(
        package='zlac8015d_control',
        executable='zlac8015d_control_node',
        name='zlac8015d_control_node',
        parameters=[
            robot_params_file,
            PathJoinSubstitution([
                FindPackageShare('zlac8015d_control'), 'config',
                LaunchConfiguration('motor_config')
            ]),
            {'use_mock': LaunchConfiguration('use_mock')},
        ],
        output='screen',
    )

    # ── Odometry ───────────────────────────────────────────────
    # robot_params(공통: wheel_radius/base/offset_x) 먼저, odometry(프레임명 등) 나중에
    odom_config = os.path.join(pkg_share, 'config', 'odometry.yaml')
    odometry_node = Node(
        package='umma_slam',
        executable='diff_drive_odometry',
        name='diff_drive_odometry_node',
        parameters=[
            robot_params_file,
            odom_config,
        ],
        output='screen',
    )

    # ── YDLidar ────────────────────────────────────────────────
    lidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        parameters=[LaunchConfiguration('lidar_params')],
        output='screen',
    )

    # ── Motor auto-initialization ──────────────────────────────
    #  모터 노드가 올라온 직후(3 s 대기) /initialize → /enable 순서로 서비스 호출
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
    motor_enable_after_init = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=motor_init_cmd,
            on_start=[
                TimerAction(period=1.0, actions=[motor_enable_cmd])
            ],
        )
    )
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
