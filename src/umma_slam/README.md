# UMMA Robot SLAM & 자율주행 프레임워크

ZLAC8015D 허브 모터 + YDLidar 기반 실제 로봇용 SLAM / 자율주행 패키지입니다.

---

## 목차

1. [시스템 구조](#1-시스템-구조)
2. [설치](#2-설치)
3. [로봇 파라미터 설정](#3-로봇-파라미터-설정)
4. [1단계 — 수동 매핑 (지도 만들기)](#4-1단계--수동-매핑-지도-만들기)
5. [2단계 — 자율주행 (목적지 지정)](#5-2단계--자율주행-목적지-지정)
6. [비상정지 (E-stop)](#6-비상정지-e-stop)
7. [상태 진단](#7-상태-진단)
8. [패키지 구조](#8-패키지-구조)

---

## 1. 시스템 구조

### 전체 데이터 흐름

```
【수동 매핑 모드】
  teleop ──► /cmd_vel_raw ─┐
                            ▼
                   [emergency_stop_node]  ◄── Watchdog (1초 타임아웃)
                            │ /cmd_vel
                            ▼
                   [zlac8015d_control]  ←──── CANopen ──── 모터
                            │ /joint_states
                            ▼
                   [diff_drive_odometry]
                            │ /odom  +  TF: odom→base_footprint
                            │
              /scan ◄── [YDLidar driver]
                            │
                   [slam_toolbox (mapping)]
                            │ TF: map→odom  +  /map
                            ▼
                          RViz2

【자율주행 모드】
  RViz2 (목적지 클릭)
       │ /goal_pose
       ▼
    [nav2]  ──► /cmd_vel_raw ─┐
                               ▼
                      [emergency_stop_node]
                               │ /cmd_vel
                               ▼
                      [zlac8015d_control] ── 모터
```

### TF 체인

```
map ──► odom ──► base_footprint ──► base_link ──► laser_frame
 │       │
slam    odom
toolbox  node
```

---

## 2. 설치

### 2-1. ROS 2 패키지

```bash
sudo apt install -y \
  ros-$ROS_DISTRO-slam-toolbox \
  ros-$ROS_DISTRO-nav2-bringup \
  ros-$ROS_DISTRO-xacro \
  ros-$ROS_DISTRO-robot-state-publisher \
  ros-$ROS_DISTRO-teleop-twist-keyboard \
  ros-$ROS_DISTRO-tf2-ros
```

### 2-2. YDLidar SDK 빌드

```bash
cd /home/mingun/close_ws/pkgs/YDLidar-SDK
mkdir -p build && cd build
cmake .. && make -j$(nproc)
sudo make install
```

### 2-3. 워크스페이스 빌드

```bash
# YDLidar ROS2 드라이버
cd /home/mingun/close_ws/pkgs/ydlidar_ros2_ws
colcon build --symlink-install

# UMMA SLAM 패키지 (motor controller 포함)
cd /home/mingun/umma_ws
colcon build --symlink-install
```

### 2-4. CAN 인터페이스 설정

실행할 때마다 필요합니다:

```bash
sudo ip link set can1 up type can bitrate 500000
```

부팅 시 자동 활성화하려면:

```bash
sudo tee /etc/network/interfaces.d/can1 << 'EOF'
auto can1
iface can1 inet manual
    pre-up ip link set can1 type can bitrate 500000
    up ifconfig can1 up
    down ifconfig can1 down
EOF
```

---

## 3. 로봇 파라미터 설정

실제 로봇 치수에 맞게 두 파일을 수정하세요. **두 파일의 값이 반드시 일치**해야 합니다.

### `config/odometry.yaml`

```yaml
diff_drive_odometry_node:
  ros__parameters:
    wheel_radius: 0.1    # ← 바퀴 반지름 (m) 실측값으로 변경
    wheel_base:   0.5    # ← 좌우 바퀴 중심 간격 (m) 실측값으로 변경
```

### `src/ROS2_ZLAC8015D_canopen/config/default.yaml`

```yaml
zlac8015d_control_node:
  ros__parameters:
    wheel_radius: 0.1    # ← 위와 동일한 값
    wheel_base:   0.5    # ← 위와 동일한 값
    can_interface: "can1"
    encoder_resolution: 1024
```

### `config/lidar.yaml`

```yaml
ydlidar_ros2_driver_node:
  ros__parameters:
    port: /dev/ttyUSB0   # ← 실제 LiDAR 포트 (ls /dev/ttyUSB* 로 확인)
    frequency: 10.0
    range_max: 16.0      # ← 사용 중인 LiDAR 모델의 최대 거리
```

### `config/nav2_params.yaml` (자율주행 시)

```yaml
# 로봇 크기 — 두 곳을 같은 값으로
local_costmap:
  local_costmap:
    ros__parameters:
      robot_radius: 0.35   # ← 로봇 반지름 + 여유 (m)

global_costmap:
  global_costmap:
    ros__parameters:
      robot_radius: 0.35   # ← 동일하게

# 자율주행 속도 제한
controller_server:
  ros__parameters:
    FollowPath:
      desired_linear_vel: 0.3   # ← 최대 직선 속도 (m/s)
```

---

## 4. 1단계 — 수동 매핑 (지도 만들기)

처음 한 번만 하면 됩니다. 공간을 직접 돌아다니며 LiDAR로 지도를 생성합니다.

### 4-1. 터미널 1 — SLAM 기동

```bash
source /home/mingun/close_ws/pkgs/ydlidar_ros2_ws/install/setup.bash
source /home/mingun/umma_ws/install/setup.bash

ros2 launch umma_slam slam.launch.py
```

> 기동 후 약 3초 뒤 모터가 자동으로 초기화됩니다.

### 4-2. 터미널 2 — teleop으로 로봇 조종

```bash
source /home/mingun/umma_ws/install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=cmd_vel_raw
```

> `-r cmd_vel:=cmd_vel_raw` 옵션이 **반드시** 필요합니다.

**조종 키:**

| 키 | 동작 |
|:---:|---|
| `i` | 전진 |
| `,` | 후진 |
| `j` | 제자리 좌회전 |
| `l` | 제자리 우회전 |
| `u` / `o` | 좌/우 전진 곡선 |
| `k` | 정지 |
| `q` / `z` | 전체 속도 증가 / 감소 |

공간 전체를 천천히 돌아다니며 RViz에서 지도가 완성되는 것을 확인합니다.

### 4-3. 지도 저장

```bash
mkdir -p ~/umma_ws/maps

ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/mingun/maps/my_map'}"
```

저장 완료 후 `~/maps/my_map.posegraph`, `~/maps/my_map.data` 두 파일이 생깁니다.

### 4-4. 종료

터미널 1에서 `Ctrl+C` → 자동으로 모터 정지 후 모든 노드 종료.

---

## 5. 2단계 — 자율주행 (목적지 지정)

저장된 지도를 불러와 nav2가 경로를 계획하고 자율 주행합니다.

### 5-1. 터미널 1 — 자율주행 시스템 기동

```bash
source /home/mingun/close_ws/pkgs/ydlidar_ros2_ws/install/setup.bash
source /home/mingun/umma_ws/install/setup.bash

ros2 launch umma_slam navigation.launch.py \
  map_file:=/home/mingun/maps/my_map
```

> 이 명령 하나로 모터 + LiDAR + 오도메트리 + 로컬라이제이션 + nav2 + RViz2 모두 기동됩니다.

### 5-2. RViz에서 목적지 지정

1. RViz 좌측 상단 툴바에서 **"Nav2 Goal"** 버튼 클릭
2. 지도 위 목적지를 클릭 + 드래그로 방향 지정
3. nav2가 자동으로 경로 계획 → 로봇 주행 시작

### 5-3. 코드로 목적지 지정 (옵션)

RViz 없이 터미널에서 직접 목적지를 보낼 수도 있습니다:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}}"
```

### 5-4. 종료

터미널 1에서 `Ctrl+C` → 자동으로 모터 정지 후 전체 종료.

---

## 6. 비상정지 (E-stop)

### 동작 원리

모든 속도 명령(`/cmd_vel_raw`)은 `emergency_stop_node`를 반드시 거쳐야만 모터에 전달됩니다.

```
/cmd_vel_raw ──► [E-stop 게이트] ──► /cmd_vel ──► 모터
                        │
               Watchdog: 1초 동안 입력 없으면 자동 정지
```

| 상황 | 결과 |
|---|---|
| 정상 주행 중 | 명령 통과 |
| teleop 터미널 강제 종료 | 1초 후 자동 정지 |
| 네트워크 끊김 (원격 조종) | 1초 후 자동 정지 |
| nav2 계획 완료 / 목적지 도착 | nav2가 zero 명령 전송 → 정지 |
| `Ctrl+C` | OnShutdown 핸들러 → 즉시 정지 |

### 수동 E-stop 명령

```bash
# 즉시 정지
ros2 service call /estop/activate std_srvs/srv/Trigger '{}'

# 해제
ros2 service call /estop/release std_srvs/srv/Trigger '{}'

# 상태 확인
ros2 topic echo /estop/state
```

---

## 7. 상태 진단

시스템이 제대로 동작하는지 확인하는 명령입니다.

```bash
# 실행 중인 노드 목록
ros2 node list

# 토픽 주파수 확인 (정상값)
ros2 topic hz /scan          # LiDAR: ~10 Hz
ros2 topic hz /joint_states  # 모터 피드백: ~50 Hz
ros2 topic hz /odom          # 오도메트리: ~50 Hz
ros2 topic hz /map           # 지도 업데이트: ~0.2 Hz (5초마다)

# E-stop 상태
ros2 topic echo /estop/state   # false = 정상, true = 정지 중

# TF 체인 시각화 (map→odom→base_footprint→base_link→laser_frame)
ros2 run tf2_tools view_frames
```

---

## 8. 패키지 구조

```
umma_slam/
├── urdf/
│   └── umma_robot.urdf.xacro       # 로봇 3D 모델 (base, 바퀴, LiDAR)
│
├── umma_slam/
│   ├── diff_drive_odometry.py       # 엔코더 → /odom + TF
│   └── emergency_stop.py            # E-stop 게이트 + Watchdog
│
├── config/
│   ├── odometry.yaml                # 바퀴 파라미터 ← 실측값 입력
│   ├── lidar.yaml                   # LiDAR 포트/모델 설정 ← 확인 필요
│   ├── slam_toolbox.yaml            # SLAM 매핑 설정
│   ├── localization.yaml            # 로컬라이제이션 설정
│   ├── nav2_params.yaml             # nav2 경로계획/속도 설정 ← 로봇 크기 입력
│   ├── slam.rviz                    # 매핑용 RViz 레이아웃
│   └── navigation.rviz              # 자율주행용 RViz 레이아웃
│
└── launch/
    ├── bringup.launch.py            # 하드웨어 공통 기동 (모터+LiDAR+odom+URDF)
    ├── slam.launch.py               # 수동 매핑 (bringup + slam_toolbox)
    ├── localization.launch.py       # 위치 추정만 (저장된 맵 사용)
    └── navigation.launch.py         # 자율주행 (bringup + 로컬라이제이션 + nav2)
```


ZLAC8015D 모터 드라이버 + YDLidar를 기반으로 한 실제 로봇용 SLAM 프레임워크입니다.

---

## 전체 시스템 구조

```
[teleop / nav2]                              [SLAM / Localization]
       │                                              │
       ▼ /cmd_vel_raw                         /scan ◄─┤
[emergency_stop_node] ──► /cmd_vel ──► [zlac8015d_control_node]
   (watchdog + E-stop)                         │ ▲
                                   /joint_states │ │ CAN bus
                                               │ ▼
                               [diff_drive_odometry_node]   [YDLidar driver]
                                               │ /odom           │ /scan
                                        TF broadcast            │
                                  odom → base_footprint         │
                                               │                 │
                              ┌────────────────┴─────────────────┘
                              ▼
                      [slam_toolbox]
                    map → odom TF broadcast
                              │
                              ▼
                          [RViz2]
```

### TF 체인

```
map ──► odom ──► base_footprint ──► base_link ──► laser_frame
 │       │             │
slam   odom          URDF
toolbox node       (robot_state_publisher)
```

---

## 설치

### 1. ROS 2 의존성

```bash
sudo apt install -y \
  ros-$ROS_DISTRO-slam-toolbox \
  ros-$ROS_DISTRO-xacro \
  ros-$ROS_DISTRO-robot-state-publisher \
  ros-$ROS_DISTRO-teleop-twist-keyboard \
  ros-$ROS_DISTRO-tf2-ros \
  ros-$ROS_DISTRO-nav-msgs
```

### 2. YDLidar SDK 설치

```bash
cd /home/mingun/close_ws/pkgs/YDLidar-SDK
mkdir -p build && cd build
cmake .. && make -j$(nproc)
sudo make install
```

### 3. ydlidar_ros2_driver 빌드

```bash
cd /home/mingun/close_ws/pkgs/ydlidar_ros2_ws
colcon build --symlink-install
```

### 4. umma_slam 빌드

```bash
cd /home/mingun/umma_ws
colcon build --symlink-install
```

### 5. CAN 인터페이스 활성화

```bash
sudo ip link set can0 up type can bitrate 1000000
```

**부팅마다 자동 활성화 (선택):**

```bash
# /etc/network/interfaces.d/can0 파일 생성
sudo tee /etc/network/interfaces.d/can0 << 'EOF'
auto can0
iface can0 inet manual
    pre-up ip link set can0 type can bitrate 1000000
    up ifconfig can0 up
    down ifconfig can0 down
EOF
```

---

## 설정값 조정

실제 로봇 치수에 맞게 반드시 수정하세요.

### 바퀴/오도메트리 파라미터

`config/odometry.yaml`:
```yaml
diff_drive_odometry_node:
  ros__parameters:
    wheel_radius: 0.1    # 바퀴 반지름 (m) ← 실측값으로 변경
    wheel_base: 0.5      # 좌우 바퀴 간격 (m) ← 실측값으로 변경
```

`src/ROS2_ZLAC8015D_canopen/config/default.yaml`:
```yaml
zlac8015d_control_node:
  ros__parameters:
    wheel_radius: 0.1    # 위와 동일한 값으로 맞출 것
    wheel_base: 0.5
```

### LiDAR 파라미터

`config/lidar.yaml`:
```yaml
ydlidar_ros2_driver_node:
  ros__parameters:
    port: /dev/ttyUSB0   # LiDAR 실제 포트 확인: ls /dev/ttyUSB*
    frequency: 10.0      # 스캔 주파수 (Hz)
    range_max: 16.0      # 최대 측정 거리 (m) — 모델별 상이
```

---

## SLAM 실행 (지도 만들기)

### 터미널 1 — 전체 시스템 기동

```bash
source /home/mingun/close_ws/pkgs/ydlidar_ros2_ws/install/setup.bash
source /home/mingun/umma_ws/install/setup.bash

ros2 launch umma_slam slam.launch.py

# 모니터 없는 헤드리스 운용 시
ros2 launch umma_slam slam.launch.py rviz:=false
```

이 명령 하나로 다음이 모두 기동됩니다:

| 노드 | 역할 |
|---|---|
| `robot_state_publisher` | URDF → TF (base_link, laser_frame 등) |
| `zlac8015d_control_node` | CAN 통신, 모터 제어 (3초 후 자동 초기화) |
| `ydlidar_ros2_driver_node` | LiDAR → `/scan` |
| `diff_drive_odometry_node` | 엔코더 → `/odom` + TF |
| `emergency_stop_node` | 비상정지 게이트 |
| `async_slam_toolbox_node` | 실시간 매핑 |
| `rviz2` | 시각화 |

### 터미널 2 — teleop

```bash
source /home/mingun/umma_ws/install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=cmd_vel_raw
```

> ⚠️ `-r cmd_vel:=cmd_vel_raw` 리맵핑 필수. 없으면 비상정지를 우회해 모터에 직접 연결됩니다.

**조종 키:**

```
u  i  o    ← 전진 (i=직진, u=좌, o=우)
j  k  l    ← k=정지, j=좌회전, l=우회전
m  ,  .    ← 후진
q/z : 전체 속도 증가/감소
w/x : 선속도 증가/감소
e/c : 각속도 증가/감소
```

### 지도 저장

공간을 충분히 돌아다닌 후:

```bash
mkdir -p ~/maps

ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/mingun/maps/my_map'}"
```

`my_map.posegraph`, `my_map.data` 두 파일이 생성됩니다.

---

## 로컬라이제이션 실행 (저장된 지도로 위치 추정)

```bash
source /home/mingun/close_ws/pkgs/ydlidar_ros2_ws/install/setup.bash
source /home/mingun/umma_ws/install/setup.bash

ros2 launch umma_slam localization.launch.py \
  map_file:=/home/mingun/maps/my_map
```

---

## 비상정지 (E-stop)

### 동작 방식 — Watchdog + Passthrough

```
teleop ──► /cmd_vel_raw ──► [emergency_stop_node] ──► /cmd_vel ──► 모터
                                      │
                              Watchdog (10Hz 주기 체크)
                                      │
                         /cmd_vel_raw 수신 후 1초 초과?
                           YES → E-stop 발동, zero 퍼블리시
```

| 상황 | 결과 |
|---|---|
| 키 누르는 중 | 속도 명령 통과 |
| 키에서 손 뗌 | teleop가 zero 전송 → 정지 |
| **teleop 터미널 강제 종료** | 메시지 끊김 → **1초 후 자동 정지** |
| **네트워크 연결 끊김** | 동일하게 1초 후 자동 정지 |
| Ctrl+C (launch 종료) | OnShutdown 핸들러 → 즉시 모터 정지 후 전체 종료 |

### 수동 E-stop 명령

```bash
# 발동 (즉시 정지)
ros2 service call /estop/activate std_srvs/srv/Trigger '{}'

# 해제 (teleop 터미널이 살아있어야 함)
ros2 service call /estop/release std_srvs/srv/Trigger '{}'

# 상태 확인
ros2 topic echo /estop/state
```

### Ctrl+C 종료 순서

```
Ctrl+C 입력
   ├─ /estop/activate 서비스 호출 → /cmd_vel zero 즉시 퍼블리시
   ├─ /stop 서비스 호출 → 모터 드라이버 직접 정지
   └─ 모든 자식 프로세스 SIGINT → 각 노드 finally 블록 실행
         ├─ emergency_stop: Twist() 한 번 더 퍼블리시
         └─ zlac8015d: driver.stop() + CAN disconnect
```

---

## teleop과 자율주행의 관계

### 현재 구조 (SLAM 수동 매핑)

teleop 없이는 로봇이 움직이지 않습니다. `/cmd_vel_raw`에 아무 메시지도 없으면 Watchdog이 E-stop을 발동합니다.

```
teleop(필수) ──► /cmd_vel_raw ──► E-stop ──► /cmd_vel ──► 모터
```

SLAM 매핑 단계는 공간 전체를 사람이 직접 운전하며 스캔해야 하므로 이 구조가 맞습니다.

### 자율주행 연동 시 (nav2 등)

nav2를 붙일 경우, nav2가 `/cmd_vel_raw`로 publish하도록 리맵핑하면 E-stop 안전장치를 그대로 유지할 수 있습니다.

```bash
# nav2 실행 시 리맵핑 예시
ros2 launch nav2_bringup navigation_launch.py \
  --ros-args -r cmd_vel:=cmd_vel_raw
```

이렇게 하면 teleop 없이 자율주행 상태에서도:
- nav2가 `/cmd_vel_raw`로 명령 → Watchdog 리셋 유지 → 정상 주행
- nav2가 멈추면 1초 후 Watchdog 발동 → 자동 안전 정지

---

## 상태 확인 (진단)

```bash
# 노드 목록
ros2 node list

# 토픽 주파수 확인
ros2 topic hz /scan          # LiDAR: ~10Hz
ros2 topic hz /joint_states  # 모터 피드백: ~50Hz
ros2 topic hz /odom          # 오도메트리: ~50Hz
ros2 topic hz /cmd_vel       # 모터 명령: E-stop 통과 후

# TF 체인 시각화 (map→odom→base_footprint→base_link→laser_frame)
ros2 run tf2_tools view_frames
evince frames.pdf

# E-stop 상태
ros2 topic echo /estop/state
```

---

## 패키지 구조

```
umma_slam/
├── urdf/
│   └── umma_robot.urdf.xacro     # 로봇 모델 (base, wheel, LiDAR)
├── umma_slam/
│   ├── diff_drive_odometry.py    # 엔코더 → odom + TF
│   └── emergency_stop.py         # 비상정지 + Watchdog
├── config/
│   ├── odometry.yaml             # 바퀴 파라미터
│   ├── lidar.yaml                # YDLidar 파라미터
│   ├── slam_toolbox.yaml         # SLAM 설정
│   ├── localization.yaml         # 로컬라이제이션 설정
│   └── slam.rviz                 # RViz 레이아웃
└── launch/
    ├── bringup.launch.py         # 모터 + LiDAR + odom + URDF
    ├── slam.launch.py            # bringup + SLAM + E-stop
    └── localization.launch.py    # bringup + 저장된 맵 로컬라이제이션
```
