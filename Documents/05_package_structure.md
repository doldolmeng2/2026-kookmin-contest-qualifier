# 05. 패키지 구조 및 개발 환경

## 1. 워크스페이스 구조

```
~/xycar_ws/
└── src/
    └── ericar/
        ├── perception/     # A 담당: 인식
        ├── driving/        # B 담당: 차선/라바콘/좌회전
        ├── main/           # C 담당: 상태 머신 + 제어
        ├── function/       # 센서 뷰어 + 수동 조종
        └── ericar_msgs/    # 공용 메시지 패키지
```

워크스페이스 이름은 기존 관례에 맞춰 `xycar_ws`를 유지한다.

---

## 2. 패키지별 상세

### 2-1. `ericar_msgs` (메시지)

```
ericar_msgs/
├── CMakeLists.txt
├── package.xml
└── msg/
    ├── Perception.msg
    ├── DrivingOffset.msg
    ├── ControlState.msg
    └── ActivePerceptions.msg
```

**`package.xml`**:
```xml
<?xml version="1.0"?>
<package format="3">
  <name>ericar_msgs</name>
  <version>0.0.1</version>
  <description>Custom messages for ERICAR team</description>
  <maintainer email="ktypet13@hanyang.ac.kr">ericar</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
</package>
```

**`CMakeLists.txt`**:
```cmake
cmake_minimum_required(VERSION 3.5)
project(ericar_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Perception.msg"
  "msg/DrivingOffset.msg"
  "msg/ControlState.msg"
  "msg/ActivePerceptions.msg"
  DEPENDENCIES std_msgs
)

ament_package()
```

### 2-2. `perception` (A 담당)

```
perception/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── perception
├── perception/
│   ├── __init__.py
│   ├── perception_node.py        # 메인 노드
│   └── detectors/                # 인식기 모듈들
│       ├── __init__.py
│       ├── traffic_light.py
│       ├── pedestrian.py
│       ├── obstacle_vehicle.py
│       ├── school_zone.py
│       ├── police_car.py
│       ├── lap_counter.py
│       └── shortcut_exit.py
└── launch/
    └── perception.launch.py
```

**`package.xml`**:
```xml
<?xml version="1.0"?>
<package format="3">
  <name>perception</name>
  <version>0.0.1</version>
  <description>Perception node for ERICAR</description>
  <maintainer email="ktypet13@hanyang.ac.kr">ericar</maintainer>
  <license>MIT</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>ericar_msgs</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### 2-3. `driving` (B 담당)

```
driving/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── driving
├── driving/
│   ├── __init__.py
│   ├── driving_node.py           # 메인 노드
│   ├── lane_detector.py          # 차선 인식
│   ├── cone_detector.py          # 라바콘 인식
│   └── left_turn_offset.py       # 좌회전 offset 계산
└── launch/
    └── driving.launch.py
```

### 2-4. `main` (C 담당)

```
main/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── main
├── main/
│   ├── __init__.py
│   ├── main_node.py              # 상태 머신
│   ├── control_node.py           # 제어 (또는 main_node 안에 통합)
│   └── stage_config.py           # STAGE_TO_MODE, STAGE_PERCEPTIONS 정의
└── launch/
    ├── main.launch.py
    └── all.launch.py             # 전체 시스템 통합 launch
```

### 2-5. `function` (센서 뷰어 + 수동 조종)

```
function/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── function
├── function/
│   ├── __init__.py
│   ├── cam_viewer.py             # 카메라 뷰어
│   ├── lidar_viewer.py           # LiDAR 뷰어
│   ├── imu_viewer.py             # IMU 뷰어
│   ├── motor_viewer.py           # 모터 뷰어
│   └── manual_control.py         # 수동 조종
└── launch/
    └── function.launch.py
```

---

## 3. 빌드 방법

### 전체 빌드
```bash
cw          # cd ~/xycar_ws
cbs         # colcon build --symlink-install
```

### 특정 패키지만 빌드
```bash
cbp ericar_msgs
cbp perception
cbp driving
cbp main
cbp function
```

### 메시지 변경 시
메시지 패키지는 의존성 때문에 항상 가장 먼저 빌드되어야 한다.
```bash
cbp ericar_msgs
source ~/xycar_ws/install/setup.bash
cbs
```

### 빌드 후 환경 적용
```bash
source ~/xycar_ws/install/setup.bash
```

`~/.bashrc`에 추가해두는 것을 권장:
```bash
source /opt/ros/humble/setup.bash
source ~/xycar_ws/install/setup.bash
export ROS_DOMAIN_ID=7
```

---

## 4. 실행 방법

### 개별 노드 실행
```bash
# 인식 노드
ros2 run perception perception_node

# 주행 노드
ros2 run driving driving_node

# main + control 노드
ros2 run main main_node

# 수동 조종
ros2 run function manual_control
```

### Launch로 전체 실행
```bash
ros2 launch main all.launch.py
```

`all.launch.py` 예시:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception',
            executable='perception_node',
            name='ericar_perception',
            output='screen',
        ),
        Node(
            package='driving',
            executable='driving_node',
            name='ericar_driving',
            output='screen',
        ),
        Node(
            package='main',
            executable='main_node',
            name='ericar_main',
            output='screen',
        ),
    ])
```

---

## 5. 디버깅 도구

### 토픽 모니터링
```bash
# 토픽 목록
ros2 topic list

# 특정 토픽 내용 보기
ros2 topic echo /ericar/perception
ros2 topic echo /ericar/control_state
ros2 topic echo /ericar/current_stage

# 발행 주기 확인
ros2 topic hz /ericar/perception
ros2 topic hz /xycar_motor

# 메시지 타입 확인
ros2 topic info /ericar/perception
```

### 노드 정보
```bash
ros2 node list
ros2 node info /ericar_main_control
```

### 메시지 타입 확인
```bash
ros2 interface show ericar_msgs/msg/Perception
```

### 그래프 시각화
```bash
rqt_graph
```

### 수동 발행 테스트
```bash
# 시작 신호 강제 발행 테스트
ros2 topic pub --once /ericar/active_perceptions ericar_msgs/msg/ActivePerceptions "{names: ['traffic_light_start']}"
```

---

## 6. Git 워크플로우 제안

### 브랜치 전략
- `main`: 안정 버전 (대회 출제용)
- `dev`: 개발 통합 브랜치
- `feature/perception-traffic-light` (A 담당 작업)
- `feature/driving-lane` (B 담당 작업)
- `feature/main-state-machine` (C 담당 작업)

### .gitignore (워크스페이스 차원)
```
# Build artifacts
build/
install/
log/

# IDE
.vscode/
.idea/

# Python
__pycache__/
*.pyc

# OS
.DS_Store
```

> **주의**: 실제 코드는 `~/xycar_ws/src` 안에서 작업하므로, 워크스페이스 Git이 아닌 패키지별 Git을 사용하거나 `src/` 폴더 자체를 Git 루트로 두는 방식 중 선택.

### 권장: `src/` 자체를 이 repo로 두는 방식
1. 이 repo(`2026-kookmin-contest-qualifier`)를 클론
2. `~/xycar_ws/src` 폴더를 이 repo로 심볼릭 링크 또는 직접 clone
3. 빌드는 `~/xycar_ws`에서 수행

```bash
mkdir -p ~/xycar_ws
cd ~/xycar_ws
git clone <repo> src
cbs
```

---

## 7. 환경 정보 정리

| 항목 | 값 |
|------|-----|
| OS | Ubuntu 22.04 (WSL2) |
| ROS2 | Humble |
| Python | 3.10 |
| 워크스페이스 | `~/xycar_ws` |
| ROS_DOMAIN_ID | 7 |
| 네임스페이스 | xycar |
| 시뮬레이터 | Windows 호스트에서 실행 |
| 통신 | TCP |

---

## 8. 다음 문서
- [`06_message_definitions.md`](06_message_definitions.md) — .msg 파일 실제 예시
- [`07_design_decisions.md`](07_design_decisions.md) — 설계 결정 배경
