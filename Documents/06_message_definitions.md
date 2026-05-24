# 06. 메시지 정의 (Message Definitions)

`ericar_msgs/msg/` 폴더에 들어갈 4개의 `.msg` 파일 정의.

---

## 1. `Perception.msg`

인식 담당이 발행하는 통합 인식 메시지.

```
# Perception.msg
# 인식 담당 → main
# 모든 인식 결과를 통합한 단일 메시지

# === 신호등 enum 값 ===
uint8 SIGNAL_NONE = 0
uint8 SIGNAL_RED = 1
uint8 SIGNAL_YELLOW = 2
uint8 SIGNAL_GREEN = 3
uint8 SIGNAL_LEFT = 4

# === 차선 enum 값 (obstacle_faster_lane용) ===
uint8 LANE_UNKNOWN = 0
uint8 LANE_1 = 1
uint8 LANE_2 = 2

# === 경찰차 enum 값 ===
uint8 POLICE_UNKNOWN = 0
uint8 POLICE_PRESENT = 1
uint8 POLICE_ABSENT = 2

# ────────────────────────────────────────
# === 필드 ===
# ────────────────────────────────────────

# Header (timestamp 추적용, 선택)
std_msgs/Header header

# 신호등 (총 2개)
uint8 traffic_light_start         # 시작 신호등
uint8 traffic_light_track         # 트랙 위 신호등 = 경로선택 신호등

# 보행자
bool pedestrian_detected          # 위험 위치 보행자 감지 시 true

# 방해차량
bool obstacle_vehicle_detected    # 앞 차량 인식 여부
uint8 obstacle_faster_lane        # 어느 차선이 더 빠른지 (LANE_*)
bool obstacle_overtake_done       # 느린 차량 추월 완료 신호

# 어린이 보호구역
bool school_zone_entry            # 시작 표시 인식
bool school_zone_exit             # 종료 표시 인식

# 경찰차
uint8 police_car_status           # POLICE_*

# lap 카운트
bool start_line_detected          # 출발선 인식

# 지름길
bool shortcut_exit_trigger        # 두 번째 좌회전 트리거
```

### 사용 예시 (발행)
```python
from ericar_msgs.msg import Perception

msg = Perception()
msg.header.stamp = self.get_clock().now().to_msg()
msg.traffic_light_start = Perception.SIGNAL_GREEN
msg.pedestrian_detected = False
msg.obstacle_vehicle_detected = True
msg.obstacle_faster_lane = Perception.LANE_2
msg.police_car_status = Perception.POLICE_UNKNOWN
# ... 다른 필드들 (자동으로 0/false 초기화됨)
self.perception_pub.publish(msg)
```

### 사용 예시 (구독)
```python
def perception_callback(self, msg: Perception):
    if msg.traffic_light_start == Perception.SIGNAL_GREEN:
        self.transition_to("CONE_DRIVING")
```

---

## 2. `DrivingOffset.msg`

차선/라바콘/좌회전 담당이 발행하는 주행 offset.

```
# DrivingOffset.msg
# ericar_driving → control
# 차선/라바콘/좌회전 offset 통합

# === source enum ===
uint8 SOURCE_LANE = 0       # 차선 인식
uint8 SOURCE_CONE = 1       # 라바콘 인식
uint8 SOURCE_LEFT_TURN = 2  # 좌회전 모듈

# Header (선택)
std_msgs/Header header

# 횡방향 오차
# 부호 규칙은 팀에서 합의 (권장: 좌-, 우+)
float32 offset

# 어디서 온 offset인지 (디버깅용)
uint8 source
```

### 사용 예시
```python
from ericar_msgs.msg import DrivingOffset

msg = DrivingOffset()
msg.header.stamp = self.get_clock().now().to_msg()
msg.offset = -0.15  # 좌측으로 약간 치우침
msg.source = DrivingOffset.SOURCE_LANE
self.driving_offset_pub.publish(msg)
```

---

## 3. `ControlState.msg`

main이 control에 보내는 모드 + 정지 명령.

```
# ControlState.msg
# main → control
# 모드와 정지 명령을 묶어 전달

# === 모드 enum ===
uint8 MODE_WAITING = 0
uint8 MODE_CONE_DRIVING = 1
uint8 MODE_LANE_DRIVING = 2
uint8 MODE_LANE_DRIVING_2 = 3
uint8 MODE_LANE_OBSTACLE_FOLLOW = 4
uint8 MODE_LANE_CHANGE = 5
uint8 MODE_LEFT_TURN = 6

# Header (선택)
std_msgs/Header header

# 현재 모드
uint8 mode

# 정지 명령 (모드와 무관하게 true면 speed=0)
bool stop_request
```

### 사용 예시 (발행)
```python
from ericar_msgs.msg import ControlState

state = ControlState()
state.mode = ControlState.MODE_LANE_DRIVING
state.stop_request = False
self.control_state_pub.publish(state)
```

### 사용 예시 (구독, control 측)
```python
def control_state_callback(self, msg: ControlState):
    self.current_mode = msg.mode
    self.stop_request = msg.stop_request
```

---

## 4. `ActivePerceptions.msg`

main이 인식 담당에 보내는 활성 인식기 목록.

```
# ActivePerceptions.msg
# main → ericar_perception
# 현재 활성화할 인식기 이름 목록

# Header (선택)
std_msgs/Header header

# 활성 인식기 이름들
# 가능한 값:
#   "traffic_light_start"
#   "traffic_light_track"
#   "pedestrian"
#   "obstacle_vehicle"
#   "school_zone_entry"
#   "school_zone_exit"
#   "police_car"
#   "lap_counter"
#   "shortcut_exit_trigger"
string[] names
```

### 사용 예시 (발행)
```python
from ericar_msgs.msg import ActivePerceptions

msg = ActivePerceptions()
msg.names = ["pedestrian"]
self.active_perceptions_pub.publish(msg)
```

### 사용 예시 (구독, 인식 측)
```python
def active_perceptions_callback(self, msg: ActivePerceptions):
    self.active = set(msg.names)

def perception_loop(self):
    out = Perception()
    if "traffic_light_start" in self.active:
        out.traffic_light_start = self.detect_start_light()
    # ...
    self.perception_pub.publish(out)
```

---

## 5. 인식기 이름 상수 (선택)

오타 방지를 위해 인식기 이름들을 별도 메시지 파일이나 공용 Python 모듈에 상수로 두는 것을 권장.

### 옵션 A. `ericar_msgs`에 상수 메시지 추가
```
# PerceptionNames.msg (참조용 상수만 담은 빈 메시지)
string TRAFFIC_LIGHT_START="traffic_light_start"
string TRAFFIC_LIGHT_TRACK="traffic_light_track"
string PEDESTRIAN="pedestrian"
string OBSTACLE_VEHICLE="obstacle_vehicle"
string SCHOOL_ZONE_ENTRY="school_zone_entry"
string SCHOOL_ZONE_EXIT="school_zone_exit"
string POLICE_CAR="police_car"
string LAP_COUNTER="lap_counter"
string SHORTCUT_EXIT_TRIGGER="shortcut_exit_trigger"
```
> ROS2 .msg는 string 상수를 지원함.

### 옵션 B. 공용 Python 모듈
`ericar_main_control/ericar_main_control/perception_names.py`:
```python
TRAFFIC_LIGHT_START = "traffic_light_start"
TRAFFIC_LIGHT_TRACK = "traffic_light_track"
PEDESTRIAN = "pedestrian"
OBSTACLE_VEHICLE = "obstacle_vehicle"
SCHOOL_ZONE_ENTRY = "school_zone_entry"
SCHOOL_ZONE_EXIT = "school_zone_exit"
POLICE_CAR = "police_car"
LAP_COUNTER = "lap_counter"
SHORTCUT_EXIT_TRIGGER = "shortcut_exit_trigger"
```

권장: **옵션 A** (메시지 패키지가 공용 의존성이라 모든 패키지에서 접근 가능).

---

## 6. `CMakeLists.txt`에 추가할 내용 (`ericar_msgs`)

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Perception.msg"
  "msg/DrivingOffset.msg"
  "msg/ControlState.msg"
  "msg/ActivePerceptions.msg"
  DEPENDENCIES std_msgs
)
```

---

## 7. 메시지 패키지 빌드 후 확인

```bash
cbp ericar_msgs
source ~/xycar_ws/install/setup.bash

# 메시지 타입 확인
ros2 interface show ericar_msgs/msg/Perception
ros2 interface show ericar_msgs/msg/DrivingOffset
ros2 interface show ericar_msgs/msg/ControlState
ros2 interface show ericar_msgs/msg/ActivePerceptions

# 모든 인터페이스 목록
ros2 interface list | grep ericar
```

---

## 8. Header 사용 여부

각 메시지에 `std_msgs/Header header`를 포함시킬지 결정.

| 장점 | 단점 |
|------|------|
| timestamp 추적 가능 (디버깅, 동기화) | 메시지 약간 무거워짐 |
| 향후 멀티 센서 동기화 시 유리 | 모든 발행자가 stamp를 채워야 함 |

**권장**: 처음에는 **Header 포함**으로 시작. 발행자가 stamp만 채워주면 되며, 구독자는 무시 가능. 나중에 빼는 것보다 추가하는 게 더 번거로움.

---

## 9. 다음 문서
- [`07_design_decisions.md`](07_design_decisions.md) — 설계 결정 배경
