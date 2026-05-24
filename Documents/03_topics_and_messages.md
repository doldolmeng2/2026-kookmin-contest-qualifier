# 03. 토픽과 메시지 (Topics and Messages)

## 1. 통신 설계 원칙

1. **인식 결과는 통합 메시지로 발행**: 인식 담당자가 만드는 모든 인식 결과는 하나의 커스텀 메시지(`Perception`)로 묶어 단일 토픽으로 발행한다. 비활성 인식기는 0/false로 채운다.
2. **주행 offset도 통합 토픽으로 발행**: 차선/라바콘/좌회전 offset을 같은 토픽 `/ericar/driving_offset`에 발행한다. control은 모드와 무관하게 같은 방식으로 처리.
3. **main이 single source of truth**: 모드, 스테이지, 활성 인식기 목록, 목표 차선은 모두 main이 발행.
4. **QoS는 best-effort + depth=10**: 실시간성이 중요하고, 메시지는 30Hz로 빠르게 갱신되므로 신뢰성보다 최신성이 우선.
5. **prefix는 `/ericar/`**: 팀 네임스페이스. 단, 시뮬레이터 제어 토픽 `/xycar_motor`는 예외(시뮬레이터 규약).

---

## 2. 전체 토픽 다이어그램

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           시뮬레이터 (Windows)                          │
└─────────────────────────────────────────────────────────────────────────┘
        │                            ▲
        │ 센서 데이터                │ /xycar_motor (XycarMotor)
        ▼                            │
   ┌──────────────────────────────────────────────────────────────┐
   │   (입력) 카메라, IMU, LiDAR 등                                │
   └──────────────────────────────────────────────────────────────┘
        │                            │                            │
        ▼                            ▼                            ▼
   ┌──────────────┐          ┌───────────────────┐         ┌────────────┐
   │ ericar_      │          │  ericar_driving   │         │ ericar_    │
   │ perception   │          │  (차선/라바콘/    │         │ main_      │
   │              │          │   좌회전 offset)  │         │ control    │
   └──────────────┘          └───────────────────┘         └────────────┘
        │                            │                            │
        │ /ericar/perception         │ /ericar/driving_offset     │
        │ (Perception, 30Hz)         │ (DrivingOffset, 30Hz)      │
        │                            │                            │
        ▼                            ▼                            │
   ┌──────────────────────────────────────────────────────────────┘
   │           main 노드 (상태 머신)                ◀─────────────┐
   └──────────────────────────────────────────────────────────────┤
        │                            │                            │
        │ /ericar/active_perceptions │ /ericar/target_lane        │ /ericar/control_state
        │ (ActivePerceptions, 변경시) │ (Int8, 변경시)             │ (ControlState, 30Hz)
        │ /ericar/current_stage       │                            │
        │ (String, 변경시)            │                            │
        ▼                            ▼                            ▼
   ┌──────────────┐          ┌───────────────────┐         ┌────────────┐
   │ ericar_      │          │  ericar_driving   │         │ control    │
   │ perception   │          │  (차선 인식)      │         │ (main 패키지│
   │ (각 인식기)  │          │                   │         │  내부)      │
   └──────────────┘          └───────────────────┘         └────────────┘
```

---

## 3. 토픽 요약표

| 토픽 | 발행자 | 구독자 | 메시지 타입 | 주기 | QoS |
|------|--------|--------|-------------|------|-----|
| `/ericar/perception` | ericar_perception | main | `ericar_msgs/Perception` | 30Hz | best-effort, depth=10 |
| `/ericar/driving_offset` | ericar_driving | control | `ericar_msgs/DrivingOffset` | 30Hz | best-effort, depth=10 |
| `/ericar/control_state` | main | control | `ericar_msgs/ControlState` | 30Hz | best-effort, depth=10 |
| `/ericar/target_lane` | main | ericar_driving | `std_msgs/Int8` | 변경 시 | best-effort, depth=10 |
| `/ericar/active_perceptions` | main | ericar_perception | `ericar_msgs/ActivePerceptions` | 변경 시 | best-effort, depth=10 |
| `/ericar/current_stage` | main | ericar_driving, 디버깅 | `std_msgs/String` | 변경 시 | best-effort, depth=10 |
| `/xycar_motor` | control | 시뮬레이터 | `xycar_msgs/XycarMotor` | 30Hz | (시뮬레이터 규약 따름) |

> **변경 시 발행 토픽에 대한 보강**: best-effort + depth=10이면 늦게 구독한 노드가 마지막 값을 못 받을 수 있다. 안정성을 위해 변경 시 발행 토픽은 **변경 시점에 5~10회 연속 발행**하거나, **별도로 1Hz heartbeat**를 추가하는 것을 권장한다. 또는 QoS의 reliable + transient_local 조합으로 latch처럼 동작시킬 수도 있다 (단, 구현 복잡도 증가).

---

## 4. 메시지 정의 (요약)

> 실제 `.msg` 파일 예시는 [`06_message_definitions.md`](06_message_definitions.md) 참고.

### 4-1. `ericar_msgs/Perception`

인식 담당이 모든 인식 결과를 묶어 발행하는 메시지.

| 필드 | 타입 | 값 의미 |
|------|------|---------|
| `traffic_light_start` | uint8 | 0:N/A, 1:빨강, 2:주황, 3:초록(직진), 4:좌회전 |
| `traffic_light_track` | uint8 | 동일 (트랙 위 신호등 = 경로선택 신호등) |
| `pedestrian_detected` | bool | 보행자가 위험 위치에 있을 때 true |
| `obstacle_vehicle_detected` | bool | 앞 차량 인식 |
| `obstacle_faster_lane` | uint8 | 0:미판단, 1:1차선 빠름, 2:2차선 빠름 |
| `obstacle_overtake_done` | bool | 느린 차량 추월 완료 |
| `school_zone_entry` | bool | 어린이 보호구역 시작 표시 |
| `school_zone_exit` | bool | 어린이 보호구역 종료 표시 |
| `police_car_status` | uint8 | 0:미판단, 1:있음, 2:없음 |
| `start_line_detected` | bool | 출발선(체스판) 인식 |
| `shortcut_exit_trigger` | bool | 지름길 두 번째 좌회전 트리거 |

**중요**:
- 인식기가 비활성 상태인 필드는 **항상 0/false**로 채운다.
- 활성 상태이지만 아직 판단 결과가 없는 경우(예: `obstacle_faster_lane` 측정 중)에도 0으로 둔다.

### 4-2. `ericar_msgs/DrivingOffset`

차선/라바콘/좌회전 담당이 발행하는 주행 offset.

| 필드 | 타입 | 값 의미 |
|------|------|---------|
| `offset` | float32 | 횡방향 오차 (양수/음수 부호 규칙은 팀 합의) |
| `source` | uint8 | 0:차선, 1:라바콘, 2:좌회전 (디버깅용) |

**중요**:
- 차선이 일시적으로 안 보일 때는 B 담당자가 내부적으로 이전 값을 활용하는 등 적절히 처리. 별도 valid 플래그 없음.

### 4-3. `ericar_msgs/ControlState`

main이 control에 보내는 모드 + 정지 명령.

| 필드 | 타입 | 값 의미 |
|------|------|---------|
| `mode` | uint8 | `MODE_*` 상수 (메시지 내부 상수로 정의) |
| `stop_request` | bool | true면 speed=0 |

상수 정의:
```
uint8 MODE_WAITING = 0
uint8 MODE_CONE_DRIVING = 1
uint8 MODE_LANE_DRIVING = 2
uint8 MODE_LANE_DRIVING_2 = 3
uint8 MODE_LANE_OBSTACLE_FOLLOW = 4
uint8 MODE_LANE_CHANGE = 5
uint8 MODE_LEFT_TURN = 6
```

> 모드를 string 대신 uint8 상수로 정의한 이유: 오타로 인한 버그 방지. 디버깅 시에는 `/ericar/current_stage`의 string 값을 활용.

### 4-4. `ericar_msgs/ActivePerceptions`

main이 인식 담당에 보내는 활성 인식기 목록.

| 필드 | 타입 | 값 의미 |
|------|------|---------|
| `names` | string[] | 활성화할 인식기 이름 배열 |

`names`에 들어갈 수 있는 문자열:
```
"traffic_light_start"
"traffic_light_track"
"pedestrian"
"obstacle_vehicle"
"school_zone_entry"
"school_zone_exit"
"police_car"
"lap_counter"
"shortcut_exit_trigger"
```

### 4-5. `std_msgs/Int8` (target_lane)

값: `1` (1차선) 또는 `2` (2차선)

### 4-6. `std_msgs/String` (current_stage)

값: 현재 스테이지의 enum 이름 문자열. 예: `"LANE_OBSTACLE_FOLLOW_SLOW"`

### 4-7. `xycar_msgs/XycarMotor` (motor cmd)

시뮬레이터 제공 메시지. 필드:
- `angle`: float32, -100 ~ 100 (음수=좌, 0=직진, 양수=우)
- `speed`: float32, -50 ~ 50 (음수=후진)

발행 예시 (Python):
```python
from xycar_msgs.msg import XycarMotor

self.motor_pub = self.create_publisher(XycarMotor, 'xycar_motor', 10)

def drive(self, angle, speed):
    motor_msg = XycarMotor()
    motor_msg.angle = float(angle)
    motor_msg.speed = float(speed)
    self.motor_pub.publish(motor_msg)
```

---

## 5. QoS 설정 가이드

ROS2 Python 기준 QoS 설정 예시:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# 발행자
self.pub = self.create_publisher(MyMsg, '/ericar/topic', qos)
# 구독자도 같은 qos로
self.sub = self.create_subscription(MyMsg, '/ericar/topic', self.cb, qos)
```

### 변경 시 발행 토픽 (target_lane, active_perceptions, current_stage)

이런 토픽들은 메시지가 자주 발행되지 않으므로 다음 중 하나를 권장:

**옵션 A. 변경 시 연속 발행 (가장 단순)**
```python
def publish_target_lane(self, lane: int):
    msg = Int8(); msg.data = lane
    for _ in range(5):
        self.target_lane_pub.publish(msg)
        time.sleep(0.01)
```

**옵션 B. 변경 시 + 1Hz heartbeat (안전)**
- 변경 시점에 한 번 발행
- 그리고 1초마다 timer로 현재 값을 다시 발행

**옵션 C. transient_local QoS (latch와 유사)**
```python
qos_latched = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
```
- 구독자가 늦게 들어와도 마지막 메시지를 받을 수 있음
- 단, reliable이라 약간 오버헤드 있음

권장: 처음에는 **옵션 A**로 시작하고, 문제가 보이면 **옵션 C**로 변경.

---

## 6. 노드 간 통신 흐름 (예시 시나리오)

### 시나리오: 보행자 인식 후 정지/재출발

```
1. main: stage = LANE_PEDESTRIAN_ZONE
2. main → /ericar/active_perceptions ["pedestrian"]
3. main → /ericar/control_state {mode: LANE_DRIVING, stop_request: false}

4. ericar_perception: 보행자 발견
5. ericar_perception → /ericar/perception {pedestrian_detected=true, ...}

6. main: pedestrian_detected=true 수신
7. main → /ericar/control_state {mode: LANE_DRIVING, stop_request: true}
8. control: speed=0 발행 → 차량 정지

9. ericar_perception: 보행자 클리어
10. ericar_perception → /ericar/perception {pedestrian_detected=false, ...}

11. main: pedestrian_detected=false 수신
12. main → /ericar/control_state {mode: LANE_DRIVING, stop_request: false}
13. control: 정상 속도/조향 재개

14. main: stage 전환 → LANE_OBSTACLE_DETECT
15. main → /ericar/active_perceptions ["obstacle_vehicle"]
```

### 시나리오: 지름길 진입 좌회전

```
1. main: stage = LANE_WAIT_LEFT_SIGNAL
2. main → /ericar/active_perceptions ["traffic_light_track"]
3. main → /ericar/control_state {mode: LANE_DRIVING, stop_request: true}

4. ericar_perception → /ericar/perception {traffic_light_track=4 (좌회전), ...}

5. main: 좌회전 신호 수신
6. main → stage 전환 → LEFT_TURN_SHORTCUT_ENTRY
7. main → /ericar/current_stage "LEFT_TURN_SHORTCUT_ENTRY"
8. main → /ericar/control_state {mode: LEFT_TURN, stop_request: false}
9. main → /ericar/active_perceptions []

10. ericar_driving: current_stage 수신 → 좌회전 모듈 호출
11. ericar_driving → /ericar/driving_offset {offset=..., source=2}

12. control: mode=LEFT_TURN + driving_offset 기반으로 angle/speed 계산
13. control → /xycar_motor

14. ericar_driving: IMU yaw가 yaw1 도달 → main에 알림
    (별도 토픽 또는 main이 직접 IMU 구독)
15. main → stage 전환 → LANE_SHORTCUT
```

---

## 7. 네임스페이스와 Domain ID

- ROS2 네임스페이스: `xycar` (프로젝트 룰)
- ROS_DOMAIN_ID: `7`

각 노드는 launch 파일이나 환경 변수로 도메인을 맞춰야 한다.

```bash
export ROS_DOMAIN_ID=7
```

토픽 이름의 `/ericar/...` prefix는 네임스페이스와 별개로 명시한다. 즉, 실제 토픽은 `/xycar/ericar/perception` 같은 형태가 아니라 `/ericar/perception`. 네임스페이스를 토픽 prefix로 활용하려면 launch에서 `remap`을 활용하면 된다.

> **결정 필요**: 토픽 이름에 ericar prefix를 직접 박을지, ROS2 namespace로 처리할지는 패키지 설정 시 협의. 기본 권장은 **토픽 이름에 ericar 직접 박기** (혼동 적음).

---

## 8. 다음 문서

- [`04_part_responsibilities.md`](04_part_responsibilities.md) — 파트별 구현 항목
- [`06_message_definitions.md`](06_message_definitions.md) — 실제 .msg 파일 예시
