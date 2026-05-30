# 04. 파트별 구현 가이드 (Part Responsibilities)

3명이 다음 3개 파트를 나누어 구현한다. 메시지 패키지(`ericar_msgs`)는 공용이라 별도로 분리.

| 파트 | 패키지 | 담당 영역 |
|------|--------|-----------|
| A | `perception` | 모든 인식 (신호등, 보행자, 차량, 노면 표시 등) |
| B | `driving` | 차선 인식 + 라바콘 인식 + 좌회전 offset 계산 |
| C | `main` | 상태 머신(main) + 최종 제어(control) |
| - | `function` | 센서 뷰어 + 수동 조종 (테스트/디버깅용) |

---

## A. 인식 담당 (`perception`)

### 책임
모든 인식 관련 처리. 카메라/LiDAR/IMU 등 센서 데이터를 받아 `Perception` 메시지로 통합 발행.

### 구현해야 할 인식기

| 인식기 이름 | 입력 | 출력 필드 | 사용 시점 |
|-----------|------|----------|-----------|
| `traffic_light_start` | 카메라 | `traffic_light_start` | 시작 대기 |
| `traffic_light_track` | 카메라 | `traffic_light_track` | 트랙 위 신호등 (lap1 진입, 경로선택) |
| `pedestrian` | 카메라 또는 LiDAR | `pedestrian_detected` | 보행자 구간 |
| `obstacle_vehicle` | LiDAR + 카메라 | `obstacle_vehicle_detected`, `obstacle_faster_lane`, `obstacle_overtake_done` | 방해차량 구간 |
| `school_zone_entry` | 카메라 (노면 색) | `school_zone_entry` | 어린이 보호구역 시작 표시 인식 |
| `school_zone_exit` | 카메라 (노면 표시) | `school_zone_exit` | 어린이 보호구역 종료 표시 인식 |
| `police_car` | 카메라 | `police_car_status` | 경로선택 직전 |
| `lap_counter` | 카메라 | `start_line_detected` | 출발선(체스판) 인식 |
| `shortcut_exit_trigger` | 카메라 또는 차선 패턴 | `shortcut_exit_trigger` | 지름길에서 두 번째 좌회전 직전 |

### 구독해야 할 토픽

| 토픽 | 메시지 타입 | 용도 |
|------|-------------|------|
| `/ericar/active_perceptions` | `ericar_msgs/ActivePerceptions` | 자기 이름이 있을 때만 실제 처리 |
| (센서 토픽들) | (시뮬레이터 규약) | 카메라, LiDAR 등 |

### 발행해야 할 토픽

| 토픽 | 메시지 타입 | 주기 |
|------|-------------|------|
| `/ericar/perception` | `ericar_msgs/Perception` | 30Hz |

### 구현 가이드

1. **단일 노드 vs 다중 노드**: 인식기마다 노드를 나누는 것보다, **하나의 노드 안에서 여러 인식 함수를 호출하고 통합 메시지로 발행**하는 게 단순. 노드 분리는 카메라 frame을 여러 번 처리할 때 비효율적.

2. **비활성 인식기 처리**: `active_perceptions`에 자기 이름이 없으면 해당 처리를 스킵하고 결과 필드를 0/false로 둠.
   ```python
   def perception_loop(self):
       msg = Perception()  # 모든 필드 0/false로 초기화
       active = self.active_perceptions  # set 형태로 유지

       if "traffic_light_start" in active:
           msg.traffic_light_start = self.detect_start_light()

       if "pedestrian" in active:
           msg.pedestrian_detected = self.detect_pedestrian()

       # ... 다른 인식기들

       self.perception_pub.publish(msg)
   ```

3. **신호등 enum 매핑**:
   ```
   0: 인식 안됨/대상 아님
   1: 빨강
   2: 주황 (또는 황색)
   3: 초록 (직진)
   4: 좌회전
   ```

4. **3-state 처리** (`obstacle_faster_lane`, `police_car_status`): "아직 판단 안 함"과 "확정적으로 없음"을 구분. main이 잘못된 판단을 하지 않도록 확신이 들 때만 1/2를 채움.

5. **연속 발행 권장**: 발행 주기 30Hz를 일정하게 유지. 인식 결과가 변하지 않아도 매 프레임 같은 값을 발행.

### 체크리스트
- [ ] `ericar_msgs/Perception` 메시지 사용
- [ ] `/ericar/perception` 30Hz 발행
- [ ] `/ericar/active_perceptions` 구독 및 활성 인식기 관리
- [ ] 9개 인식기 함수 구현
- [ ] 신호등 enum 값 정확히 매핑 (0~4)
- [ ] 3-state 필드는 확신이 들 때만 1/2 채움
- [ ] 비활성 인식기 필드는 0/false로 유지

---

## B. 주행 담당 (`driving`)

### 책임
차선/라바콘/좌회전에 따른 주행 offset 계산. `DrivingOffset` 메시지로 단일 토픽 발행.

### 구현해야 할 모듈

| 모듈 | 입력 | 출력 (source) | 사용 시점 |
|------|------|---------------|-----------|
| 차선 인식 | 카메라 | offset, source=0 | 모든 차선 주행 모드 |
| 라바콘 인식 | LiDAR (또는 카메라) | offset, source=1 | `CONE_DRIVING` 모드 |
| 좌회전 offset | IMU yaw + current_stage | offset, source=2 | `LEFT_TURN` 모드 |

### 구독해야 할 토픽

| 토픽 | 메시지 타입 | 용도 |
|------|-------------|------|
| `/ericar/target_lane` | `std_msgs/Int8` | 추종할 차선 (1 또는 2) |
| `/ericar/current_stage` | `std_msgs/String` | 좌회전 모드일 때 어느 좌회전인지 판단 |
| (센서 토픽들) | (시뮬레이터 규약) | 카메라, LiDAR, IMU |

### 발행해야 할 토픽

| 토픽 | 메시지 타입 | 주기 |
|------|-------------|------|
| `/ericar/driving_offset` | `ericar_msgs/DrivingOffset` | 30Hz |

### 구현 가이드

1. **모드별 source 결정**: control이 어느 source의 offset인지 알 수 있도록 정확히 채움. 라바콘 종료 시 자연스럽게 차선 offset으로 전환.

2. **차선 미검출 처리**:
   - 일시적 미검출: 이전 offset 값 유지 또는 점진적 감쇠
   - 지속적 미검출 (지름길 차선 안 보이는 구간): 직진 또는 마지막 차선 정보 기반으로 적절한 offset 발행
   - 별도의 valid 플래그는 사용하지 않음

3. **target_lane 처리**:
   - 1차선 추종: 1차선 중심 기준 offset 발행
   - 2차선 추종: 2차선 중심 기준 offset 발행
   - 변경 시점에는 부드럽게(offset jump 완화) 처리 권장

4. **좌회전 offset 계산**:
   ```python
   def calc_left_turn_offset(self, current_yaw, target_yaw):
       error = target_yaw - current_yaw
       # error를 적절한 offset 값으로 환산
       return offset
   ```
   - 좌회전 모드 진입 시점에 현재 yaw를 기록해두고 목표 yaw₁, yaw₂를 계산.
   - current_stage가 `LEFT_TURN_SHORTCUT_ENTRY`인지 `LEFT_TURN_SHORTCUT_EXIT`인지에 따라 어느 목표를 쓸지 결정.

5. **좌회전 완료 알림 방법** (협의 필요):
   - 옵션 a: B가 `/ericar/left_turn_done` 같은 토픽으로 알림
   - 옵션 b: main이 직접 IMU yaw를 구독해 판단
   - 어느 쪽이든 좋지만 **a를 권장** (책임 분리).

6. **차선 변경 완료 알림**:
   - target_lane이 바뀐 후 일정 시간 내 양 차선 offset을 모니터링.
   - 새 target_lane 기준 offset이 충분히 작아지면 완료.
   - `/ericar/lane_change_done` 같은 별도 토픽으로 main에 알림 (또는 내부적으로 처리하지 않고 main이 알아서 처리)

### 체크리스트
- [ ] `ericar_msgs/DrivingOffset` 사용
- [ ] `/ericar/driving_offset` 30Hz 발행
- [ ] `/ericar/target_lane` 구독 및 차선 전환 처리
- [ ] `/ericar/current_stage` 구독
- [ ] 차선 인식 (source=0), 1·2차선 모두 처리 가능
- [ ] 라바콘 인식 (source=1)
- [ ] 좌회전 offset (source=2), yaw1·yaw2 모두 처리 가능
- [ ] 차선 미검출 시 내부적으로 적절히 대응
- [ ] 좌회전 완료 알림 메커니즘 구현 (협의 후)
- [ ] 차선 변경 완료 알림 메커니즘 구현 (협의 후)

---

## C. main + control 담당 (`main`)

### 책임
- **main**: 상태 머신(스테이지 + 모드 관리), lap 카운트, 차선 변경 결정, 인식 활성화 관리
- **control**: 모드와 offset을 받아 angle/speed 결정 후 `/xycar_motor` 발행

### 단일 노드 vs 분리 노드
- **권장**: **단일 노드** (또는 같은 패키지의 두 노드). main과 control 사이는 토픽보다 직접 함수 호출이 더 빠르고 단순.
- 분리하더라도 같은 패키지 내에서 토픽으로 연결 (`/ericar/control_state`).

### 구현해야 할 기능

#### main 측
1. 스테이지 머신 (`STAGE_TO_MODE`, `STAGE_PERCEPTIONS` 딕셔너리 기반)
2. 인식 결과 구독 → 스테이지 전환 조건 평가
3. lap 카운터 (0 → 3, 3 도달 시 종료)
4. 활성 인식기 목록 발행
5. target_lane 발행 (차선 변경 결정)
6. control_state 발행 (모드 + 정지)
7. current_stage 발행 (디버깅용)
8. 좌회전 완료/차선 변경 완료 신호 처리

#### control 측
9. mode와 offset 받아 angle/speed 결정
10. 모드별 속도 프로파일 관리
11. `/xycar_motor` 발행 (30Hz)

### 구독해야 할 토픽

| 토픽 | 메시지 타입 | 용도 |
|------|-------------|------|
| `/ericar/perception` | `ericar_msgs/Perception` | 모든 인식 결과 |
| `/ericar/driving_offset` | `ericar_msgs/DrivingOffset` | control용 offset |
| (선택) `/ericar/left_turn_done` | (협의) | 좌회전 완료 |
| (선택) `/ericar/lane_change_done` | (협의) | 차선 변경 완료 |

### 발행해야 할 토픽

| 토픽 | 메시지 타입 | 주기 |
|------|-------------|------|
| `/ericar/control_state` | `ericar_msgs/ControlState` | 30Hz |
| `/ericar/active_perceptions` | `ericar_msgs/ActivePerceptions` | 변경 시 (+ 권장: 1Hz heartbeat) |
| `/ericar/target_lane` | `std_msgs/Int8` | 변경 시 |
| `/ericar/current_stage` | `std_msgs/String` | 변경 시 |
| `/xycar_motor` | `xycar_msgs/XycarMotor` | 30Hz |

### 구현 가이드

1. **상태 머신 구조**:
   ```python
   class MainNode(Node):
       def __init__(self):
           super().__init__('ericar_main')
           self.stage = "WAITING_START"
           self.lap = 0
           self.stop_request = False
           self.target_lane = 1
           # ...

       def perception_callback(self, msg):
           # stage별 전환 조건 평가
           handler = getattr(self, f"on_{self.stage.lower()}")
           handler(msg)

       def on_waiting_start(self, msg):
           if msg.traffic_light_start == 3:  # 초록
               self.transition_to("CONE_DRIVING")

       def on_lane_pedestrian_zone(self, msg):
           self.stop_request = msg.pedestrian_detected
           if not msg.pedestrian_detected:
               # 보행자 클리어 후 다음 단계는 시간 기반? 또는 즉시 전환?
               # 본 설계에서는 즉시 전환 가정
               self.transition_to("LANE_OBSTACLE_DETECT")

       def transition_to(self, new_stage):
           self.stage = new_stage
           self.publish_current_stage()
           self.publish_active_perceptions()
           # control_state는 30Hz 타이머에서 발행
   ```

2. **30Hz 타이머**:
   ```python
   self.timer = self.create_timer(1.0/30.0, self.tick)

   def tick(self):
       # control_state 발행
       state = ControlState()
       state.mode = STAGE_TO_MODE_ENUM[self.stage]
       state.stop_request = self.stop_request
       self.control_state_pub.publish(state)

       # control 측: angle/speed 계산 + xycar_motor 발행
       self.control_tick()
   ```

3. **lap 카운트 처리**:
   ```python
   def on_lane_before_lap_count(self, msg):
       if msg.start_line_detected:
           self.lap += 1
           if self.lap >= 3:
               self.stop_request = True  # 안전
               # 시뮬레이션이 자동 종료됨
           else:
               self.transition_to("LANE_BEFORE_ROUTE_SELECT")
   ```

4. **차선 변경 결정**:
   - `LANE_OBSTACLE_FOLLOW_SLOW` 상태에서 `obstacle_faster_lane`이 1 또는 2가 되면 main이 결정.
   - `target_lane`을 더 빠른 차선으로 변경하고 `LANE_OBSTACLE_CHASE` 진입.
   - `obstacle_overtake_done`이 true가 되면 `target_lane`을 비어있는 차선으로 변경하고 `LANE_OBSTACLE_OVERTAKE` 진입.

5. **속도 프로파일** (control 측, 모드별):
   ```python
   MODE_SPEED = {
       "WAITING": 0,
       "CONE_DRIVING": 20,
       "LANE_DRIVING": 30,
       "LANE_DRIVING_2": 15,         # 어린이 보호구역 감속
       "LANE_OBSTACLE_FOLLOW": 10,    # 느린 추종
       "LANE_CHANGE": 25,             # 약간 감속하여 안정적 변경
       "LEFT_TURN": 15,               # 좌회전 시 감속
   }
   ```
   숫자는 추후 튜닝.

6. **stop_request 우선**:
   ```python
   def control_tick(self):
       if self.stop_request:
           self.drive(angle=0.0, speed=0.0)
           return

       offset = self.latest_offset.offset  # 최신 driving_offset 값
       angle = self.angle_pid(offset)
       speed = MODE_SPEED[self.current_mode]
       self.drive(angle=angle, speed=speed)
   ```

### 체크리스트
- [ ] `STAGE_TO_MODE`, `STAGE_PERCEPTIONS` 딕셔너리 정의
- [ ] 15개 스테이지 핸들러 구현
- [ ] `/ericar/perception` 구독 및 전환 로직
- [ ] `/ericar/driving_offset` 구독
- [ ] `/ericar/control_state` 30Hz 발행
- [ ] `/ericar/active_perceptions` 변경 시 발행
- [ ] `/ericar/target_lane` 변경 시 발행
- [ ] `/ericar/current_stage` 변경 시 발행
- [ ] `/xycar_motor` 30Hz 발행
- [ ] lap 카운트 (0~3)
- [ ] stop_request 로직 (보행자, 빨간불, 좌회전 대기)
- [ ] 모드별 속도 프로파일
- [ ] 좌회전 완료/차선 변경 완료 처리

---

## 공통 규칙

### 1. ROS_DOMAIN_ID
```bash
export ROS_DOMAIN_ID=7
```
모든 노드 실행 전 설정.

### 2. 단축 명령어 (~/.bashrc)
```bash
alias cw='cd ~/xycar_ws'
alias cbs='cd ~/xycar_ws && colcon build --symlink-install'
cbp() {
    cd ~/xycar_ws && colcon build --symlink-install --packages-select "$1"
}
```

### 3. 메시지 패키지 의존
모든 패키지의 `package.xml`에 다음 추가:
```xml
<depend>ericar_msgs</depend>
```

`CMakeLists.txt` (C++) 또는 `setup.py` (Python)에도 의존성 추가.

### 4. 코드 스타일
- Python: PEP 8 기본
- ROS2 노드 클래스명: PascalCase (예: `EricarMainNode`)
- 토픽명: snake_case (예: `/ericar/driving_offset`)

### 5. 공동 결정 사항 (협의 필요)
- [ ] 차선 변경 완료 신호 방법 (B → main)
- [ ] 좌회전 완료 신호 방법 (B → main)
- [ ] offset 부호 규칙 (좌+/우-? 또는 반대?)
- [ ] 모드별 속도 값 튜닝
- [ ] target_lane 1차선/2차선 어느 쪽이 안쪽인지 (안쪽=1, 바깥=2 권장)

---

## D. 센서 뷰어 + 수동 조종 (`function`)

### 책임
대회 준비 및 디버깅용 유틸리티 패키지. 각 센서 데이터를 시각적으로 확인하고, 차량을 수동으로 조종할 수 있다.

### 구현해야 할 노드

| 노드 | 구독 토픽 | 기능 |
|------|----------|------|
| `cam_viewer` | 카메라 토픽 | 카메라 영상 실시간 표시 |
| `lidar_viewer` | LiDAR 토픽 | LiDAR 포인트클라우드 시각화 |
| `imu_viewer` | IMU 토픽 | IMU 데이터 (yaw 등) 표시 |
| `motor_viewer` | `/xycar_motor` | 현재 모터 명령 표시 |
| `manual_control` | (키보드 입력) | 키보드로 `/xycar_motor` 직접 발행 |

### 실행
```bash
# 개별 뷰어
ros2 run function cam_viewer
ros2 run function lidar_viewer
ros2 run function imu_viewer
ros2 run function motor_viewer

# 수동 조종
ros2 run function manual_control
```

---

## 다음 문서
- [`05_package_structure.md`](05_package_structure.md) — ROS2 패키지 구조와 빌드 방법
- [`06_message_definitions.md`](06_message_definitions.md) — .msg 파일 실제 예시
