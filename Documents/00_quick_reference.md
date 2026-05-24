# 00. 빠른 참조 (Quick Reference)

자주 찾아볼 정보 모음. 자세한 내용은 각 문서로 이동.

---

## 모드 (Mode) — 7개

| 모드 | 의미 | 권장 속도 |
|------|------|----------|
| `WAITING` (0) | 정지 대기 | 0 |
| `CONE_DRIVING` (1) | 라바콘 주행 | ~20 |
| `LANE_DRIVING` (2) | 일반 차선 주행 | ~30 |
| `LANE_DRIVING_2` (3) | 어린이 보호구역 (감속) | ~15 |
| `LANE_OBSTACLE_FOLLOW` (4) | 방해차량 느린 추종 | ~10 |
| `LANE_CHANGE` (5) | 차선 변경 | ~25 |
| `LEFT_TURN` (6) | IMU 좌회전 | ~15 |

---

## 스테이지 (Stage) — 15개

```
WAITING_START → CONE_DRIVING → LANE_AFTER_CONE → LANE_PEDESTRIAN_ZONE
  → LANE_OBSTACLE_DETECT → LANE_OBSTACLE_FOLLOW_SLOW
  → LANE_OBSTACLE_CHASE → LANE_OBSTACLE_OVERTAKE
  → LANE_SCHOOL_ZONE_ENTRY_WATCH → SCHOOL_ZONE_DRIVING
  → LANE_BEFORE_LAP_COUNT
      ├ lap<3 → LANE_BEFORE_ROUTE_SELECT
      │           ├ 경찰차 있음 → LANE_PEDESTRIAN_ZONE
      │           └ 경찰차 없음 → LANE_WAIT_LEFT_SIGNAL
      │                            → LEFT_TURN_SHORTCUT_ENTRY
      │                            → LANE_SHORTCUT
      │                            → LEFT_TURN_SHORTCUT_EXIT
      │                            → SCHOOL_ZONE_DRIVING
      └ lap==3 → 시뮬레이션 자동 종료
```

---

## 토픽 — 7개

| 토픽 | 발행 | 구독 | 메시지 | 주기 |
|------|-----|-----|--------|------|
| `/ericar/perception` | A | C | `ericar_msgs/Perception` | 30Hz |
| `/ericar/driving_offset` | B | C | `ericar_msgs/DrivingOffset` | 30Hz |
| `/ericar/control_state` | C(main) | C(control) | `ericar_msgs/ControlState` | 30Hz |
| `/ericar/target_lane` | C(main) | B | `std_msgs/Int8` | 변경 시 |
| `/ericar/active_perceptions` | C(main) | A | `ericar_msgs/ActivePerceptions` | 변경 시 |
| `/ericar/current_stage` | C(main) | B + 디버깅 | `std_msgs/String` | 변경 시 |
| `/xycar_motor` | C(control) | 시뮬레이터 | `xycar_msgs/XycarMotor` | 30Hz |

QoS: best-effort, KEEP_LAST, depth=10

---

## 신호등 enum

```
0: 인식 안됨
1: 빨강
2: 주황
3: 초록 (직진)
4: 좌회전
```

## 차선 enum (obstacle_faster_lane)

```
0: 미판단
1: 1차선 빠름
2: 2차선 빠름
```

## 경찰차 enum

```
0: 미판단
1: 있음
2: 없음
```

## 드라이빙 source enum

```
0: 차선
1: 라바콘
2: 좌회전
```

---

## 정지 (stop_request) 발생 조건

| 조건 | 어디서 |
|------|-------|
| 보행자 감지 | `LANE_PEDESTRIAN_ZONE` |
| 트랙 신호 빨강/주황 | `LANE_AFTER_CONE`, `LANE_BEFORE_ROUTE_SELECT`(경찰차 있는 경우) |
| 좌회전 신호 대기 | `LANE_WAIT_LEFT_SIGNAL` |
| lap == 3 완주 | 모든 스테이지 (안전) |

---

## 자주 쓰는 명령어

```bash
# 빌드
cw                          # cd ~/xycar_ws
cbs                         # 전체 빌드
cbp <패키지명>              # 패키지 단일 빌드

# 환경
source ~/xycar_ws/install/setup.bash
export ROS_DOMAIN_ID=7

# 실행
ros2 launch ericar_main_control all.launch.py

# 디버깅
ros2 topic list
ros2 topic echo /ericar/current_stage
ros2 topic echo /ericar/control_state
ros2 topic hz /ericar/perception
ros2 interface show ericar_msgs/msg/Perception
rqt_graph
```

---

## 미정 사항 (협의 필요)

자세한 내용은 [`07_design_decisions.md`](07_design_decisions.md) 하단.

- offset 부호 규칙 (좌+ / 우+)
- 모드별 속도 값 튜닝
- target_lane 정의 (안쪽=1, 바깥=2?)
- 차선 변경 완료 신호 토픽
- 좌회전 완료 신호 토픽
- 어린이 보호구역 정확한 속도 제한
- 보행자 클리어 후 즉시 vs 시간 대기

---

## 문서 내비게이션

- [`01_mission_overview.md`](01_mission_overview.md) — 미션 분석
- [`02_mode_and_stage.md`](02_mode_and_stage.md) — 모드/스테이지 상세
- [`03_topics_and_messages.md`](03_topics_and_messages.md) — 토픽/메시지 상세
- [`04_part_responsibilities.md`](04_part_responsibilities.md) — 파트별 구현 가이드
- [`05_package_structure.md`](05_package_structure.md) — 패키지/빌드/실행
- [`06_message_definitions.md`](06_message_definitions.md) — .msg 파일 예시
- [`07_design_decisions.md`](07_design_decisions.md) — 설계 결정 배경
