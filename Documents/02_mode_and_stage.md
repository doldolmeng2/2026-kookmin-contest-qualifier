# 02. 모드와 스테이지 (Mode and Stage)

## 1. 두 층(layer)의 상태 관리

전체 주행 상태를 **모드(Mode)** 와 **스테이지(Stage)** 라는 두 층으로 분리해 관리한다.

### 모드 (Mode)
- **누가 사용**: `control` 노드
- **목적**: "차량을 어떻게 움직일지" (속도 프로파일, 조향 처리 방식)를 결정
- **개수**: 7개

### 스테이지 (Stage)
- **누가 사용**: `main` 노드 내부, 그리고 좌회전 offset 계산 등 일부 파트
- **목적**: "지금이 어느 시점인지"를 표현해 인식기 활성화와 모드 전환의 단위로 활용
- **개수**: 15개 (현 시점 기준)

> **왜 두 층으로 나누었나?**
> 같은 `LANE_DRIVING` 모드에서도 보행자 인식이 필요한 시점, 방해차량 인식이 필요한 시점, lap 카운터가 필요한 시점이 모두 다르다. 모드만으로는 이 차이를 표현할 수 없기 때문에, 더 세분화된 스테이지를 두고 각 스테이지가 어떤 모드와 어떤 인식기를 쓰는지 매핑한다.

---

## 2. 모드 (Mode) 상세

| 모드 | control 동작 | 사용처 |
|------|--------------|--------|
| `WAITING` | speed=0, angle=0 | 시작 대기, 좌회전 신호 대기, 빨간불 대기 등 |
| `CONE_DRIVING` | 라바콘 offset 기반 주행, 중속 | 출발 직후 라바콘 구간 |
| `LANE_DRIVING` | 차선 offset 기반 주행, 일반 속도 | 트랙 위 대부분의 차선 주행 |
| `LANE_DRIVING_2` | 차선 offset 기반 주행, **감속** | 어린이 보호구역 |
| `LANE_OBSTACLE_FOLLOW` | 차선 offset 기반 주행, **저속** | 방해차량 속도 비교 중 느린 추종 |
| `LANE_CHANGE` | 차선 offset 기반 주행, 부드러운 전환 | 차선 변경 (target_lane 변경 직후) |
| `LEFT_TURN` | 좌회전 모듈이 발행하는 offset 기반 주행 | 지름길 진입/탈출 좌회전 |

### 정지(stop_request) 처리
- 정지는 별도 모드로 두지 않는다.
- `main`이 `/ericar/control_state.stop_request = true`를 발행하면 control은 모드와 무관하게 `speed=0`을 출력한다.
- 이렇게 한 이유는 "정지"가 모드와 직교(orthogonal)하는 횡단 동작이기 때문이다.

---

## 3. 스테이지 (Stage) 전체 목록

```python
# main이 관리하는 스테이지 enum (개념적 표현)
STAGES = [
    # === 시작 / lap1 공통 ===
    "WAITING_START",
    "CONE_DRIVING",
    "LANE_AFTER_CONE",
    "LANE_PEDESTRIAN_ZONE",

    # === 방해차량 추월 (4단계) ===
    "LANE_OBSTACLE_DETECT",
    "LANE_OBSTACLE_FOLLOW_SLOW",
    "LANE_OBSTACLE_CHASE",
    "LANE_OBSTACLE_OVERTAKE",

    # === 어린이 보호구역 + lap 카운트 ===
    "LANE_SCHOOL_ZONE_ENTRY_WATCH",
    "SCHOOL_ZONE_DRIVING",
    "LANE_BEFORE_LAP_COUNT",

    # === lap2/lap3 경로 선택 ===
    "LANE_BEFORE_ROUTE_SELECT",
    "LANE_WAIT_LEFT_SIGNAL",

    # === 지름길 ===
    "LEFT_TURN_SHORTCUT_ENTRY",
    "LANE_SHORTCUT",
    "LEFT_TURN_SHORTCUT_EXIT",
]
```

---

## 4. 스테이지 → 모드 / 활성 인식기 매핑

> 이 두 딕셔너리는 `main` 노드 내부에 정의되어 single source of truth로 동작한다.

```python
STAGE_TO_MODE = {
    "WAITING_START":                "WAITING",
    "CONE_DRIVING":                 "CONE_DRIVING",
    "LANE_AFTER_CONE":              "LANE_DRIVING",
    "LANE_PEDESTRIAN_ZONE":         "LANE_DRIVING",
    "LANE_OBSTACLE_DETECT":         "LANE_DRIVING",
    "LANE_OBSTACLE_FOLLOW_SLOW":    "LANE_OBSTACLE_FOLLOW",
    "LANE_OBSTACLE_CHASE":          "LANE_CHANGE",  # 변경 완료 후 LANE_DRIVING로
    "LANE_OBSTACLE_OVERTAKE":       "LANE_CHANGE",
    "LANE_SCHOOL_ZONE_ENTRY_WATCH": "LANE_DRIVING",
    "SCHOOL_ZONE_DRIVING":          "LANE_DRIVING_2",
    "LANE_BEFORE_LAP_COUNT":        "LANE_DRIVING",
    "LANE_BEFORE_ROUTE_SELECT":     "LANE_DRIVING",
    "LANE_WAIT_LEFT_SIGNAL":        "LANE_DRIVING",  # stop_request로 정지 처리
    "LEFT_TURN_SHORTCUT_ENTRY":     "LEFT_TURN",
    "LANE_SHORTCUT":                "LANE_DRIVING",
    "LEFT_TURN_SHORTCUT_EXIT":      "LEFT_TURN",
}

STAGE_PERCEPTIONS = {
    "WAITING_START":                ["traffic_light_start"],
    "CONE_DRIVING":                 [],
    "LANE_AFTER_CONE":              ["traffic_light_track"],
    "LANE_PEDESTRIAN_ZONE":         ["pedestrian"],
    "LANE_OBSTACLE_DETECT":         ["obstacle_vehicle"],
    "LANE_OBSTACLE_FOLLOW_SLOW":    ["obstacle_vehicle"],
    "LANE_OBSTACLE_CHASE":          ["obstacle_vehicle"],
    "LANE_OBSTACLE_OVERTAKE":       [],
    "LANE_SCHOOL_ZONE_ENTRY_WATCH": ["school_zone_entry"],
    "SCHOOL_ZONE_DRIVING":          ["school_zone_exit"],
    "LANE_BEFORE_LAP_COUNT":        ["lap_counter"],
    "LANE_BEFORE_ROUTE_SELECT":     ["police_car", "traffic_light_track"],
    "LANE_WAIT_LEFT_SIGNAL":        ["traffic_light_track"],
    "LEFT_TURN_SHORTCUT_ENTRY":     [],
    "LANE_SHORTCUT":                ["shortcut_exit_trigger"],
    "LEFT_TURN_SHORTCUT_EXIT":      [],
}
```

---

## 5. 스테이지 전환 다이어그램

### 5-1. lap1 흐름

```
[WAITING_START]
   │ 시작 신호등 초록불 인식
   ▼
[CONE_DRIVING]
   │ 라바콘 종료 + 차선 검출
   ▼
[LANE_AFTER_CONE]
   │ 트랙 신호등 초록(직진/좌회전) 인식
   │ (빨강/주황이면 stop_request=true로 대기)
   ▼
[LANE_PEDESTRIAN_ZONE]
   │ 보행자 미감지 (pedestrian_detected=false)
   │ (감지 중에는 stop_request=true)
   ▼
[LANE_OBSTACLE_DETECT]
   │ 앞 차량 인식 (obstacle_vehicle_detected=true)
   ▼
[LANE_OBSTACLE_FOLLOW_SLOW]
   │ 더 빠른 차선 판단 완료 (obstacle_faster_lane ∈ {1, 2})
   ▼
[LANE_OBSTACLE_CHASE]
   │ target_lane = 빠른 차량이 있는 차선
   │ 인식 담당이 "느린 차량 추월 완료" 발행 (obstacle_overtake_done=true)
   ▼
[LANE_OBSTACLE_OVERTAKE]
   │ target_lane = 비어있는 차선(원래 느린 차가 있던 곳)
   │ 차선 변경 완료 (B 담당이 판단, 별도 토픽 또는 내부 처리)
   ▼
[LANE_SCHOOL_ZONE_ENTRY_WATCH]
   │ 어린이 보호구역 시작 표시 인식 (school_zone_entry=true)
   ▼
[SCHOOL_ZONE_DRIVING]
   │ 어린이 보호구역 종료 표시 인식 (school_zone_exit=true)
   ▼
[LANE_BEFORE_LAP_COUNT]
   │ 출발선 인식 (start_line_detected=true) → lap += 1
   │
   ├─ lap < 3 → [LANE_BEFORE_ROUTE_SELECT]
   └─ lap == 3 → 시뮬레이션 자동 종료
```

### 5-2. lap2 / lap3 흐름

```
[LANE_BEFORE_ROUTE_SELECT]
   │ police_car_status, traffic_light_track 동시 확인
   │
   ├─ 경찰차 있음 (police_car_status=1)
   │    ├─ 신호 빨강 → stop_request=true (대기)
   │    └─ 신호 초록(직진) → [LANE_PEDESTRIAN_ZONE] (lap1과 동일 흐름)
   │
   └─ 경찰차 없음 (police_car_status=2)
        │
        ▼
[LANE_WAIT_LEFT_SIGNAL]
   │ stop_request=true (필요 시 — 이미 좌회전 신호면 바로 통과)
   │ traffic_light_track == 좌회전 신호 (4)
   ▼
[LEFT_TURN_SHORTCUT_ENTRY]
   │ 좌회전 모듈이 yaw1 목표로 offset 발행
   │ IMU yaw가 yaw1에 도달
   ▼
[LANE_SHORTCUT]
   │ 차선 인식 담당이 offset 발행 (안 보이는 구간은 자체 대응)
   │ shortcut_exit_trigger=true
   ▼
[LEFT_TURN_SHORTCUT_EXIT]
   │ 좌회전 모듈이 yaw2 목표로 offset 발행
   │ IMU yaw가 yaw2에 도달
   ▼
[SCHOOL_ZONE_DRIVING]
   │ (지름길 탈출 직후 곧바로 어린이 보호구역으로 진입)
   │ school_zone_exit=true
   ▼
[LANE_BEFORE_LAP_COUNT]
   │ 출발선 인식 → lap += 1 → 다음 lap 또는 종료
```

---

## 6. 방해차량 추월 4단계 상세

방해차량 미션이 가장 복잡하므로 별도로 정리한다.

| 스테이지 | target_lane | 모드 | 의미 |
|---------|-------------|------|------|
| `LANE_OBSTACLE_DETECT` | 현재 차선 유지 | `LANE_DRIVING` | 앞 차량 인식 대기 |
| `LANE_OBSTACLE_FOLLOW_SLOW` | 현재 차선 유지 | `LANE_OBSTACLE_FOLLOW` | 천천히 따라가며 양 차선 속도 비교 |
| `LANE_OBSTACLE_CHASE` | **빠른 차량 차선** | `LANE_CHANGE` → 도착 후 `LANE_DRIVING` | 빠른 차량을 따라가며 느린 차량 추월 시점 대기 |
| `LANE_OBSTACLE_OVERTAKE` | **비어있는 차선** (느린 차량이 있던 곳) | `LANE_CHANGE` → 도착 후 `LANE_DRIVING` | 추월 직후 비어있는 차선으로 옮겨감 |

### 차선 변경 완료 판정
- 차선 변경 완료는 차선/라바콘 담당(B)이 판단한다.
- 양 차선 offset을 모니터링하다가 target_lane 기준 offset이 충분히 작아지면 완료.
- 별도의 완료 토픽을 둘지(`/ericar/lane/change_complete`), B 내부 처리만으로 갈지는 B 담당자가 결정.
  - 기본 방향: B가 별도 토픽으로 알린다 → main이 다음 스테이지로 전환한다.

### 차선 변경 미발생 케이스
- 현재 차선의 차량이 더 빠르다고 판단되면 `LANE_OBSTACLE_CHASE`의 target_lane은 현재 차선과 같으므로 사실상 차선 변경 없이 따라가기만 한다.
- 그래도 모드는 `LANE_CHANGE` → `LANE_DRIVING` 동일하게 흘러도 무방하며, control이 부드럽게 처리.

---

## 7. 정지(stop_request)가 필요한 시점

스테이지와 무관하게 `stop_request=true`로 정지하는 조건은 다음과 같다.

| 조건 | 어디서 발생 |
|------|-------------|
| 보행자 감지 (`pedestrian_detected=true`) | `LANE_PEDESTRIAN_ZONE` |
| 트랙 신호 빨강/주황 | `LANE_AFTER_CONE`, `LANE_BEFORE_ROUTE_SELECT` (경찰차 있는 경우) |
| 좌회전 신호 대기 | `LANE_WAIT_LEFT_SIGNAL` |

해당 조건이 사라지면 `stop_request=false`로 풀고 차량은 자연스럽게 재출발한다.

---

## 8. 좌회전 모드 처리

### 좌회전이 2번 발생함
- `LEFT_TURN_SHORTCUT_ENTRY` (지름길 진입): 목표 yaw₁
- `LEFT_TURN_SHORTCUT_EXIT` (지름길 탈출): 목표 yaw₂

### 좌회전 offset 계산 책임
- **B 담당자**(차선/라바콘/좌회전)가 모두 책임짐.
- 좌회전 모드일 때, 현재 stage(`/ericar/current_stage`)를 확인하여 어느 좌회전인지 판단.
- IMU yaw 값과 목표 yaw 차이를 기반으로 offset을 계산해 `/ericar/driving_offset`에 발행 (source=2).

### 좌회전 종료 판정
- 좌회전 모듈이 "목표 yaw 도달" 신호를 어떻게 main에 알릴지는 두 가지 옵션:
  - (a) 별도 토픽(`/ericar/left_turn_done` 같은)으로 발행
  - (b) main이 자체적으로 IMU yaw를 구독해 도달 판정
- 본 설계에서는 **B 담당자가 main에 명시적 신호로 알리는 방향(a)** 을 권장한다 (책임 분리). 실제 구현 시 협의로 결정.

---

## 9. lap 카운트

- 트리거: `start_line_detected=true` (체스판 인식)
- 카운트 주체: `main`
- 동작:
  1. lap 카운터 +1
  2. lap < 3: 다음 lap 시작 → `LANE_BEFORE_ROUTE_SELECT` 스테이지로 전환
  3. lap == 3: 시뮬레이션이 자동 종료되므로 별도 종료 처리 불필요 (안전을 위해 stop_request=true 유지 권장)

---

## 10. 전체 스테이지 머신 (한눈에 보기)

```
                        ┌─────────────────────────────────────────────┐
                        │                                             │
[WAITING_START] ──→ [CONE_DRIVING] ──→ [LANE_AFTER_CONE] ──→ [LANE_PEDESTRIAN_ZONE]
                                                                       │
                                                                       ▼
                                                      [LANE_OBSTACLE_DETECT]
                                                                       │
                                                                       ▼
                                                    [LANE_OBSTACLE_FOLLOW_SLOW]
                                                                       │
                                                                       ▼
                                                      [LANE_OBSTACLE_CHASE]
                                                                       │
                                                                       ▼
                                                    [LANE_OBSTACLE_OVERTAKE]
                                                                       │
                                                                       ▼
                                                [LANE_SCHOOL_ZONE_ENTRY_WATCH]
                                                                       │
                                                                       ▼
              ┌─────────────────────────────────────→  [SCHOOL_ZONE_DRIVING]
              │                                                        │
              │                                                        ▼
              │                                          [LANE_BEFORE_LAP_COUNT]
              │                                                        │
              │            lap < 3                          lap == 3   │
              │      ┌─────────────────────────────────────────────────┘
              │      ▼                                        │
              │  [LANE_BEFORE_ROUTE_SELECT]                  종료
              │      │
              │      ├─ 경찰차 있음 ──→ [LANE_PEDESTRIAN_ZONE] (위로 복귀)
              │      │
              │      └─ 경찰차 없음
              │             │
              │             ▼
              │      [LANE_WAIT_LEFT_SIGNAL]
              │             │ 좌회전 신호
              │             ▼
              │      [LEFT_TURN_SHORTCUT_ENTRY]
              │             │ yaw1 도달
              │             ▼
              │      [LANE_SHORTCUT]
              │             │ shortcut_exit_trigger
              │             ▼
              │      [LEFT_TURN_SHORTCUT_EXIT]
              │             │ yaw2 도달
              └─────────────┘ (다시 SCHOOL_ZONE_DRIVING으로 합류)
```

---

## 11. 다음 문서

- [`03_topics_and_messages.md`](03_topics_and_messages.md) — 위 스테이지 머신을 실제로 구현하는 토픽/메시지 명세
