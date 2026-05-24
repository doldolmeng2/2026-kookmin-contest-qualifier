# 2026-kookmin-contest-qualifier

제 9회 국민대학교 자율주행 경진대회 (2026) 예선 — **Team ERICAR**

ROS2 Humble + WSL2 Ubuntu 22.04 환경에서 동작하는 자율주행 시스템.
Windows 호스트의 시뮬레이터와 TCP로 통신하며, 카메라/IMU/LiDAR 데이터를 받아 차량을 제어한다.

---

## 시스템 개요

총 **3 lap** 주행하며 7가지 미션(신호등, 라바콘, 차선, 보행자, 경찰차, 방해차량, 어린이 보호구역)을 자율적으로 수행한다.

### 아키텍처 한눈에 보기

```
┌──────────────────┐    /ericar/perception    ┌──────────────────┐
│ ericar_          │ ───────────────────────→ │                  │
│ perception (A)   │                           │  ericar_main_    │
└──────────────────┘                           │  control (C)     │
        ▲                                      │  (main + control)│
        │ /ericar/active_perceptions           │                  │
        │                                      └──────────────────┘
                                                       │  ▲
                                                       │  │ /ericar/driving_offset
                                       /ericar/        │  │
                                       control_state   │  │
                                       /xycar_motor    │  │
                                                       ▼  │
┌──────────────────┐    /ericar/target_lane    ┌──────────────────┐
│ ericar_          │ ←──────────────────────── │                  │
│ driving (B)      │    /ericar/current_stage  │                  │
│ (차선/라바콘/    │ ←──────────────────────── │   (위와 동일)    │
│  좌회전)         │                           │                  │
└──────────────────┘                           └──────────────────┘
```

### 패키지 구성 (총 4개)

| 패키지 | 담당 | 책임 |
|--------|------|------|
| `ericar_msgs` | 공용 | 커스텀 메시지 정의 |
| `ericar_perception` | A | 모든 인식 (신호등, 보행자, 차량, 노면 표시 등) |
| `ericar_driving` | B | 차선 / 라바콘 / 좌회전 offset 계산 |
| `ericar_main_control` | C | 상태 머신(main) + 최종 제어(control) |

---

## 핵심 설계 원칙

1. **Mode + Stage 두 층 상태 관리** — control은 7개 모드만, main은 15개 스테이지로 세분화된 상태 머신을 관리
2. **인식 결과 통합 메시지** — 모든 인식 결과를 단일 `Perception` 메시지에 묶어 발행
3. **주행 offset 통합 토픽** — 차선/라바콘/좌회전 offset 모두 `/ericar/driving_offset`으로 통일
4. **정지는 플래그로** — 정지를 별도 모드로 두지 않고 `stop_request` 플래그로 처리
5. **main이 single source of truth** — 모드, 스테이지, 활성 인식기, target_lane 모두 main이 결정·발행

자세한 배경은 [`Documents/07_design_decisions.md`](Documents/07_design_decisions.md) 참고.

---

## 빠른 시작

### 환경
- WSL2, Ubuntu 22.04, ROS2 Humble
- 워크스페이스: `~/xycar_ws`
- `ROS_DOMAIN_ID=7`, 네임스페이스 `xycar`

### 빌드
```bash
cw          # cd ~/xycar_ws
cbs         # colcon build --symlink-install
source install/setup.bash
```

### 실행
```bash
ros2 launch ericar_main_control all.launch.py
```

### 디버깅
```bash
# 현재 스테이지 확인
ros2 topic echo /ericar/current_stage

# 모드 + 정지 상태 확인
ros2 topic echo /ericar/control_state

# 인식 결과 확인
ros2 topic echo /ericar/perception

# 노드 그래프
rqt_graph
```

자세한 빌드/실행/디버깅은 [`Documents/05_package_structure.md`](Documents/05_package_structure.md) 참고.

---

## 문서

| 문서 | 내용 |
|------|------|
| [`Documents/00_quick_reference.md`](Documents/00_quick_reference.md) | 자주 찾는 정보 한 페이지 요약 |
| [`Documents/01_mission_overview.md`](Documents/01_mission_overview.md) | 대회 개요, 미션 리스트, 트랙 구조, lap별 시나리오 |
| [`Documents/02_mode_and_stage.md`](Documents/02_mode_and_stage.md) | 모드(7개) / 스테이지(15개) 정의, 전환 다이어그램 |
| [`Documents/03_topics_and_messages.md`](Documents/03_topics_and_messages.md) | 토픽 구조, 메시지 타입, QoS, 통신 흐름 |
| [`Documents/04_part_responsibilities.md`](Documents/04_part_responsibilities.md) | 파트별(A/B/C) 구현 항목과 체크리스트 |
| [`Documents/05_package_structure.md`](Documents/05_package_structure.md) | 패키지 구조, 빌드, 실행 방법 |
| [`Documents/06_message_definitions.md`](Documents/06_message_definitions.md) | `.msg` 파일 실제 예시 |
| [`Documents/07_design_decisions.md`](Documents/07_design_decisions.md) | 설계 결정 배경 (ADR) |

---

## 진행 상태

### 완료
- [x] 미션 분석 및 lap별 흐름 정리
- [x] 모드 / 스테이지 설계
- [x] 토픽 / 메시지 설계
- [x] 패키지 구조 결정
- [x] 파트별 책임 분배

### 진행 예정
- [ ] `ericar_msgs` 패키지 구현
- [ ] A 파트: 인식 노드 구현
- [ ] B 파트: 차선/라바콘/좌회전 노드 구현
- [ ] C 파트: main + control 노드 구현
- [ ] 통합 테스트 (시뮬레이터 연동)
- [ ] 미정 사항 결정 (offset 부호, 속도 튜닝 등 — `07_design_decisions.md` 하단 참조)

---

## 팀

**Team ERICAR**
- 3인 협업 (인식 / 주행 / 메인+제어)
- 책임 영역과 인터페이스는 [`Documents/04_part_responsibilities.md`](Documents/04_part_responsibilities.md)에 명시됨

---

## 참고 자료

- 대회 공식 자료: 상위 폴더의 `과제1번설명.pdf`, `대회소개.pdf`, `시뮬레이터사용법.pdf`
- ROS2 Humble 문서: https://docs.ros.org/en/humble/
