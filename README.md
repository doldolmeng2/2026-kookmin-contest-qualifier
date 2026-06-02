# ERICAR 주행 로직 개발 지침

## 패키지 구성

```
src/ericar/
├── perception/   # 인식 (카메라, 라이다 → 상황 배열 발행)
├── driving/      # 오프셋 계산 (차선/라바콘/좌회전)
├── main/         # 상태머신 + 최종 제어
└── function/     # 유지 (뷰어, 수동 조종)
```

---

## 파일 구조 및 import 규칙

각 패키지는 ROS 노드 파일과 로직 모듈로 나뉜다.
**토픽 sub/pub 은 노드 파일(`main.py`, `control.py`, `perception.py`, `driving.py`)에서만 한다.**
나머지 로직 파일들은 토픽을 직접 다루지 않고, 노드 파일이 `import` 해서 함수/클래스로 사용한다.

```
ericar/
├── perception/perception/
│   ├── perception.py       # [노드] 카메라/라이다 구독 → /perception/status 발행
│   └── yolo_detector.py    #  └ import: YOLO 객체 인식 래퍼
├── driving/driving/
│   ├── driving.py          # [노드] 센서 구독 + /main/* 구독 → /driving/offset 발행
│   ├── lane_detection.py   #  └ import: 차선 인식 → offset
│   ├── turn_left.py        #  └ import: IMU yaw 기반 좌회전 offset
│   └── rubbercone.py       #  └ import: 라이다 라바콘 중심 → offset
├── main/main/
│   ├── main.py             # [노드] 상태머신 + 디버그 시각화, /main/mode·/main/stage 발행
│   └── control.py          # [노드] offset → angle/speed 계산 → /xycar_motor 발행
└── function/               # 유지 (뷰어, 수동 조종)
```

- 노드: `main`, `control`, `perception`, `driving` 만 토픽을 주고받는다.
- 로직 모듈(`yolo_detector`, `lane_detection`, `turn_left`, `rubbercone`)은
  순수 입출력 변환만 하며 `rclpy` 를 import 하지 않는다.
- 디버그 시각화(OpenCV `imshow`) 창은 `main.py` 에서 띄운다.
  현재 모드(텍스트 변환)·stage(차선/좌회전 종류)·perception 상황·
  `/xycar_motor` 의 angle/speed·lap 수·현재 offset 을 표시한다.
  (`main` 은 모터값 표시를 위해 `/xycar_motor` 를 구독만 한다.)

---

## 센서

| 센서 | 토픽 | 메시지 타입 |
| --- | --- | --- |
| 카메라 (전/후/좌/우) | `/usb_cam/image_raw/front` | `sensor_msgs/Image` |
| 라이다 | `/scan` | `sensor_msgs/LaserScan` |
| IMU | `/imu` | `sensor_msgs/Imu` (quaternion → yaw 변환해서 사용) |

---

## 토픽 구조

### perception → main

```
/perception/status   std_msgs/Int32MultiArray
```

**data 인덱스 정의:**

```
[0] start_signal      : 0=대기, 1=초록불
[1] traffic_signal    : 0=wait, 1=green(직진), 2=left_turn
[2] obstacle_front    : 0=없음, 1=전방 방해차량 있음
[3] obstacle_passed   : 0=아직, 1=첫 번째 방해차량 지나침
[4] police_detected   : 0=없음, 1=경찰차 있음
[5] shortcut_exit     : 0=아직, 1=지름길 출구 위치 감지
[6] lap_line          : 0=아직, 1=출발선 통과 감지
```

### driving → main

```
/driving/offset          std_msgs/Float32     # 조향 오프셋 
/driving/lane_change_done std_msgs/Bool       # 차선 변경 완료 신호
```

### main → driving / perception

```
/main/mode   std_msgs/Int32              # 현재 모드 (아래 모드 정의 참고)
/main/stage  std_msgs/Int32MultiArray   # 세부 단계 (아래 stage 정의 참고)
```

### main → 모터

```
/xycar_motor   xycar_msgs/XycarMotor    # angle(-100~100), speed(-50~50)
```

---

## 모드 정의 (`/main/mode`)

```python
MODE_WAIT        = 0  # 시작 대기 (초록불 대기, 정지)
MODE_CONE        = 1  # 라바콘 주행 (라이다 기반)
MODE_LANE        = 2  # 차선 주행
MODE_LEFT_TURN   = 3  # 좌회전
MODE_LANE_CHANGE = 4  # 차선 변경 중
MODE_FOLLOW      = 5  # 앞차 따라가기 (2차선 저속)
MODE_SIGNAL_WAIT = 6  # 신호 대기 (정지)
```

---

## stage 정의 (`/main/stage`, Int32MultiArray)

```
[0] lane_target   : 0=1차선 주행, 1=2차선 주행
[1] turn_type     : 0=좌회전A(지름길 진입), 1=좌회전B(지름길 탈출)
```

- `lane_target`: MODE_LANE, MODE_LANE_CHANGE, MODE_FOLLOW에서 driving이 참조
- `turn_type`: MODE_LEFT_TURN에서 driving이 참조 (목표 yaw가 다름)

---

## 제어 계산 (main)

```python
angle = offset * mode_ratio   # steer(-100~100)
# 모드별 speed 예시
speed_table = {
    MODE_WAIT:        0,
    MODE_CONE:       15,
    MODE_LANE:       25,
    MODE_LEFT_TURN:  12,
    MODE_LANE_CHANGE:20,
    MODE_FOLLOW:     10,
    MODE_SIGNAL_WAIT: 0,
}
```

---

## 전체 주행 시퀀스

```
[lap 0]
MODE_WAIT
  └─(start_signal=1)→ MODE_CONE
      └─(아스팔트 진입 완료)→ MODE_LANE [lane=0, 1차선]

[lap 1]
MODE_LANE [1차선]
  └─(obstacle_front=1)→ MODE_LANE_CHANGE [lane=1, 2차선으로]
      └─(lane_change_done)→ MODE_FOLLOW [2차선 저속]
          └─(라이다 왼쪽 미감지)→ MODE_LANE [lane=0, 1차선]
              └─(lap_line=1)→ lap++, MODE_LANE [1차선]

[lap 2/3 - 경찰 없음]
MODE_LANE [1차선]
  └─(신호등 앞 도달)→ MODE_SIGNAL_WAIT
      └─(traffic_signal=2, 좌회전)→ MODE_LEFT_TURN [turn_type=0, 지름길 진입]
          └─(목표 yaw 도달)→ MODE_LANE [lane=1, 2차선 숏컷 주행]
              └─(shortcut_exit=1)→ MODE_LEFT_TURN [turn_type=1, 지름길 탈출]
                  └─(목표 yaw 도달)→ MODE_LANE [lane=0, 1차선]
                      └─(lap_line=1)→ lap++

[lap 2/3 - 경찰 있음]
MODE_LANE [1차선]
  └─(신호등 앞 도달)→ MODE_SIGNAL_WAIT
      └─(traffic_signal=1, 직진)→ MODE_LANE [1차선]
          └─(obstacle_front=1)→ MODE_LANE_CHANGE ... (lap1과 동일)
```

---

## driving 동작 요약

| 모드 | 동작 |
| --- | --- |
| MODE_CONE | 라이다로 라바콘 중심 계산 → offset 발행 |
| MODE_LANE | 카메라 차선 인식, stage[0]에 따라 1차선/2차선 기준 offset 발행 |
| MODE_LANE_CHANGE | LANE과 동일 계산, offset이 목표 범위 진입 시 lane_change_done 발행 |
| MODE_FOLLOW | LANE과 동일 계산 (속도 조절은 main에서) |
| MODE_LEFT_TURN | stage[1]에 따라 목표 yaw 설정, 도달 전까지 최대 왼쪽 offset 발행 (control 비례값과 상의 필요) |
| MODE_WAIT / MODE_SIGNAL_WAIT | offset 발행 불필요 |

---

## perception 동작 요약

- 카메라: 신호등 색상/화살표 인식, 경찰차 인식, 방해차량 인식, 출발선 감지, 지름길 출구 감지
- 라이다: 방해차량 전방 감지 보조, 라바콘 감지 (driving에서 직접 구독)
- 모든 인식 결과를 `Int32MultiArray` 하나로 통합 발행

---

## 전략 메모

- 1차선 유지 주행 시 보행자는 그냥 통과 가능 (충돌 없음)
- 어린이보호구역: 벌점 감수하고 빠르게 통과 (빠르게 통과시 5초 벌점 받음 vs 느리게 통과 하면 빠르게 통과보다 10초 더 걸림)
- 신호 무시 가능 여부: 신호 대기 시간이 15초 벌점보다 길 때만 무시 (타이밍 판단 필요)
- 중앙선 침범: 감점 없음 그러나 일단 차선 주행으로 할 것임