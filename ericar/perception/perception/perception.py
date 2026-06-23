#!/usr/bin/env python3
"""perception 노드.

카메라(전/후/좌/우)와 라이다를 구독해 대회 상황을 인식하고,
모든 결과를 하나의 std_msgs/Int32MultiArray 로 통합해 /perception/status 로 발행한다.

실제 인식 알고리즘(신호등 색/화살표, 경찰차, 방해차량, 출발선, 지름길 출구 등)은
yolo_detector 모듈 및 영상처리 함수로 분리해 import 해서 사용한다.
이 파일은 ROS 토픽 sub/pub 과 콜백, 발행 타이밍만 담당한다.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data

from std_msgs.msg import Int32, Int32MultiArray
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge

import cv2

# 로직 모듈
from perception.yolo_detector import YoloDetector
from perception.traffic_light import (
    TrafficLightDetector, SIGNAL_NONE, SIGNAL_GREEN, SIGNAL_LEFT)
from perception.start_line import detect_start_line
from perception.shortcut_exit import detect_shortcut_exit
from perception.obstacle import detect_obstacle_front, left_min, sector_min
from perception.left_car import car_in_left
from perception.school_zone import SchoolZoneDetector

# 시뮬레이터는 카메라/라이다를 RELIABLE 로 발행하므로 맞춰서 구독
# (BEST_EFFORT 로 받으면 큰 이미지가 UDP 조각 유실로 대부분 드롭됨)
_QOS_RELIABLE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# ---------------------------------------------------------------------------
# /perception/status data 인덱스 정의 (README 참고)
# ---------------------------------------------------------------------------
IDX_START_SIGNAL   = 0   # 0=대기, 1=초록불
IDX_TRAFFIC_SIGNAL = 1   # 0=wait, 1=green(직진), 2=left_turn
IDX_OBSTACLE_FRONT = 2   # 0=없음, 1=전방 방해차량 있음
IDX_OBSTACLE_PASSED = 3  # 0=아직, 1=첫 번째 방해차량 지나침
IDX_POLICE_DETECTED = 4  # 0=없음, 1=경찰차 있음
IDX_SHORTCUT_EXIT  = 5   # 0=아직, 1=지름길 출구 위치 감지
IDX_LAP_LINE       = 6   # 0=아직, 1=출발선 통과 감지
IDX_PEDESTRIAN     = 7   # 0=무시(없음/멀리/옆), 1=바로 앞 위험 → 정지
IDX_SCHOOL_ZONE    = 8   # 0=아님, 1=시작 인식(감속), 2=해제 인식(가속)
IDX_TRAFFIC_PRESENT = 9  # 0=신호등 미검출, 1=트랙 신호등 검출
STATUS_LEN = 10

# ===========================================================================
# 🔧 튜닝 파라미터  (인식 임계값 — 여기만 고치면 됨)
#   ※ 신호등/스쿨존의 'HSV 색 범위'는 양이 많아 각 모듈 상단에 둠
#     (traffic_light.py / school_zone.py 의 HSV_* 참고)
# ===========================================================================

# --- YOLO (경찰차 / 보행자) ---
YOLO_CONF  = 0.5         # confidence 하한 (낮추면 더 잘 잡지만 오검출↑)
YOLO_EVERY = 5           # N틱마다 1회 추론 (30Hz/5=6Hz). CPU 무거우면 ↑

# --- 보행자 (정지 판단) ---  박스가 가깝고(큼) 진행경로(중앙)일 때만 정지
PED_DANGER_MIN_H  = 0.35          # 박스 세로크기(0~1) 이 이상이면 '바로 앞' → 정지
PED_DANGER_X_BAND = (0.30, 0.70)  # 박스 중심 x 가 이 범위(진행경로) 안일 때만

# --- 경찰차 ---
POLICE_MIN_H = 0.21     # 박스 세로크기(0~1) 이 이상(가까움)일 때만 인식. 창 'hXX' 보고 튜닝

# --- 방해차량 (라이다) ---  ※각도 규약: 0°=전방, +각도=좌측 (index=각도 0~359°)
OBS_FRONT_DEG  = list(range(350, 360)) + list(range(0, 11))  # 전방 섹터(±10°) — 앞차 보는 방향
OBS_FRONT_MAX  = 4.0    # 전방 이 거리(m) 이내에 차 있으면 → 앞차(data[2]=1, 충돌방지)
OBS_LEFT_DEG   = list(range(60, 96))   # 좌측 섹터(60~95°) — 추월 중 옆 1차선 차 보는 방향
OBS_CAR_BESIDE = 2.5    # 좌측이 이 거리(m) 이내면 → '옆에 차 붙음'
OBS_PASSED_GAP = 1.0    # 붙었던 거리보다 이만큼(m) 더 멀어지면 → 추월완료(data[3]=1)
# 앞차 인식에서 보행자 제외: 전방 경로에 가까운 보행자면 앞차(data[2]) 끔 (data[7]이 따로 처리)
OBS_PED_SUPPRESS_H    = 0.18          # 보행자 박스 세로크기 ≥ 이 값(=라이다에 잡힐 거리)이면 억제
OBS_PED_SUPPRESS_BAND = (0.35, 0.65)  # 보행자 박스 중심 x 가 이 범위(전방 경로)면 억제

# --- 트랙 신호등 (4구) ---  키 이름은 traffic_light.BASE_PARAMS 와 일치해야 함
TL_TRACK_PARAMS = dict(
    black_min_count=120,       # ROI 내 검은 픽셀 총량 하한
    black_min_blob_area=18000,  # ★ 하우징 박스 면적 ≥ 이 값(=가까움)이면 인식. 로그 blob= 보고 튜닝
    color_min_count=50,       # 게이트 통과 후 색 구분 최소 픽셀
    bbox_pad=0,
    aspect_min=2.0,            # 하우징 가로/세로 비 하한 (세로 기둥/나무 배제)
)

# --- 시작 신호등 (3구) ---  가까이·정면이라 게이트 높게
TL_START_PARAMS = dict(
    black_min_count=600,
    black_min_blob_area=400,
    color_min_count=40,
    aspect_min=1.5,
)

# --- 어린이 보호구역 (하단 ROI 노랑/흰색 상태기계) ---
SZ_ROI_TOP      = 0.80   # 하단 ROI 시작(0~1). 차에 가까운 노면만
SZ_YELLOW_ENTER = 10000  # 노란 픽셀 ≥ → 시작(감속, data[8]=1)
SZ_WHITE_EXIT   = 1000   # (보호구역 안) 흰 픽셀 ≥ → 해제(data[8]=2)
SZ_WHITE_NORMAL = 3000   # (해제 후) 흰 픽셀 ≥ → 일반도로 복귀(data[8]=0)

# --- 디버그 시각화 ---
VIZ_DEFAULT = True       # perception 창(카메라+bbox+status) 기본 표시 여부

# main 모드 정의 (퍼셉션이 모드별로 인식 항목을 골라 켜기 위해 참조)
MODE_WAIT, MODE_CONE, MODE_LANE, MODE_LEFT_TURN, \
    MODE_LANE_CHANGE, MODE_FOLLOW, MODE_SIGNAL_WAIT = range(7)


class Perception(Node):

    def __init__(self):
        super().__init__('perception')

        self._bridge = CvBridge()
        self._detector = YoloDetector(conf_threshold=YOLO_CONF)  # CPU 추론, weights/perception.pt
        # 첫 추론 지연(수 초)을 주행 중이 아니라 시작 시점에 미리 처리.
        # torch/ultralytics 미설치 환경에선 경고만 내고 계속 진행.
        try:
            self.get_logger().info('YOLO 모델 로딩 중...')
            self._detector.warmup()
            self.get_logger().info('YOLO 모델 준비 완료')
        except Exception as e:
            self.get_logger().warn(f'YOLO 로드 실패(경찰차 검출 비활성): {e}')

        # 신호등 검출기 (검은 픽셀 게이팅, ericar_msgs 불필요)
        self._tl_start = TrafficLightDetector(
            'start', four_lamp=False, overrides=TL_START_PARAMS,
            show=False, logger=self.get_logger())
        self._tl_track = TrafficLightDetector(
            'track', four_lamp=True, overrides=TL_TRACK_PARAMS,
            show=False, logger=self.get_logger())

        # 어린이 보호구역 (하단 ROI 노랑/흰색 상태기계)
        self._school_zone = SchoolZoneDetector(
            logger=self.get_logger(), debug=True, show=False,
            roi_top=SZ_ROI_TOP, yellow_enter=SZ_YELLOW_ENTER,
            white_exit=SZ_WHITE_EXIT, white_normal=SZ_WHITE_NORMAL)

        # 디버그 시각화: 카메라 + YOLO bbox + status 를 'perception' 창에 표시
        #   팀원들이 인식 상태를 눈으로 확인용. 끄려면: -p viz:=false
        self.declare_parameter('viz', VIZ_DEFAULT)
        self._viz = self.get_parameter('viz').value

        # 최신 입력 버퍼
        self._img_front = None
        self._img_left = None    # 좌측 카메라 (추월 중 옆 차 검출용)
        self._scan = None

        # main 으로부터 현재 모드/스테이지 수신 (필요한 인식만 수행하기 위함)
        self._mode = MODE_WAIT
        self._stage = [0, 0]

        # 누적 상태 (한 번 1이 된 래치성 값은 유지)
        self._status = [0] * STATUS_LEN

        # YOLO 추론은 CPU 라 무겁다 → 저빈도(every N틱)로만 돌리고 결과 캐시
        self._det = {}
        self._tick_n = 0
        self._yolo_every = YOLO_EVERY

        # 추월완료 판정용(라이다 좌측, B안): 옆에 붙었던 차의 최소거리 기록
        #   → 그보다 확 멀어지면 추월완료 (도로변 절대거리 무관). FOLLOW 진입시 리셋.
        self._left_beside_min = None

        # ---- 구독 ----
        # 시뮬이 RELIABLE 로 발행 → RELIABLE 로 받아야 이미지 유실 없음
        self.create_subscription(
            Image, '/usb_cam/image_raw/front',
            self._front_cb, _QOS_RELIABLE)
        self.create_subscription(
            Image, '/usb_cam/image_raw/left',
            self._left_cb, _QOS_RELIABLE)
        self.create_subscription(
            LaserScan, '/scan',
            self._scan_cb, _QOS_RELIABLE)
        self.create_subscription(
            Int32, '/main/mode', self._mode_cb, 10)
        self.create_subscription(
            Int32MultiArray, '/main/stage', self._stage_cb, 10)

        # ---- 발행 ----
        self._status_pub = self.create_publisher(
            Int32MultiArray, '/perception/status', 10)

        # 인식 + 발행 주기 (30Hz)
        self.create_timer(1.0 / 30.0, self._tick)

        self.get_logger().info('perception node ready')

    # ------------------------------------------------------------------
    # 콜백: 최신 데이터만 저장
    # ------------------------------------------------------------------
    def _front_cb(self, msg):
        self._img_front = self._bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _left_cb(self, msg):
        self._img_left = self._bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _scan_cb(self, msg):
        self._scan = msg

    def _mode_cb(self, msg):
        # FOLLOW(2차선 추월) 진입 시 추월완료 추적 상태 초기화 (이번 추월 새로)
        if msg.data == MODE_FOLLOW and self._mode != MODE_FOLLOW:
            self._left_beside_min = None
            self._status[IDX_OBSTACLE_PASSED] = 0   # 래치 해제
        self._mode = msg.data

    def _stage_cb(self, msg):
        if msg.data:
            self._stage = list(msg.data)

    # ------------------------------------------------------------------
    # 주기 처리: 현재 모드에 필요한 인식만 돌려 status 갱신 후 발행
    # ------------------------------------------------------------------
    def _tick(self):
        if self._img_front is None:
            return
        self._tick_n += 1

        # YOLO 가 필요한 모드에서만, 저빈도로 추론하고 결과를 캐시한다.
        # (신호등은 YOLO 안 쓰고 self._img_front 로 매 틱 검출 → 영향 없음)
        yolo_modes = (MODE_LANE, MODE_FOLLOW, MODE_SIGNAL_WAIT)
        if self._mode in yolo_modes and self._tick_n % self._yolo_every == 0:
            self._det = self._detector.infer(self._img_front)
        det = self._det

        if self._mode == MODE_WAIT:
            self._status[IDX_START_SIGNAL] = self._detect_start_signal(det)

        if self._mode in (MODE_LANE, MODE_SIGNAL_WAIT):
            self._status[IDX_TRAFFIC_SIGNAL] = self._detect_traffic_signal(det)
            self._status[IDX_POLICE_DETECTED] = self._detect_police(det)

        if self._mode in (MODE_LANE, MODE_FOLLOW):
            self._status[IDX_OBSTACLE_FRONT] = self._detect_obstacle_front(det, self._scan)
            self._status[IDX_LAP_LINE] = self._detect_lap_line(self._img_front)
            self._status[IDX_PEDESTRIAN] = self._detect_pedestrian(det)

        if self._mode == MODE_FOLLOW:
            self._status[IDX_OBSTACLE_PASSED] = self._detect_obstacle_passed(self._scan)

        if self._mode == MODE_LANE and self._stage[0] == 1:
            self._status[IDX_SHORTCUT_EXIT] = self._detect_shortcut_exit(det)

        # 어린이 보호구역: 노랑(시작)→1, 흰색(해제)→2 상태기계
        if self._mode == MODE_LANE:
            self._status[IDX_SCHOOL_ZONE] = self._school_zone.update(self._img_front)

        self._publish_status()

        if self._viz:
            self._show_debug()

    # ------------------------------------------------------------------
    # 디버그 시각화: 카메라 + YOLO bbox + status 값
    # ------------------------------------------------------------------
    def _show_debug(self):
        if self._img_front is None:
            return
        try:
            vis = self._img_front.copy()
            Himg = vis.shape[0]
            colors = {'police_car': (0, 0, 255), 'pedestrian': (0, 255, 0)}
            for label, dets in self._det.items():
                col = colors.get(label, (255, 255, 0))
                for d in dets:
                    x1, y1, x2, y2 = (int(v) for v in d.bbox)
                    hf = (y2 - y1) / float(Himg)   # 박스 세로크기 비율 (거리 가늠 + 튜닝)
                    cv2.rectangle(vis, (x1, y1), (x2, y2), col, 2)
                    cv2.putText(vis, f'{label} {d.confidence:.2f} h{hf:.2f}',
                                (x1, max(12, y1 - 5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
            names = ['start', 'traffic', 'obsF', 'obsP', 'police',
                     'short', 'lap', 'ped', 'school', 'tlPre']
            txt = ' '.join(f'{n}:{v}' for n, v in zip(names, self._status))
            cv2.putText(vis, txt, (6, 18), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (0, 255, 255), 1)
            # 신호등 하우징 박스 (현재 모드의 검출기) + 어린이보호구역 ROI
            if self._mode in (MODE_LANE, MODE_SIGNAL_WAIT):
                self._tl_track.draw(vis)
            elif self._mode == MODE_WAIT:
                self._tl_start.draw(vis)
            if self._mode == MODE_LANE:
                self._school_zone.draw(vis)
            cv2.imshow('perception', vis)
            # 방해차량(라이다) 조감도 창
            if self._scan is not None:
                self._draw_lidar_viz(self._scan)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().warn(f'viz 실패: {e}')

    # ------------------------------------------------------------------
    # 방해차량 라이다 시각화 (조감도): 전방/좌측 섹터 + 검출 상태
    #   빨강점=전방섹터 위험거리내 / 주황점=좌측 '차붙음'거리내 / 회색=기타
    # ------------------------------------------------------------------
    def _draw_lidar_viz(self, scan):
        import numpy as np
        H = Wd = 440
        img = np.full((H, Wd, 3), 30, np.uint8)
        cx, cy = Wd // 2, H - 55
        scale = 32.0                      # px/m
        maxr = (cy - 10) / scale
        front_set, left_set = set(OBS_FRONT_DEG), set(OBS_LEFT_DEG)

        # 거리 링
        for rr in range(2, int(maxr) + 1, 2):
            cv2.circle(img, (cx, cy), int(rr * scale), (55, 55, 55), 1)
            cv2.putText(img, f'{rr}m', (cx + 3, cy - int(rr * scale) + 13),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.34, (90, 90, 90), 1)
        # 섹터 경계선 (전방 ±10°, 좌측 60~95°)
        for deg, c in [(10, (0, 90, 200)), (350, (0, 90, 200)),
                       (60, (0, 170, 170)), (95, (0, 170, 170))]:
            a = math.radians(deg)
            ex = int(cx - maxr * math.sin(a) * scale)
            ey = int(cy - maxr * math.cos(a) * scale)
            cv2.line(img, (cx, cy), (ex, ey), c, 1)

        # 라이다 점 찍기 (차체 가림 99~262° 제외)
        r = scan.ranges
        fmin = lmin = float('inf')
        for d in range(min(360, len(r))):
            dist = r[d]
            if not math.isfinite(dist) or dist <= 0.3:
                continue
            if d in front_set:
                fmin = min(fmin, dist)
            if d in left_set:
                lmin = min(lmin, dist)
            if dist > maxr:
                continue
            a = math.radians(d)
            px = int(cx - dist * math.sin(a) * scale)
            py = int(cy - dist * math.cos(a) * scale)
            if not (0 <= px < Wd and 0 <= py < H):
                continue
            if d in front_set:
                col = (0, 0, 255) if dist < OBS_FRONT_MAX else (0, 150, 255)
            elif d in left_set:
                col = (0, 165, 255) if dist < OBS_CAR_BESIDE else (0, 255, 255)
            else:
                col = (140, 140, 140)
            cv2.circle(img, (px, py), 2, col, -1)

        # 임계 마커: 전방 위험거리 / 좌측 차붙음거리
        cv2.circle(img, (cx, cy - int(OBS_FRONT_MAX * scale)), 4, (0, 0, 255), -1)
        a60 = math.radians(77)   # 좌측 섹터 중앙쯤에 표시
        cv2.circle(img, (int(cx - OBS_CAR_BESIDE * math.sin(a60) * scale),
                         int(cy - OBS_CAR_BESIDE * math.cos(a60) * scale)),
                   4, (0, 165, 255), -1)
        # 차
        cv2.rectangle(img, (cx - 9, cy - 4), (cx + 9, cy + 16), (0, 0, 180), -1)

        # 상태 텍스트
        bm = self._left_beside_min
        f = fmin if math.isfinite(fmin) else 0.0
        l = lmin if math.isfinite(lmin) else 0.0
        lines = [
            f"mode={self._mode}",
            f"front={f:4.1f}m  obsF={self._status[IDX_OBSTACLE_FRONT]}",
            f"left ={l:4.1f}m  obsP={self._status[IDX_OBSTACLE_PASSED]}",
            f"besideMin={('%.1f' % bm) if bm is not None else '-'}",
        ]
        for i, t in enumerate(lines):
            cv2.putText(img, t, (8, 18 + i * 17), cv2.FONT_HERSHEY_SIMPLEX,
                        0.46, (255, 255, 255), 1)
        cv2.imshow('lidar_obstacle', img)

    # ------------------------------------------------------------------
    # 인식 세부 함수 (골격만; 추후 구현)
    # ------------------------------------------------------------------
    def _detect_start_signal(self, det):
        # 시작등(3구): 초록불이면 1, 아니면 0
        sig = self._tl_start.detect(self._img_front)
        return 1 if sig == SIGNAL_GREEN else 0

    def _detect_traffic_signal(self, det):
        # 트랙 신호등 원본 판정 결과를 한 번만 계산한다.
        sig = self._tl_track.detect(self._img_front)

        # 빨강·노랑·초록·좌회전 중 하나라도 검출되면 신호등이 존재한다.
        # NONE과 빨간불을 구분하기 위해 별도 상태값으로 발행한다.
        self._status[IDX_TRAFFIC_PRESENT] = (
            0 if sig == SIGNAL_NONE else 1
        )

        if sig == SIGNAL_GREEN:
            return 1

        if sig == SIGNAL_LEFT:
            return 2
        # 빨강·노랑은 정지 대기이며, NONE도 action 값은 0이다.
        # 두 경우의 구분은 IDX_TRAFFIC_PRESENT가 담당한다.
        return 0

    def _detect_police(self, det):
        # 경찰차 박스가 충분히 클 때(=가까울 때)만 1 → 너무 일찍 인식 방지
        cars = det.get('police_car')
        if not cars or self._img_front is None:
            return 0
        H = self._img_front.shape[0]
        for d in cars:
            x1, y1, x2, y2 = d.bbox
            if (y2 - y1) / H >= POLICE_MIN_H:
                return 1
        return 0

    def _detect_pedestrian(self, det):
        # 보행자 박스 중 '바로 앞 위험'한 것이 있으면 1, 아니면 0(무시하고 통과).
        #   위험 = 가깝고(박스 h 큼) AND 진행경로(중앙 x밴드) 안.
        #   멀거나 길 가장자리 보행자는 0 → 그냥 지나감.
        peds = det.get('pedestrian')
        if not peds or self._img_front is None:
            return 0
        H, W = self._img_front.shape[:2]
        for d in peds:
            x1, y1, x2, y2 = d.bbox
            h_norm = (y2 - y1) / H
            cx_norm = ((x1 + x2) * 0.5) / W
            if (h_norm >= PED_DANGER_MIN_H
                    and PED_DANGER_X_BAND[0] <= cx_norm <= PED_DANGER_X_BAND[1]):
                return 1
        return 0

    def _detect_obstacle_front(self, det, scan):
        # 라이다 전방 섹터에 가까운 물체가 있으면 앞차 후보 (임계값은 OBS_* 중앙 파라미터)
        if not detect_obstacle_front(scan, OBS_FRONT_DEG, OBS_FRONT_MAX):
            return 0
        # 단, 그 물체가 '보행자'(전방 경로에 가까이)면 앞차로 치지 않음 → data[7]이 처리
        if self._ped_in_front_path(det):
            return 0
        return 1

    def _ped_in_front_path(self, det):
        # YOLO 보행자 중, 전방 경로(중앙 x밴드)에 가까이(박스 큼) 있는 게 있으면 True
        peds = det.get('pedestrian')
        if not peds or self._img_front is None:
            return False
        H, W = self._img_front.shape[:2]
        for d in peds:
            x1, y1, x2, y2 = d.bbox
            h = (y2 - y1) / H
            cx = ((x1 + x2) * 0.5) / W
            if (h >= OBS_PED_SUPPRESS_H
                    and OBS_PED_SUPPRESS_BAND[0] <= cx <= OBS_PED_SUPPRESS_BAND[1]):
                return True
        return False

    def _detect_obstacle_passed(self, scan):
        # 추월완료(B안): 좌측 라이다에 차가 '붙었다(가까움)' → '확 멀어졌다' 변화로 판정.
        #   '있었던 적' 있어야 추월완료가 뜨므로, 평소 빈 영역으로 인한 오검출 방지.
        #   붙었던 거리 대비 상대 판정이라 도로변 절대거리에도 둔감.
        # ★ 래치: 한 번 추월완료(1)되면 이 세션 동안 1 고정 → 추월 직후 경찰차가
        #   좌측을 막아 다시 '차붙음→사라짐'으로 1이 두 번 뜨는 것 방지.
        #   (FOLLOW 재진입 시 _mode_cb 에서 0 으로 리셋)
        if self._status[IDX_OBSTACLE_PASSED] == 1:
            return 1
        left = left_min(scan, OBS_LEFT_DEG)
        if left < OBS_CAR_BESIDE:
            self._left_beside_min = (left if self._left_beside_min is None
                                     else min(self._left_beside_min, left))
            return 0
        if self._left_beside_min is not None and left > self._left_beside_min + OBS_PASSED_GAP:
            return 1
        return 0

    def _detect_shortcut_exit(self, det):
        # 숏컷(직선) 끝 삼거리: 정면에 잔디(길 끝남)가 차면 1 → main이 좌회전 시작
        return 1 if detect_shortcut_exit(self._img_front) else 0

    def _detect_lap_line(self, img):
        # 출발선(흑백 체커보드)이 하단에 가까이 보이면 1
        return 1 if detect_start_line(img) else 0

    # ------------------------------------------------------------------
    def _publish_status(self):
        msg = Int32MultiArray()
        msg.data = [int(v) for v in self._status]
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Perception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
