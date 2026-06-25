#!/usr/bin/env python3
"""perception 노드.

카메라(전/후/좌/우)와 라이다를 구독해 대회 상황을 인식하고,
모든 결과를 하나의 std_msgs/Int32MultiArray 로 통합해 /perception/status 로 발행한다.

실제 인식 알고리즘(신호등 색/화살표, 경찰차, 방해차량, 출발선, 지름길 출구 등)은
yolo_detector 모듈 및 영상처리 함수로 분리해 import 해서 사용한다.
이 파일은 ROS 토픽 sub/pub 과 콜백, 발행 타이밍만 담당한다.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Int32, Int32MultiArray
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np

# 로직 모듈 (구현은 추후 팀원이 채움)
from perception.yolo_detector import YoloDetector

# ---------------------------------------------------------------------------
# /perception/status data 인덱스 정의 (README 참고)
# ---------------------------------------------------------------------------
IDX_START_SIGNAL      = 0   # 0=대기, 1=초록불
IDX_TRAFFIC_SIGNAL    = 1   # 0=wait, 1=green(직진), 2=left_turn
IDX_OBSTACLE_FRONT    = 2   # 0=없음, 1=전방 방해차량 있음
IDX_OBSTACLE_PASSED   = 3   # 0=아직, 1=첫 번째 방해차량 지나침
IDX_POLICE_DETECTED   = 4   # 0=없음, 1=경찰차 있음
IDX_SHORTCUT_EXIT     = 5   # 0=아직, 1=지름길 출구 위치 감지
IDX_LAP_LINE          = 6   # 0=아직, 1=출발선 통과 감지
IDX_CONE_FINISHED     = 7   # 0=라바콘 주행 중, 1=아스팔트 진입(라바콘 구간 종료)
IDX_SCHOOL_ZONE_SIGNAL = 8  # 0=없음, 1=어린이보호구역 시작 감지
IDX_SCHOOL_ZONE_END    = 9  # 0=없음, 1=어린이보호구역 종료 감지
STATUS_LEN = 10

YELLOW_PIXEL_MIN = 500   # 라바콘 구간 종료 판정용 노란색 픽셀 임계값 (튜닝 필요)

# 출발선(lap line) 검출 파라미터 (튜닝 필요)
LAP_ROI = (0, 270, 640, 300)          # (x1, y1, x2, y2) — 이미지 하단
LAP_HSV_LOWER = np.array([0, 0, 0]) # 흰색 계열 (H·S·V)
LAP_HSV_UPPER = np.array([20, 20, 2])
LAP_PIXEL_MIN = 100

# 어린이보호구역 시작 마커 검출 파라미터 (튜닝 필요)
SCHOOL_ROI = (0, 270, 640, 310)              # (x1, y1, x2, y2) — lane_detection ROI_Y1~ROI_Y2 참고
SCHOOL_HSV_LOWER = np.array([25, 205, 80])   # lane_detection YELLOW_LOWER 동일
SCHOOL_HSV_UPPER = np.array([35, 255, 255])  # lane_detection YELLOW_UPPER 동일
SCHOOL_PIXEL_MIN = 2400

# 어린이보호구역 종료 마커 검출 파라미터 (흰색 픽셀, 튜닝 필요)
SCHOOL_END_ROI = (0, 270, 640, 310)              # (x1, y1, x2, y2) — 튜닝 필요
SCHOOL_END_HSV_LOWER = np.array([0, 0, 104])
SCHOOL_END_HSV_UPPER = np.array([4, 255, 255])
SCHOOL_END_PIXEL_MIN = 1500

# main 모드 정의 (퍼셉션이 모드별로 인식 항목을 골라 켜기 위해 참조)
MODE_WAIT, MODE_CONE, MODE_LANE, MODE_LEFT_TURN, \
    MODE_LANE_CHANGE, MODE_FOLLOW, MODE_SIGNAL_WAIT, MODE_SCHOOL_ZONE = range(8)


class Perception(Node):

    def __init__(self):
        super().__init__('perception')

        self._bridge = CvBridge()
        self._detector = YoloDetector()  # 추후 weight 경로 등 파라미터 주입
        self.visualization_enabled = True  # 디버깅용 시각화 켜기/끄기

        # 최신 입력 버퍼
        self._img_front = None
        self._scan = None

        # main 으로부터 현재 모드/스테이지 수신 (필요한 인식만 수행하기 위함)
        self._mode = MODE_WAIT
        self._stage = [0, 0]

        # 누적 상태 (한 번 1이 된 래치성 값은 유지)
        self._status = [0] * STATUS_LEN

        # ---- 구독 ----
        self.create_subscription(
            Image, '/usb_cam/image_raw/front',
            self._front_cb, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, '/scan',
            self._scan_cb, qos_profile_sensor_data)
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

    def _scan_cb(self, msg):
        self._scan = msg

    def _mode_cb(self, msg):
        if msg.data != self._mode:
            self._status = [0] * STATUS_LEN
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
        # print("self._status[IDX_START_SIGNAL]", self._status[IDX_START_SIGNAL])
        # YOLO 1회 추론 결과(딕셔너리 형태 예상)
        det = self._detector.infer(self._img_front)

        # TODO(team): 아래는 자리만 잡아둔 골격. 실제 임계값/로직은 추후 구현.
        # 모드별로 필요한 항목만 갱신하면 연산 절약 가능.
        if self._mode == MODE_WAIT and self._status[IDX_START_SIGNAL] == 0:
            self._status[IDX_START_SIGNAL] = self._detect_start_signal(det)

        if self._mode == MODE_CONE and self._status[IDX_CONE_FINISHED] == 0:
            self._status[IDX_CONE_FINISHED] = self._detect_cone_finished()

        if self._mode in (MODE_LANE, MODE_SIGNAL_WAIT):
            self._status[IDX_TRAFFIC_SIGNAL] = self._detect_traffic_signal(det)
            self._status[IDX_POLICE_DETECTED] = self._detect_police(det)

        if self._mode in (MODE_LANE, MODE_FOLLOW):
            self._status[IDX_OBSTACLE_FRONT] = self._detect_obstacle_front(det, self._scan)
            self._status[IDX_LAP_LINE] = self._detect_lap_line(self._img_front)

        if self._mode == MODE_FOLLOW:
            self._status[IDX_OBSTACLE_PASSED] = self._detect_obstacle_passed(self._scan)

        if self._mode == MODE_LANE and self._stage[0] == 1:
            self._status[IDX_SHORTCUT_EXIT] = self._detect_shortcut_exit(det)

        # 1차선 주행 중 어린이보호구역 시작 마커 감지
        if self._mode == MODE_LANE and self._stage[0] == 0 \
                and self._status[IDX_SCHOOL_ZONE_SIGNAL] == 0:
            self._status[IDX_SCHOOL_ZONE_SIGNAL] = self._detect_school_zone_signal()

        # 어린이보호구역 주행 중 종료 마커(흰색) 감지
        if self._mode == MODE_SCHOOL_ZONE and self._status[IDX_SCHOOL_ZONE_END] == 0:
            self._status[IDX_SCHOOL_ZONE_END] = self._detect_school_zone_end()
        
        if self.visualization_enabled:
            self._visualize_school_zone()
        self._publish_status()

    # ------------------------------------------------------------------
    # 인식 세부 함수 (골격만; 추후 구현)
    # ------------------------------------------------------------------
    def _detect_start_signal(self, det):
        GREEN_PIXEL_MIN = 2900 # 실제로 2991개 검출됨
        ROI_X1, ROI_Y1, ROI_X2, ROI_Y2 = 200, 80, 400, 150

        roi = self._img_front[ROI_Y1:ROI_Y2, ROI_X1:ROI_X2]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,
                           np.array([40, 50, 50]),
                           np.array([80, 255, 255]))
        count = int(np.count_nonzero(mask))
        # print(f'[start_signal] green pixels: {count}')
        return 1 if count >= GREEN_PIXEL_MIN else 0

    def _detect_cone_finished(self):
        ROI_X1, ROI_Y1, ROI_X2, ROI_Y2 = 180, 340, 400, 430
        roi = self._img_front[ROI_Y1:ROI_Y2, ROI_X1:ROI_X2]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,
                           np.array([20, 50, 50]),
                           np.array([35, 255, 255]))
        count = int(np.count_nonzero(mask))
        # print(f'[cone_finished] yellow pixels: {count}')
        return 1 if count >= YELLOW_PIXEL_MIN else 0

    def _detect_traffic_signal(self, det):
        # TODO: 0=wait, 1=green(직진), 2=left_turn(좌회전 화살표)
        return self._status[IDX_TRAFFIC_SIGNAL]

    def _detect_police(self, det):
        # TODO: 경찰차 검출
        return self._status[IDX_POLICE_DETECTED]

    def _detect_obstacle_front(self, det, scan):
        FRONT_HALF_DEG = 5.0  # 전방 ±5도 (총 10도)
        MAX_DIST_M     = 7.0
        MIN_POINTS     = 10

        if scan is None:
            return self._status[IDX_OBSTACLE_FRONT]

        front_rad = np.deg2rad(FRONT_HALF_DEG)
        count = 0
        for i, r in enumerate(scan.ranges):
            if not np.isfinite(r) or r <= 0.0 or r > MAX_DIST_M:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            angle = (angle + np.pi) % (2 * np.pi) - np.pi  # -π ~ π 정규화
            if abs(angle) <= front_rad:
                count += 1
        # print(f'[obstacle_front] detected points: {count}')
        return 1 if count >= MIN_POINTS else 0

    def _detect_obstacle_passed(self, scan):
        LEFT_CENTER_DEG = 90.0   # 왼쪽 중심
        LEFT_HALF_DEG   = 45.0   # ±45도 (총 90도)
        MIN_DIST_M      = 1
        MAX_DIST_M      = 5.0
        MAX_POINTS      = 3

        if scan is None:
            return self._status[IDX_OBSTACLE_PASSED]

        if self._status[IDX_OBSTACLE_PASSED] == 1:
            return 1

        left_center = np.deg2rad(LEFT_CENTER_DEG)
        left_half   = np.deg2rad(LEFT_HALF_DEG)
        count = 0
        for i, r in enumerate(scan.ranges):
            if not np.isfinite(r) or r <= MIN_DIST_M or r > MAX_DIST_M:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            angle = (angle + np.pi) % (2 * np.pi) - np.pi
            if abs(angle - left_center) <= left_half:
                count += 1
        print(f'[obstacle_passed] left points: {count}')
        return 1 if count <= MAX_POINTS else 0

    def _detect_shortcut_exit(self, det):
        # TODO: 지름길 출구 위치 감지
        return self._status[IDX_SHORTCUT_EXIT]

    def _detect_lap_line(self, img):
        x1, y1, x2, y2 = LAP_ROI
        roi = img[y1:y2, x1:x2]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LAP_HSV_LOWER, LAP_HSV_UPPER)
        count = int(np.count_nonzero(mask))
        # print(f'[lap_line] pixels: {count}')
        return 1 if count >= LAP_PIXEL_MIN else 0

    def _detect_school_zone_signal(self):
        if self._img_front is None:
            return 0
        x1, y1, x2, y2 = SCHOOL_ROI
        roi = self._img_front[y1:y2, x1:x2]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, SCHOOL_HSV_LOWER, SCHOOL_HSV_UPPER)
        count = int(np.count_nonzero(mask))
        # print(f'[school_zone_start] pixels: {count}')
        return 1 if count >= SCHOOL_PIXEL_MIN else 0

    def _detect_school_zone_end(self):
        if self._img_front is None:
            return 0
        x1, y1, x2, y2 = SCHOOL_END_ROI
        roi = self._img_front[y1:y2, x1:x2]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, SCHOOL_END_HSV_LOWER, SCHOOL_END_HSV_UPPER)
        count = int(np.count_nonzero(mask))
        # print(f'[school_zone_end] pixels: {count}')
        return 1 if count >= SCHOOL_END_PIXEL_MIN else 0

    def _visualize_school_zone(self):
        if self._img_front is None:
            return

        vis = self._img_front.copy()
        hsv = cv2.cvtColor(vis, cv2.COLOR_BGR2HSV)

        # ── 시작 ROI (노란색) ──
        sx1, sy1, sx2, sy2 = SCHOOL_ROI
        yellow_mask = cv2.inRange(hsv[sy1:sy2, sx1:sx2],
                                  SCHOOL_HSV_LOWER, SCHOOL_HSV_UPPER)
        yellow_cnt = int(np.count_nonzero(yellow_mask))
        s_active = yellow_cnt >= SCHOOL_PIXEL_MIN
        s_color = (0, 255, 0) if s_active else (0, 180, 255)   # 초록=감지, 주황=미감지
        txt_color = (0, 0, 220)  # 빨간색 (BGR)
        cv2.rectangle(vis, (sx1, sy1), (sx2, sy2), s_color, 2)
        cv2.putText(vis, f'SCHOOL_START  yellow={yellow_cnt} (>={SCHOOL_PIXEL_MIN})',
                    (sx1, max(sy1 - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, txt_color, 2)

        # ── 종료 ROI (흰색) ──
        ex1, ey1, ex2, ey2 = SCHOOL_END_ROI
        white_mask = cv2.inRange(hsv[ey1:ey2, ex1:ex2],
                                 SCHOOL_END_HSV_LOWER, SCHOOL_END_HSV_UPPER)
        white_cnt = int(np.count_nonzero(white_mask))
        e_active = white_cnt >= SCHOOL_END_PIXEL_MIN
        e_color = (255, 0, 0) if e_active else (200, 200, 0)
        cv2.rectangle(vis, (ex1 + 2, ey1 + 2), (ex2 - 2, ey2 - 2), e_color, 2)
        cv2.putText(vis, f'SCHOOL_END  white={white_cnt} (>={SCHOOL_END_PIXEL_MIN})',
                    (ex1, ey2 + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, txt_color, 2)

        # ── 마스크 오버레이 ──
        yellow_overlay = np.zeros_like(vis)
        yellow_overlay[sy1:sy2, sx1:sx2][yellow_mask > 0] = (0, 200, 255)
        white_overlay = np.zeros_like(vis)
        white_overlay[ey1:ey2, ex1:ex2][white_mask > 0] = (255, 100, 0)
        vis = cv2.addWeighted(vis, 1.0, yellow_overlay, 0.5, 0)
        vis = cv2.addWeighted(vis, 1.0, white_overlay, 0.5, 0)

        cv2.imshow('school_zone_detect', vis)
        cv2.waitKey(1)

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
