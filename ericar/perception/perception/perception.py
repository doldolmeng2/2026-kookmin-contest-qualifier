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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data

from std_msgs.msg import Int32, Int32MultiArray
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge

# 로직 모듈
from perception.yolo_detector import YoloDetector
from perception.traffic_light import (
    TrafficLightDetector, START_OVERRIDES, TRACK_OVERRIDES,
    SIGNAL_NONE, SIGNAL_GREEN, SIGNAL_LEFT)
from perception.start_line import detect_start_line
from perception.shortcut_exit import detect_shortcut_exit
from perception.obstacle import detect_obstacle_front
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

# 보행자 '위험(정지)' 판정 기준 — 박스가 가깝고(큼) 진행경로(중앙) 안일 때만.
#   h(세로크기, 0~1)가 클수록 가깝다. 실측 분포: 중앙0.16 / p90 0.42 / max 0.57
PED_DANGER_MIN_H = 0.35          # 이 이상이면 '바로 앞'
PED_DANGER_X_BAND = (0.30, 0.70)  # 박스 중심 x 가 이 범위(차 진행경로) 안

# main 모드 정의 (퍼셉션이 모드별로 인식 항목을 골라 켜기 위해 참조)
MODE_WAIT, MODE_CONE, MODE_LANE, MODE_LEFT_TURN, \
    MODE_LANE_CHANGE, MODE_FOLLOW, MODE_SIGNAL_WAIT = range(7)


class Perception(Node):

    def __init__(self):
        super().__init__('perception')

        self._bridge = CvBridge()
        self._detector = YoloDetector()  # CPU 추론, 패키지 내부 police.pt 사용
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
            'start', four_lamp=False, overrides=START_OVERRIDES,
            debug=True, logger=self.get_logger())
        self._tl_track = TrafficLightDetector(
            'track', four_lamp=True, overrides=TRACK_OVERRIDES,
            debug=True, logger=self.get_logger())

        # 어린이 보호구역 (노면 노란 글자 토글). show=True 면 cv2 창으로 화면 표시
        self._school_zone = SchoolZoneDetector(
            logger=self.get_logger(), debug=True, show=True)

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
        self._yolo_every = 5   # 30Hz / 5 = 6Hz 추론

        # 추월완료 판정용(좌측 카메라): 옆에 차를 봤는지 + 사라짐 디바운스
        self._left_car_seen = False
        self._left_gone = 0
        self._GONE_FRAMES = 5   # 차가 5틱(~0.17s) 연속 안 보이면 '추월 완료'(깜빡임 방지)

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
        # FOLLOW(2차선 추월) 진입 시 추월완료 추적 상태 초기화
        if msg.data == MODE_FOLLOW and self._mode != MODE_FOLLOW:
            self._left_car_seen = False
            self._left_gone = 0
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
        # YOLO 가 police_car 를 하나라도 잡으면 1, 아니면 0
        return 1 if det.get('police_car') else 0

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
        # 라이다 전방 섹터에 앞차가 가까이 있으면 1
        return 1 if detect_obstacle_front(scan) else 0

    def _detect_obstacle_passed(self, scan):
        # 추월완료: 좌측 카메라로 옆 방해차량(라임색)을 '봤다가 → 사라지면' 1.
        #   라이다는 도로변과 안 갈라져서, 차 색을 보는 카메라로 판정(더 견고).
        if car_in_left(self._img_left):
            self._left_car_seen = True
            self._left_gone = 0
            return 0
        # 안 보임: 본 적 있고 + 연속으로 충분히 안 보이면(깜빡임 방지) 추월완료
        if self._left_car_seen:
            self._left_gone += 1
            if self._left_gone >= self._GONE_FRAMES:
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
