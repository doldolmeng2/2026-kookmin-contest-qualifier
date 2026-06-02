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

# 로직 모듈 (구현은 추후 팀원이 채움)
from perception.yolo_detector import YoloDetector

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
STATUS_LEN = 7

# main 모드 정의 (퍼셉션이 모드별로 인식 항목을 골라 켜기 위해 참조)
MODE_WAIT, MODE_CONE, MODE_LANE, MODE_LEFT_TURN, \
    MODE_LANE_CHANGE, MODE_FOLLOW, MODE_SIGNAL_WAIT = range(7)


class Perception(Node):

    def __init__(self):
        super().__init__('perception')

        self._bridge = CvBridge()
        self._detector = YoloDetector()  # 추후 weight 경로 등 파라미터 주입

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

        # YOLO 1회 추론 결과(딕셔너리 형태 예상)
        det = self._detector.infer(self._img_front)

        # TODO(team): 아래는 자리만 잡아둔 골격. 실제 임계값/로직은 추후 구현.
        # 모드별로 필요한 항목만 갱신하면 연산 절약 가능.
        if self._mode == MODE_WAIT:
            self._status[IDX_START_SIGNAL] = self._detect_start_signal(det)

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

        self._publish_status()

    # ------------------------------------------------------------------
    # 인식 세부 함수 (골격만; 추후 구현)
    # ------------------------------------------------------------------
    def _detect_start_signal(self, det):
        # TODO: 초록불 검출 → 1
        return self._status[IDX_START_SIGNAL]

    def _detect_traffic_signal(self, det):
        # TODO: 0=wait, 1=green(직진), 2=left_turn(좌회전 화살표)
        return self._status[IDX_TRAFFIC_SIGNAL]

    def _detect_police(self, det):
        # TODO: 경찰차 검출
        return self._status[IDX_POLICE_DETECTED]

    def _detect_obstacle_front(self, det, scan):
        # TODO: 카메라 박스 + 라이다 전방 거리 보조로 방해차량 판정
        return self._status[IDX_OBSTACLE_FRONT]

    def _detect_obstacle_passed(self, scan):
        # TODO: 왼쪽/측면 라이다로 첫 방해차량 추월 완료 판정
        return self._status[IDX_OBSTACLE_PASSED]

    def _detect_shortcut_exit(self, det):
        # TODO: 지름길 출구 위치 감지
        return self._status[IDX_SHORTCUT_EXIT]

    def _detect_lap_line(self, img):
        # TODO: 출발선(가로선) 통과 감지
        return self._status[IDX_LAP_LINE]

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
