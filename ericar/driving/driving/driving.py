#!/usr/bin/env python3
"""driving 노드.

main 으로부터 현재 모드(/main/mode)와 스테이지(/main/stage)를 받고,
카메라/라이다/IMU 를 구독해 모드별 조향 오프셋을 계산해 /driving/offset 으로 발행한다.
차선 변경이 끝나면 /driving/lane_change_done 을 발행한다.

오프셋 계산 알고리즘은 아래 로직 모듈로 분리하고 여기서 import 해서 쓴다:
    lane_detection.LaneDetector   : 카메라 차선 인식 → offset
    rubbercone.ConeDriver         : 카메라 라바콘 중심 → offset (카메라 기반으로 변경)
    turn_left.LeftTurner          : IMU yaw 기반 좌회전 offset
이 파일은 토픽 sub/pub, 콜백, 모드 분기만 담당한다.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Int32, Int32MultiArray, Float32, Bool
from sensor_msgs.msg import Image, LaserScan, Imu
from cv_bridge import CvBridge

# 로직 모듈
from driving.lane_detection import LaneDetector
from driving.rubbercone import ConeDriver
from driving.turn_left import LeftTurner

# ---------------------------------------------------------------------------
# 모드 정의 (README 와 동일)
# ---------------------------------------------------------------------------
MODE_WAIT, MODE_CONE, MODE_LANE, MODE_LEFT_TURN, \
    MODE_LANE_CHANGE, MODE_FOLLOW, MODE_SIGNAL_WAIT = range(7)

# stage 인덱스
STAGE_LANE_TARGET = 0   # 0=1차선, 1=2차선
STAGE_TURN_TYPE = 1     # 0=좌회전A(진입), 1=좌회전B(탈출)

# 차선 변경 완료로 볼 offset 허용 범위
LANE_CHANGE_DONE_TOL = 0.1

class Driving(Node):
    def __init__(self):
        super().__init__('driving')
        self._bridge = CvBridge()

        # 로직 모듈 인스턴스
        self._lane = LaneDetector()
        self._cone = ConeDriver()
        self._turn = LeftTurner()

        # 입력 버퍼
        self._img_front = None
        self._scan = None
        self._yaw = 0.0

        # main 상태
        self._mode = MODE_WAIT
        self._stage = [0, 0]
        self._lane_change_sent = False

        # 좌회전 완료 중복 발행 방지 플래그
        self._turn_reached = False

        # 카메라 구독 시 RELIABLE QoS 사용 (시뮬레이터 발행자가 RELIABLE이므로)
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---- 구독 ----
        self.create_subscription(
            Image, '/usb_cam/image_raw/front',
            self._front_cb, qos_reliable)
        self.create_subscription(
            LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(
            Imu, '/imu', self._imu_cb, qos_profile_sensor_data)
        self.create_subscription(
            Int32, '/main/mode', self._mode_cb, 10)
        self.create_subscription(
            Int32MultiArray, '/main/stage', self._stage_cb, 10)

        # ---- 발행 ----
        self._offset_pub = self.create_publisher(Float32, '/driving/offset', 10)
        self._lane_change_pub = self.create_publisher(
            Bool, '/driving/lane_change_done', 10)
        # 좌회전 완료 신호 발행 (main이 구독해서 모드 전환에 활용)
        self._turn_done_pub = self.create_publisher(
            Bool, '/driving/turn_done', 10)

        # 계산 + 발행 주기 (30Hz)
        self.create_timer(1.0 / 30.0, self._tick)
        self.get_logger().info('driving node ready')

    # ------------------------------------------------------------------
    # 콜백
    # ------------------------------------------------------------------
    def _front_cb(self, msg):
        self._img_front = self._bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _scan_cb(self, msg):
        self._scan = msg

    def _imu_cb(self, msg):
        self._yaw = self._quat_to_yaw(msg.orientation)

    def _mode_cb(self, msg):
        if msg.data != self._mode:
            # 모드 진입 시 1회 초기화
            self._on_mode_enter(msg.data)
        self._mode = msg.data

    def _stage_cb(self, msg):
        if msg.data:
            self._stage = list(msg.data)

    def _on_mode_enter(self, new_mode):
        if new_mode == MODE_LANE_CHANGE:
            self._lane_change_sent = False
        if new_mode == MODE_LEFT_TURN:
            # turn_type 에 따라 목표 yaw 설정
            self._turn.start(self._stage[STAGE_TURN_TYPE], self._yaw)
            # 좌회전 완료 플래그 리셋
            self._turn_reached = False

    # ------------------------------------------------------------------
    # 주기 처리: 모드별 offset 계산 → 발행
    # ------------------------------------------------------------------
    def _tick(self):
        offset = None

        if self._mode == MODE_CONE:
            # 라바콘 구간: 카메라 기반 주황색 검출로 중앙 offset 계산
            offset = self._cone.compute_offset(self._img_front)
        elif self._mode in (MODE_LANE, MODE_FOLLOW):
            lane_target = self._stage[STAGE_LANE_TARGET]
            offset = self._lane.compute_offset(self._img_front, lane_target)
        elif self._mode == MODE_LANE_CHANGE:
            lane_target = self._stage[STAGE_LANE_TARGET]
            offset = self._lane.compute_offset(self._img_front, lane_target)
            self._check_lane_change_done(offset)
        elif self._mode == MODE_LEFT_TURN:
            offset = self._turn.compute_offset(self._yaw)
            # 좌회전 완료 시 turn_done 토픽 발행 (중복 발행 방지)
            if self._turn.reached(self._yaw) and not self._turn_reached:
                self._turn_done_pub.publish(Bool(data=True))
                self._turn_reached = True
                self.get_logger().info('left turn done')

        # MODE_WAIT / MODE_SIGNAL_WAIT : offset 발행 불필요
        if offset is not None:
            self._publish_offset(offset)

    def _check_lane_change_done(self, offset):
        if self._lane_change_sent or offset is None:
            return
        # offset 이 목표 차선 기준 정렬 범위에 들어오면 완료로 판단
        if abs(offset) < LANE_CHANGE_DONE_TOL:
            self._lane_change_pub.publish(Bool(data=True))
            self._lane_change_sent = True
            self.get_logger().info('lane change done')

    # ------------------------------------------------------------------
    def _publish_offset(self, offset):
        self._offset_pub.publish(Float32(data=float(offset)))

    @staticmethod
    def _quat_to_yaw(q):
        import math
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = Driving()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
