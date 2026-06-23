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

import cv2
import numpy as np

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

# 차선 변경 직후 offset이 일시적으로 0에 가까워지는 오검출을 막는다.
LANE_CHANGE_MIN_FRAMES = 15
LANE_CHANGE_STABLE_FRAMES = 6

# 라바콘 구간 종료 판정에 사용하는 연속 프레임 수
# driving 노드가 30Hz이므로 18프레임은 약 0.6초이다.
CONE_MISSING_FRAMES = 18
LANE_VISIBLE_FRAMES = 12

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
        self._lane_change_ticks = 0
        self._lane_change_stable_count = 0

        # 좌회전 완료 중복 발행 방지 플래그
        self._turn_reached = False

        # 라바콘 구간 종료 판정 상태
        self._cone_seen_once = False
        self._cone_missing_count = 0
        self._lane_visible_count = 0
        self._cone_done_sent = False
        # 방해차량 차선 결정 (perception 연계)
        self._obs_lane        = 0   # 0=1차선, 1=2차선
        self._lap_count       = 0
        self._lap_line_prev   = 0
        self._obs_front_latch = False

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
        self.create_subscription(
            Int32MultiArray, '/perception/status',
            self._perception_cb, 10)

        # ---- 발행 ----
        self._offset_pub = self.create_publisher(Float32, '/driving/offset', 10)
        self._lane_change_pub = self.create_publisher(
            Bool, '/driving/lane_change_done', 10)
        # 좌회전 완료 신호 발행 (main이 구독해서 모드 전환에 활용)
        self._turn_done_pub = self.create_publisher(
            Bool, '/driving/turn_done', 10)


        # 라바콘 통과 후 차선 주행으로 전환할 때 main에 한 번 발행한다.
        self._cone_done_pub = self.create_publisher(
            Bool, '/driving/cone_done', 10)
        # 차선 디버그 이미지 publisher
        self._lane_debug_pub = self.create_publisher(Image, '/lane_debug', 10)
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
            old_stage = list(self._stage)
            self._stage = list(msg.data)
            if self._stage != old_stage:
                self.get_logger().info(
                    f'STAGE 수신: {old_stage} -> {self._stage}, '
                    f'현재 mode={self._mode}'
                )

    def _perception_cb(self, msg):
        """LAP1: 홈=1차선, 앞차→2차선, 왼쪽차 사라짐→1차선
        LAP2/3: 홈=2차선, 앞차→1차선, 왼쪽차 사라짐→2차선"""
        if len(msg.data) < 7:
            return
        status = list(msg.data)

        # 랩라인 엣지 감지
        lap_line = status[6]
        if lap_line == 1 and self._lap_line_prev == 0:
            self._lap_count += 1
            self.get_logger().info(f'[OBS] lap={self._lap_count}')
            if self._lap_count >= 2:
                self._obs_lane = 1
                self._obs_front_latch = False
                self.get_logger().info('[OBS] LAP2+ 홈=2차선')
        self._lap_line_prev = lap_line

        home = 1 if self._lap_count >= 2 else 0
        away = 1 - home

        # 앞차 감지 → 회피 차선
        if status[2] == 1 and self._obs_lane == home:
            self._obs_front_latch = True
        if self._obs_front_latch and self._obs_lane == home:
            if self._is_merge_clear():
                self._obs_lane = away
                self.get_logger().info(f'[OBS] 앞차 → {away+1}차선')

        # 왼쪽 차 사라짐 → 홈 복귀
        if status[3] == 1 and self._obs_lane == away:
            self._obs_lane = home
            self._obs_front_latch = False
            self.get_logger().info(f'[OBS] 복귀 → {home+1}차선')

    def _is_merge_clear(self):
        import math
        if self._scan is None:
            return True
        r = self._scan.ranges
        n = len(r)
        vals = [r[d] for d in range(315, 350)
                if d < n and math.isfinite(r[d]) and r[d] > 0.3]
        return (min(vals) if vals else float('inf')) > 3.0

            if self._stage != old_stage:
                self.get_logger().info(
                    f'STAGE 수신: {old_stage} -> {self._stage}, '
                    f'현재 mode={self._mode}'
                )

    def _on_mode_enter(self, new_mode):
        if new_mode == MODE_CONE:
            # 새 라바콘 미션에 진입할 때 종료 판정 상태를 초기화한다.
            self._cone_seen_once = False
            self._cone_missing_count = 0
            self._lane_visible_count = 0
            self._cone_done_sent = False

        if new_mode == MODE_LANE_CHANGE:
            self._lane_change_sent = False
            self._lane_change_ticks = 0
            self._lane_change_stable_count = 0
        if new_mode == MODE_LEFT_TURN:
            turn_type = self._stage[STAGE_TURN_TYPE]

            self.get_logger().info(
                'LEFT_TURN 시작: '
                f'turn_type={turn_type}, '
                f'stage={self._stage}, '
                f'start_yaw={self._yaw:.3f}'
            )

            # turn_type에 따라 목표 yaw 설정
            self._turn.start(turn_type, self._yaw)

            # 좌회전 완료 플래그 리셋
            self._turn_reached = False

    # ------------------------------------------------------------------
    # 주기 처리: 모드별 offset 계산 → 발행
    # ------------------------------------------------------------------
    def _tick(self):
        offset = None

        if self._mode == MODE_CONE:
            # 라바콘 구간: 카메라 기반 주황색 검출로 중앙 offset 계산
            offset = self._cone.compute_offset(self._scan)  # 라이다 기반으로 변경

            # 기존 조향 계산 이후 라바콘 구간 종료 조건을 확인한다.
            self._check_cone_done()
        elif self._mode in (MODE_LANE, MODE_FOLLOW):
            lane_target = self._obs_lane
            offset = self._lane.compute_offset(self._img_front, lane_target)
            if self._lane.last_debug_img is not None:
                self._lane_debug_pub.publish(
                    self._bridge.cv2_to_imgmsg(self._lane.last_debug_img, 'bgr8'))
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


    def _check_cone_done(self):
        """라바콘 통로가 사라지고 차선이 나타나면 완료 신호를 발행한다."""
        if self._cone_done_sent:
            return

        # ConeDriver는 현재 프레임에서 좌우 클러스터를 찾으면
        # 해당 방향의 age를 0으로 갱신한다.
        left_age = int(getattr(self._cone, 'left_age', 999))
        right_age = int(getattr(self._cone, 'right_age', 999))

        # 좌우 라바콘이 같은 프레임에서 모두 검출되어야
        # 실제 라바콘 통로를 한 번 통과한 것으로 인정한다.
        cone_pair_visible = left_age == 0 and right_age == 0

        if cone_pair_visible:
            if not self._cone_seen_once:
                self.get_logger().info('좌우 라바콘 통로 최초 감지')

            self._cone_seen_once = True
            self._cone_missing_count = 0

        elif self._cone_seen_once:
            self._cone_missing_count += 1

        if self._is_lane_visible():
            self._lane_visible_count += 1
        else:
            self._lane_visible_count = 0

        # 라바콘 통로를 실제로 한 번 감지한 뒤,
        # 좌우 라바콘 쌍이 약 0.6초 동안 사라지고
        # 노란 차선이 연속으로 검출될 때 차선 모드로 전환한다.
        if (
            self._cone_seen_once
            and self._cone_missing_count >= CONE_MISSING_FRAMES
            and self._lane_visible_count >= LANE_VISIBLE_FRAMES
        ):
            self._cone_done_pub.publish(Bool(data=True))
            self._cone_done_sent = True

            self.get_logger().info(
                '라바콘 구간 종료 감지: '
                f'cone_missing={self._cone_missing_count}, '
                f'lane_visible={self._lane_visible_count}'
            )

    def _is_lane_visible(self):
        """차선 전환 판정용으로 노란 차선의 존재 여부만 확인한다."""
        if self._img_front is None:
            return False

        image = self._img_front
        height = image.shape[0]

        # 기존 LaneDetector와 동일하게 영상 아래쪽 40%를 사용한다.
        roi_top = int(height * 0.6)
        roi = image[roi_top:height, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([20, 100, 100], dtype=np.uint8)
        upper_yellow = np.array([35, 255, 255], dtype=np.uint8)

        mask = cv2.inRange(
            hsv,
            lower_yellow,
            upper_yellow,
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            np.ones((5, 5), dtype=np.uint8),
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            np.ones((3, 3), dtype=np.uint8),
        )

        ys, xs = np.where(mask > 0)

        if len(xs) < 80:
            return False

        # 점 형태의 노란 물체보다 세로로 이어진 차선을 우선한다.
        vertical_span = int(np.max(ys) - np.min(ys))

        return vertical_span >= 25

    def _check_lane_change_done(self, offset):
        if self._lane_change_sent or offset is None:
            return

        self._lane_change_ticks += 1

        # 새 목표 차선이 적용되기 전에 들어온 이전 offset은 무시한다.
        if self._lane_change_ticks < LANE_CHANGE_MIN_FRAMES:
            return

        if abs(offset) < LANE_CHANGE_DONE_TOL:
            self._lane_change_stable_count += 1
        else:
            self._lane_change_stable_count = 0

        if (
            self._lane_change_stable_count
            >= LANE_CHANGE_STABLE_FRAMES
        ):
            self._lane_change_pub.publish(
                Bool(data=True)
            )
            self._lane_change_sent = True

            self.get_logger().info(
                'lane change done: '
                f'offset={offset:+.3f}, '
                f'stable={self._lane_change_stable_count}'
            )

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
