#!/usr/bin/env python3
"""driving 노드.

main 으로부터 현재 모드(/main/mode)와 스테이지(/main/stage)를 받고,
카메라/라이다/IMU 를 구독해 모드별 조향 오프셋을 계산해 /driving/offset 으로 발행한다.
차선 변경이 끝나면 /driving/lane_change_done 을 발행한다.

오프셋 계산 알고리즘은 아래 로직 모듈로 분리하고 여기서 import 해서 쓴다:
    lane_detection.LaneDetector   : 카메라 차선 인식 → offset
    rubbercone.ConeDriver         : 라이다 라바콘 중심 → offset
    turn_left.LeftTurner          : IMU yaw 기반 좌회전 offset
이 파일은 토픽 sub/pub, 콜백, 모드 분기만 담당한다.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Int32, Int32MultiArray, Float32, Bool
from sensor_msgs.msg import Image, LaserScan, Imu
import numpy as np

# 로직 모듈
from driving.lane_detection import LaneDetector, SCurveDetector
from driving.rubbercone import ConeDriver
from driving.turn_left import LeftTurner

# ---------------------------------------------------------------------------
# 모드 정의 (README 와 동일)
# ---------------------------------------------------------------------------
MODE_WAIT, MODE_CONE, MODE_LANE, MODE_LEFT_TURN, \
    MODE_LANE_CHANGE, MODE_FOLLOW, MODE_SIGNAL_WAIT, MODE_SCHOOL_ZONE, \
    MODE_S_CURVE = range(9)

# stage 인덱스
STAGE_LANE_TARGET = 0   # 0=1차선, 1=2차선
STAGE_TURN_TYPE = 1     # 0=좌회전A(진입), 1=좌회전B(탈출)

# 차선 변경 완료 판정 파라미터
LANE_CHANGE_DONE_TOL    = 20   # offset 허용 범위 (픽셀)
LANE_CHANGE_MIN_ENTRY   = 15   # 모드 진입 후 무시할 틱 수 (0.5s @30Hz) — 오래된 offset 버림
LANE_CHANGE_STABLE_TICKS = 10  # 연속으로 조건을 만족해야 하는 틱 수 (~0.33s)


class Driving(Node):

    def __init__(self):
        super().__init__('driving')

        # 로직 모듈 인스턴스
        self._lane   = LaneDetector()
        self._scurve = SCurveDetector()
        self._cone   = ConeDriver()
        self._turn   = LeftTurner()

        # 입력 버퍼
        self._img_front = None
        self._scan = None
        self._yaw = 0.0

        # main 상태
        self._mode = MODE_WAIT
        self._stage = [0, 0]
        self._lane_change_sent = False
        self._lane_change_entry_ticks = 0   # 모드 진입 후 경과 틱
        self._lane_change_stable_count = 0  # 연속 조건 만족 틱

        # ---- 구독 ----
        self.create_subscription(
            Image, '/usb_cam/image_raw/front',
            self._front_cb, qos_profile_sensor_data)
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

        # 계산 + 발행 주기 (30Hz)
        self.create_timer(1.0 / 30.0, self._tick)

        self.get_logger().info('driving node ready')

    # ------------------------------------------------------------------
    # 콜백
    # ------------------------------------------------------------------
    def _front_cb(self, msg):
        img = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape((msg.height, msg.width, 3))
        if msg.encoding == 'rgb8':
            img = img[:, :, ::-1]
        self._img_front = img

    def _scan_cb(self, msg):
        # print("scan msg received")
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
            self._lane_change_entry_ticks = 0
            self._lane_change_stable_count = 0
        if new_mode == MODE_LEFT_TURN:
            # turn_type 에 따라 목표 yaw 설정
            self._turn.start(self._stage[STAGE_TURN_TYPE], self._yaw)

    # ------------------------------------------------------------------
    # 주기 처리: 모드별 offset 계산 → 발행
    # ------------------------------------------------------------------
    def _tick(self):
        offset = None

        if self._mode == MODE_CONE:
            offset = self._cone.compute_offset(self._scan)
            self._cone.visualize(self._scan)

        elif self._mode in (MODE_LANE, MODE_FOLLOW):
            lane_target = self._stage[STAGE_LANE_TARGET]
            offset = self._lane.compute_offset(self._img_front, lane_target)
            if self._lane.reuse_reason:
                self.get_logger().warn(
                    f'[LANE] reusing last offset={offset:.1f}  reason: {self._lane.reuse_reason}')

        elif self._mode == MODE_SCHOOL_ZONE:
            offset = self._school_zone_offset()
            self.get_logger().info(f'[SCHOOL_ZONE] yaw={math.degrees(self._yaw):.1f}deg  offset={math.degrees(offset):.1f}deg')

        elif self._mode == MODE_LANE_CHANGE:
            lane_target = self._stage[STAGE_LANE_TARGET]
            offset = self._lane.compute_offset(self._img_front, lane_target)
            if self._lane.reuse_reason:
                self.get_logger().warn(
                    f'[LANE_CHANGE] reusing last offset={offset:.1f}  reason: {self._lane.reuse_reason}')
            self._check_lane_change_done(offset)

        elif self._mode == MODE_S_CURVE:
            offset = self._scurve.compute_offset(self._img_front, self._yaw)

        elif self._mode == MODE_LEFT_TURN:
            offset = self._turn.compute_offset(self._yaw)

        if self._img_front is not None:
            self._lane.visualize_yellow(self._img_front)

        # MODE_WAIT / MODE_SIGNAL_WAIT : offset 발행 불필요
        if offset is not None:
            self._publish_offset(offset)

    def _check_lane_change_done(self, offset):
        if self._lane_change_sent or offset is None:
            return

        self._lane_change_entry_ticks += 1
        # 진입 직후 오래된 offset 값은 무시
        if self._lane_change_entry_ticks < LANE_CHANGE_MIN_ENTRY:
            return

        if abs(offset) < LANE_CHANGE_DONE_TOL:
            self._lane_change_stable_count += 1
        else:
            self._lane_change_stable_count = 0

        if self._lane_change_stable_count >= LANE_CHANGE_STABLE_TICKS:
            self._lane_change_pub.publish(Bool(data=True))
            self._lane_change_sent = True
            self.get_logger().info(
                f'lane change done (stable {LANE_CHANGE_STABLE_TICKS} ticks)')

    def _school_zone_offset(self):
        # target yaw = ±180° (= ±π rad). offset이 0이면 정렬 완료.
        return self._normalize_angle(self._yaw + math.pi)

    @staticmethod
    def _normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    # ------------------------------------------------------------------
    def _publish_offset(self, offset):
        self._offset_pub.publish(Float32(data=float(offset)))

    @staticmethod
    def _quat_to_yaw(q):
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
