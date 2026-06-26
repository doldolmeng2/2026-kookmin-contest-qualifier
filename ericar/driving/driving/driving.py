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

import cv2
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
    MODE_S_CURVE, MODE_SHORTCUT = range(10)

# stage 인덱스
STAGE_LANE_TARGET = 0   # 0=1차선, 1=2차선
STAGE_TURN_TYPE = 1     # 0=좌회전A(진입), 1=좌회전B(탈출)

# 차선 변경 완료 판정 파라미터
LANE_CHANGE_DONE_TOL    = 20   # offset 허용 범위 (픽셀)
LANE_CHANGE_MIN_ENTRY   = 15   # 모드 진입 후 무시할 틱 수 (0.5s @30Hz) — 오래된 offset 버림
LANE_CHANGE_STABLE_TICKS = 10  # 연속으로 조건을 만족해야 하는 틱 수 (~0.33s)

# FOLLOW 모드 파라미터
FOLLOW_LANE_SCALE = 1.0   # lane offset에 곱하는 비례값
FOLLOW_YAW_SCALE  = 43.0  # yaw 오차(rad) → offset 단위 변환 스케일

# SHORTCUT 모드: 첫 좌회전 후 yaw 90도 방향으로 직진
SHORTCUT_TARGET_YAW = math.radians(90)


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

        # FOLLOW 모드: lane_offset 이상값 필터용 이전 값
        self._last_follow_lane_offset = 0.0

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

        elif self._mode == MODE_LANE:
            lane_target = self._stage[STAGE_LANE_TARGET]
            offset = self._lane.compute_offset(self._img_front, lane_target)
            if self._lane.reuse_reason:
                self.get_logger().warn(
                    f'[LANE] reusing last offset={offset:.1f}  reason: {self._lane.reuse_reason}')

        elif self._mode == MODE_FOLLOW:
            lane_target = self._stage[STAGE_LANE_TARGET]
            lane_offset = self._lane.compute_offset(self._img_front, lane_target)
            if self._lane.reuse_reason:
                self.get_logger().warn(
                    f'[FOLLOW] reusing last offset={lane_offset:.1f}  reason: {self._lane.reuse_reason}')
            if lane_offset >= 380:
                self.get_logger().warn(
                    f'[FOLLOW] lane_offset={lane_offset:.1f} >= 380, using prev={self._last_follow_lane_offset:.1f}')
                lane_offset = self._last_follow_lane_offset
            else:
                self._last_follow_lane_offset = lane_offset
            yaw_err = self._normalize_angle(self._yaw + math.pi)
            offset = lane_offset * FOLLOW_LANE_SCALE + yaw_err * FOLLOW_YAW_SCALE
            self.get_logger().info(
                f'[FOLLOW] lane={lane_offset:.1f}  yaw_err={math.degrees(yaw_err):+.1f}deg  offset={offset:.1f}')
            self._visualize_follow(lane_offset, yaw_err, offset)

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

        elif self._mode == MODE_SHORTCUT:
            offset = self._shortcut_offset()
            self.get_logger().info(
                f'[SHORTCUT] yaw={math.degrees(self._yaw):.1f}deg  '
                f'offset={math.degrees(offset):.1f}deg')

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

    def _visualize_follow(self, lane_offset, yaw_err, offset):
        W, H = 360, 240
        panel = np.zeros((H, W, 3), dtype=np.uint8)

        # 제목
        cv2.putText(panel, 'FOLLOW DEBUG', (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

        # 바 그래프 그리기
        BAR_CX   = 155   # 바 중앙 x
        BAR_HALF = 120   # 바 절반 너비 (픽셀)

        def draw_bar(y, label, value, max_val, color):
            cv2.putText(panel, label, (10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (170, 170, 170), 1)
            # 배경 + 중앙선
            cv2.rectangle(panel, (BAR_CX - BAR_HALF, y - 13),
                          (BAR_CX + BAR_HALF, y + 3), (45, 45, 45), -1)
            cv2.line(panel, (BAR_CX, y - 13), (BAR_CX, y + 3), (90, 90, 90), 1)
            # 값 막대
            fill = int(np.clip(value / max_val, -1.0, 1.0) * BAR_HALF)
            if fill > 0:
                cv2.rectangle(panel, (BAR_CX, y - 12),
                              (BAR_CX + fill, y + 2), color, -1)
            elif fill < 0:
                cv2.rectangle(panel, (BAR_CX + fill, y - 12),
                              (BAR_CX, y + 2), color, -1)
            cv2.putText(panel, f'{value:+.1f}',
                        (BAR_CX + BAR_HALF + 5, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (220, 220, 220), 1)

        draw_bar(55,  'Lane offset', lane_offset,              200.0, (100, 220, 100))
        draw_bar(90,  'Yaw err(deg)', math.degrees(yaw_err),  180.0, (100, 180, 255))
        draw_bar(125, 'Combined',     offset,                  300.0, (0, 200, 255))

        # 스케일 표시
        cv2.putText(panel,
                    f'lane x{FOLLOW_LANE_SCALE:.1f}  +  yaw_err x{FOLLOW_YAW_SCALE:.1f}',
                    (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.33, (90, 90, 90), 1)

        # ── 미니 나침반 (imu_visualizer 와 동일 방향 규칙) ────────────────────
        # tip = (cx - R*sin(yaw), cy - R*cos(yaw))
        # yaw=0 → 위(N), yaw=±π → 아래(S=-180°)
        CC = (290, 145)   # 나침반 중심
        R  = 55
        cv2.circle(panel, CC, R, (60, 60, 60), 2)

        # N/S 눈금
        for deg, lbl in [(0, 'N'), (180, 'S')]:
            rad = math.radians(deg)
            ex = int(CC[0] - R * math.sin(rad))
            ey = int(CC[1] - R * math.cos(rad))
            cv2.circle(panel, (ex, ey), 3, (70, 70, 70), -1)
            cv2.putText(panel, lbl, (ex - 4, ey + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.30, (90, 90, 90), 1)

        # 목표(-180°=S) 강조 표시
        target_pt = (CC[0], CC[1] + R)
        cv2.circle(panel, target_pt, 5, (0, 80, 255), -1)
        cv2.putText(panel, '-180', (target_pt[0] - 14, target_pt[1] + 13),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.28, (0, 80, 255), 1)

        # 현재 yaw 화살표 (녹색)
        tip = (int(CC[0] - (R - 10) * math.sin(self._yaw)),
               int(CC[1] - (R - 10) * math.cos(self._yaw)))
        cv2.arrowedLine(panel, CC, tip, (0, 220, 0), 2, tipLength=0.22)

        # yaw 수치
        cv2.putText(panel, f'{math.degrees(self._yaw):+.0f}',
                    (CC[0] - 14, CC[1] + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 180), 1)

        cv2.imshow('follow_debug', panel)
        cv2.waitKey(1)

    def _school_zone_offset(self):
        # target yaw = ±180° (= ±π rad). offset이 0이면 정렬 완료.
        return self._normalize_angle(self._yaw + math.pi)

    def _shortcut_offset(self):
        # target yaw = 90° (= π/2 rad). offset이 0이면 정렬 완료.
        return self._normalize_angle(self._yaw - SHORTCUT_TARGET_YAW)

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
