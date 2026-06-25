#!/usr/bin/env python3
"""main 노드 — 상태머신 + 디버그 시각화.

역할:
  - /perception/status, /driving/offset, /driving/lane_change_done, /imu 를 구독
  - 주행 시퀀스(README) 에 따라 모드/스테이지 전환
  - /main/mode, /main/stage 발행 (driving, perception, control 이 참조)
  - 디버그용 OpenCV 창에 현재 상태를 시각화
    (모드 텍스트 / stage / perception 상황 / xycar_motor angle·speed / lap / offset)

실제 모터 명령(/xycar_motor) 발행은 control.py 가 담당한다.
main 은 디버그 창에서 모터값을 보기 위해 /xycar_motor 를 '구독'만 한다.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import cv2
import numpy as np

from std_msgs.msg import Int32, Int32MultiArray, Float32, Bool
from sensor_msgs.msg import Imu
from xycar_msgs.msg import XycarMotor

# ---------------------------------------------------------------------------
# 모드 정의
# ---------------------------------------------------------------------------
MODE_WAIT, MODE_CONE, MODE_LANE, MODE_LEFT_TURN, \
    MODE_LANE_CHANGE, MODE_FOLLOW, MODE_SIGNAL_WAIT, MODE_SCHOOL_ZONE = range(8)

MODE_NAME = {
    MODE_WAIT: 'WAIT',
    MODE_CONE: 'CONE',
    MODE_LANE: 'LANE',
    MODE_LEFT_TURN: 'LEFT_TURN',
    MODE_LANE_CHANGE: 'LANE_CHANGE',
    MODE_FOLLOW: 'FOLLOW',
    MODE_SIGNAL_WAIT: 'SIGNAL_WAIT',
    MODE_SCHOOL_ZONE: 'SCHOOL_ZONE',
}

# stage 인덱스
STAGE_LANE_TARGET = 0   # 0=1차선, 1=2차선
STAGE_TURN_TYPE = 1     # 0=좌회전A(진입/서쪽), 1=좌회전B(탈출/남쪽)

# 좌회전 완료 판정용 절대 heading (ROS yaw 기준)
# imu_visualizer 좌표계: yaw=0→N, yaw=π/2→W, yaw=±π→S, yaw=-π/2→E
TURN_A_TARGET_YAW = math.pi / 2    # 서쪽 (1차 좌회전 완료)
TURN_B_TARGET_YAW = math.pi        # 남쪽 (2차 좌회전 완료)
TURN_YAW_TOL = math.radians(20)    # 허용 오차 ±20도

# /perception/status 인덱스
IDX_START_SIGNAL, IDX_TRAFFIC_SIGNAL, IDX_OBSTACLE_FRONT, IDX_OBSTACLE_PASSED, \
    IDX_POLICE_DETECTED, IDX_SHORTCUT_EXIT, IDX_LAP_LINE, IDX_CONE_FINISHED, \
    IDX_SCHOOL_ZONE_SIGNAL, IDX_SCHOOL_ZONE_END = range(10)
STATUS_LEN = 10

STATUS_LABEL = [
    'start_signal', 'traffic_signal', 'obstacle_front', 'obstacle_passed',
    'police_detected', 'shortcut_exit', 'lap_line', 'cone_finished',
    'school_zone_sig', 'school_zone_end',
]

TRAFFIC_GREEN = 1
TRAFFIC_LEFT = 2


class Main(Node):

    def __init__(self):
        super().__init__('main')

        # 상태
        self._mode = MODE_WAIT
        self._stage = [0, 0]
        self._lap = 0
        # lap 카운트 잠금: True이면 어린이보호구역 통과 전까지 lap 증가 차단
        self._lap_locked = False

        # 입력 버퍼
        self._status = [0] * STATUS_LEN
        self._offset = 0.0
        self._lane_change_done = False
        self._yaw = 0.0
        self._motor_angle = 0.0
        self._motor_speed = 0.0

        # ---- 구독 ----
        self.create_subscription(
            Int32MultiArray, '/perception/status', self._status_cb, 10)
        self.create_subscription(Float32, '/driving/offset', self._offset_cb, 10)
        self.create_subscription(
            Bool, '/driving/lane_change_done', self._lane_change_cb, 10)
        self.create_subscription(Imu, '/imu', self._imu_cb, qos_profile_sensor_data)
        self.create_subscription(
            XycarMotor, '/xycar_motor', self._motor_cb, 10)

        # ---- 발행 ----
        self._mode_pub = self.create_publisher(Int32, '/main/mode', 10)
        self._stage_pub = self.create_publisher(Int32MultiArray, '/main/stage', 10)

        # 상태머신 주기 (30Hz)
        self.create_timer(1.0 / 30.0, self._fsm_tick)
        # 디버그 창 갱신 주기 (15Hz)
        self.create_timer(1.0 / 15.0, self._debug_tick)

        # 첫 모드 발행
        self._publish_mode_stage()
        self.get_logger().info('main node ready')

    # ==================================================================
    # 콜백
    # ==================================================================
    def _status_cb(self, msg):
        if len(msg.data) >= STATUS_LEN:
            self._status = list(msg.data[:STATUS_LEN])

    def _offset_cb(self, msg):
        self._offset = msg.data

    def _lane_change_cb(self, msg):
        self._lane_change_done = msg.data

    def _imu_cb(self, msg):
        self._yaw = self._quat_to_yaw(msg.orientation)

    def _motor_cb(self, msg):
        self._motor_angle = msg.angle
        self._motor_speed = msg.speed

    # ==================================================================
    # 상태머신 (README '전체 주행 시퀀스' 골격)
    # ==================================================================
    def _fsm_tick(self):
        s = self._status
        prev_mode, prev_stage = self._mode, list(self._stage)

        if self._mode == MODE_WAIT:
            # 초록불 → 라바콘 주행
            if s[IDX_START_SIGNAL] == 1:
                self._status[IDX_START_SIGNAL] = 0
                self._set_mode(MODE_CONE)

        elif self._mode == MODE_CONE:
            # 아스팔트 진입 완료 → 1차선 차선주행
            # TODO(team): 아스팔트 진입 완료 판정 조건
            if self._cone_finished():
                self._stage[STAGE_LANE_TARGET] = 0
                self._set_mode(MODE_LANE)

        elif self._mode == MODE_LANE:
            self._fsm_lane()

        elif self._mode == MODE_LANE_CHANGE:
            # 차선 변경 완료 → 2차선 저속 추종
            if self._lane_change_done:
                self._lane_change_done = False
                self._set_mode(MODE_FOLLOW)

        elif self._mode == MODE_FOLLOW:
            # 라이다 왼쪽 미감지(추월 완료) → 1차선 복귀
            # TODO(team): 왼쪽 라이다 미감지 판정 (obstacle_passed 활용 가능)
            if s[IDX_OBSTACLE_PASSED] == 1:
                self._stage[STAGE_LANE_TARGET] = 0
                self._set_mode(MODE_LANE)

        elif self._mode == MODE_SIGNAL_WAIT:
            self._fsm_signal_wait()

        elif self._mode == MODE_SCHOOL_ZONE:
            # 어린이보호구역 종료 마커(흰색) 감지 → 1차선 복귀
            if s[IDX_SCHOOL_ZONE_END] == 1:
                self._status[IDX_SCHOOL_ZONE_END] = 0
                self._stage[STAGE_LANE_TARGET] = 0
                self._lap_locked = False  # 구역 통과 완료 → lap 잠금 해제
                self._set_mode(MODE_LANE)

        elif self._mode == MODE_LEFT_TURN:
            # 목표 yaw 도달 → 다음 단계
            # TODO(team): driving.turn 의 reached 판정과 동기화 (별도 토픽 or yaw 직접 비교)
            if self._left_turn_reached():
                self._after_left_turn()

        self._publish_mode_stage()

    def _fsm_lane(self):
        s = self._status
        # 1차선 주행 중 어린이보호구역 신호 → 어린이보호구역 모드
        if s[IDX_SCHOOL_ZONE_SIGNAL] == 1 and self._stage[STAGE_LANE_TARGET] == 0:
            self._status[IDX_SCHOOL_ZONE_SIGNAL] = 0
            self._set_mode(MODE_SCHOOL_ZONE)
            return

        # 1차선 주행 중 방해차량 → 차선 변경
        if s[IDX_OBSTACLE_FRONT] == 1 and self._stage[STAGE_LANE_TARGET] == 0:
            self._stage[STAGE_LANE_TARGET] = 1   # 2차선으로
            self._set_mode(MODE_LANE_CHANGE)
            return

        # 2차선 숏컷 주행 중 지름길 출구 감지 → 좌회전B(탈출)
        if self._stage[STAGE_LANE_TARGET] == 1 and s[IDX_SHORTCUT_EXIT] == 1:
            self._stage[STAGE_TURN_TYPE] = 1
            self._set_mode(MODE_LEFT_TURN)
            return

        # 신호등 앞 도달 → 신호 대기
        # TODO(team): 신호등 앞 도달 판정 (위치/거리). 우선 traffic_signal 수신으로 대체 가능.
        if self._reached_traffic_light():
            self._set_mode(MODE_SIGNAL_WAIT)
            return

        # 출발선 통과 → lap++ (어린이보호구역 통과 전까지 중복 차단)
        if s[IDX_LAP_LINE] == 1 and not self._lap_locked:
            self._lap += 1
            self._lap_locked = True
            self._consume_lap_line()

    def _fsm_signal_wait(self):
        s = self._status
        sig = s[IDX_TRAFFIC_SIGNAL]
        if sig == TRAFFIC_LEFT:
            # 좌회전 → 지름길 진입
            self._stage[STAGE_TURN_TYPE] = 0
            self._set_mode(MODE_LEFT_TURN)
        elif sig == TRAFFIC_GREEN:
            # 직진 → 1차선 차선주행 복귀
            self._stage[STAGE_LANE_TARGET] = 0
            self._set_mode(MODE_LANE)

    def _after_left_turn(self):
        if self._stage[STAGE_TURN_TYPE] == 0:
            # 1차 좌회전(진입/서쪽) 완료 → 2차선 숏컷 주행
            self._stage[STAGE_LANE_TARGET] = 1
            self._set_mode(MODE_LANE)
        else:
            # 2차 좌회전(탈출/남쪽) 완료 → 어린이보호구역 모드 진입
            self._stage[STAGE_LANE_TARGET] = 0
            self._lap_locked = False   # 어린이보호구역 진입으로 lap 잠금 해제
            self._set_mode(MODE_SCHOOL_ZONE)

    # ------------------------------------------------------------------
    # 전환 조건 헬퍼 (구현은 추후; 자리만 잡음)
    # ------------------------------------------------------------------
    def _cone_finished(self):
        if self._status[IDX_CONE_FINISHED] == 1:
            self._status[IDX_CONE_FINISHED] = 0
            return True
        return False

    def _reached_traffic_light(self):
        # TODO(team): 신호등 앞 도달 판정
        return False

    def _left_turn_reached(self):
        # turn_type에 따른 절대 heading 기준 좌회전 완료 판정
        if self._stage[STAGE_TURN_TYPE] == 0:
            target = TURN_A_TARGET_YAW   # 서쪽 (π/2)
        else:
            target = TURN_B_TARGET_YAW   # 남쪽 (π)
        err = abs(math.atan2(math.sin(self._yaw - target),
                             math.cos(self._yaw - target)))
        return err < TURN_YAW_TOL

    def _consume_lap_line(self):
        # lap_line 래치 소비 (중복 카운트 방지)
        self._status[IDX_LAP_LINE] = 0

    # ------------------------------------------------------------------
    def _set_mode(self, mode):
        if mode != self._mode:
            self.get_logger().info(
                f'mode: {MODE_NAME.get(self._mode)} -> {MODE_NAME.get(mode)}')
        self._mode = mode

    def _publish_mode_stage(self):
        self._mode_pub.publish(Int32(data=int(self._mode)))
        m = Int32MultiArray()
        m.data = [int(v) for v in self._stage]
        self._stage_pub.publish(m)

    # ==================================================================
    # 디버그 시각화
    # ==================================================================
    def _debug_tick(self):
        img = np.zeros((500, 460, 3), dtype=np.uint8)
        white = (255, 255, 255)
        green = (0, 255, 0)
        gray = (160, 160, 160)
        yellow = (0, 220, 255)
        cyan = (255, 220, 0)
        red = (0, 80, 255)

        def put(text, y, color=white, scale=0.6):
            cv2.putText(img, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX,
                        scale, color, 1, cv2.LINE_AA)

        put('ERICAR DEBUG', 28, yellow, 0.7)
        cv2.line(img, (10, 38), (450, 38), gray, 1)

        mode_color = cyan if self._mode == MODE_SCHOOL_ZONE else green
        put(f'MODE : {MODE_NAME.get(self._mode, "?")} ({self._mode})', 66, mode_color)
        lane_txt = '2nd' if self._stage[STAGE_LANE_TARGET] == 1 else '1st'
        turn_txt = 'B(exit/S)' if self._stage[STAGE_TURN_TYPE] == 1 else 'A(enter/W)'
        put(f'STAGE: lane={lane_txt}  turn={turn_txt}', 92)
        lap_color = red if self._lap_locked else white
        put(f'LAP  : {self._lap}  {"[LOCKED]" if self._lap_locked else ""}', 118, lap_color)
        put(f'YAW  : {math.degrees(self._yaw):+.1f} deg', 144)
        put(f'OFFSET: {self._offset:+.3f}', 170)
        put(f'MOTOR : angle={self._motor_angle:+6.1f}  speed={self._motor_speed:+5.1f}', 196)

        cv2.line(img, (10, 210), (450, 210), gray, 1)
        put('PERCEPTION', 232, yellow)
        y = 256
        for i, label in enumerate(STATUS_LABEL):
            v = self._status[i] if i < len(self._status) else 0
            color = green if v else gray
            put(f'{label:<16}: {v}', y, color, 0.5)
            y += 22

        cv2.imshow('ericar_debug', img)
        key = cv2.waitKey(1) & 0xFF
        if ord('0') <= key <= ord('7'):
            self._set_mode(key - ord('0'))

    # ------------------------------------------------------------------
    @staticmethod
    def _quat_to_yaw(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = Main()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
