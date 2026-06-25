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
from sensor_msgs.msg import Imu, Image
from xycar_msgs.msg import XycarMotor

# ---------------------------------------------------------------------------
# 모드 정의
# ---------------------------------------------------------------------------
MODE_WAIT, MODE_CONE, MODE_LANE, MODE_LEFT_TURN, \
    MODE_LANE_CHANGE, MODE_FOLLOW, MODE_SIGNAL_WAIT, MODE_SCHOOL_ZONE, \
    MODE_S_CURVE = range(9)

MODE_NAME = {
    MODE_WAIT: 'WAIT',
    MODE_CONE: 'CONE',
    MODE_LANE: 'LANE',
    MODE_LEFT_TURN: 'LEFT_TURN',
    MODE_LANE_CHANGE: 'LANE_CHANGE',
    MODE_FOLLOW: 'FOLLOW',
    MODE_SIGNAL_WAIT: 'SIGNAL_WAIT',
    MODE_SCHOOL_ZONE: 'SCHOOL_ZONE',
    MODE_S_CURVE: 'S_CURVE',
}

# stage 인덱스
STAGE_LANE_TARGET = 0   # 0=1차선, 1=2차선
STAGE_TURN_TYPE = 1     # 0=좌회전A(진입/서쪽), 1=좌회전B(탈출/남쪽)

# /perception/status 인덱스 — perception.py의 IDX_* 정의와 반드시 일치해야 함
IDX_START_SIGNAL    = 0
IDX_TRAFFIC_SIGNAL  = 1   # 0=미인식, 1=직진, 2=좌회전, 3=주황/빨간(정지)
IDX_OBSTACLE_FRONT  = 2
IDX_OBSTACLE_PASSED = 3
IDX_POLICE_DETECTED = 4
IDX_SHORTCUT_EXIT   = 5
IDX_LAP_LINE        = 6
IDX_PEDESTRIAN      = 7
IDX_SCHOOL_ZONE     = 8   # 0=없음, 1=시작(감속)
IDX_TRAFFIC_PRESENT = 9
IDX_POLICE_READY    = 10
IDX_LEFT_TURN_DONE  = 11  # perception이 좌회전 완료 시 1로 세팅
IDX_S_CURVE         = 12  # 0=S자 아님, 1=S자 구간 끝 감지
STATUS_LEN = 13

STATUS_LABEL = [
    'start_signal', 'traffic_signal', 'obstacle_front', 'obstacle_passed',
    'police_detected', 'shortcut_exit', 'lap_line', 'pedestrian',
    'school_zone', 'traffic_present', 'police_ready', 'left_turn_done',
    's_curve',
]

TRAFFIC_NONE  = 0
TRAFFIC_GREEN = 1   # 직진
TRAFFIC_LEFT  = 2   # 좌회전
TRAFFIC_STOP  = 3   # 주황/빨간

# cone → lane 전환: lane_detection.py 와 동일한 HSV 범위·ROI
_YELLOW_LOWER = np.array([25, 205,  80])
_YELLOW_UPPER = np.array([31, 255, 255])
_YELLOW_ROI_Y1 = 270
_YELLOW_ROI_Y2 = 310
_CONE_YELLOW_MIN = 350   # ROI 내 노란 픽셀 수 ≥ 이 값이면 중앙 차선 검출 완료

# lane → s_curve 전환: offset 절대값이 CURVE_OFFSET_TOL 이상이면 S자 구간 진입
CURVE_OFFSET_TOL = 43 

class Main(Node):

    def __init__(self):
        super().__init__('main')

        self._img_front = None

        # 상태
        self._mode = MODE_WAIT
        self._stage = [0, 0]
        self._lap = 0
        self._lap_locked = False      # 출발선 중복 카운트 방지 (스쿨존 통과 후 해제)
        self._police_seen = False     # 이번 lap에서 경찰차 인식 여부 (시나리오 분기)
        self._ignore_police = False   # 방해차량 추월 시작 후 경찰차 무시 플래그
        self._tl_done = False         # 이번 lap에서 신호등 처리 완료 여부
        self._s_curve_flag = 0        # MODE_LANE에서 신호등 감지 후 S자 구간 대비 플래그
        self._left_exit_flag = 0      # LEFT_TURN A 진입 후 숏컷 출구 감지 대기 플래그

        # 입력 버퍼
        self._status = [0] * STATUS_LEN
        self._offset = 0.0
        self._lane_change_done = False
        self._yaw = 0.0
        self._motor_angle = 0.0
        self._motor_speed = 0.0

        # ---- 구독 ----
        self.create_subscription(
            Image, '/usb_cam/image_raw/front', self._front_cb, 10)
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
    def _front_cb(self, msg):
        img = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape((msg.height, msg.width, 3))
        if msg.encoding == 'rgb8':
            img = img[:, :, ::-1]
        self._img_front = img

    def _status_cb(self, msg):
        for i, v in enumerate(msg.data):
            if i < STATUS_LEN:
                self._status[i] = int(v)

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

        if self._mode == MODE_WAIT:
            if s[IDX_START_SIGNAL] == 1:
                self._status[IDX_START_SIGNAL] = 0
                self._set_mode(MODE_CONE)

        elif self._mode == MODE_CONE:
            if self._cone_finished():
                self._stage[STAGE_LANE_TARGET] = 0
                self._set_mode(MODE_LANE)

        elif self._mode == MODE_LANE:
            self._fsm_lane()

        elif self._mode == MODE_LANE_CHANGE:
            if self._lane_change_done:
                self._lane_change_done = False
                if self._stage[STAGE_LANE_TARGET] == 0:
                    # 1차선 변경 완료 → 차선 주행
                    self._set_mode(MODE_LANE)
                else:
                    # 2차선 변경 완료 → 방해차량 추종
                    self._set_mode(MODE_FOLLOW)

        elif self._mode == MODE_FOLLOW:
            if s[IDX_OBSTACLE_PASSED] == 1:
                # 추월 완료 → 반대 차선으로 LANE_CHANGE
                self._stage[STAGE_LANE_TARGET] = 1 - self._stage[STAGE_LANE_TARGET]
                self._set_mode(MODE_LANE_CHANGE)

        elif self._mode == MODE_SIGNAL_WAIT:
            self._fsm_signal_wait()

        elif self._mode == MODE_SCHOOL_ZONE:
            if s[IDX_SCHOOL_ZONE] == 0:
                self._lap_locked = False
                if self._stage[STAGE_LANE_TARGET] == 0:
                    # 1차선 스쿨존(lap1) → 바로 LANE
                    self._set_mode(MODE_LANE)
                else:
                    # 2차선 스쿨존(lap2/3) → 1차선으로 LANE_CHANGE
                    self._stage[STAGE_LANE_TARGET] = 0
                    self._set_mode(MODE_LANE_CHANGE)

        elif self._mode == MODE_LEFT_TURN:
            if s[IDX_LEFT_TURN_DONE] == 1:
                self._after_left_turn()

        elif self._mode == MODE_S_CURVE:
            if s[IDX_S_CURVE] == 1:
                self._stage[STAGE_LANE_TARGET] = 1
                self._set_mode(MODE_FOLLOW)

        self._publish_mode_stage()

    def _fsm_lane(self):
        s = self._status

        if self._lap == 0:
            # ── lap1: 방해차량 추월 → 스쿨존(1차선) → 출발선 ──

            # 신호등 감지 시 S자 구간 대비 플래그 설정
            if s[IDX_TRAFFIC_SIGNAL] in (1, 2, 3):
                self._s_curve_flag = 1

            if self._s_curve_flag == 1 and abs(self._offset) >= CURVE_OFFSET_TOL:
                self._set_mode(MODE_S_CURVE)
                return

            if s[IDX_SCHOOL_ZONE] == 1 and self._stage[STAGE_LANE_TARGET] == 0:
                self._set_mode(MODE_SCHOOL_ZONE)
                return

            if s[IDX_OBSTACLE_FRONT] == 1 and self._stage[STAGE_LANE_TARGET] == 0:
                self._stage[STAGE_LANE_TARGET] = 1
                self._set_mode(MODE_LANE_CHANGE)
                return

            if s[IDX_LAP_LINE] == 1 and not self._lap_locked:
                self._lap += 1
                self._lap_locked = True
                self._consume_lap_line()

        else:
            # ── lap2/3: 경찰차 flag → 신호등 → 시나리오 분기 ──

            # 경찰차 인식 기억 (추월 시작 후에는 무시)
            if s[IDX_POLICE_DETECTED] == 1 and not self._ignore_police:
                self._police_seen = True

            # 신호등 감지 → SIGNAL_WAIT (이번 lap에서 아직 처리 안 했을 때만)
            if s[IDX_TRAFFIC_PRESENT] == 1 and not self._tl_done:
                self._set_mode(MODE_SIGNAL_WAIT)
                return

            # 신호등 통과 후 S자 구간 대비 플래그 설정 (SIGNAL_WAIT 분기 이후에만 실행)
            if s[IDX_TRAFFIC_SIGNAL] in (1, 2, 3):
                self._s_curve_flag = 1

            if self._s_curve_flag == 1 and abs(self._offset) >= CURVE_OFFSET_TOL:
                self._set_mode(MODE_S_CURVE)
                return

            # 시나리오1: 신호 통과 후 방해차량 감지 → 2차선으로 LANE_CHANGE
            # 이 시점부터 lap_line까지 경찰차 재인식 무시
            if (s[IDX_OBSTACLE_FRONT] == 1
                    and self._stage[STAGE_LANE_TARGET] == 0
                    and self._police_seen):
                self._ignore_police = True
                self._stage[STAGE_LANE_TARGET] = 1
                self._set_mode(MODE_LANE_CHANGE)
                return

            # 시나리오2: LEFT_TURN A 이후 숏컷 주행 중 출구 감지 → LEFT_TURN B
            if self._left_exit_flag == 1 and s[IDX_SHORTCUT_EXIT] == 1:
                self._left_exit_flag = 0
                self._stage[STAGE_TURN_TYPE] = 1
                self._set_mode(MODE_LEFT_TURN)
                return

            # 스쿨존 진입 (1차선에서 감지되더라도 2차선 모드로 전환)
            if s[IDX_SCHOOL_ZONE] == 1:
                self._stage[STAGE_LANE_TARGET] = 1
                self._set_mode(MODE_SCHOOL_ZONE)
                return

            # 출발선 통과 → lap++ 및 플래그 초기화
            if s[IDX_LAP_LINE] == 1 and not self._lap_locked:
                self._lap += 1
                self._lap_locked = True
                self._police_seen = False
                self._ignore_police = False
                self._tl_done = False
                self._consume_lap_line()

    def _fsm_signal_wait(self):
        s = self._status
        sig = s[IDX_TRAFFIC_SIGNAL]
        if self._police_seen:
            # 시나리오1: 직진(1) 신호 → LANE 복귀 후 방해차량 대기
            if sig == TRAFFIC_GREEN:
                self._tl_done = True
                self._set_mode(MODE_LANE)
        else:
            # 시나리오2: 좌회전(2) 신호 → LEFT_TURN A(진입)
            if sig == TRAFFIC_LEFT:
                self._tl_done = True
                self._stage[STAGE_TURN_TYPE] = 0
                self._left_exit_flag = 1  # 숏컷 출구 감지 대기 시작
                self._set_mode(MODE_LEFT_TURN)

    def _after_left_turn(self):
        if self._stage[STAGE_TURN_TYPE] == 0:
            # A 완료(진입) → 2차선 숏컷 주행
            self._stage[STAGE_LANE_TARGET] = 1
            self._set_mode(MODE_LANE)
        else:
            # B 완료(탈출) → 스쿨존 진입 (2차선 유지)
            self._set_mode(MODE_SCHOOL_ZONE)

    # ------------------------------------------------------------------
    # 전환 조건 헬퍼 (구현은 추후; 자리만 잡음)
    # ------------------------------------------------------------------
    def _cone_finished(self):
        if self._img_front is None:
            return False
        roi = self._img_front[_YELLOW_ROI_Y1:_YELLOW_ROI_Y2, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, _YELLOW_LOWER, _YELLOW_UPPER)
        print(f"yellow pixel count: {np.count_nonzero(mask)}")
        return int(np.count_nonzero(mask)) >= _CONE_YELLOW_MIN

    def _consume_lap_line(self):
        # lap_line 래치 소비 (중복 카운트 방지)
        self._status[IDX_LAP_LINE] = 0

    # ------------------------------------------------------------------
    def _set_mode(self, mode):
        if mode != self._mode:
            self.get_logger().info(
                f'mode: {MODE_NAME.get(self._mode)} -> {MODE_NAME.get(mode)}')
            if self._mode == MODE_LANE and mode != MODE_LANE:
                self._s_curve_flag = 0
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
        img = np.zeros((630, 460, 3), dtype=np.uint8)
        white = (255, 255, 255)
        green = (0, 255, 0)
        gray = (160, 160, 160)
        yellow = (0, 220, 255)
        cyan = (255, 220, 0)
        red = (0, 80, 255)
        orange = (0, 165, 255)

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
        police_col = orange if self._police_seen else gray
        put(
            f'POLICE: seen={int(self._police_seen)}'
            f'  ignore={int(self._ignore_police)}'
            f'  tl_done={int(self._tl_done)}',
            222, police_col, 0.5,
        )
        scurve_col = orange if self._s_curve_flag else gray
        put(
            f'S_CURVE_FLAG: {self._s_curve_flag}'
            f'  LEFT_EXIT_FLAG: {self._left_exit_flag}'
            f'  offset: {self._offset:+.1f}',
            244, scurve_col, 0.5,
        )

        cv2.line(img, (10, 258), (450, 258), gray, 1)
        put('PERCEPTION', 278, yellow)
        y = 298
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
