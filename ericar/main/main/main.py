#!/usr/bin/env python3
"""
main 노드 — 상태머신 + 디버그 시각화.

역할:
- /perception/status, /driving/offset, /driving/*_done, /imu 구독
- 인식 결과와 주행 완료 이벤트에 따라 모드/스테이지 전환
- /main/mode, /main/stage 발행
- 디버그 창에 현재 상태와 모터 명령 표시

실제 모터 명령(/xycar_motor)은 control.py가 담당한다.
"""

import math

import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, Int32, Int32MultiArray
from xycar_msgs.msg import XycarMotor


# ---------------------------------------------------------------------------
# 모드 정의
# ---------------------------------------------------------------------------

MODE_WAIT, MODE_CONE, MODE_LANE, MODE_LEFT_TURN, \
    MODE_LANE_CHANGE, MODE_FOLLOW, MODE_SIGNAL_WAIT = range(7)

MODE_NAME = {
    MODE_WAIT: 'WAIT',
    MODE_CONE: 'CONE',
    MODE_LANE: 'LANE',
    MODE_LEFT_TURN: 'LEFT_TURN',
    MODE_LANE_CHANGE: 'LANE_CHANGE',
    MODE_FOLLOW: 'FOLLOW',
    MODE_SIGNAL_WAIT: 'SIGNAL_WAIT',
}


# ---------------------------------------------------------------------------
# stage 정의
# ---------------------------------------------------------------------------

STAGE_LANE_TARGET = 0  # 0=1차선, 1=2차선
STAGE_TURN_TYPE = 1    # 0=좌회전A(진입), 1=좌회전B(탈출)


# ---------------------------------------------------------------------------
# /perception/status 인덱스
# ---------------------------------------------------------------------------

IDX_START_SIGNAL = 0
IDX_TRAFFIC_SIGNAL = 1
IDX_OBSTACLE_FRONT = 2
IDX_OBSTACLE_PASSED = 3
IDX_POLICE_DETECTED = 4
IDX_SHORTCUT_EXIT = 5
IDX_LAP_LINE = 6
IDX_PEDESTRIAN = 7
IDX_SCHOOL_ZONE = 8
IDX_TRAFFIC_PRESENT = 9
IDX_POLICE_READY = 10

STATUS_LEN = 11

STATUS_LABEL = [
    'start_signal',
    'traffic_signal',
    'obstacle_front',
    'obstacle_passed',
    'police_detected',
    'shortcut_exit',
    'lap_line',
    'pedestrian',
    'school_zone',
    'traffic_present',
    'police_ready',
]

TRAFFIC_NONE = 0
TRAFFIC_GREEN = 1
TRAFFIC_LEFT = 2
TRAFFIC_STOP = 3

# 연속 프레임 기반 디바운스 값
TRAFFIC_PRESENT_FRAMES = 5
PEDESTRIAN_DANGER_FRAMES = 3
PEDESTRIAN_CLEAR_FRAMES = 8
SIGNAL_COOLDOWN_FRAMES = 60
STOP_LINE_CONFIRM_FRAMES = 2


class Main(Node):
    def __init__(self):
        super().__init__('main')

        # ------------------------------------------------------------------
        # 상태
        # ------------------------------------------------------------------

        self._mode = MODE_WAIT
        self._stage = [0, 0]
        self._lap = 0

        # 방해차량 인식이 완성되기 전까지 기본적으로 비활성화한다.
        self._enable_obstacle_mission = bool(
            self.declare_parameter(
                'enable_obstacle_mission',
                False,
            ).value
        )

        # 디버그 창이 필요 없거나 WSL에서 Qt 경고가 많을 때 끌 수 있다.
        self._show_debug = bool(
            self.declare_parameter(
                'show_debug',
                True,
            ).value
        )

        # ------------------------------------------------------------------
        # 입력 버퍼와 이벤트
        # ------------------------------------------------------------------

        self._status = [0] * STATUS_LEN
        self._offset = 0.0

        self._lane_change_done = False
        self._turn_done = False
        self._cone_done = False
        self._lap_line_event = False

        self._lane_change_reason = 'obstacle'
        self._follow_phase = 'passing'

        self._yaw = 0.0
        self._motor_angle = 0.0
        self._motor_speed = 0.0

        # 보행자 정지 후 복귀할 모드
        self._wait_reason = None
        self._resume_mode = MODE_LANE
        self._pedestrian_danger_count = 0
        self._pedestrian_clear_count = 0

        # 트랙 신호등 및 경로 선택 상태
        self._traffic_present_count = 0
        self._traffic_armed = False

        self._stop_line_near = False
        self._stop_line_count = 0

        self._signal_cooldown_frames = 0
        # 경찰차가 현재 lap에서 한 번이라도 검출되면,
        # 화면에서 사라져도 다음 lap 시작 전까지 기억한다.
        self._police_seen_this_lap = False

        # 동일 lap에서 같은 트랙 신호등을 다시 처리하지 않는다.
        self._route_decision_lap = -1

        self._shortcut_active = False

        # ------------------------------------------------------------------
        # 구독
        # ------------------------------------------------------------------

        self.create_subscription(
            Int32MultiArray,
            '/perception/status',
            self._status_cb,
            10,
        )

        self.create_subscription(
            Bool,
            '/perception/stop_line_near',
            self._stop_line_cb,
            10,
        )

        self.create_subscription(
            Float32,
            '/driving/offset',
            self._offset_cb,
            10,
        )

        self.create_subscription(
            Bool,
            '/driving/lane_change_done',
            self._lane_change_cb,
            10,
        )

        self.create_subscription(
            Bool,
            '/driving/turn_done',
            self._turn_done_cb,
            10,
        )

        self.create_subscription(
            Bool,
            '/driving/cone_done',
            self._cone_done_cb,
            10,
        )

        self.create_subscription(
            Imu,
            '/imu',
            self._imu_cb,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            XycarMotor,
            '/xycar_motor',
            self._motor_cb,
            10,
        )

        # ------------------------------------------------------------------
        # 발행
        # ------------------------------------------------------------------

        self._mode_pub = self.create_publisher(
            Int32,
            '/main/mode',
            10,
        )

        self._stage_pub = self.create_publisher(
            Int32MultiArray,
            '/main/stage',
            10,
        )

        # 상태머신 주기
        self.create_timer(
            1.0 / 30.0,
            self._fsm_tick,
        )

        # 다른 노드가 늦게 실행돼도 현재 상태를 받을 수 있도록 반복 발행한다.
        self.create_timer(
            0.1,
            self._publish_mode_stage,
        )

        if self._show_debug:
            self.create_timer(
                1.0 / 15.0,
                self._debug_tick,
            )

        self._publish_mode_stage()

        self.get_logger().info(
            'main node ready '
            f'(obstacle_mission={self._enable_obstacle_mission}, '
            f'debug={self._show_debug})'
        )

    # ==================================================================
    # 콜백
    # ==================================================================

    def _status_cb(self, msg):
        if len(msg.data) < STATUS_LEN:
            self.get_logger().warn(
                f'perception status 길이 부족: '
                f'{len(msg.data)} < {STATUS_LEN}'
            )
            return

        new_status = list(msg.data[:STATUS_LEN])

        previous_lap_line = self._status[IDX_LAP_LINE]
        current_lap_line = new_status[IDX_LAP_LINE]

        # 출발선이 여러 프레임 유지돼도 0→1 순간만 이벤트로 저장한다.
        if current_lap_line == 1 and previous_lap_line == 0:
            self._lap_line_event = True

        self._status = new_status

    def _stop_line_cb(self, msg):
        self._stop_line_near = bool(msg.data)

    def _offset_cb(self, msg):
        self._offset = float(msg.data)

    def _lane_change_cb(self, msg):
        if msg.data:
            self._lane_change_done = True

    def _turn_done_cb(self, msg):
        if msg.data:
            self._turn_done = True

    def _cone_done_cb(self, msg):
        if msg.data:
            self._cone_done = True

    def _imu_cb(self, msg):
        self._yaw = self._quat_to_yaw(msg.orientation)

    def _motor_cb(self, msg):
        self._motor_angle = float(msg.angle)
        self._motor_speed = float(msg.speed)

    # ==================================================================
    # 상태머신
    # ==================================================================

    def _fsm_tick(self):
        previous_mode = self._mode
        previous_stage = list(self._stage)

        if self._signal_cooldown_frames > 0:
            self._signal_cooldown_frames -= 1

        # 경찰차는 현재 lap에서 한 번이라도 검출되면 유지한다.
        if self._status[IDX_POLICE_DETECTED] == 1:
            self._police_seen_this_lap = True

        if self._status[IDX_TRAFFIC_PRESENT] == 1:
            self._traffic_present_count += 1
        else:
            self._traffic_present_count = 0

        if (
            self._traffic_present_count
            >= TRAFFIC_PRESENT_FRAMES
        ):
            self._traffic_armed = True

        if self._stop_line_near:
            self._stop_line_count += 1
        else:
            self._stop_line_count = 0

        # 주행 중 위험 보행자는 모드 처리보다 우선한다.
        if self._handle_pedestrian():
            self._publish_if_changed(previous_mode, previous_stage)
            return

        if self._mode == MODE_WAIT:
            if self._status[IDX_START_SIGNAL] == 1:
                self._set_mode(
                    MODE_CONE,
                    '출발 초록불 감지',
                )

        elif self._mode == MODE_CONE:
            if self._cone_finished():
                self._stage[STAGE_LANE_TARGET] = 0
                self._stage[STAGE_TURN_TYPE] = 0
                self._signal_cooldown_frames = SIGNAL_COOLDOWN_FRAMES

                self._set_mode(
                    MODE_LANE,
                    '라바콘 종료 및 차선 안정 검출',
                )

        elif self._mode == MODE_LANE:
            self._fsm_lane()

        elif self._mode == MODE_LANE_CHANGE:
            self._fsm_lane_change()

        elif self._mode == MODE_FOLLOW:
            self._fsm_follow()

        elif self._mode == MODE_SIGNAL_WAIT:
            self._fsm_signal_wait()

        elif self._mode == MODE_LEFT_TURN:
            if self._left_turn_reached():
                self._after_left_turn()

        self._publish_if_changed(previous_mode, previous_stage)

    def _fsm_lane(self):
        s = self._status

        # 지름길 주행 중 정면 도로가 끝나면 좌회전B로 탈출한다.
        if (
            self._shortcut_active
            and self._stage[STAGE_LANE_TARGET] == 1
            and s[IDX_SHORTCUT_EXIT] == 1
        ):
            self._stage[STAGE_TURN_TYPE] = 1

            self._set_mode(
                MODE_LEFT_TURN,
                '지름길 출구 감지',
            )
            return

        # 첫 외곽 lap을 마친 뒤부터 트랙 신호등 경로 선택을 수행한다.
        if self._reached_traffic_light():
            self._wait_reason = 'signal'

            self._set_mode(
                MODE_SIGNAL_WAIT,
                '트랙 신호등 감지',
            )
            return

        # 최신 perception status에서는 우측 차선 합류 가능 신호가 제거되었다.
        # 안전한 합류 조건을 다시 정의하기 전까지 방해차량 차선 변경은 실행하지 않는다.

        if self._lap_line_event:
            self._lap += 1
            self._consume_lap_line()

            self._traffic_armed = False
            self._traffic_present_count = 0
            self._stop_line_near = False
            self._stop_line_count = 0

            # 경찰차 검출 기록은 경로 결정 직후가 아니라
            # 새로운 lap이 시작되는 시점에만 초기화한다.
            self._police_seen_this_lap = False
            self._signal_cooldown_frames = SIGNAL_COOLDOWN_FRAMES

            self.get_logger().info(
                f'lap 증가: {self._lap}, '
                '경찰차 검출 latch 초기화'
            )

    def _fsm_lane_change(self):
        if not self._lane_change_done:
            return

        self._lane_change_done = False

        if self._lane_change_reason == 'obstacle':
            self._follow_phase = 'passing'

            self._set_mode(
                MODE_FOLLOW,
                '2차선 진입 완료, 방해차량 추월 시작',
            )
        else:
            self._set_mode(
                MODE_LANE,
                '1차선 복귀 완료',
            )

    def _fsm_follow(self):
        s = self._status


        # 2차선에서 옆 방해차량을 지나친 뒤 1차선으로 복귀한다.
        if s[IDX_OBSTACLE_PASSED] == 1:
            self._stage[STAGE_LANE_TARGET] = 0
            self._lane_change_reason = 'return'

            self._set_mode(
                MODE_LANE_CHANGE,
                '방해차량 추월 완료, 1차선 복귀',
            )

    def _fsm_signal_wait(self):
        if self._wait_reason == 'pedestrian':
            # 보행자 해제 처리는 _handle_pedestrian에서 수행한다.
            return

        signal = self._status[IDX_TRAFFIC_SIGNAL]
        police_ready = (
            self._status[IDX_POLICE_READY] == 1
        )

        # --------------------------------------------------------------
        # 첫 번째 lap:
        # 경찰차 검출 결과와 관계없이 초록불에 무조건 직진한다.
        # --------------------------------------------------------------
        if self._lap == 1:
            if signal != TRAFFIC_GREEN:
                return

            self._stage[STAGE_LANE_TARGET] = 0
            self._shortcut_active = False
            self._wait_reason = None
            self._signal_cooldown_frames = SIGNAL_COOLDOWN_FRAMES
            self._route_decision_lap = self._lap

            self._set_mode(
                MODE_LANE,
                '첫 번째 lap: 초록불 직진',
            )
            return

        # --------------------------------------------------------------
        # 두 번째 lap 이후:
        # 이번 lap에서 경찰차를 한 번이라도 봤다면,
        # 이후 화면에서 사라져도 경찰차가 있었던 것으로 판단한다.
        # --------------------------------------------------------------
        if self._police_seen_this_lap:
            if signal != TRAFFIC_GREEN:
                return

            self._stage[STAGE_LANE_TARGET] = 0
            self._shortcut_active = False
            self._wait_reason = None
            self._signal_cooldown_frames = SIGNAL_COOLDOWN_FRAMES
            self._route_decision_lap = self._lap

            self._set_mode(
                MODE_LANE,
                '경찰차 감지 기록 유지: 초록불 직진',
            )
            return

        # 경찰차가 검출되지 않았더라도 detector가 준비되지 않았다면
        # 경찰차 없음으로 판단하지 않고 정지 상태를 유지한다.
        if not police_ready:
            return

        # detector가 준비됐고 이번 lap에서 경찰차를 한 번도 보지 못한
        # 경우에만 좌회전 신호를 받아 지름길로 진입한다.
        if signal != TRAFFIC_LEFT:
            return

        self._stage[STAGE_TURN_TYPE] = 0
        self._shortcut_active = True
        self._wait_reason = None
        self._signal_cooldown_frames = SIGNAL_COOLDOWN_FRAMES
        self._route_decision_lap = self._lap

        self._set_mode(
            MODE_LEFT_TURN,
            '경찰차 미검출 확정: 좌회전 신호 후 지름길 진입',
        )

    def _after_left_turn(self):
        turn_type = self._stage[STAGE_TURN_TYPE]

        if turn_type == 0:
            # 좌회전A 완료 후 지름길 2차선 기준으로 차선 주행한다.
            self._stage[STAGE_LANE_TARGET] = 1
            self._shortcut_active = True
            reason = '좌회전A 완료, 지름길 진입'

        else:
            # 좌회전B 완료 후 외곽 1차선 기준으로 복귀한다.
            self._stage[STAGE_LANE_TARGET] = 0
            self._shortcut_active = False
            reason = '좌회전B 완료, 외곽 1차선 복귀'

        self._signal_cooldown_frames = SIGNAL_COOLDOWN_FRAMES

        self._set_mode(
            MODE_LANE,
            reason,
        )

    # ==================================================================
    # 공통 안전 처리
    # ==================================================================

    def _handle_pedestrian(self):
        # 보행자 미션은 어린이 보호구역과 독립적이다.
        # 곡선 차선 구간에서 전방 위험 보행자를 연속 감지하면 정지한다.
        pedestrian_danger = (
            self._status[IDX_PEDESTRIAN] == 1
        )

        moving_modes = (
            MODE_LANE,
            MODE_LANE_CHANGE,
            MODE_FOLLOW,
        )

        if self._mode in moving_modes:
            if pedestrian_danger:
                self._pedestrian_danger_count += 1
            else:
                self._pedestrian_danger_count = 0

            # 순간적인 한 프레임 오검출에는 정지하지 않는다.
            if (
                self._pedestrian_danger_count
                >= PEDESTRIAN_DANGER_FRAMES
            ):
                self._resume_mode = self._mode
                self._wait_reason = 'pedestrian'
                self._pedestrian_danger_count = 0
                self._pedestrian_clear_count = 0

                self._set_mode(
                    MODE_SIGNAL_WAIT,
                    '곡선 차선 전방 보행자 연속 감지',
                )
                return True

            return False

        if (
            self._mode == MODE_SIGNAL_WAIT
            and self._wait_reason == 'pedestrian'
        ):
            if pedestrian_danger:
                self._pedestrian_clear_count = 0
            else:
                self._pedestrian_clear_count += 1

            if (
                self._pedestrian_clear_count
                >= PEDESTRIAN_CLEAR_FRAMES
            ):
                resume_mode = self._resume_mode

                self._wait_reason = None
                self._pedestrian_danger_count = 0
                self._pedestrian_clear_count = 0

                self._set_mode(
                    resume_mode,
                    '보행자 위험 연속 해제',
                )

            return True

        self._pedestrian_danger_count = 0
        return False

    # ==================================================================
    # 전환 조건 헬퍼
    # ==================================================================

    def _cone_finished(self):
        if self._cone_done:
            self._cone_done = False
            self.get_logger().info(
                '라바콘 종료 이벤트 수신'
            )
            return True

        return False

    def _reached_traffic_light(self):
        # 첫 번째 lap부터 트랙 신호등 경로 판단을 수행한다.
        if self._lap < 1:
            return False

        # 이미 현재 lap의 직진/좌회전 경로를 결정했다면
        # 화면에 같은 신호등이 계속 남아 있어도 다시 정지하지 않는다.
        if self._route_decision_lap == self._lap:
            return False

        if self._shortcut_active:
            return False

        if self._signal_cooldown_frames > 0:
            return False

        reached = (
            self._traffic_armed
            and self._stop_line_count
            >= STOP_LINE_CONFIRM_FRAMES
        )

        if reached:
            # 같은 검출값으로 재진입하지 않도록 즉시 소모한다.
            self._traffic_armed = False
            self._traffic_present_count = 0
            self._stop_line_count = 0

        return reached

    def _left_turn_reached(self):
        if self._turn_done:
            self._turn_done = False
            return True

        return False

    def _consume_lap_line(self):
        self._lap_line_event = False

    # ==================================================================
    # 모드 및 stage 발행
    # ==================================================================

    def _set_mode(self, mode, reason=''):
        if mode == self._mode:
            return

        suffix = f' / 이유: {reason}' if reason else ''

        self.get_logger().info(
            f'mode: {MODE_NAME.get(self._mode)} '
            f'-> {MODE_NAME.get(mode)}{suffix}'
        )

        self._mode = mode

    def _publish_if_changed(
        self,
        previous_mode,
        previous_stage,
    ):
        if (
            self._mode != previous_mode
            or self._stage != previous_stage
        ):
            self._publish_mode_stage()

    def _publish_mode_stage(self):
        # driving이 새 mode를 받을 때 최신 stage가 준비되도록 stage부터 발행한다.
        stage_msg = Int32MultiArray()
        stage_msg.data = [
            int(value) for value in self._stage
        ]
        self._stage_pub.publish(stage_msg)

        self._mode_pub.publish(
            Int32(data=int(self._mode))
        )

    # ==================================================================
    # 디버그 시각화
    # ==================================================================

    def _debug_tick(self):
        image = np.zeros(
            (570, 500, 3),
            dtype=np.uint8,
        )

        white = (255, 255, 255)
        green = (0, 255, 0)
        gray = (160, 160, 160)
        yellow = (0, 220, 255)

        def put(text, y, color=white, scale=0.55):
            cv2.putText(
                image,
                text,
                (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                scale,
                color,
                1,
                cv2.LINE_AA,
            )

        put('ERICAR DEBUG', 28, yellow, 0.7)
        cv2.line(image, (10, 38), (490, 38), gray, 1)

        put(
            f'MODE : {MODE_NAME.get(self._mode, "?")} '
            f'({self._mode})',
            65,
            green,
        )

        lane_text = (
            '2nd'
            if self._stage[STAGE_LANE_TARGET] == 1
            else '1st'
        )

        turn_text = (
            'B(exit)'
            if self._stage[STAGE_TURN_TYPE] == 1
            else 'A(enter)'
        )

        put(
            f'STAGE: lane={lane_text} turn={turn_text}',
            90,
        )

        put(f'LAP : {self._lap}', 115)
        put(f'OFFSET: {self._offset:+.3f}', 140)

        put(
            f'MOTOR: angle={self._motor_angle:+6.1f} '
            f'speed={self._motor_speed:+5.1f}',
            165,
        )

        put(
            f'WAIT: {self._wait_reason or "-"}  '
            f'POLICE_LATCH: {int(self._police_seen_this_lap)}',
            190,
        )

        cv2.line(image, (10, 205), (490, 205), gray, 1)
        put('PERCEPTION', 228, yellow)

        y = 252

        for index, label in enumerate(STATUS_LABEL):
            value = (
                self._status[index]
                if index < len(self._status)
                else 0
            )

            color = green if value else gray

            put(
                f'{label:<18}: {value}',
                y,
                color,
                0.48,
            )

            y += 22

        ped_raw = int(
            self._status[IDX_PEDESTRIAN]
        )
        ped_wait = int(
            self._wait_reason == 'pedestrian'
        )

        put(
            (
                f'PED RAW:{ped_raw} '
                f'DANGER:{self._pedestrian_danger_count}/'
                f'{PEDESTRIAN_DANGER_FRAMES} '
                f'WAIT:{ped_wait}'
            ),
            525,
            green if ped_raw else gray,
        )

        put(
            (
                f'STOP RAW:{int(self._stop_line_near)} '
                f'COUNT:{self._stop_line_count}/'
                f'{STOP_LINE_CONFIRM_FRAMES} '
                f'ARMED:{int(self._traffic_armed)}'
            ),
            550,
            green if self._stop_line_near else gray,
        )

        cv2.imshow('ericar_debug', image)
        cv2.waitKey(1)

    # ------------------------------------------------------------------

    @staticmethod
    def _quat_to_yaw(q):
        siny_cosp = 2.0 * (
            q.w * q.z + q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y + q.z * q.z
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp,
        )


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
