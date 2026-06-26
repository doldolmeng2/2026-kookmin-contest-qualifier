#!/usr/bin/env python3
"""control 노드.

driving 이 발행한 /driving/offset 과 main 이 발행한 /main/mode, /main/stage 를 받아
모드별 비례 계수와 속도 테이블로 최종 angle/speed 를 계산해 /xycar_motor 로 발행한다.

main.py 와는 별도 노드이며, "offset → 모터 명령" 변환을 전담한다.
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Int32MultiArray, Float32
from xycar_msgs.msg import XycarMotor

# ---------------------------------------------------------------------------
# 모드 정의 (README 와 동일)
# ---------------------------------------------------------------------------
MODE_WAIT, MODE_CONE, MODE_LANE, MODE_LEFT_TURN, \
    MODE_LANE_CHANGE, MODE_FOLLOW, MODE_SIGNAL_WAIT, MODE_SCHOOL_ZONE, \
    MODE_S_CURVE, MODE_SHORTCUT = range(10)

# 모드별 offset → angle 비례 계수 (angle = offset * ratio, -100~100 로 clip)
# MODE_SCHOOL_ZONE: offset은 라디안 단위 yaw 오차 → 30 곱하면 최대 ±94 수준
MODE_RATIO = {
    MODE_WAIT:          0.0,
    MODE_CONE:        250.0,
    MODE_LANE:          0.48,
    MODE_LEFT_TURN:     100.0,
    MODE_LANE_CHANGE:   0.37,
    MODE_FOLLOW:        0.45,
    MODE_SIGNAL_WAIT:   0.0,
    MODE_SCHOOL_ZONE:  200.0,
    MODE_S_CURVE:       1.4,
    MODE_SHORTCUT:    200.0,
}

# 모드별 speed (-50~50)
SPEED_TABLE = {
    MODE_WAIT:          0,
    MODE_CONE:         15,
    MODE_LANE:         17,
    MODE_LEFT_TURN:    11,
    MODE_LANE_CHANGE:  13,
    MODE_FOLLOW:       12,
    MODE_SIGNAL_WAIT:   0,
    MODE_SCHOOL_ZONE:   16,
    MODE_S_CURVE:      12,
    MODE_SHORTCUT:     18,
}

ANGLE_LIMIT = 100.0
SPEED_LIMIT = 50.0

# MODE_LANE: angle 절대값이 ANGLE_LIMIT 일 때의 최저 속도
LANE_SPEED_MIN = 7


class Control(Node):

    def __init__(self):
        super().__init__('control')

        self._mode = MODE_WAIT
        self._stage = [0, 0]
        self._offset = 0.0

        # ---- 구독 ----
        self.create_subscription(Float32, '/driving/offset', self._offset_cb, 10)
        self.create_subscription(Int32, '/main/mode', self._mode_cb, 10)
        self.create_subscription(Int32MultiArray, '/main/stage', self._stage_cb, 10)

        # ---- 발행 ----
        self._motor_pub = self.create_publisher(XycarMotor, '/xycar_motor', 10)

        # 제어 주기 (30Hz)
        self.create_timer(1.0 / 30.0, self._tick)

        self.get_logger().info('control node ready')

    # ------------------------------------------------------------------
    def _offset_cb(self, msg):
        self._offset = msg.data

    def _mode_cb(self, msg):
        self._mode = msg.data

    def _stage_cb(self, msg):
        if msg.data:
            self._stage = list(msg.data)

    # ------------------------------------------------------------------
    def _tick(self):
        angle, speed = self._compute()
        self._publish_motor(angle, speed)

    def _compute(self):
        ratio = MODE_RATIO.get(self._mode, 0.0)
        speed = SPEED_TABLE.get(self._mode, 0)

        angle = self._offset * ratio
        angle = max(-ANGLE_LIMIT, min(ANGLE_LIMIT, angle))

        if self._mode == MODE_LANE:
            base = float(SPEED_TABLE[MODE_LANE])
            drop = (base - LANE_SPEED_MIN) * abs(angle) / ANGLE_LIMIT
            speed = base - drop
        else:
            speed = max(-SPEED_LIMIT, min(SPEED_LIMIT, float(speed)))

        # 정지 모드는 조향도 0
        if self._mode in (MODE_WAIT, MODE_SIGNAL_WAIT):
            angle = 0.0

        return angle, speed

    def _publish_motor(self, angle, speed):
        msg = XycarMotor()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.angle = float(angle)
        msg.speed = float(speed)
        self._motor_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Control()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
