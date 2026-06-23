#!/usr/bin/env python3
"""좌회전 → 조향 오프셋 계산.

driving.py 에서 import 해서 사용한다. ROS 토픽은 다루지 않고,
IMU yaw 를 기준으로 목표 yaw 까지 회전하도록 offset 을 만든다.

turn_type:
    0 = 좌회전A (지름길 진입)
    1 = 좌회전B (지름길 탈출)
두 경우 목표 yaw 가 다르다 (README 참고).
"""

import math

# 좌회전은 일반적으로 +90도. 진입/탈출에서 목표가 다르면 여기서 조정.
TURN_A_DELTA = math.radians(90)   # 지름길 진입 시 회전량
TURN_B_DELTA = math.radians(90)   # 지름길 탈출 시 회전량

# 좌회전 중 최대 왼쪽 조향 offset (control 비례식과 합의 필요)
MAX_LEFT_OFFSET = -1.0
YAW_TOLERANCE = math.radians(5)   # 목표 도달 판정 허용 오차


class LeftTurner:

    def __init__(self):
        self._target_yaw = None
        self._active = False

    def start(self, turn_type, current_yaw):
        """좌회전 진입 시 1회 호출. 현재 yaw 기준 목표 yaw 를 확정한다."""
        delta = TURN_A_DELTA if turn_type == 0 else TURN_B_DELTA
        self._target_yaw = self._normalize(current_yaw + delta)
        self._active = True

    def compute_offset(self, current_yaw):
        """목표 yaw 도달 전까지 최대 왼쪽 offset, 도달하면 0.0 반환."""
        if not self._active or self._target_yaw is None:
            return 0.0
        if self.reached(current_yaw):
            self._active = False
            return 0.0
        return MAX_LEFT_OFFSET

    def reached(self, current_yaw):
        """목표 yaw 에 도달했는지 여부 (main 이 모드 전환 판단에 참고)."""
        if self._target_yaw is None:
            return False
        err = self._normalize(self._target_yaw - current_yaw)
        return abs(err) < YAW_TOLERANCE

    @staticmethod
    def _normalize(angle):
        """[-pi, pi] 로 정규화."""
        return math.atan2(math.sin(angle), math.cos(angle))
