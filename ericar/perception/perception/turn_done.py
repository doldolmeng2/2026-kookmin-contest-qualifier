#!/usr/bin/env python3
"""IMU yaw 기반 좌회전 완료 감지.

절대 yaw 기준으로 목표 각도에 도달했는지 판정한다.
  STAGE_TURN_TYPE = 0 (좌회전A, 지름길 진입): 목표 yaw = 90°
  STAGE_TURN_TYPE = 1 (좌회전B, 지름길 탈출): 목표 yaw = 180°  (±π 동일)
"""

import math

TARGET_YAW_A = math.radians(90)   # 좌회전A: yaw ≈ 90°
TARGET_YAW_B = math.radians(180)  # 좌회전B: yaw ≈ ±180°

def detect_turn_done(turn_type, yaw_rad, tolerance):
    """turn_type에 맞는 목표 yaw에 도달했으면 True를 반환한다.

    Args:
        turn_type: 0=좌회전A, 1=좌회전B
        yaw_rad: IMU yaw (radians, -π ~ π)
        tolerance: 허용 오차 (radians)
    """
    target = TARGET_YAW_A if turn_type == 0 else TARGET_YAW_B
    diff = abs(math.atan2(
        math.sin(yaw_rad - target),
        math.cos(yaw_rad - target),
    ))
    return diff < tolerance
