#!/usr/bin/env python3
"""라바콘(러버콘) 주행 → 조향 오프셋 계산.

driving.py 에서 import 해서 사용한다. ROS 토픽은 다루지 않고,
"라이다 LaserScan → 양쪽 라바콘 사이 중심 → offset(float)" 변환만 책임진다.
"""

import math


class ConeDriver:

    def __init__(self):
        # TODO(team): 라바콘 탐색 각도 범위, 거리 임계값 등 파라미터
        self.max_range = 3.0          # m, 이보다 먼 점은 무시
        self.front_half_angle = math.radians(80)  # 전방 ±각도만 사용

    def compute_offset(self, scan):
        """좌/우 라바콘 군집의 중심을 잡아 차량이 가운데로 가도록 offset 반환.

        Args:
            scan: sensor_msgs/LaserScan (없으면 None)
        Returns:
            float offset (-1 ~ 1), 미검출 시 0.0
        """
        if scan is None:
            return 0.0

        left_pts, right_pts = self._split_left_right(scan)
        if not left_pts and not right_pts:
            return 0.0

        # TODO(team): 군집화/대표점 선정 후 좌우 중심 평균으로 목표 횡방향 위치 산출
        target_lateral = self._estimate_center(left_pts, right_pts)
        # 정규화 (차폭 기준 등) — 추후 조정
        offset = max(-1.0, min(1.0, target_lateral))
        return float(offset)

    def _split_left_right(self, scan):
        """스캔 포인트를 (x 전방, y 횡방향) 로 변환해 전방 좌/우로 분리."""
        left, right = [], []
        angle = scan.angle_min
        for r in scan.ranges:
            a = angle
            angle += scan.angle_increment
            if not (0.0 < r < self.max_range) or math.isinf(r) or math.isnan(r):
                continue
            if abs(a) > self.front_half_angle:
                continue
            x = r * math.cos(a)   # 전방
            y = r * math.sin(a)   # 좌(+)/우(-)
            (left if y >= 0 else right).append((x, y))
        return left, right

    def _estimate_center(self, left_pts, right_pts):
        # TODO(team): 좌우 콘 중심의 y 평균 → 차량이 향해야 할 횡방향 오차
        ly = sum(p[1] for p in left_pts) / len(left_pts) if left_pts else None
        ry = sum(p[1] for p in right_pts) / len(right_pts) if right_pts else None
        if ly is not None and ry is not None:
            return (ly + ry) / 2.0
        if ly is not None:
            return ly  # 오른쪽 콘만 없을 때 보정 필요(추후)
        if ry is not None:
            return ry
        return 0.0
