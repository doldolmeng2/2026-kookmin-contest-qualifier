#!/usr/bin/env python3
"""라바콘(러버콘) 주행 → 조향 오프셋 계산 + 시각화.

driving.py 에서 import 해서 사용한다. ROS 토픽은 다루지 않는다.

좌표계 (lidar 프레임):
  x = 전방(+), y = 좌(+)/우(-)
  pixel 변환: px = cx - ppm*y,  py = cy - ppm*x  (상=전방, 좌=왼쪽)
"""

import math
import cv2
import numpy as np

# ── 파라미터 ──────────────────────────────────────────────────────────────────
MAX_RANGE        = 6.0           # m, 이보다 먼 점 무시
MIN_RANGE        = 0.15          # m, 차체 노이즈 제거
CLUSTER_DIST     = 0.20          # m, 이 거리 이내 점 → 같은 라바콘
MAX_CLUSTER_PTS  = 4             # 이 이상 점 → 큰 장애물 → 제거
FRONT_HALF_ANGLE = math.radians(40)   # 탐색 각도: 전방 ±40°

# 한쪽 라바콘만 보일 때 반대쪽 예상 위치 계산용 횡방향 간격 (m)
# 실제 환경에서 L-R gap 출력값을 보고 조정할 것
CONE_SEPARATION  = 6.153


class ConeDriver:

    def compute_offset(self, scan):
        """라이다 스캔 → 조향 오프셋 (-1.0 ~ 1.0) 반환. 미검출 시 0.0."""
        if scan is None:
            return 0.0
        pts = self._scan_to_points(scan)
        clusters = self._cluster(pts)
        left_cone, right_cone = self._select_cones(clusters)
        return self._calc_offset(left_cone, right_cone)

    def visualize(self, scan):
        """디버그 시각화 창('Cone Drive')을 갱신한다."""
        SIZE   = 600
        cx, cy = SIZE // 2, SIZE // 2
        margin = 20
        ppm    = (SIZE // 2 - margin) / MAX_RANGE

        img = np.zeros((SIZE, SIZE, 3), dtype=np.uint8)
        self._draw_grid(img, cx, cy, ppm, SIZE, margin)

        # 로봇 마커
        cv2.circle(img, (cx, cy), 5, (0, 200, 255), -1)
        cv2.arrowedLine(img, (cx, cy), (cx, cy - 18), (0, 200, 255), 2, tipLength=0.4)

        if scan is None:
            cv2.imshow('Cone Drive', img)
            cv2.waitKey(1)
            return

        pts        = self._scan_to_points(scan)
        clusters   = self._cluster(pts)
        left_cone, right_cone = self._select_cones(clusters)
        offset     = self._calc_offset(left_cone, right_cone)

        # 탐색 각도 범위선
        for sign in (+1, -1):
            ang = sign * FRONT_HALF_ANGLE
            ex = int(cx - ppm * MAX_RANGE * math.sin(ang))
            ey = int(cy - ppm * MAX_RANGE * math.cos(ang))
            cv2.line(img, (cx, cy), (ex, ey), (50, 50, 50), 1)

        # 모든 클러스터 centroid (cyan 점)
        for x, y in clusters:
            px, py = self._to_px(x, y, cx, cy, ppm)
            if 0 <= px < SIZE and 0 <= py < SIZE:
                cv2.circle(img, (px, py), 5, (0, 180, 180), -1)

        # 선택된 왼쪽 라바콘 (녹색 링)
        if left_cone is not None:
            px, py = self._to_px(*left_cone, cx, cy, ppm)
            cv2.circle(img, (px, py), 10, (0, 255, 80), 2)
            cv2.putText(img, 'L', (px + 8, py - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 80), 1)

        # 선택된 오른쪽 라바콘 (주황 링)
        if right_cone is not None:
            px, py = self._to_px(*right_cone, cx, cy, ppm)
            cv2.circle(img, (px, py), 10, (30, 120, 255), 2)
            cv2.putText(img, 'R', (px + 8, py - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (30, 120, 255), 1)

        # 한쪽만 보일 때 추정 반대쪽 위치 (점선 링)
        if left_cone is not None and right_cone is None:
            est_y  = left_cone[1] - CONE_SEPARATION
            px, py = self._to_px(left_cone[0], est_y, cx, cy, ppm)
            if 0 <= px < SIZE and 0 <= py < SIZE:
                cv2.circle(img, (px, py), 8, (30, 120, 255), 1)
                cv2.putText(img, 'R?', (px + 8, py - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (30, 120, 255), 1)
        elif right_cone is not None and left_cone is None:
            est_y  = right_cone[1] + CONE_SEPARATION
            px, py = self._to_px(right_cone[0], est_y, cx, cy, ppm)
            if 0 <= px < SIZE and 0 <= py < SIZE:
                cv2.circle(img, (px, py), 8, (0, 255, 80), 1)
                cv2.putText(img, 'L?', (px + 8, py - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 80), 1)

        # 중심 y 위치 수직선 (노란색)
        mid_y = self._midpoint_y(left_cone, right_cone)
        if mid_y is not None:
            mid_px = max(margin, min(SIZE - margin, int(cx - ppm * mid_y)))
            cv2.line(img, (mid_px, margin), (mid_px, SIZE - margin), (255, 255, 0), 1)
            cv2.putText(img, f'mid_y={mid_y:+.3f}m',
                        (mid_px + 4, cy + 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 0), 1)

        # HUD 텍스트
        hud = [
            f'offset : {offset:+.4f}',
            f'cones  : {len(clusters)}',
        ]
        if left_cone and right_cone:
            gap = left_cone[1] - right_cone[1]
            hud.append(f'L-R gap: {gap:.3f} m')
        elif left_cone:
            hud.append(f'only L  estR_y={left_cone[1] - CONE_SEPARATION:+.3f}')
        elif right_cone:
            hud.append(f'only R  estL_y={right_cone[1] + CONE_SEPARATION:+.3f}')

        for i, txt in enumerate(hud):
            cv2.putText(img, txt, (8, 18 + i * 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.46, (200, 200, 200), 1)

        cv2.imshow('Cone Drive', img)
        cv2.waitKey(1)

    # ── 내부 헬퍼 ─────────────────────────────────────────────────────────────

    def _scan_to_points(self, scan):
        """유효 범위 내 (x, y) 리스트 반환."""
        pts   = []
        angle = scan.angle_min
        for r in scan.ranges:
            a      = angle
            angle += scan.angle_increment
            if not math.isfinite(r) or not (MIN_RANGE < r <= MAX_RANGE):
                continue
            pts.append((r * math.cos(a), r * math.sin(a)))
        return pts

    def _cluster(self, pts):
        """순차 클러스터링 → centroid 목록.
        인접한 두 점 거리가 CLUSTER_DIST 이하면 같은 라바콘.
        MAX_CLUSTER_PTS 초과 클러스터는 큰 장애물로 간주해 제거.
        """
        if not pts:
            return []
        clusters = []
        current  = [pts[0]]
        for p in pts[1:]:
            prev = current[-1]
            if math.hypot(p[0] - prev[0], p[1] - prev[1]) <= CLUSTER_DIST:
                current.append(p)
            else:
                if len(current) <= MAX_CLUSTER_PTS:
                    clusters.append(self._centroid(current))
                current = [p]
        if current and len(current) <= MAX_CLUSTER_PTS:
            clusters.append(self._centroid(current))
        return clusters

    def _select_cones(self, clusters):
        """전방 ±FRONT_HALF_ANGLE 이내에서 좌/우 가장 가까운 콘 반환."""
        left_cone = right_cone = None
        left_d    = right_d = float('inf')
        for x, y in clusters:
            if abs(math.atan2(y, x)) > FRONT_HALF_ANGLE:
                continue
            d = math.hypot(x, y)
            if y >= 0 and d < left_d:
                left_d, left_cone  = d, (x, y)
            elif y < 0 and d < right_d:
                right_d, right_cone = d, (x, y)

        # 좌우 라바콘의 횡방향(y) 간격이 3m 이내면 좌회전 중 오른쪽 라바콘 2개가
        # 잡힌 것으로 판단 → y<0(실제 오른쪽)인 right_cone만 유지, 왼쪽은 CONE_SEPARATION으로 추정
        if left_cone is not None and right_cone is not None:
            if (left_cone[1] - right_cone[1]) < 3.0:
                left_cone = None

        return left_cone, right_cone

    def _midpoint_y(self, left_cone, right_cone):
        """좌우 라바콘 중심 y 좌표 반환. 한쪽 없으면 CONE_SEPARATION 으로 추정."""
        if left_cone and right_cone:
            return (left_cone[1] + right_cone[1]) / 2.0
        if right_cone:
            # 오른쪽만 보임: 왼쪽 = right_y + CONE_SEPARATION 으로 추정
            return right_cone[1] + CONE_SEPARATION / 2.0
        if left_cone:
            # 왼쪽만 보임: 오른쪽 = left_y - CONE_SEPARATION 으로 추정
            return left_cone[1] - CONE_SEPARATION / 2.0
        return None

    def _calc_offset(self, left_cone, right_cone):
        mid_y = self._midpoint_y(left_cone, right_cone)
        if mid_y is None:
            return 0.0
        # mid_y > 0 → 중심이 왼쪽 → 차가 오른쪽으로 치우침 → 왼쪽 조향 → negative
        norm = CONE_SEPARATION / 2.0
        return float(max(-1.0, min(1.0, -mid_y / norm)))

    @staticmethod
    def _centroid(pts):
        n = len(pts)
        return (sum(p[0] for p in pts) / n, sum(p[1] for p in pts) / n)

    @staticmethod
    def _to_px(x, y, cx, cy, ppm):
        return int(cx - ppm * y), int(cy - ppm * x)

    @staticmethod
    def _draw_grid(img, cx, cy, ppm, SIZE, margin):
        for i in range(1, 5):
            r_m  = MAX_RANGE * i / 4
            r_px = int(ppm * r_m)
            cv2.circle(img, (cx, cy), r_px, (45, 45, 45), 1)
            cv2.putText(img, f'{r_m:.1f}m', (cx + r_px + 3, cy - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.32, (90, 90, 90), 1)
        cv2.line(img, (cx, margin), (cx, SIZE - margin), (35, 35, 35), 1)
        cv2.line(img, (margin, cy), (SIZE - margin, cy), (35, 35, 35), 1)
