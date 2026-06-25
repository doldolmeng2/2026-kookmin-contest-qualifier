#!/usr/bin/env python3
"""차선 인식 → 조향 오프셋 계산.

driving.py 에서 import 해서 사용한다. ROS 토픽은 다루지 않고,
이미지/yaw → offset(float) 변환만 책임진다.

LaneDetector  : BEV + 슬라이딩 윈도우 + 다항식 피팅 (일반 차선 주행)
SCurveDetector: yaw + 노란 차선 블렌딩 (S자 구간)
"""

import math

import cv2
import numpy as np

# ── ROI 설정 (이미지 하단 기준 %) ────────────────────────────────────────────
ROI_BOTTOM_PCT = 0.10
ROI_TOP_PCT    = 0.40

# ── 노란색 HSV 범위 ───────────────────────────────────────────────────────────
YELLOW_LOWER = np.array([25, 205,  80])
YELLOW_UPPER = np.array([31, 255, 255])

# ── BEV 출력 크기 ─────────────────────────────────────────────────────────────
BEV_W = 400
BEV_H = 300
DST_X_MARGIN = 0.32

# ── 슬라이딩 윈도우 파라미터 ─────────────────────────────────────────────────
N_WINDOWS  = 16 # 윈도우 개수
WIN_MARGIN = 40 # 윈도우 좌우 여유 픽셀
MIN_PIX    = 20 # 최소 픽셀 수 (윈도우 내) — 이보다 적으면 무시
MAX_X_SPAN = 50 # 윈도우 내 x 좌표 span 허용 범위 (픽셀) — 이보다 크면 무시

# ── offset 기준값 (BEV 픽셀, 차선별) ─────────────────────────────────────────
LANE_REF_X = {0: 230, 1: 165}  # 0=1차선, 1=2차선

# ── 곡률 가중치 ───────────────────────────────────────────────────────────────
K_CURVE = 200.0

# ── BEV 픽셀 offset → 발행값 스케일 ──────────────────────────────────────────
OFFSET_SCALE = 200.0

# ── S자 구간 파라미터 ──────────────────────────────────────────────────────────
S_TARGET_YAW_DEG = 90.0
S_YAW_WEIGHT     = 0.78
S_LANE_WEIGHT    = 0.22
S_LANE_Y_RATIO   = 0.60
S_LANE_Y_BAND    = 5
S_OFFSET_SCALE   = 100.0


def _build_bev_matrix(h, w):
    y_top    = int(h * (1.0 - ROI_TOP_PCT))
    y_bottom = int(h * (1.0 - ROI_BOTTOM_PCT))
    src = np.float32([[0, y_bottom], [w, y_bottom], [w, y_top], [0, y_top]])
    x_m = int(BEV_W * DST_X_MARGIN)
    dst = np.float32([[x_m, BEV_H], [BEV_W - x_m, BEV_H], [BEV_W, 0], [0, 0]])
    return cv2.getPerspectiveTransform(src, dst), y_top, y_bottom


class LaneDetector:

    def __init__(self):
        self._M           = None
        self._y_top       = None
        self._y_bottom    = None
        self._last_offset = 0.0
        self.reuse_reason = None  # None=fresh, str=재활용 이유

    def compute_offset(self, image, lane_target):
        """BEV + 슬라이딩 윈도우 + 다항식 피팅으로 offset 계산.

        Args:
            image: BGR 이미지 (없으면 None)
            lane_target: 0=1차선, 1=2차선
        Returns:
            float offset, 검출 실패 시 이전 값 유지
        """
        if image is None:
            self.reuse_reason = 'no image (topic not received)'
            return self._last_offset

        h, w = image.shape[:2]
        if self._M is None:
            self._M, self._y_top, self._y_bottom = _build_bev_matrix(h, w)

        bev      = cv2.warpPerspective(image, self._M, (BEV_W, BEV_H))
        bev_hsv  = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
        bev_mask = cv2.inRange(bev_hsv, YELLOW_LOWER, YELLOW_UPPER)

        cx_list, cy_list = self._sliding_window(bev_mask)

        if len(cx_list) < 2:
            self.reuse_reason = f'insufficient windows ({len(cx_list)} / need >= 2)'
            return self._last_offset

        ref_x = LANE_REF_X.get(lane_target, LANE_REF_X[0])

        if len(cx_list) >= 3:
            coeffs     = np.polyfit(cy_list, cx_list, 2)
            poly       = np.poly1d(coeffs)
            a          = coeffs[0]
            x_at_top   = float(poly(0))
            lat_offset = x_at_top - ref_x
            curve_corr = K_CURVE * abs(a)
            raw_offset = lat_offset * (1.0 + curve_corr)
        else:
            coeffs     = np.polyfit(cy_list, cx_list, 1)
            poly       = np.poly1d(coeffs)
            x_at_top   = float(poly(0))
            raw_offset = x_at_top - ref_x

        self.reuse_reason = None
        self._last_offset = raw_offset / (BEV_W / 2) * OFFSET_SCALE
        return self._last_offset

    def visualize_yellow(self, image):
        """BEV 차선 검출 결과를 화면에 표시. 확인용."""
        if image is None or self._M is None:
            return

        bev      = cv2.warpPerspective(image, self._M, (BEV_W, BEV_H))
        bev_hsv  = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
        bev_mask = cv2.inRange(bev_hsv, YELLOW_LOWER, YELLOW_UPPER)

        vis = bev.copy()
        vis[bev_mask > 0] = (0, 220, 255)

        cx_list, cy_list = self._sliding_window(bev_mask)
        for cx, cy in zip(cx_list, cy_list):
            cv2.circle(vis, (cx, cy), 4, (0, 230, 0), -1)

        if len(cx_list) >= 3:
            coeffs = np.polyfit(cy_list, cx_list, 2)
            poly   = np.poly1d(coeffs)
            y_pts  = np.linspace(0, BEV_H, 120).astype(int)
            x_pts  = np.clip(poly(y_pts).astype(int), 0, BEV_W - 1)
            cv2.polylines(vis, [np.column_stack([x_pts, y_pts])], False, (255, 60, 0), 2)
        elif len(cx_list) >= 2:
            coeffs = np.polyfit(cy_list, cx_list, 1)
            poly   = np.poly1d(coeffs)
            y_pts  = np.linspace(min(cy_list), max(cy_list), 60).astype(int)
            x_pts  = np.clip(poly(y_pts).astype(int), 0, BEV_W - 1)
            cv2.polylines(vis, [np.column_stack([x_pts, y_pts])], False, (100, 180, 255), 2)

        cv2.putText(vis, f'OFFSET={self._last_offset:+.1f}',
                    (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
        cv2.imshow('lane_detect', vis)
        cv2.waitKey(1)

    def _sliding_window(self, bev_mask):
        win_h   = BEV_H // N_WINDOWS
        cx_list = []
        cy_list = []
        for i in range(N_WINDOWS):
            y_end   = BEV_H - i * win_h
            y_start = y_end - win_h
            strip   = bev_mask[y_start:y_end, :]
            _, xs   = np.where(strip > 0)
            if len(xs) < MIN_PIX:
                continue
            if int(xs.max()) - int(xs.min()) > MAX_X_SPAN:
                continue
            cx_list.append(int(xs.mean()))
            cy_list.append((y_start + y_end) // 2)
        return cx_list, cy_list


class SCurveDetector:
    """yaw + 노란 차선 블렌딩으로 S자 구간 offset 계산."""

    def __init__(self):
        self._last_lane_cx = None

    def compute_offset(self, image, yaw):
        """offset(float) 반환. scale: S_OFFSET_SCALE 기준 (≈ -100 ~ 100).

        Args:
            image: BGR 이미지 (없으면 None)
            yaw:   현재 yaw (rad)
        """
        if image is not None:
            self._detect_yellow_cx(image)

        target_rad = math.radians(S_TARGET_YAW_DEG)
        yaw_err    = (yaw - target_rad + math.pi) % (2 * math.pi) - math.pi
        yaw_norm   = yaw_err / (math.pi / 2)

        if self._last_lane_cx is not None and image is not None:
            w         = image.shape[1]
            lane_norm = (self._last_lane_cx - w / 2) / (w / 2)
        else:
            lane_norm = 0.0

        offset = S_YAW_WEIGHT * yaw_norm + S_LANE_WEIGHT * lane_norm
        return float(offset * S_OFFSET_SCALE)

    def _detect_yellow_cx(self, frame):
        h, w   = frame.shape[:2]
        lane_y = int(h * S_LANE_Y_RATIO)
        y0     = max(0, lane_y - S_LANE_Y_BAND)
        y1     = min(h, lane_y + S_LANE_Y_BAND)
        roi    = frame[y0:y1, :]
        hsv    = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask   = cv2.inRange(hsv, YELLOW_LOWER, YELLOW_UPPER)
        _, xs  = np.where(mask > 0)
        if len(xs) > 0:
            self._last_lane_cx = int(xs.mean())
