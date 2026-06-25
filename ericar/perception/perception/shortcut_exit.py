#!/usr/bin/env python3
"""지름길 탈출 삼거리 검출 — 노란색 + 초록색(잔디) 이중 조건.

노란 픽셀 수 조건과 정면 초록 비율 조건을 모두 만족해야 True 를 반환한다.
"""

import cv2
import numpy as np

# ── 노란색 조건 파라미터 ──────────────────────────────────────────────────────
ROI_TOP_FRAC    = 0.53   # 노란색 ROI 상단 (화면 높이 비율)
ROI_BOTTOM_FRAC = 0.70   # 노란색 ROI 하단 (화면 높이 비율)

HSV_YEL_LO = (25, 205,  80)   # 노란색 HSV 하한
HSV_YEL_HI = (31, 255, 255)   # 노란색 HSV 상한

YELLOW_THRESHOLD = 1200   # 노란 픽셀 수 임계값 (이상이면 조건 충족)

# ── 초록색(잔디) 조건 파라미터 ───────────────────────────────────────────────
ROI_TOP   = 0.45   # 초록색 ROI 상단 (화면 높이 비율)
ROI_BOT   = 0.62   # 초록색 ROI 하단 (화면 높이 비율)
ROI_LEFT  = 0.35   # 초록색 ROI 좌측 (화면 너비 비율)
ROI_RIGHT = 0.65   # 초록색 ROI 우측 (화면 너비 비율)

GRN_H_LO  = 35    # 초록 Hue 하한
GRN_H_HI  = 90    # 초록 Hue 상한
GRN_S_MIN = 60    # 초록 Saturation 최솟값
GRN_V_MIN = 60    # 초록 Value 최솟값

GREEN_MIN_RATIO = 0.50   # 초록 픽셀 비율 임계값 (이상이면 조건 충족)


def detect_shortcut_exit(image):
    """BGR 이미지 → 노란색 + 초록색 조건을 모두 만족하면 True."""
    if image is None:
        return False
    h, w = image.shape[:2]

    # ── 노란색 조건 ──────────────────────────────────────────────────────────
    y0 = int(h * ROI_TOP_FRAC)
    y1 = int(h * ROI_BOTTOM_FRAC)
    roi_y = image[y0:y1, :]
    if roi_y.size == 0:
        return False
    hsv_y = cv2.cvtColor(roi_y, cv2.COLOR_BGR2HSV)
    ymask = cv2.inRange(hsv_y, HSV_YEL_LO, HSV_YEL_HI)
    yellow = int(cv2.countNonZero(ymask))
    yellow_ok = yellow >= YELLOW_THRESHOLD

    # ── 초록색(잔디) 조건 ────────────────────────────────────────────────────
    roi_g = image[int(h * ROI_TOP):int(h * ROI_BOT),
                  int(w * ROI_LEFT):int(w * ROI_RIGHT)]
    if roi_g.size == 0:
        return False
    hsv_g = cv2.cvtColor(roi_g, cv2.COLOR_BGR2HSV)
    H, S, V = hsv_g[:, :, 0], hsv_g[:, :, 1], hsv_g[:, :, 2]
    green = ((H >= GRN_H_LO) & (H <= GRN_H_HI)
             & (S >= GRN_S_MIN) & (V >= GRN_V_MIN)).mean()
    green_ok = float(green) >= GREEN_MIN_RATIO

    print(f"shortcut exit  yellow={yellow}(>={YELLOW_THRESHOLD}:{yellow_ok})  green_ratio={green:.3f}(>={GREEN_MIN_RATIO}:{green_ok})")
    return yellow_ok and green_ok
