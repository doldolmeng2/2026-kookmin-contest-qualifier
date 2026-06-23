#!/usr/bin/env python3
"""트랙 신호등 아래의 흰색 가로 정지선 검출."""

import cv2
import numpy as np


# 검사 영역: 화면 중앙 도로의 중하단부
ROI_TOP = 0.50
ROI_BOTTOM = 0.82
ROI_LEFT = 0.05
ROI_RIGHT = 0.95

# 흰색 조건
WHITE_S_MAX = 60
WHITE_V_MIN = 185

# 한 행에서 흰색이 차지하는 비율.
# 차선 경계는 좁지만 정지선은 화면을 가로질러 길게 나타난다.
ROW_RATIO_MIN = 0.55
MIN_BAND_ROWS = 3

# 이 높이보다 아래로 내려온 정지선만 "정지할 거리"로 판단한다.
# 값이 작을수록 일찍 정지한다.
TRIGGER_Y_MIN = 0.58
TRIGGER_Y_MAX = 0.80


def detect_stop_line(image):
    """BGR 이미지 -> (검출 여부, 정지선 y 비율, 최대 흰색 행 비율)."""

    if image is None:
        return False, -1.0, 0.0

    height, width = image.shape[:2]

    x0 = int(width * ROI_LEFT)
    x1 = int(width * ROI_RIGHT)
    y0 = int(height * ROI_TOP)
    y1 = int(height * ROI_BOTTOM)

    roi = image[y0:y1, x0:x1]
    if roi.size == 0:
        return False, -1.0, 0.0

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    white_mask = cv2.inRange(
        hsv,
        np.array([0, 0, WHITE_V_MIN], dtype=np.uint8),
        np.array([179, WHITE_S_MAX, 255], dtype=np.uint8),
    )

    # 가로 방향의 작은 끊김을 연결한다.
    kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (17, 3),
    )
    white_mask = cv2.morphologyEx(
        white_mask,
        cv2.MORPH_CLOSE,
        kernel,
    )

    row_ratios = np.count_nonzero(
        white_mask,
        axis=1,
    ) / float(max(1, white_mask.shape[1]))

    if row_ratios.size == 0:
        return False, -1.0, 0.0

    best_index = int(np.argmax(row_ratios))
    best_ratio = float(row_ratios[best_index])
    line_y_ratio = float(y0 + best_index) / float(height)

    strong_rows = int(
        np.count_nonzero(row_ratios >= ROW_RATIO_MIN)
    )

    detected = bool(
        best_ratio >= ROW_RATIO_MIN
        and strong_rows >= MIN_BAND_ROWS
        and TRIGGER_Y_MIN <= line_y_ratio <= TRIGGER_Y_MAX
    )

    return detected, line_y_ratio, best_ratio
