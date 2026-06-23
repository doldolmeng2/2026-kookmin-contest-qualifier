#!/usr/bin/env python3
"""지름길 탈출 삼거리 검출 — 색 처리.

perception.py 에서 import. 숏컷은 직선이라, 끝의 삼거리에 도달하면
'정면 도로가 끊기고 그 자리에 잔디(초록)' 가 나타난다.
정면 중앙 ROI 의 초록 비율이 기준 이상이면 삼거리 도달로 본다.

※ shortcut 주행 중(mode=LANE, stage[0]=1)에만 호출되므로, 그 구간에서
  길이 끝나는 곳은 탈출 삼거리뿐 → 다른 커브/교차로와 안 헷갈린다.
"""

import cv2

# 정면 중앙 ROI (height/width 비율): 직선 도로면 회색, 삼거리면 잔디
ROI_TOP, ROI_BOT, ROI_LEFT, ROI_RIGHT = 0.45, 0.62, 0.35, 0.65

# 초록(잔디) HSV 범위
GRN_H_LO, GRN_H_HI = 35, 90
GRN_S_MIN, GRN_V_MIN = 60, 60

# 정면 초록 비율 하한. 실측: 일반/접근 ≤0.34, 삼거리 도달 0.48~0.74
GREEN_MIN_RATIO = 0.45


def detect_shortcut_exit(image):
    """BGR 이미지 → 지름길 탈출 삼거리가 정면에 오면 True."""
    if image is None:
        return False
    h, w = image.shape[:2]
    roi = image[int(h * ROI_TOP):int(h * ROI_BOT),
                int(w * ROI_LEFT):int(w * ROI_RIGHT)]
    if roi.size == 0:
        return False
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    H, S, V = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    green = ((H >= GRN_H_LO) & (H <= GRN_H_HI)
             & (S >= GRN_S_MIN) & (V >= GRN_V_MIN)).mean()
    return bool(float(green) >= GREEN_MIN_RATIO)
