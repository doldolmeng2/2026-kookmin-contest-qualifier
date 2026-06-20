#!/usr/bin/env python3
"""좌측 카메라에서 방해차량(라임 연두색) 검출 — 색 처리.

추월 중(2차선) 옆(좌측)에 있는 1차선 방해차량을 좌측 카메라로 본다.
방해차량은 채도 높은 라임(H~37, S~138, V~242)이고, 배경 잔디/나무는
채도가 낮음(S~51) → 색조+채도로 구분되어 색만으로 잡힌다.

perception 에서: 좌측에 차 '보임' → 추월 중,  '사라짐' → 추월 완료(복귀).
"""

import cv2

# ROI: 좌측 카메라 '하단'만 (옆 차는 가까워서 화면 아래, 나무는 위 배경 → 하단으로 제한해 나무 배제)
ROI_TOP, ROI_BOT, ROI_LEFT, ROI_RIGHT = 0.45, 1.00, 0.15, 1.00

# 방해차량 라임색 HSV (채도/명도 높게 → 흐린 잔디 배제)
CAR_LO = (28, 95, 130)
CAR_HI = (45, 255, 255)

CAR_BLOB_MIN = 4500   # 라임 덩어리(최대) 이 이상이면 옆에 차 있음
                      #   실측(하단ROI): 차 옆 5000~11000 / 차 없음(나무 빠짐) <1300


def car_in_left(image):
    """좌측 카메라 BGR → 옆(좌측)에 방해차량이 있으면 True."""
    if image is None:
        return False
    h, w = image.shape[:2]
    roi = image[int(h * ROI_TOP):int(h * ROI_BOT),
                int(w * ROI_LEFT):int(w * ROI_RIGHT)]
    if roi.size == 0:
        return False
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, CAR_LO, CAR_HI)
    num, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if num <= 1:
        return False
    return int(stats[1:, cv2.CC_STAT_AREA].max()) >= CAR_BLOB_MIN
