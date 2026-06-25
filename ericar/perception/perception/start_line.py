#!/usr/bin/env python3
"""출발선(흑백 체커보드) 검출 — 색/명암 처리.

perception.py 에서 import. 체커보드는 도로 하단에서 '검은 픽셀과 흰 픽셀이
동시에 많이' 나오는 게 특징(일반 도로는 회색이라 순수 검정이 거의 없음).

도로 하단 중앙 ROI 에서 검정/흰색 비율을 재서 둘 다 기준 이상이면 출발선으로 본다.
"""

import cv2

# ROI (height/width 비율): 화면 '맨 아래 띠'만 본다.
#   → 체커보드가 정면(차가 밟기 직전)일 때만 잡히고, 멀리 접근 중(화면 중앙)일
#     때는 안 잡힌다. 주행 중 멀리서 한순간 오검출하는 문제 방지.
ROI_TOP, ROI_BOT, ROI_LEFT, ROI_RIGHT = 0.80, 1.00, 0.15, 0.85

BLACK_V_MAX = 50          # 이보다 어두우면 '검정'(체커보드 검은 칸)
WHITE_V_MIN = 200         # 이보다 밝으면 '흰색'(체커보드 흰 칸)
BLACK_MIN_RATIO = 0.05    # 맨아래 ROI 검정 비율 하한 (정면크로싱 0.20~0.47 / 접근중 0~0.9% / 도로 0)
WHITE_MIN_RATIO = 0.11    # 맨아래 ROI 흰색 비율 하한 (정면크로싱 0.29~0.53 / 도로 0.11~0.13)


def detect_start_line(image):
    """BGR 이미지 → 출발선(체커보드)이 가까이 보이면 True."""
    if image is None:
        return False
    h, w = image.shape[:2]
    roi = image[int(h * ROI_TOP):int(h * ROI_BOT),
                int(w * ROI_LEFT):int(w * ROI_RIGHT)]
    if roi.size == 0:
        return False
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    black = float((gray < BLACK_V_MAX).mean())
    white = float((gray > WHITE_V_MIN).mean())
    return bool(black >= BLACK_MIN_RATIO and white >= WHITE_MIN_RATIO)
