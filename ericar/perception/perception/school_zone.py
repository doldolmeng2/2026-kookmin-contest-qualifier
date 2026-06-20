#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""어린이 보호구역 인식 (HSV, 하단 ROI 3단계 상태기계).

data[8] 의미:
    0 = 보호구역 아님 (일반 도로)
    1 = 시작 인식 (main 감속) — 보호구역 안에서 계속 1 유지
    2 = 해제 인식 (main 가속) — 일반도로로 복귀하면 0 으로 돌아감

노면 색 실측:
    - 일반 도로     : yellow 낮음(~0~3600), white 높음(~5000)  (흰 차선)
    - 시작 문구/주행: yellow 높음(시작 ~18000),  white 낮음(~250~450)
    - 해제 문구     : white 중간(~1700)  (양옆 흰 선이 막 돌아옴)

상태기계:
    out(0):  yellow >= YELLOW_ENTER 면 → in(1)   (밖에선 흰색 무시)
    in(1):   white  >= WHITE_EXIT   면 → exited(2)  (안에선 노랑 무시)
    exited(2): white >= WHITE_NORMAL 면 → out(0)  (일반도로 흰 차선 복귀)

  → 해제 흰색(~1700)은 WHITE_EXIT~WHITE_NORMAL 사이라 '해제'로는 잡히지만
    '일반도로 복귀'로는 안 잡히고, 이후 일반도로 흰색(~5000)에서 0으로 복귀.

차 바로 앞에서만 반응하도록 화면 하단 띠만 본다 (ROI_TOP_FRAC).
"""

import time

import cv2
import numpy as np

# 하단 ROI (차에 가장 가까운 노면)
ROI_TOP_FRAC = 0.80

# 노란색 / 흰색 HSV 범위 (시뮬 기준)
HSV_YEL_LO = (15, 80, 120);  HSV_YEL_HI = (40, 255, 255)
HSV_WHT_LO = (0, 0, 190);    HSV_WHT_HI = (179, 40, 255)   # 저채도 + 고명도 = 흰색

# 임계값 (실측 기반)
YELLOW_ENTER = 10000   # 노란 픽셀 ≥ → 시작(진입). 시작문구 ~18000, 일반도로 <3700
WHITE_EXIT = 1000      # (보호구역 안) 흰 픽셀 ≥ → 해제. 주행중 ~450, 해제 ~1700
WHITE_NORMAL = 3000    # (해제 후) 흰 픽셀 ≥ → 일반도로 복귀(0). 일반도로 ~5000


class SchoolZoneDetector:
    """update(image) → 0(아님) / 1(시작·주행중) / 2(해제). 3단계 상태기계."""

    def __init__(self, logger=None, debug=False, show=False):
        self.logger = logger
        self.debug = debug
        self.show = show
        self.state = 'out'      # 'out' / 'in' / 'exited'
        self.signal = 0
        self._last_log = 0.0

    def update(self, image):
        if image is None:
            return self.signal

        h, w = image.shape[:2]
        y0 = int(h * ROI_TOP_FRAC)
        roi = image[y0:, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        ymask = cv2.inRange(hsv, HSV_YEL_LO, HSV_YEL_HI)
        wmask = cv2.inRange(hsv, HSV_WHT_LO, HSV_WHT_HI)
        yellow = int(cv2.countNonZero(ymask))
        white = int(cv2.countNonZero(wmask))

        if self.state == 'out':
            # 일반도로/진입 전: 노랑 급증으로만 진입 (흰색 무시 → 일반 흰차선 무시)
            if yellow >= YELLOW_ENTER:
                self.state = 'in'
                self.signal = 1
                if self.logger:
                    self.logger.info(f'[SchoolZone] 시작 인식 (yellow={yellow}) → 1')
        elif self.state == 'in':
            # 보호구역 안: 흰색 급증으로만 해제 (노랑 무시 → 1 유지)
            if white >= WHITE_EXIT:
                self.state = 'exited'
                self.signal = 2
                if self.logger:
                    self.logger.info(f'[SchoolZone] 해제 인식 (white={white}) → 2')
        else:  # 'exited'
            # 해제 후: 일반도로 흰 차선이 충분히 돌아오면 0 으로 복귀
            if white >= WHITE_NORMAL:
                self.state = 'out'
                self.signal = 0
                if self.logger:
                    self.logger.info(f'[SchoolZone] 일반도로 복귀 (white={white}) → 0')

        if self.debug and self.logger and time.time() - self._last_log > 0.5:
            self._last_log = time.time()
            self.logger.info(
                f'[SchoolZone] yellow={yellow} white={white} '
                f'state={self.state} signal={self.signal}')

        if self.show:
            self._show(image, y0, ymask, wmask, yellow, white)
        return self.signal

    def _show(self, image, y0, ymask, wmask, yellow, white):
        try:
            vis = image.copy()
            h, w = image.shape[:2]
            cv2.rectangle(vis, (0, y0), (w, h), (255, 0, 255), 2)
            sub = vis[y0:, :]
            sub[ymask > 0] = (0, 255, 255)    # 노랑
            sub[wmask > 0] = (255, 255, 255)  # 흰색
            cv2.putText(vis, f'Y={yellow} W={white} {self.state} sig={self.signal}',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow('school_zone', vis)
            cv2.waitKey(1)
        except Exception as e:
            if self.logger:
                self.logger.warn(f'[SchoolZone] imshow 실패: {e}')
