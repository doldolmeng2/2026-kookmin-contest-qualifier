#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# traffic_light_base.py  (Team ERICAR perception - 신호등 공통 엔진)
#
# 시작등/트랙등이 공유하는 영상처리 엔진. 직접 쓰지 않고 상속해서 쓴다.
#   - traffic_light_start.py : 시작 신호등(3구)
#   - traffic_light_track.py : 트랙 위 신호등(4구, 좌회전 포함)
#
# 약속된 enum (Perception.msg): 0 없음 / 1 빨강 / 2 주황 / 3 초록(직진) / 4 좌회전
#
# 전략: 회색·어두운·가로로 긴 '하우징'을 먼저 찾고, 그 안에서만 램프를 본다.
#       (나무는 채도가 높아 회색 마스크에 안 들어옴 → 나무 오검출이 사라짐)
#
# 자식 클래스가 해야 할 일:
#   - DEFAULT_PARAMS : 파라미터 딕셔너리 (크기/임계값 등)
#   - NAME           : 로그/디버그 폴더 이름 ('start'/'track')
#   - _classify(lamps): 검출된 램프들로 최종 신호색 결정
# =============================================================================

import os
import time
from collections import deque, Counter

import cv2
import numpy as np

from ericar_msgs.msg import Perception


# 램프 색 HSV 범위 (공통 — 같은 시뮬레이터 색)
HSV_RED1_LO = (0,   110, 110);  HSV_RED1_HI = (10,  255, 255)
HSV_RED2_LO = (168, 110, 110);  HSV_RED2_HI = (179, 255, 255)
HSV_YEL_LO  = (10,  80,  110);  HSV_YEL_HI  = (35,  255, 255)   # 오렌지~노랑
HSV_GRN_LO  = (40,  70,  120);  HSV_GRN_HI  = (95,  255, 255)

TL_HISTORY_LEN = 7

_NAME = {0: 'NONE', 1: 'RED', 2: 'YELLOW', 3: 'GREEN', 4: 'LEFT'}
_DBG_BGR = {1: (0, 0, 255), 2: (0, 255, 255), 3: (0, 255, 0)}

# 공통 기본 파라미터 (자식이 일부만 덮어씀). 면적류는 '하한'이라 크게 보여도 통과.
BASE_PARAMS = dict(
    roi_top=0.00, roi_bottom=0.65,
    housing_max_v=135, housing_max_s=85,     # 회색 하우징 (어둡고 무채색)
    close_k=15, open_k=9,                     # 램프 구멍 메움 / 얇은 기둥 제거
    housing_min_area=1200,                    # 하우징 최소 면적(하한). 가까우면 더 큼 → OK
    aspect_lo=1.6, aspect_hi=6.0,             # 가로로 긴 사각형
    min_extent=0.30,                          # 채움비율(켜진 램프 구멍 고려해 낮게)
    max_cy=0.60,                              # 하우징은 화면 위쪽
    above_max_green=0.25,                     # 위가 나무면 지상 오검출
    lamp_min_area=20,                         # 램프 최소 면적(하한)
    lamp_pad=3,                               # 램프 탐색 시 하우징 밖 여백(작을수록 나무 배제)
)


class TrafficLightBase:
    NAME = 'base'
    DEFAULT_PARAMS = BASE_PARAMS

    def __init__(self, debug=False, logger=None, overrides=None):
        self.debug = debug
        self.logger = logger
        self.hist = deque(maxlen=TL_HISTORY_LEN)
        self._last_debug = 0.0
        self._last_log = 0.0
        p = dict(self.DEFAULT_PARAMS)
        if overrides:
            p.update(overrides)
        self.p = p
        if self.logger:
            self.logger.info(f'{self.__class__.__name__} loaded (v5)')

    # -------------------------------------------------------------------------
    # public: 노드는 detect(image)만 호출
    # -------------------------------------------------------------------------
    def detect(self, image):
        lamps = self._analyze(image)
        color = self._classify(lamps)          # 자식이 구현
        self.hist.append(color)
        return Counter(self.hist).most_common(1)[0][0]

    # 자식 클래스가 반드시 구현: 검출된 램프들 → 최종 신호색
    def _classify(self, lamps):
        raise NotImplementedError

    # -------------------------------------------------------------------------
    # 공통 영상처리
    # -------------------------------------------------------------------------
    # 반환: lamps = {color: (area, pos, bbox)}  하우징 안 색별 최대 블롭
    def _analyze(self, image):
        if image is None:
            if self.logger and time.time() - self._last_log > 1.0:
                self._last_log = time.time()
                self.logger.warn(f'[TL:{self.NAME}] 카메라 영상 없음 (image=None)')
            return {}

        p = self.p
        h, w = image.shape[:2]
        roi = image[int(h * p['roi_top']):int(h * p['roi_bottom']), :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        housing, gray_mask = self._find_housing(hsv)
        if housing is None:
            if self.debug:
                self._save_debug(roi, gray_mask, None, {})
            self._log(None, {})
            return {}

        lamps = self._lit_color_in_housing(hsv, housing)
        if self.debug:
            self._save_debug(roi, gray_mask, housing, lamps)
        self._log(housing, lamps)
        return lamps

    def _find_housing(self, hsv):
        p = self.p
        s = hsv[:, :, 1]
        v = hsv[:, :, 2]
        gray = ((s < p['housing_max_s']) & (v < p['housing_max_v'])).astype(np.uint8) * 255
        gray = cv2.morphologyEx(gray, cv2.MORPH_CLOSE,
                                np.ones((p['close_k'], p['close_k']), np.uint8))
        gray = cv2.morphologyEx(gray, cv2.MORPH_OPEN,
                                np.ones((p['open_k'], p['open_k']), np.uint8))

        H = hsv.shape[0]
        contours, _ = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        best_area = 0
        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            box_area = w * h
            if box_area < p['housing_min_area']:
                continue
            aspect = w / float(h)
            if not (p['aspect_lo'] <= aspect <= p['aspect_hi']):
                continue
            if cv2.contourArea(c) / float(box_area) < p['min_extent']:
                continue
            if (y + h / 2.0) / H > p['max_cy']:
                continue
            above = hsv[max(0, y - h):y, x:x + w]
            if above.size:
                g = cv2.inRange(above, HSV_GRN_LO, HSV_GRN_HI)
                if float((g > 0).mean()) > p['above_max_green']:
                    continue
            if box_area > best_area:
                best_area = box_area
                best = (x, y, w, h)
        return best, gray

    def _lit_color_in_housing(self, hsv, housing):
        p = self.p
        x, y, w, h = housing
        pad = p['lamp_pad']      # 하우징 밖으로 거의 안 나가게(나무 조각 배제)
        H, W = hsv.shape[:2]
        x0, y0 = max(0, x - pad), max(0, y - pad)
        x1, y1 = min(W, x + w + pad), min(H, y + h + pad)
        sub = hsv[y0:y1, x0:x1]

        masks = {
            Perception.SIGNAL_RED: cv2.bitwise_or(
                cv2.inRange(sub, HSV_RED1_LO, HSV_RED1_HI),
                cv2.inRange(sub, HSV_RED2_LO, HSV_RED2_HI)),
            Perception.SIGNAL_YELLOW: cv2.inRange(sub, HSV_YEL_LO, HSV_YEL_HI),
            Perception.SIGNAL_GREEN: cv2.inRange(sub, HSV_GRN_LO, HSV_GRN_HI),
        }
        lamps = {}
        for color, m in masks.items():
            contours, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            best_a = 0
            best_bbox = None
            for c in contours:
                a = cv2.contourArea(c)
                if a < p['lamp_min_area']:
                    continue
                if a > best_a:
                    bx, by, bw, bh = cv2.boundingRect(c)
                    best_a = a
                    best_bbox = (x0 + bx, y0 + by, bw, bh)
            if best_bbox is not None:
                lamp_cx = best_bbox[0] + best_bbox[2] / 2.0
                pos = (lamp_cx - x) / float(w) if w > 0 else -1.0
                lamps[color] = (best_a, pos, best_bbox)
        return lamps

    # -------------------------------------------------------------------------
    # 디버그 / 로그
    # -------------------------------------------------------------------------
    def _log(self, housing, lamps):
        if not self.logger:
            return
        now = time.time()
        if now - self._last_log < 1.0:
            return
        self._last_log = now
        if housing is None:
            self.logger.info(f'[TL:{self.NAME}] 하우징 미검출')
            return
        x, y, w, h = housing
        parts = [f'{_NAME[c]}@{pp:.2f}' for c, (a, pp, b) in
                 sorted(lamps.items(), key=lambda kv: kv[1][1])]
        self.logger.info(f'[TL:{self.NAME}] housing=({w}x{h}) lamps=[{" ".join(parts)}]')

    def _save_debug(self, roi, gray_mask, housing, lamps):
        now = time.time()
        if now - self._last_debug < 0.5:
            return
        self._last_debug = now
        d = f'/tmp/tl_debug/{self.NAME}'
        try:
            os.makedirs(d, exist_ok=True)
            vis = roi.copy()
            if housing is not None:
                x, y, w, h = housing
                cv2.rectangle(vis, (x, y), (x + w, y + h), (255, 0, 255), 2)
            for color, (a, pos, bb) in lamps.items():
                bx, by, bw, bh = bb
                bgr = _DBG_BGR.get(color, (255, 255, 255))
                cv2.rectangle(vis, (bx, by), (bx + bw, by + bh), bgr, 2)
                cv2.putText(vis, f'{_NAME[color]} {pos:.2f}', (bx, max(12, by - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr, 2)
            ts = int(now * 2)
            cv2.imwrite(f'{d}/roi_{ts}.png', vis)
            cv2.imwrite(f'{d}/mask_gray_{ts}.png', gray_mask)
        except Exception as e:
            if self.logger:
                self.logger.warn(f'tl debug save failed: {e}')
