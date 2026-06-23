#!/usr/bin/env python3
"""신호등 검출기 (검은 픽셀 게이팅 방식).

perception.py 에서 import 해서 사용한다. ROS 토픽은 다루지 않으며,
"BGR 이미지 → 신호 정수" 변환만 책임진다.

신호 반환값:
    SIGNAL_NONE   = 0
    SIGNAL_RED    = 1
    SIGNAL_YELLOW = 2
    SIGNAL_GREEN  = 3
    SIGNAL_LEFT   = 4  (빨강+초록 동시 = 좌회전)

전략 (검은 픽셀 게이팅):
    ROI 안에서 검은 픽셀은 신호등 하우징에만 있다.
    ① ROI 자르기 → ② 검은 픽셀 수 측정 → ③ 기준치 이상이면 신호등으로 인정
    → ④ 하우징 bbox 안에서만 색 픽셀을 세서 판정
"""

import os
import time
from collections import deque, Counter

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# 신호 상수
# ---------------------------------------------------------------------------
SIGNAL_NONE   = 0
SIGNAL_RED    = 1
SIGNAL_YELLOW = 2
SIGNAL_GREEN  = 3
SIGNAL_LEFT   = 4

# ---------------------------------------------------------------------------
# HSV 색 범위 (시뮬레이터 기준)
# ---------------------------------------------------------------------------
HSV_RED1_LO = (0,   110, 110);  HSV_RED1_HI = (10,  255, 255)
HSV_RED2_LO = (168, 110, 110);  HSV_RED2_HI = (179, 255, 255)
HSV_YEL_LO  = (10,  120, 120);  HSV_YEL_HI  = (35,  255, 255)
HSV_GRN_LO  = (40,   70, 120);  HSV_GRN_HI  = (95,  255, 255)

TL_HISTORY_LEN = 7
_NAME = {0: 'NONE', 1: 'RED', 2: 'YELLOW', 3: 'GREEN', 4: 'LEFT'}

# ---------------------------------------------------------------------------
# 파라미터 기본값
# ---------------------------------------------------------------------------
BASE_PARAMS = dict(
    roi_top=0.00, roi_bottom=0.60,
    black_v_max=60,
    black_s_max=255,
    open_k=3,
    close_k=7,
    black_min_count=200,
    black_min_blob_area=120,
    aspect_min=1.5,   # 하우징 가로/세로 비율 하한 (세로 기둥·나무 오검출 배제)
    bbox_pad=4,
    # 색 카운트 세로 밴드: 램프가 있는(하우징이 꽉 찬) 구간만.
    # 상단 마운트바·그 옆 배경(나무 초록)을 제외하기 위함.
    lamp_top_frac=0.18,
    lamp_bot_frac=0.97,
    color_min_count=25,
    debug_period=0.5,
    log_period=1.0,
)

# 시작등(3구): 가까이 크게 보임 → 게이트 높게
START_OVERRIDES = dict(
    black_min_count=600,
    black_min_blob_area=400,
    color_min_count=40,
    aspect_min=1.5,   # 3구 가로형
)

# 트랙등(4구): 멀리 작게 보임 → 게이트 낮게, bbox 딱 맞게(배경 나무 배제)
TRACK_OVERRIDES = dict(
    black_min_count=120,
    black_min_blob_area=3000,  # ★ 하우징 박스 면적이 이 값 이상(=가까울 때)이면 신호등 인식
                               #   → 그 다음 색 분류. 로그의 'blob=NNN'을 원하는 거리에서
                               #   보고 그 값 근처로 튜닝 (너무 일찍이면 ↑, 늦으면 ↓)
    color_min_count=50,        # 게이트 통과 후 색 구분용 (작게 — 거리 게이트는 하우징이 담당)
    bbox_pad=0,
    aspect_min=2.0,   # 4구는 매우 가로로 김(비율 ~5). 기둥(<1) 확실히 배제
)


class TrafficLightDetector:
    """검은 픽셀 게이팅 기반 신호등 검출기.

    four_lamp=False : 시작등(3구) — 가장 많이 검출된 색 반환.
    four_lamp=True  : 트랙등(4구) — 빨강+초록 동시 점등이면 좌회전.
    """

    def __init__(self, name='tl', four_lamp=True,
                 debug=False, show=False, logger=None, overrides=None):
        self.name = name
        self.four_lamp = four_lamp
        self.debug = debug
        self.show = show
        self.logger = logger
        self.hist = deque(maxlen=TL_HISTORY_LEN)
        self.last_box = None            # 마지막 하우징 박스 (ROI 좌표)
        self.last_color = SIGNAL_NONE
        self._roi_y0 = 0                # ROI 상단 오프셋 (full-image 변환용)
        self._last_debug = 0.0
        self._last_log = 0.0

        p = dict(BASE_PARAMS)
        if overrides:
            p.update(overrides)
        self.p = p

        if self.logger:
            self.logger.info(f'TrafficLightDetector[{name}] ready '
                             f'(four_lamp={four_lamp})')

    # ------------------------------------------------------------------
    def detect(self, image):
        """BGR 이미지 → 신호 정수 (SIGNAL_*)."""
        counts = self._analyze(image)
        color = self._classify(counts)
        self.hist.append(color)
        self.last_color = Counter(self.hist).most_common(1)[0][0]
        return self.last_color

    def draw(self, image):
        """마지막 하우징 박스를 full-image 위에 그린다 (perception 창 통합용)."""
        if self.last_box is None:
            return
        x, y, w, h = self.last_box
        y += self._roi_y0
        cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 255), 2)
        cv2.putText(image, f'{self.name}:{_NAME[self.last_color]}',
                    (x, max(12, y - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255, 0, 255), 2)

    # ------------------------------------------------------------------
    def _analyze(self, image):
        if image is None:
            return {}

        p = self.p
        h, w = image.shape[:2]
        roi = image[int(h * p['roi_top']):int(h * p['roi_bottom']), :]
        self._roi_y0 = int(h * p['roi_top'])
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        v = hsv[:, :, 2]
        s = hsv[:, :, 1]

        # 검은 픽셀 마스크
        black = ((v < p['black_v_max']) & (s < p['black_s_max'])).astype(np.uint8) * 255
        if p['open_k'] > 1:
            black = cv2.morphologyEx(black, cv2.MORPH_OPEN,
                                     np.ones((p['open_k'], p['open_k']), np.uint8))
        if p['close_k'] > 1:
            black = cv2.morphologyEx(black, cv2.MORPH_CLOSE,
                                     np.ones((p['close_k'], p['close_k']), np.uint8))

        n_black = int(cv2.countNonZero(black))
        contours, _ = cv2.findContours(black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # 가장 큰 블롭 중 '가로로 긴'(aspect>=aspect_min) 것만 하우징으로 인정.
        # → 세로 기둥/나무 줄기(비율<1)는 면적이 커도 탈락.
        best_box, best_area = None, 0.0
        for c in contours:
            a = cv2.contourArea(c)
            if a <= best_area:
                continue
            x, y, bw, bh = cv2.boundingRect(c)
            if bh <= 0 or (bw / float(bh)) < p['aspect_min']:
                continue
            best_area = a
            best_box = (x, y, bw, bh)

        # 게이트 체크
        if (n_black < p['black_min_count'] or best_box is None
                or best_area < p['black_min_blob_area']):
            self.last_box = None
            if self.debug or self.show:
                self._save_debug(roi, black, None, {})
            self._log(n_black, best_area, {})
            return {}

        # 색 카운트 영역: 하우징의 세로 중앙 밴드(램프가 있는, 꽉 찬 구간)만.
        # 상단 마운트바 옆 배경 나무(가짜 초록)를 원천 배제.
        bx, by, bw, bh = best_box
        pad = p['bbox_pad']
        H, W = hsv.shape[:2]
        x0 = max(0, bx - pad)
        x1 = min(W, bx + bw + pad)
        y0 = max(0, by + int(bh * p['lamp_top_frac']))
        y1 = min(H, by + int(bh * p['lamp_bot_frac']))
        sub = hsv[y0:y1, x0:x1]

        masks = {
            SIGNAL_RED: cv2.bitwise_or(
                cv2.inRange(sub, HSV_RED1_LO, HSV_RED1_HI),
                cv2.inRange(sub, HSV_RED2_LO, HSV_RED2_HI)),
            SIGNAL_YELLOW: cv2.inRange(sub, HSV_YEL_LO, HSV_YEL_HI),
            SIGNAL_GREEN:  cv2.inRange(sub, HSV_GRN_LO, HSV_GRN_HI),
        }
        counts = {}
        for color, m in masks.items():
            # 가장 큰 연결 덩어리(=켜진 램프)만 인정. 흩어진 배경 잡색은 탈락.
            num, _, stats, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
            if num <= 1:
                continue
            max_area = int(stats[1:, cv2.CC_STAT_AREA].max())
            if max_area >= p['color_min_count']:
                counts[color] = max_area

        self.last_box = best_box
        if self.debug or self.show:
            self._save_debug(roi, black, best_box, counts)
        self._log(n_black, best_area, counts)
        return counts

    # ------------------------------------------------------------------
    def _classify(self, counts):
        if not counts:
            return SIGNAL_NONE

        has_r = SIGNAL_RED   in counts
        has_y = SIGNAL_YELLOW in counts
        has_g = SIGNAL_GREEN  in counts

        if self.four_lamp:
            n_r = counts.get(SIGNAL_RED, 0)
            n_g = counts.get(SIGNAL_GREEN, 0)
            # 초록이 빨강의 30% 이상일 때만 좌회전 인정 (배경 나무 오판 방지)
            if has_g and has_r and n_g >= n_r * 0.3:
                return SIGNAL_LEFT
            if has_g:
                return SIGNAL_GREEN
            if has_r:
                return SIGNAL_RED
            if has_y:
                return SIGNAL_YELLOW
            return SIGNAL_NONE

        # 3구: 가장 많이 검출된 색
        return max(counts.items(), key=lambda kv: kv[1])[0]

    # ------------------------------------------------------------------
    def _log(self, n_black, best_area, counts):
        if not self.logger:
            return
        now = time.time()
        if now - self._last_log < self.p['log_period']:
            return
        self._last_log = now
        if not counts:
            self.logger.info(f'[TL:{self.name}] 미검출 '
                             f'(black={n_black} blob={int(best_area)})')
            return
        parts = [f'{_NAME[c]}:{n}' for c, n in
                 sorted(counts.items(), key=lambda kv: -kv[1])]
        self.logger.info(f'[TL:{self.name}] black={n_black} '
                         f'blob={int(best_area)} [{" ".join(parts)}]')

    def _save_debug(self, roi, black_mask, housing, counts):
        now = time.time()
        if now - self._last_debug < self.p['debug_period']:
            return
        self._last_debug = now
        try:
            vis = roi.copy()
            if housing is not None:
                x, y, w, h = housing
                cv2.rectangle(vis, (x, y), (x + w, y + h), (255, 0, 255), 2)
                txt = ' '.join(f'{_NAME[c]}:{n}' for c, n in counts.items())
                cv2.putText(vis, txt, (x, max(12, y - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
            if self.debug:
                d = f'/tmp/tl_debug/{self.name}'
                os.makedirs(d, exist_ok=True)
                ts = int(now * 2)
                cv2.imwrite(f'{d}/roi_{ts}.png', vis)
                cv2.imwrite(f'{d}/mask_black_{ts}.png', black_mask)
            if self.show:
                cv2.imshow(f'tl_{self.name}', vis)
                cv2.waitKey(1)
        except Exception as e:
            if self.logger:
                self.logger.warn(f'tl debug failed: {e}')
