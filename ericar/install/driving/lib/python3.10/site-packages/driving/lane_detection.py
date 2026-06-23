#!/usr/bin/env python3
"""
lane_detection.py
-----------------
노란색 차선을 검출하여 차량 중심 대비 lateral offset(-1.0 ~ +1.0)을 계산한다.

알고리즘 개요:
  1. HSV + HLS 이중 마스킹으로 노란색 픽셀 추출
  2. Blob 필터로 소형 노이즈(교차로 화살표, 점선 등) 제거
  3. 2단 ROI:
     - 하단 ROI (차 바로 앞): 히스토그램 peak로 현재 차선 위치 정밀 추정
     - 원거리 ROI (앞쪽 도로): centroid(mean)으로 커브 방향 조기 감지
  4. 커브 boost: 원거리 ROI가 하단 ROI보다 같은 방향으로 더 큰 error를 보이면
     error를 CURVE_BOOST배로 증폭 → 커브 진입 전 핸들을 더 빨리 꺾음
  5. Velocity prediction: 차선이 일시적으로 사라졌을 때 직전 이동속도(dx)로
     최대 MAX_PRED_FRAMES 프레임 동안 위치를 예측해 offset 유지
  6. NO_LINE 턴 유지: 차선 완전 소실 시 턴 중(|offset|>0.2)이면 핸들 유지,
     직진 중이면 서서히 0으로 감쇠
  7. Normalized PID: error를 화면 폭 기준으로 정규화(-1~1) 후 PID 적용
"""
import cv2
import numpy as np
import sys
import time

# ── 색상 범위 ────────────────────────────────────────────────────────────────
# HSV + HLS 이중 조건으로 조명 변화에 강인하게 노란색 검출
# HSV: 색상(H) 15~40, 채도(S) 70+, 명도(V) 70+
# HLS: 색상(H) 15~45, 밝기(L) 45+, 채도(S) 60+
YELLOW_HSV_LOWER = np.array([15,  70,  70], dtype=np.uint8)
YELLOW_HSV_UPPER = np.array([40, 255, 255], dtype=np.uint8)
YELLOW_HLS_LOWER = np.array([15,  45,  60], dtype=np.uint8)
YELLOW_HLS_UPPER = np.array([45, 255, 255], dtype=np.uint8)

# ── 타겟 / 탐색 파라미터 ─────────────────────────────────────────────────────
# LANE1_TARGET_X: 1차선 주행 시 차선이 위치해야 할 이미지 x좌표 (직선 보정값)
# LANE2_TARGET_X: 2차선 주행 시 타겟 x좌표
# MIN_YELLOW_PX : 유효 차선으로 인정하는 최소 노란 픽셀 수
# YELLOW_MAX_RATIO: 이 비율 이상이면 교차로/오탐지로 판단하여 스킵
# HIST_KERNEL   : 히스토그램 스무딩 커널 크기 (클수록 peak가 부드러워짐)
# SEARCH_HALF   : 히스토그램 peak 탐색 범위 (target_x ± SEARCH_HALF px)
# BLOB_MIN_AREA : 이 면적(px²) 미만인 연결요소는 노이즈로 제거
LANE1_TARGET_X   = 310
LANE2_TARGET_X   = 420
MIN_YELLOW_PX    = 30
YELLOW_MAX_RATIO = 0.30
HIST_KERNEL      = 40
SEARCH_HALF      = 150
BLOB_MIN_AREA    = 80

# ── ROI ──────────────────────────────────────────────────────────────────────
# 이미지 높이를 기준으로 비율로 지정 (0.0 = 이미지 상단, 1.0 = 이미지 하단)
# ROI_NEAR_START: 하단 ROI 시작 위치 (차 바로 앞 노면, 이미지 50%~100%)
#   → 히스토그램 peak로 현재 차선 중심 정밀 추정
ROI_NEAR_START   = 0.80   # Near zone: 이미지 80%~100% (차 바로 앞)
ROI_MID_START    = 0.65   # Mid zone:  이미지 65%~80%
ROI_FAR_START    = 0.50   # Far zone:  이미지 50%~65%

ROI_WEIGHT_NEAR  = 0.2    # 가까울수록 낮은 가중치
ROI_WEIGHT_MID   = 0.3
ROI_WEIGHT_FAR   = 0.5    # 멀수록 높은 가중치 (커브 조기 감지)


# ── velocity prediction ──────────────────────────────────────────────────────
# 차선이 일시적으로 사라졌을 때 (예: S자 커브 중간, 그림자 등)
# 직전 프레임들의 이동 속도(dx)를 지수이동평균으로 추정하고,
# 그 속도로 최대 MAX_PRED_FRAMES 프레임 동안 위치를 외삽하여 offset 유지
# DX_ALPHA: EMA 계수 (클수록 최신 dx 반영 강함)
MAX_PRED_FRAMES  = 20
DX_ALPHA         = 0.3

# ── NO_LINE 처리 ─────────────────────────────────────────────────────────────
# velocity prediction도 불가능한 완전 차선 소실 시
# NO_LINE_MAX 프레임 이후부터 offset을 서서히 0으로 감쇠
# 단, 턴 중(|last_offset| > 0.2)이면 감쇠 없이 마지막 핸들각 유지
# → S자 2번째 커브 진입 시 1번째 턴 관성을 보존
NO_LINE_DECAY    = 0.95
NO_LINE_MAX      = 15

# ── PID 파라미터 ─────────────────────────────────────────────────────────────
# error_norm = error_px / (w/2)로 정규화 (-1.0 ~ +1.0)
# KP: 비례항 - 현재 오차에 즉각 반응
# KI: 적분항 - 지속적 편향 보정 (현재 비활성)
# KD: 미분항 - 급격한 변화 억제 (±0.15 클램핑으로 폭발 방지)
# PID_DERIVATIVE_ALPHA: 미분항 EMA 계수 (노이즈 스무딩)
PID_KP               = 0.55
PID_KI               = 0.0
PID_KD               = 0.20
PID_INTEGRAL_LIMIT   = 1.0
PID_DERIVATIVE_ALPHA = 0.60
CONTROL_PERIOD       = 1.0 / 30.0   # 제어 주기 30Hz


class LaneDetector:
    def __init__(self):
        self.last_offset      = 0.0   # 마지막으로 계산된 offset (소실 시 반환용)
        self.last_debug_img   = None  # driving.py가 /lane_debug로 퍼블리시할 이미지
        self._no_line_cnt     = 0     # 연속 차선 소실 프레임 수
        # PID 상태
        self._pid_integral    = 0.0
        self._pid_prev_error  = None
        self._pid_prev_time   = None
        self._pid_prev_deriv  = 0.0
        # velocity prediction 상태
        self._prev_yellow_x   = None  # 직전 프레임 차선 x좌표
        self._last_dx         = 0.0   # EMA로 추정된 프레임당 x이동량
        self._missing_frames  = 0     # 연속으로 차선 미검출된 프레임 수

    def _reset_pid(self):
        """PID 내부 상태 초기화 (차선 완전 소실 시 호출)"""
        self._pid_integral   = 0.0
        self._pid_prev_error = None
        self._pid_prev_time  = None
        self._pid_prev_deriv = 0.0

    def _compute_pid(self, error_norm: float) -> float:
        """
        정규화된 오차(-1~1)를 입력받아 PID 출력(-1~1)을 반환한다.
        미분항은 EMA 필터를 거쳐 노이즈를 억제한다.
        """
        now = time.monotonic()
        dt  = CONTROL_PERIOD if self._pid_prev_time is None else (now - self._pid_prev_time)
        if dt < 1e-4:
            dt = CONTROL_PERIOD

        # P항: 현재 오차에 비례
        p_term = PID_KP * error_norm

        # I항: 오차 누적 (wind-up 방지를 위해 클리핑)
        self._pid_integral = max(-PID_INTEGRAL_LIMIT,
                                  min(PID_INTEGRAL_LIMIT,
                                      self._pid_integral + error_norm * dt))
        i_term = PID_KI * self._pid_integral

        # D항: 오차 변화율 (EMA로 스무딩)
        raw_d  = 0.0 if self._pid_prev_error is None else (error_norm - self._pid_prev_error) / dt
        deriv  = PID_DERIVATIVE_ALPHA * self._pid_prev_deriv + (1 - PID_DERIVATIVE_ALPHA) * raw_d
        d_term = PID_KD * deriv
        d_term = max(-0.15, min(0.15, d_term))  # D항 폭발 방지

        self._pid_prev_error = error_norm
        self._pid_prev_time  = now
        self._pid_prev_deriv = deriv
        return p_term + i_term + d_term

    def _yellow_mask(self, roi: np.ndarray) -> np.ndarray:
        """
        HSV + HLS 이중 마스킹으로 노란색 픽셀을 검출한다.
        두 조건의 AND를 취해 오탐지를 줄인다.
        morphology close → 차선 내부 구멍 메우기
        morphology open  → 단독 픽셀 노이즈 제거
        """
        hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        hls  = cv2.cvtColor(roi, cv2.COLOR_BGR2HLS)
        mask = cv2.bitwise_and(
            cv2.inRange(hsv, YELLOW_HSV_LOWER, YELLOW_HSV_UPPER),
            cv2.inRange(hls, YELLOW_HLS_LOWER, YELLOW_HLS_UPPER)
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  np.ones((3, 3), np.uint8))
        return mask

    def _blob_filter(self, mask: np.ndarray) -> np.ndarray:
        """
        연결요소(blob) 분석으로 BLOB_MIN_AREA 미만의 작은 덩어리를 제거한다.
        교차로 화살표, 점선 노이즈, 신호등 반사 등 소형 오탐지 제거에 효과적.
        """
        n, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        out = np.zeros_like(mask)
        for i in range(1, n):  # 0은 배경
            if stats[i, cv2.CC_STAT_AREA] >= BLOB_MIN_AREA:
                out[labels == i] = 255
        return out

    def _histogram_peak(self, xs: np.ndarray, width: int, target_x: int) -> float:
        """
        x좌표 배열로 히스토그램을 생성하고 커널 스무딩 후 peak를 반환한다.
        탐색 범위를 target_x ± SEARCH_HALF로 제한해 반대편 차선 오탐지 방지.
        peak가 없으면 target_x를 그대로 반환 (offset=0 유지).
        """
        hist   = np.zeros(width, dtype=np.float32)
        np.add.at(hist, xs.clip(0, width - 1), 1.0)
        # 박스 커널 컨볼루션으로 스무딩 (HIST_KERNEL 크기 슬라이딩 합산)
        kernel = np.ones(HIST_KERNEL, dtype=np.float32)
        hist_s = np.convolve(hist, kernel, mode='same')
        lo = max(0, target_x - SEARCH_HALF)
        hi = min(width, target_x + SEARCH_HALF)
        local = hist_s[lo:hi]
        if local.max() == 0:
            return float(target_x)
        return float(lo + np.argmax(local))

    def compute_offset(self, image, lane_target):
        """
        입력 이미지에서 노란 차선을 검출하고 정규화된 lateral offset을 반환한다.
        offset > 0: 차선이 타겟보다 오른쪽 → 우회전 필요
        offset < 0: 차선이 타겟보다 왼쪽  → 좌회전 필요
        """
        if image is None:
            return self.last_offset

        h, w     = image.shape[:2]
        # lane_target: 0=1차선(LANE1), 1=2차선(LANE2)
        target_x = LANE1_TARGET_X if lane_target == 0 else LANE2_TARGET_X

        # ── 3단 ROI: 가중평균으로 커브 조기 감지 ───────────────────────────────
        near_top = int(h * ROI_NEAR_START)
        mid_top  = int(h * ROI_MID_START)
        far_top  = int(h * ROI_FAR_START)

        near_mask = self._blob_filter(self._yellow_mask(image[near_top:, :]))
        mid_mask  = self._blob_filter(self._yellow_mask(image[mid_top:near_top, :]))
        far_mask  = self._blob_filter(self._yellow_mask(image[far_top:mid_top, :]))

        _, near_xs = np.where(near_mask > 0)
        _, mid_xs  = np.where(mid_mask > 0)
        _, far_xs  = np.where(far_mask > 0)

        # ── 디버그 이미지 생성 ────────────────────────────────────────────────
        debug = image.copy()
        cv2.line(debug, (0, near_top), (w, near_top), (255, 255, 255), 2)  # near 경계
        cv2.line(debug, (0, mid_top),  (w, mid_top),  (200, 200,   0), 1)  # mid 경계
        cv2.line(debug, (0, far_top),  (w, far_top),  (180, 180,   0), 1)  # far 경계
        debug[near_top:][near_mask > 0]           = [0, 255, 0]    # near: 초록
        debug[mid_top:near_top][mid_mask > 0]     = [0, 200, 200]  # mid: 청록
        debug[far_top:mid_top][far_mask > 0]      = [0, 150, 255]  # far: 파랑
        cv2.line(debug, (target_x, far_top), (target_x, h), (255, 100, 0), 2)

        # ── too-many-yellow: 교차로/오탐지 필터 ──────────────────────────────
        # 노란 픽셀 비율이 YELLOW_MAX_RATIO를 초과하면 교차로나 이상 상황으로 판단
        # 이전 offset을 유지하고 스킵
        yellow_ratio = np.count_nonzero(near_mask) / float((h - near_top) * w + 1)
        if yellow_ratio > YELLOW_MAX_RATIO:
            cv2.putText(debug, f'TOO_MANY {yellow_ratio:.2f}',
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
            self.last_debug_img = debug
            return self.last_offset

        # ── NO_LINE / velocity prediction 처리 ───────────────────────────────
        if near_xs.size < MIN_YELLOW_PX:
            # 하단 ROI에서 차선을 충분히 검출하지 못한 경우
            self._missing_frames += 1
            self._no_line_cnt    += 1

            if self._prev_yellow_x is not None and self._missing_frames <= MAX_PRED_FRAMES:
                # [velocity prediction] 직전 이동속도(dx)로 현재 위치 예측
                yellow_x = self._prev_yellow_x + self._last_dx * self._missing_frames
                yellow_x = max(0.0, min(float(w - 1), yellow_x))
                source = 'predict'
            else:
                # prediction도 불가 → 완전 소실 처리
                if self._no_line_cnt > NO_LINE_MAX:
                    if abs(self.last_offset) > 0.2:
                        # [턴 유지] S자 커브 1번째 턴 중에 차선 소실 시
                        # 마지막 핸들각을 유지해 2번째 커브도 통과 가능하게 함
                        pass
                    else:
                        # 직진 중 소실 → 서서히 0으로 감쇠
                        self.last_offset *= NO_LINE_DECAY
                self._reset_pid()
                cv2.putText(debug, f'NO_LINE({self._no_line_cnt}) out={self.last_offset:.2f}',
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                self.last_debug_img = debug
                print(f'[LANE] NO_LINE({self._no_line_cnt}) -> {self.last_offset:.3f}', file=sys.stderr, flush=True)
                return self.last_offset
        else:
            # 정상 검출: 히스토그램 peak로 현재 차선 x좌표 추정
            self._missing_frames = 0
            self._no_line_cnt    = 0
            near_peak = self._histogram_peak(near_xs, w, target_x)
            mid_peak  = self._histogram_peak(mid_xs, w, target_x) if mid_xs.size >= MIN_YELLOW_PX else near_peak
            far_peak  = self._histogram_peak(far_xs, w, target_x) if far_xs.size >= MIN_YELLOW_PX else mid_peak
            yellow_x  = near_peak * ROI_WEIGHT_NEAR + mid_peak * ROI_WEIGHT_MID + far_peak * ROI_WEIGHT_FAR
            source    = 'weighted'
            # velocity prediction용 dx는 near_peak 기준
            if self._prev_yellow_x is not None:
                dx = near_peak - self._prev_yellow_x
                self._last_dx = (1 - DX_ALPHA) * self._last_dx + DX_ALPHA * dx
            self._prev_yellow_x = near_peak

        # ── error 계산 ────────────────────────────────────────────────────────
        error_px = yellow_x - target_x

        # ── PID 제어 ──────────────────────────────────────────────────────────
        # error_px를 화면 폭의 절반으로 나눠 -1~1로 정규화
        # → 해상도가 바뀌어도 동일한 PID 게인 사용 가능
        error_norm = max(-1.0, min(1.0, error_px / (w / 2.0)))
        offset     = max(-1.0, min(1.0, self._compute_pid(error_norm)))
        self.last_offset = offset

        # ── 디버그 오버레이 ───────────────────────────────────────────────────
        cv2.line(debug, (int(yellow_x), near_top), (int(yellow_x), h), (255, 255, 0), 2)
        label = f'peak={int(yellow_x)}({source}) tgt={target_x} err={int(error_px)} out={offset:.2f}'
        cv2.putText(debug, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 255), 2)
        self.last_debug_img = debug
        print(f'[LANE] peak={yellow_x:.0f} tgt={target_x} err={error_px:.1f} out={offset:.3f}',
              file=sys.stderr, flush=True)
        return self.last_offset
