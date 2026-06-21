#!/usr/bin/env python3
import cv2
import numpy as np
import sys
import time

# ── 색상 범위 ────────────────────────────────────────────
YELLOW_HSV_LOWER = np.array([15,  70,  70], dtype=np.uint8)
YELLOW_HSV_UPPER = np.array([40, 255, 255], dtype=np.uint8)
YELLOW_HLS_LOWER = np.array([15,  45,  60], dtype=np.uint8)
YELLOW_HLS_UPPER = np.array([45, 255, 255], dtype=np.uint8)

# ── 타겟 / 탐색 ──────────────────────────────────────────
LANE1_TARGET_X   = 310
LANE2_TARGET_X   = 420
MIN_YELLOW_PX    = 30
YELLOW_MAX_RATIO = 0.30
HIST_KERNEL      = 40
SEARCH_HALF      = 100
BLOB_MIN_AREA    = 80

# ── 2단 ROI ──────────────────────────────────────────────
ROI_NEAR_START   = 0.70   # 하단(정밀): 70%~100%
ROI_FAR_START    = 0.45   # 원거리(look-ahead): 45%~70%
ROI_FAR_END      = 0.70

# ── 커브 boost ────────────────────────────────────────────
CURVE_BOOST      = 3.0
CURVE_MIN_ERR    = 15     # px - 이 이상 차이날 때만 boost

# ── velocity prediction ───────────────────────────────────
MAX_PRED_FRAMES  = 12
DX_ALPHA         = 0.3

# ── NO_LINE ───────────────────────────────────────────────
NO_LINE_DECAY    = 0.85
NO_LINE_MAX      = 15

# ── PID ───────────────────────────────────────────────────
PID_KP               = 0.55
PID_KI               = 0.0
PID_KD               = 0.20
PID_INTEGRAL_LIMIT   = 1.0
PID_DERIVATIVE_ALPHA = 0.60
CONTROL_PERIOD       = 1.0 / 30.0


class LaneDetector:
    def __init__(self):
        self.last_offset      = 0.0
        self.last_debug_img   = None
        self._no_line_cnt     = 0
        self._pid_integral    = 0.0
        self._pid_prev_error  = None
        self._pid_prev_time   = None
        self._pid_prev_deriv  = 0.0
        self._prev_yellow_x   = None
        self._last_dx         = 0.0
        self._missing_frames  = 0

    def _reset_pid(self):
        self._pid_integral   = 0.0
        self._pid_prev_error = None
        self._pid_prev_time  = None
        self._pid_prev_deriv = 0.0

    def _compute_pid(self, error_norm: float) -> float:
        now = time.monotonic()
        dt  = CONTROL_PERIOD if self._pid_prev_time is None else (now - self._pid_prev_time)
        if dt < 1e-4:
            dt = CONTROL_PERIOD
        p_term = PID_KP * error_norm
        self._pid_integral = max(-PID_INTEGRAL_LIMIT,
                                  min(PID_INTEGRAL_LIMIT,
                                      self._pid_integral + error_norm * dt))
        i_term  = PID_KI * self._pid_integral
        raw_d   = 0.0 if self._pid_prev_error is None else (error_norm - self._pid_prev_error) / dt
        deriv   = PID_DERIVATIVE_ALPHA * self._pid_prev_deriv + (1 - PID_DERIVATIVE_ALPHA) * raw_d
        d_term  = PID_KD * deriv
        self._pid_prev_error = error_norm
        self._pid_prev_time  = now
        self._pid_prev_deriv = deriv
        return p_term + i_term + d_term

    def _yellow_mask(self, roi: np.ndarray) -> np.ndarray:
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
        n, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        out = np.zeros_like(mask)
        for i in range(1, n):
            if stats[i, cv2.CC_STAT_AREA] >= BLOB_MIN_AREA:
                out[labels == i] = 255
        return out

    def _histogram_peak(self, xs: np.ndarray, width: int, target_x: int) -> float:
        hist   = np.zeros(width, dtype=np.float32)
        np.add.at(hist, xs.clip(0, width - 1), 1.0)
        kernel = np.ones(HIST_KERNEL, dtype=np.float32)
        hist_s = np.convolve(hist, kernel, mode='same')
        lo = max(0, target_x - SEARCH_HALF)
        hi = min(width, target_x + SEARCH_HALF)
        local = hist_s[lo:hi]
        if local.max() == 0:
            return float(target_x)
        return float(lo + np.argmax(local))

    def compute_offset(self, image, lane_target):
        if image is None:
            return self.last_offset

        h, w     = image.shape[:2]
        target_x = LANE1_TARGET_X if lane_target == 0 else LANE2_TARGET_X

        # ── 하단 ROI (정밀 히스토그램) ───────────────────
        n_top  = int(h * ROI_NEAR_START)
        n_mask = self._blob_filter(self._yellow_mask(image[n_top:, :]))
        _, n_xs = np.where(n_mask > 0)

        # ── 원거리 ROI (커브 look-ahead, centroid) ────────
        f_top  = int(h * ROI_FAR_START)
        f_bot  = int(h * ROI_FAR_END)
        f_mask = self._blob_filter(self._yellow_mask(image[f_top:f_bot, :]))
        _, f_xs = np.where(f_mask > 0)

        # ── 디버그 ────────────────────────────────────────
        debug = image.copy()
        cv2.line(debug, (0, n_top), (w, n_top), (255, 255, 255), 2)
        cv2.line(debug, (0, f_top), (w, f_top), (180, 180, 0),   1)
        cv2.line(debug, (0, f_bot), (w, f_bot), (180, 180, 0),   1)
        debug[n_top:][n_mask > 0]         = [0, 255, 0]
        debug[f_top:f_bot][f_mask > 0]    = [0, 200, 200]
        cv2.line(debug, (target_x, f_top), (target_x, h), (255, 100, 0), 2)

        # ── too-many-yellow ───────────────────────────────
        yellow_ratio = np.count_nonzero(n_mask) / float((h - n_top) * w + 1)
        if yellow_ratio > YELLOW_MAX_RATIO:
            cv2.putText(debug, f'TOO_MANY {yellow_ratio:.2f}',
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
            self.last_debug_img = debug
            return self.last_offset

        # ── NO_LINE 처리 ──────────────────────────────────
        if n_xs.size < MIN_YELLOW_PX:
            self._missing_frames += 1
            self._no_line_cnt    += 1
            # velocity prediction
            if self._prev_yellow_x is not None and self._missing_frames <= MAX_PRED_FRAMES:
                yellow_x = self._prev_yellow_x + self._last_dx * self._missing_frames
                yellow_x = max(0.0, min(float(w - 1), yellow_x))
                source = 'predict'
            else:
                if self._no_line_cnt > NO_LINE_MAX:
                    if abs(self.last_offset) > 0.2:
                        pass   # 턴 중 → offset 유지
                    else:
                        self.last_offset *= NO_LINE_DECAY
                self._reset_pid()
                cv2.putText(debug, f'NO_LINE({self._no_line_cnt}) out={self.last_offset:.2f}',
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                self.last_debug_img = debug
                print(f'[LANE] NO_LINE({self._no_line_cnt}) -> {self.last_offset:.3f}', file=sys.stderr, flush=True)
                return self.last_offset
        else:
            self._missing_frames = 0
            self._no_line_cnt    = 0
            yellow_x = self._histogram_peak(n_xs, w, target_x)
            source   = 'hist'
            if self._prev_yellow_x is not None:
                dx = yellow_x - self._prev_yellow_x
                self._last_dx = (1 - DX_ALPHA) * self._last_dx + DX_ALPHA * dx
            self._prev_yellow_x = yellow_x

        # ── 커브 boost (원거리 ROI 기준) ─────────────────
        error_px = yellow_x - target_x
        boosted  = False
        if f_xs.size >= MIN_YELLOW_PX:
            f_mean = float(np.mean(f_xs))
            f_err  = f_mean - target_x
            if (f_err * error_px > 0
                    and abs(f_err) > abs(error_px) + CURVE_MIN_ERR):
                error_px *= CURVE_BOOST
                boosted = True

        # ── PID ───────────────────────────────────────────
        error_norm = max(-1.0, min(1.0, error_px / (w / 2.0)))
        offset     = max(-1.0, min(1.0, self._compute_pid(error_norm)))
        self.last_offset = offset

        # ── 디버그 오버레이 ───────────────────────────────
        cv2.line(debug, (int(yellow_x), n_top), (int(yellow_x), h), (255, 255, 0), 2)
        tag = '(CURVE)' if boosted else ''
        label = f'peak={int(yellow_x)}({source}){tag} tgt={target_x} err={int(error_px)} out={offset:.2f}'
        cv2.putText(debug, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 255), 2)
        self.last_debug_img = debug
        print(f'[LANE] peak={yellow_x:.0f} tgt={target_x} err={error_px:.1f} boost={boosted} out={offset:.3f}',
              file=sys.stderr, flush=True)
        return self.last_offset
