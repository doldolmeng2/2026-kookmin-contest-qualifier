#!/usr/bin/env python3
import cv2
import numpy as np
from collections import deque

class LaneDetector:
    def __init__(self):
        self.roi_y_ratio = 0.6          # 상단 60% 버림
        self.lane1_target_x = 453       # 1차선 주행 시 노란선 목표 x
        self.lane2_target_x = 217       # 2차선 주행 시 노란선 목표 x
        self.scurve_target_x = 320      # S자 구간: 이미지 중앙 (중앙선 위 주행)
        self.offset_history = deque(maxlen=5)  # smoothing (5개로 줄여 반응 빠르게)
        self.last_offset = 0.0
        self.is_scurve = False          # S자 구간 여부

        # 1차선 사다리꼴 ROI 비율 (ROI 기준)
        # 노란선이 오른쪽에 있으므로 사다리꼴도 오른쪽에 설정
        self.trap_top_left   = 0.35    # ROI 상단 왼쪽 x 비율
        self.trap_top_right  = 0.85    # ROI 상단 오른쪽 x 비율
        self.trap_bot_left   = 0.50    # ROI 하단 왼쪽 x 비율
        self.trap_bot_right  = 1.10    # ROI 하단 오른쪽 x 비율 (화면 밖 허용)

    def compute_offset(self, image, lane_target):
        if image is None:
            return 0.0

        h, w = image.shape[:2]
        roi_top = int(h * self.roi_y_ratio)
        roi = image[roi_top:h, :]
        roi_h, roi_w = roi.shape[:2]

        # BGR → HSV 변환
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # 노란색 마스킹
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([35, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # 노이즈 제거
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  np.ones((3, 3), np.uint8))

        # 1차선 주행 시 사다리꼴 ROI 적용 → S자 감지
        if lane_target == 0:
            trap_mask = self._make_trapezoid_mask(roi_h, roi_w)
            masked = cv2.bitwise_and(mask, trap_mask)
            yellow_pixels = np.where(masked > 0)

            if len(yellow_pixels[1]) < 30:
                # 사다리꼴 안에 노란선 없음 → S자 구간으로 판단
                self.is_scurve = True
            else:
                self.is_scurve = False

            if self.is_scurve:
                # S자 구간: 전체 마스크에서 노란선 찾아 중앙으로 유도
                all_pixels = np.where(mask > 0)
                if len(all_pixels[1]) < 30:
                    return self.last_offset
                mean_x = float(np.mean(all_pixels[1]))
                target_x = self.scurve_target_x
            else:
                mean_x = float(np.mean(yellow_pixels[1]))
                target_x = self.lane1_target_x
        else:
            # 2차선: 사다리꼴 없이 전체 ROI
            yellow_pixels = np.where(mask > 0)
            if len(yellow_pixels[1]) < 30:
                return self.last_offset
            mean_x = float(np.mean(yellow_pixels[1]))
            target_x = self.lane2_target_x

        # offset 계산 (-1 ~ 1 정규화)
        img_center_x = w / 2.0
        offset = (mean_x - target_x) / img_center_x

        # smoothing
        self.offset_history.append(offset)
        smoothed = float(np.mean(self.offset_history))
        self.last_offset = smoothed
        return smoothed

    def _make_trapezoid_mask(self, roi_h, roi_w):
        """1차선 주행용 사다리꼴 마스크 생성"""
        mask = np.zeros((roi_h, roi_w), dtype=np.uint8)
        points = np.array([
            [int(roi_w * self.trap_top_left),  0],
            [int(roi_w * self.trap_top_right), 0],
            [int(roi_w * self.trap_bot_right), roi_h - 1],
            [int(roi_w * self.trap_bot_left),  roi_h - 1],
        ], dtype=np.int32)
        cv2.fillPoly(mask, [points], 255)
        return mask

    def _apply_roi(self, image):
        h = image.shape[0]
        y0 = int(h * self.roi_y_ratio)
        return image[y0:, :]
