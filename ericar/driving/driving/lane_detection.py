#!/usr/bin/env python3
import cv2
import numpy as np
from collections import deque

class LaneDetector:
    def __init__(self):
        self.roi_y_ratio = 0.6
        self.lane1_target_x = 480   # 1차선 주행 시 노란선 목표 x (튜닝 필요)
        self.lane2_target_x = 217   # 2차선 주행 시 노란선 목표 x (튜닝 필요)
        self.offset_history = deque(maxlen=10)
        self.last_offset = 0.0

    def compute_offset(self, image, lane_target):
        if image is None:
            return 0.0

        h, w = image.shape[:2]
        roi_top = int(h * self.roi_y_ratio)
        roi = image[roi_top:h, :]

        # BGR → HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # 노란색 마스킹
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([35, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        yellow_pixels = np.where(mask > 0)

        if len(yellow_pixels[1]) < 50:
            return self.last_offset

        mean_x = float(np.mean(yellow_pixels[1]))

        # 차선에 따라 목표 x 선택
        target_x = self.lane2_target_x if lane_target == 1 else self.lane1_target_x

        # -1 ~ 1 정규화
        img_center_x = w / 2.0
        offset = (mean_x - target_x) / img_center_x

        # smoothing
        self.offset_history.append(offset)
        smoothed = float(np.mean(self.offset_history))
        self.last_offset = smoothed

        return smoothed

    def _apply_roi(self, image):
        h = image.shape[0]
        y0 = int(h * self.roi_y_ratio)
        return image[y0:, :]
