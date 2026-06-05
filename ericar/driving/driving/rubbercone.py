#!/usr/bin/env python3
import cv2
import numpy as np
from collections import deque

IMAGE_WIDTH = 640
IMAGE_CENTER = IMAGE_WIDTH // 2  # 320
ROI_TOP_RATIO = 0.4  # 상단 50% 버림

class ConeDriver:
    def __init__(self):
        self.offset_history = deque(maxlen=10)
        self.last_offset = 0.0

    def compute_offset(self, image):
        if image is None:
            return 0.0

        h, w = image.shape[:2]
        roi_top = int(h * ROI_TOP_RATIO)
        roi = image[roi_top:h, :]

        # BGR → HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # 주황색 마스킹
        lower_orange = np.array([5, 150, 150])
        upper_orange = np.array([20, 255, 255])
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # 좌/우 분리
        left_mask  = mask[:, :IMAGE_CENTER]
        right_mask = mask[:, IMAGE_CENTER:]

        left_pixels  = np.where(left_mask > 0)
        right_pixels = np.where(right_mask > 0)

        left_detected  = len(left_pixels[1]) >= 10
        right_detected = len(right_pixels[1]) >= 10

        if left_detected and right_detected:
            left_x  = float(np.mean(left_pixels[1]))
            right_x = float(np.mean(right_pixels[1])) + IMAGE_CENTER
            midpoint = (left_x + right_x) / 2.0
            offset = (midpoint - IMAGE_CENTER) / IMAGE_CENTER
        elif left_detected:
            left_x = float(np.mean(left_pixels[1]))
            offset = (left_x - IMAGE_CENTER * 0.5) / IMAGE_CENTER
        elif right_detected:
            right_x = float(np.mean(right_pixels[1])) + IMAGE_CENTER
            offset = (right_x - IMAGE_CENTER * 1.5) / IMAGE_CENTER
        else:
            return self.last_offset

        offset = max(-1.0, min(1.0, offset))

        self.offset_history.append(offset)
        smoothed = float(np.mean(self.offset_history))
        self.last_offset = smoothed

        return smoothed
