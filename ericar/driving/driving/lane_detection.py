#!/usr/bin/env python3
"""차선 인식 → 조향 오프셋 계산.

driving.py 에서 import 해서 사용한다. ROS 토픽은 다루지 않고,
"카메라 이미지 + 목표 차선 → offset(float)" 변환만 책임진다.

offset 부호 약속:
    음수 = 좌측으로 조향 필요, 양수 = 우측으로 조향 필요 (control 의 비례식과 합의 필요)
"""

import cv2
import numpy as np

ROI_Y1 = 270
ROI_Y2 = 310

# HSV 노란색 범위
YELLOW_LOWER = np.array([25, 205, 80])
YELLOW_UPPER = np.array([35, 255, 255])


LANE_REF_X = {0: 377, 1: 225}  # 차선별 기준 cx (offset=0 기준점)

# 노이즈 필터용 사다리꼴 ROI (좌상→우상→우하→좌하, 시계방향)
TRAP_PTS = np.array([[290, 190], [350, 190], [400, 250], [240, 250]], dtype=np.int32)
TRAP_PIXEL_MIN = 60
TRAP_SPAN_MIN  = 300


class LaneDetector:

    def __init__(self):
        self.lane1_center_x = LANE_REF_X[0]
        self.lane2_center_x = LANE_REF_X[1]
        self._last_offset = 0.0
        self.reuse_reason = None  # None=fresh, str=재활용 이유

    def compute_offset(self, image, lane_target):
        """노란색 차선 cx - 기준 cx = offset 으로 반환.

        Args:
            image: BGR 이미지 (없으면 None)
            lane_target: 0=1차선(기준 cx=400), 1=2차선(기준 cx=220)
        Returns:
            float offset (양수=오른쪽, 음수=왼쪽), 검출 실패 시 이전 값 유지
        """
        if image is None:
            self.reuse_reason = 'no image (topic not received)'
            return self._last_offset

        if self._is_wide_noise_in_trapezoid(image):
            self.reuse_reason = 'wide noise in trapezoid'
            return self._last_offset

        cx = self._yellow_cx(image)
        if cx is None:
            self.reuse_reason = 'yellow lane not detected'
            return self._last_offset

        self.reuse_reason = None
        ref_x = LANE_REF_X.get(lane_target, LANE_REF_X[0])
        self._last_offset = float(cx - ref_x)
        return self._last_offset

    def detect_yellow(self, image):
        """ROI(y=270~330) 에서 노란색 차선 픽셀을 검출해 마스크를 반환.

        Returns:
            mask: ROI 크기의 이진 마스크 (uint8)
            roi:  ROI 원본 BGR 이미지
        """
        roi = image[ROI_Y1:ROI_Y2, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, YELLOW_LOWER, YELLOW_UPPER)
        return mask, roi

    def visualize_yellow(self, image):
        """노란색 차선 검출 결과를 화면에 표시. 확인용."""
        mask, roi = self.detect_yellow(image)

        vis = image.copy()

        # 사다리꼴 픽셀 수 + 차선 ROI x 스팬 계산
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        yellow_full = cv2.inRange(hsv, YELLOW_LOWER, YELLOW_UPPER)

        trap_bg = np.zeros(image.shape[:2], dtype=np.uint8)
        cv2.fillPoly(trap_bg, [TRAP_PTS], 255)
        trap_count = int(np.count_nonzero(cv2.bitwise_and(yellow_full, trap_bg)))

        roi_mask = yellow_full[ROI_Y1:ROI_Y2, :]
        roi_cols = np.where(roi_mask.any(axis=0))[0]
        roi_span = int(roi_cols[-1] - roi_cols[0]) if len(roi_cols) > 0 else 0

        noise_detected = trap_count >= TRAP_PIXEL_MIN and roi_span >= TRAP_SPAN_MIN
        trap_color = (0, 0, 255) if noise_detected else (0, 255, 0)
        cv2.polylines(vis, [TRAP_PTS], isClosed=True, color=trap_color, thickness=2)
        label = f'trap={trap_count}px roi_span={roi_span} {"IGNORED" if noise_detected else "OK"}'
        cv2.putText(vis, label, (TRAP_PTS[:, 0].min(), TRAP_PTS[:, 1].min() - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, trap_color, 1)

        # 차선 ROI 박스
        cv2.rectangle(vis, (0, ROI_Y1), (image.shape[1] - 1, ROI_Y2), (0, 255, 0), 2)

        # 마스크에서 윤곽선 추출 → 원본 이미지에 오버레이
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if cv2.contourArea(cnt) < 50:
                continue
            cnt_shifted = cnt + np.array([[[0, ROI_Y1]]])
            cv2.drawContours(vis, [cnt_shifted], -1, (0, 0, 255), 2)

        # 무게중심 x 표시
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00']) + ROI_Y1
            cv2.circle(vis, (cx, cy), 6, (255, 0, 0), -1)
            cv2.putText(vis, f'cx={cx}', (cx + 8, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        # cv2.imshow('yellow_mask', mask)
        cv2.imshow('yellow_detect', vis)
        cv2.waitKey(1)

    def _is_wide_noise_in_trapezoid(self, image):
        """사다리꼴 픽셀 수 >= 100 AND 차선 ROI x 스팬 >= 100 이면 True."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        yellow_mask = cv2.inRange(hsv, YELLOW_LOWER, YELLOW_UPPER)

        # 조건1: 사다리꼴 안 노란색 픽셀 수
        trap_mask = np.zeros(image.shape[:2], dtype=np.uint8)
        cv2.fillPoly(trap_mask, [TRAP_PTS], 255)
        trap_count = int(np.count_nonzero(cv2.bitwise_and(yellow_mask, trap_mask)))
        if trap_count < TRAP_PIXEL_MIN:
            return False

        # 조건2: 차선 ROI(ROI_Y1~ROI_Y2) 안 노란색 x 스팬
        roi_mask = yellow_mask[ROI_Y1:ROI_Y2, :]
        cols = np.where(roi_mask.any(axis=0))[0]
        if len(cols) == 0:
            return False
        return int(cols[-1] - cols[0]) >= TRAP_SPAN_MIN

    def _yellow_cx(self, image):
        """ROI 노란색 마스크의 무게중심 x를 반환. 검출 실패 시 None."""
        mask, _ = self.detect_yellow(image)
        M = cv2.moments(mask)
        if M['m00'] == 0:
            return None
        return int(M['m10'] / M['m00'])
