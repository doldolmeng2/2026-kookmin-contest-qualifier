#!/usr/bin/env python3
"""차선 인식 → 조향 오프셋 계산.

driving.py 에서 import 해서 사용한다. ROS 토픽은 다루지 않고,
"카메라 이미지 + 목표 차선 → offset(float)" 변환만 책임진다.

offset 부호 약속:
    음수 = 좌측으로 조향 필요, 양수 = 우측으로 조향 필요 (control 의 비례식과 합의 필요)
"""

import cv2
import numpy as np


class LaneDetector:

    def __init__(self):
        # TODO(team): ROI, 색/엣지 임계값, 차선 폭(px), 차선별 기준 x 등 파라미터
        self.roi_y_ratio = 0.6        # 화면 아래쪽 비율부터 ROI
        self.lane1_center_x = None    # 1차선 주행 시 목표 중심 x
        self.lane2_center_x = None    # 2차선 주행 시 목표 중심 x

    def compute_offset(self, image, lane_target):
        """차선을 인식해 목표 중심과 현재 위치의 오차를 offset 으로 반환.

        Args:
            image: BGR 이미지 (없으면 None)
            lane_target: 0=1차선, 1=2차선
        Returns:
            float offset, 또는 인식 실패 시 0.0 (직진 유지)
        """
        if image is None:
            return 0.0

        # TODO(team): 실제 차선 검출 파이프라인
        # 1) ROI 자르기
        # 2) 색/엣지 마스크
        # 3) 좌/우 차선 픽셀 → 차선 중심 x 추정
        # 4) lane_target 에 따라 목표 중심 선택
        # 5) (목표 중심 - 화면 중심) 정규화 → offset
        lane_center_x = self._estimate_lane_center(image, lane_target)
        if lane_center_x is None:
            return 0.0

        img_center_x = image.shape[1] / 2.0
        offset = (lane_center_x - img_center_x) / img_center_x  # -1 ~ 1 정규화
        return float(offset)

    def _estimate_lane_center(self, image, lane_target):
        # TODO(team): 차선 중심 x 추정 구현
        return None

    # 참고용 유틸 (추후 사용)
    def _apply_roi(self, image):
        h = image.shape[0]
        y0 = int(h * self.roi_y_ratio)
        return image[y0:, :]
