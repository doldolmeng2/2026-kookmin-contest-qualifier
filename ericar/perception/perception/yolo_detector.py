#!/usr/bin/env python3
"""YOLO 객체 인식 래퍼.

perception.py 에서 import 해서 사용한다. ROS 토픽은 다루지 않으며,
순수하게 "이미지 → 검출 결과" 변환만 책임진다.

추후 ultralytics(YOLO) 등으로 실제 모델을 로드/추론하도록 구현한다.
대회 인식 대상 클래스 예시:
    traffic_green, traffic_left, police_car, obstacle_car, start_line, shortcut_sign
"""


class Detection:
    """단일 검출 결과."""

    def __init__(self, label, confidence, bbox):
        self.label = label            # str
        self.confidence = confidence  # float 0~1
        self.bbox = bbox              # (x1, y1, x2, y2)

    def __repr__(self):
        return f'Detection({self.label}, {self.confidence:.2f}, {self.bbox})'


class YoloDetector:
    """YOLO 추론기. infer(img) → list[Detection] 또는 라벨별 dict 를 반환."""

    def __init__(self, weights_path=None, conf_threshold=0.5, device='cpu'):
        self.weights_path = weights_path
        self.conf_threshold = conf_threshold
        self.device = device
        self._model = None
        # self._load_model()

    def _load_model(self):
        # TODO(team): ultralytics YOLO 로드
        # from ultralytics import YOLO
        # self._model = YOLO(self.weights_path)
        pass

    def infer(self, image):
        """BGR 이미지 한 장을 받아 검출 결과를 반환한다.

        Returns:
            dict: {label: [Detection, ...]} 형태로 묶어서 반환 (조회 편의).
        """
        if self._model is None or image is None:
            return {}

        # TODO(team): 실제 추론
        # results = self._model(image, conf=self.conf_threshold, device=self.device)
        # detections = self._parse(results)
        detections = []

        grouped = {}
        for d in detections:
            grouped.setdefault(d.label, []).append(d)
        return grouped
