#!/usr/bin/env python3
"""YOLO 객체 인식 래퍼 (ultralytics, CPU 추론).

perception.py 에서 import 해서 사용한다. ROS 토픽은 다루지 않으며,
"이미지 → 검출 결과" 변환만 책임진다.

CPU 로 추론하므로 노트북 등 GPU 없는 환경에서도 동일하게 동작한다.
weight 는 패키지 내부(perception/weights/police.pt)에서 자동으로 찾는다.

학습 대상 클래스(현재): police_car
추후 pedestrian, stop_line 등을 추가 학습하면 클래스만 늘면 된다.
"""

import os


class Detection:
    """단일 검출 결과."""

    def __init__(self, label, confidence, bbox):
        self.label = label            # str
        self.confidence = confidence  # float 0~1
        self.bbox = bbox              # (x1, y1, x2, y2) 픽셀

    def __repr__(self):
        return f'Detection({self.label}, {self.confidence:.2f}, {self.bbox})'


# 패키지 내부 기본 weight 경로 (이 파일 기준 상대 → 설치 위치 무관, 노트북도 동일)
_DEFAULT_WEIGHTS = os.path.join(os.path.dirname(__file__), 'weights', 'police.pt')


class YoloDetector:
    """YOLO 추론기. infer(img) → {label: [Detection, ...]} 반환."""

    def __init__(self, weights_path=None, conf_threshold=0.5, device='cpu'):
        self.weights_path = weights_path or _DEFAULT_WEIGHTS
        self.conf_threshold = conf_threshold
        self.device = device          # 'cpu' 고정 (노트북 호환)
        self._model = None
        self._names = {}
        self._failed = False          # 로드 실패 시 True → 이후 infer 는 빈 결과

    def _load_model(self):
        from ultralytics import YOLO
        self._model = YOLO(self.weights_path)
        self._names = self._model.names   # {id: label}

    def warmup(self):
        """모델 로딩 + 1회 더미 추론 (첫 추론 지연을 시작 시점으로 옮김)."""
        import numpy as np
        if self._model is None:
            self._load_model()
        self._model.predict(np.zeros((640, 640, 3), dtype=np.uint8),
                            device=self.device, verbose=False)

    def infer(self, image):
        """BGR 이미지 한 장 → {label: [Detection, ...]}.

        모델/이미지가 없으면 빈 dict 반환.
        """
        if image is None or self._failed:
            return {}
        if self._model is None:
            try:
                self._load_model()
            except Exception:
                self._failed = True
                return {}

        results = self._model.predict(
            image, conf=self.conf_threshold, device=self.device,
            verbose=False)

        grouped = {}
        if not results:
            return grouped
        r = results[0]
        if r.boxes is None:
            return grouped
        for b in r.boxes:
            cls_id = int(b.cls[0])
            label = self._names.get(cls_id, str(cls_id))
            conf = float(b.conf[0])
            x1, y1, x2, y2 = (float(v) for v in b.xyxy[0])
            grouped.setdefault(label, []).append(
                Detection(label, conf, (x1, y1, x2, y2)))
        return grouped
