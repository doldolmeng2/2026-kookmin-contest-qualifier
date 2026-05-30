#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# traffic_light_start.py  (Team ERICAR perception - 시작 신호등 3구)
#
#   [빨강][주황][초록]  — WAITING_START 에서만 사용. 가까이·정면·정지 상태.
#   한 번에 한 색만 켜지므로 '가장 큰 램프 색'을 그대로 반환한다.
#
#   ★ 시작등만 튜닝하려면 이 파일의 DEFAULT_PARAMS 만 고치면 된다.
#     (트랙등에는 전혀 영향 없음)
# =============================================================================

from ericar_msgs.msg import Perception
from perception.detectors.traffic_light_base import TrafficLightBase, BASE_PARAMS


class StartTrafficLightDetector(TrafficLightBase):
    NAME = 'start'

    # 시작등은 가까이 크게 보이고, 켜진 램프 구멍이 큼
    #   → 최소 면적 ↑, 구멍 메움(close) ↑, 채움비율 기준 ↓(구멍 허용)
    DEFAULT_PARAMS = dict(
        BASE_PARAMS,
        housing_min_area=2500,
        close_k=25,
        min_extent=0.25,
        max_cy=0.65,
    )

    # 3구: 가장 큰(=켜진) 램프의 색을 반환
    def _classify(self, lamps):
        if not lamps:
            return Perception.SIGNAL_NONE
        return max(lamps.items(), key=lambda kv: kv[1][0])[0]  # area 최대
