#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# traffic_light_track.py  (Team ERICAR perception - 트랙 위 신호등 4구)
#
#   배치(왼→오): [1 빨강][2 주황][3 좌회전(초록)][4 직진초록]
#     · 3번(좌회전)과 4번(직진)이 둘 다 초록 → '위치(pos)'로 구분
#     · 좌회전 신호 시 빨강(1번)+좌회전(3번) 동시 점등 → 좌회전 우선
#
#   우선순위:  좌회전(초록@3번) > 직진(초록@4번) > 빨강 > 주황
#
#   ★ 트랙등만 튜닝하려면 이 파일의 DEFAULT_PARAMS / left_pos_max 만 고치면 된다.
#     (시작등에는 전혀 영향 없음)
# =============================================================================

from ericar_msgs.msg import Perception
from perception.detectors.traffic_light_base import TrafficLightBase, BASE_PARAMS


class TrackTrafficLightDetector(TrafficLightBase):
    NAME = 'track'

    # 트랙등은 멀리 작게 보일 수 있어 면적·커널을 작게 (하한이라 가까워 커져도 OK)
    DEFAULT_PARAMS = dict(
        BASE_PARAMS,
        housing_min_area=600,
        lamp_min_area=15,
        close_k=11,        # 작은 하우징을 과하게 안 깎도록 커널 축소 (깜빡임 완화)
        open_k=5,
        lamp_pad=2,        # 하우징 안쪽만 → 나무 조각이 초록으로 끼는 것 방지
    )

    # 4구 판단: '좌회전이면 빨강+초록이 같이 켜진다'는 사실로 구분 (위치 안 씀)
    #   초록+빨강 동시 → 좌회전 / 초록만 → 직진 / 빨강만 → 빨강 / 주황 → 주황
    def _classify(self, lamps):
        has_g = Perception.SIGNAL_GREEN in lamps
        has_r = Perception.SIGNAL_RED in lamps
        has_y = Perception.SIGNAL_YELLOW in lamps
        if has_g and has_r:
            return Perception.SIGNAL_LEFT
        if has_g:
            return Perception.SIGNAL_GREEN
        if has_r:
            return Perception.SIGNAL_RED
        if has_y:
            return Perception.SIGNAL_YELLOW
        return Perception.SIGNAL_NONE
