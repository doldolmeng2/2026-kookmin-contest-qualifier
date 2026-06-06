#!/usr/bin/env python3
"""방해차량 추월용 라이다 감지.

perception.py 에서 import. /scan(LaserScan) → 전방/전방우측/좌측 거리로
앞차·합류가능·추월완료를 판정한다.

★ 라이다 각도 규약 (이 시뮬 실측 — 기존 rubbercone/lidar_scan 가정과 다름):
    - index = 각도(deg), 0~359, 1°/step
    - 0° = 전방,  +각도 = 좌측,  315~359°(=-45~0°) = 전방-우측
    - 99~262° = 차량 자체에 가림(사용 불가). 순수 우측(270)·후방도 거의 막힘.
  → 쓸 수 있는 건 '전방 + 좌측(~98°) + 전방우측(315~)' 아크뿐.

추월 시나리오(2차선=오른쪽):
    앞차(전방) 따라가다 → 우측(전방우측) 간격 열리면 오른쪽 2차선으로 합류
    → 추월(앞차가 좌측으로) → 좌측 비면 추월완료 → 1차선 복귀
"""

import math

FRONT_DEG  = list(range(350, 360)) + list(range(0, 11))   # 전방 ±10°
FRIGHT_DEG = list(range(340, 359))                         # 전방-우측 '좁게'(도로변 나무 회피)
LEFT_DEG   = list(range(60, 96))                           # 좌측(추월 대상차)

# 임계값 (bag 실측 기반, 튜닝 가능)
FRONT_OBSTACLE_MAX = 6.0   # 전방 이 거리 이내면 → 앞차 있음
MERGE_CLEAR_MIN    = 7.0   # 전방우측이 이 거리보다 멀면 → 합류 가능(우측 차 앞서감)
PASSED_LEFT_MIN    = 4.0   # 좌측이 이 거리보다 멀면 → 추월 완료(옆차 빠짐)

_SELF_MIN = 0.3   # 이보다 가까운 값은 차체 self-hit 로 보고 무시


def _sector_min(scan, degs):
    r = scan.ranges
    n = len(r)
    vals = [r[d] for d in degs
            if d < n and math.isfinite(r[d]) and r[d] > _SELF_MIN]
    return min(vals) if vals else float('inf')


def detect_obstacle_front(scan):
    """전방에 앞차가 가까이 있으면 True."""
    if scan is None:
        return False
    return _sector_min(scan, FRONT_DEG) < FRONT_OBSTACLE_MAX


def detect_merge_clear(scan):
    """전방-우측(2차선)이 충분히 비어 합류 가능하면 True."""
    if scan is None:
        return False
    return _sector_min(scan, FRIGHT_DEG) > MERGE_CLEAR_MIN


def left_min(scan):
    """좌측 섹터 최소 거리(m). 비었으면 inf. (추월완료 판정은 perception에서 상태로)"""
    if scan is None:
        return float('inf')
    return _sector_min(scan, LEFT_DEG)


# 추월완료(obstacle_passed) 판정용 (perception 에서 상태 추적):
#   좌측에 차가 '붙었다(beside)' → '확 멀어졌다' 변화로 감지. 도로변 절대거리 무관.
CAR_BESIDE_MAX = 2.5   # 좌측이 이보다 가까우면 '옆에 차 붙음'
PASSED_GAP     = 1.0   # 붙었던 거리보다 이만큼 더 멀어지면 '추월 완료'
