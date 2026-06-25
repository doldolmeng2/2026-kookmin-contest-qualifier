#!/usr/bin/env python3
"""S자 커브 구간 끝 감지 — 흰색 나무 + 초록 나무 + 라이다 복합 판정.

perception.py 에서 import.
ROI 내 흰색/초록 픽셀 비율과 전방 중거리(8~16 m) 라이다 포인트 수로
S자 커브 구간이 끝났음을 판정한다.

임계값 튜닝 방법:
  실행 중 터미널에 '[SCURVE]' 로그(1초마다)가 출력된다.
  S커브 끝 지점에 차를 세우고 wht= / grn= / lidar= 값을 보고
  아래 *_MIN_RATIO / LIDAR_MIN_POINTS 를 조정하면 된다.
"""

import math
import cv2
import numpy as np

# ── ROI 설정 (이미지 height/width 비율) ──────────────────────────────────
# 나무들이 나타나는 화면 중상단 영역. 하늘과 도로 제외.
ROI_TOP   = 0.20
ROI_BOT   = 0.55
ROI_LEFT  = 0.15
ROI_RIGHT = 0.85

# ── 흰색 나무 HSV 범위 ───────────────────────────────────────────────────
WHT_H_MIN =   0
WHT_H_MAX = 179
WHT_S_MIN =   4
WHT_S_MAX =  77
WHT_V_MIN = 130
WHT_V_MAX = 250

# ── 초록 나무 HSV 범위 ───────────────────────────────────────────────────
GRN_H_LO  = 35
GRN_H_HI  = 90
GRN_S_MIN = 60
GRN_V_MIN = 60

# ── 색 판정 임계값 (실측 후 조정) ────────────────────────────────────────
WHT_MIN_RATIO = 0.05   # ROI 내 흰색 픽셀 비율 하한
GRN_MIN_RATIO = 0.05   # ROI 내 초록 픽셀 비율 하한

# ── 라이다 파라미터 ──────────────────────────────────────────────────────
# 각도 규약: index = 각도(deg), 0=전방, +방향=좌측
LIDAR_FRONT_DEG  = list(range(0, 21)) + list(range(340, 360))  # 전방 ±20°
LIDAR_DIST_MIN   = 15.0   # m — 탐색 시작 거리 (이보다 가까운 건 도로/라바콘)
LIDAR_DIST_MAX   = 20.0   # m — 탐색 끝 거리
LIDAR_MIN_POINTS = 6      # 해당 구간 포인트 수 ≥ 이 값이면 전방에 물체 있음


def detect_s_curve_end(image, scan):
    """BGR 이미지 + LaserScan → (결과, 흰색비율, 초록비율, 라이다포인트수).

    결과(bool): 흰색·초록 픽셀 비율과 라이다 포인트 수가 모두 임계값 이상이면 True.
    나머지 세 값은 임계값 튜닝용 디버그 수치.
    """
    white_ratio, green_ratio = _color_ratios(image)
    lidar_pts = _lidar_front_count(scan)

    white_ok = white_ratio >= WHT_MIN_RATIO
    green_ok = green_ratio >= GRN_MIN_RATIO
    lidar_ok = lidar_pts  >= LIDAR_MIN_POINTS

    result = white_ok and green_ok and lidar_ok
    return result, white_ratio, green_ratio, lidar_pts


def draw_s_curve_debug(image, scan):
    """S커브 감지 디버그 창('scurve_debug')을 갱신한다.

    상단: 카메라 원본 + ROI 박스 + 판정 텍스트
    하단 좌: 흰색 마스크  하단 중: 초록 마스크  하단 우: 라이다 조감도
    """
    if image is None:
        return

    result, wht, grn, lidar_pts = detect_s_curve_end(image, scan)

    h, w = image.shape[:2]
    rx0, ry0 = int(w * ROI_LEFT), int(h * ROI_TOP)
    rx1, ry1 = int(w * ROI_RIGHT), int(h * ROI_BOT)

    # ── 상단: 카메라 원본 + ROI 박스 ────────────────────────────────────
    top = image.copy()
    roi_color = (0, 255, 0) if result else (0, 80, 255)
    cv2.rectangle(top, (rx0, ry0), (rx1, ry1), roi_color, 2)

    wht_ok = wht >= WHT_MIN_RATIO
    grn_ok = grn >= GRN_MIN_RATIO
    lid_ok = lidar_pts >= LIDAR_MIN_POINTS

    def _put(img, text, y, color):
        cv2.putText(img, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)

    ok_c = (0, 255, 0)
    ng_c = (0, 80, 255)
    _put(top, f'WHT  {wht:.3f} / {WHT_MIN_RATIO}  {"OK" if wht_ok else "NG"}', 22, ok_c if wht_ok else ng_c)
    _put(top, f'GRN  {grn:.3f} / {GRN_MIN_RATIO}  {"OK" if grn_ok else "NG"}', 44, ok_c if grn_ok else ng_c)
    _put(top, f'LDR  {lidar_pts:3d} / {LIDAR_MIN_POINTS}  {"OK" if lid_ok else "NG"}', 66, ok_c if lid_ok else ng_c)
    label = 'S-CURVE END DETECTED' if result else 'not detected'
    _put(top, label, 96, ok_c if result else (160, 160, 160))

    # ── 하단 패널 높이 = ROI 높이 ────────────────────────────────────────
    mask_h = max(ry1 - ry0, 1)
    panel_w  = w // 3          # 좌·중 패널 폭
    panel_w3 = w - 2 * panel_w  # 우 패널 폭 (나머지 픽셀 포함해 top 폭과 일치)

    roi_crop = image[ry0:ry1, rx0:rx1]
    if roi_crop.size > 0:
        hsv = cv2.cvtColor(roi_crop, cv2.COLOR_BGR2HSV)
        H, S, V = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]

        wht_mask = ((H >= WHT_H_MIN) & (H <= WHT_H_MAX)
                    & (S >= WHT_S_MIN) & (S <= WHT_S_MAX)
                    & (V >= WHT_V_MIN) & (V <= WHT_V_MAX)).astype(np.uint8) * 255
        grn_mask = ((H >= GRN_H_LO) & (H <= GRN_H_HI)
                    & (S >= GRN_S_MIN) & (V >= GRN_V_MIN)).astype(np.uint8) * 255

        wht_vis = cv2.cvtColor(wht_mask, cv2.COLOR_GRAY2BGR)
        wht_vis[wht_mask > 0] = (220, 180, 0)   # 하늘색

        grn_vis = cv2.cvtColor(grn_mask, cv2.COLOR_GRAY2BGR)
        grn_vis[grn_mask > 0] = (0, 220, 80)    # 초록

        cv2.putText(wht_vis, f'WHITE  {wht:.3f}', (4, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        cv2.putText(grn_vis, f'GREEN  {grn:.3f}', (4, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        wht_rs = cv2.resize(wht_vis, (panel_w, mask_h))
        grn_rs = cv2.resize(grn_vis, (panel_w, mask_h))
    else:
        wht_rs = np.zeros((mask_h, panel_w, 3), np.uint8)
        grn_rs = np.zeros((mask_h, panel_w, 3), np.uint8)

    # ── 하단 우: 라이다 조감도 ───────────────────────────────────────────
    ldr_vis = _draw_lidar_panel(scan, lidar_pts, lid_ok, panel_w3, mask_h)

    bottom = np.hstack([wht_rs, grn_rs, ldr_vis])
    canvas = np.vstack([top, bottom])
    cv2.imshow('scurve_debug', canvas)


def _draw_lidar_panel(scan, lidar_pts, lid_ok, pw, ph):
    """라이다 전방 조감도 패널을 그려 반환한다.

    - 회색 점: 유효하지만 탐색 범위(8~16 m) 밖
    - 밝은 노랑 점: 탐색 범위(8~16 m) 안 — S커브 끝 나무 후보
    - 파란 링: LIDAR_DIST_MIN(내)  빨간 링: LIDAR_DIST_MAX(외)
    """
    img = np.zeros((ph, pw, 3), np.uint8)
    cx, cy = pw // 2, ph - 10          # 차량 위치: 하단 중앙
    # 16 m 가 패널 세로 전체에 맞도록 scale 계산
    scale = max((ph - 20) / LIDAR_DIST_MAX, 1.0)

    # 거리 링
    for dist, color in [(LIDAR_DIST_MIN, (180, 100, 0)),
                        (LIDAR_DIST_MAX, (0,  80, 200))]:
        r_px = int(dist * scale)
        if r_px < pw:
            cv2.circle(img, (cx, cy), r_px, color, 1)
            cv2.putText(img, f'{dist:.0f}m', (cx + r_px + 2, cy - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.32, color, 1)

    # 섹터 경계선 (전방 ±20°)
    for sign in (+1, -1):
        ang = math.radians(sign * 20)
        ex = int(cx - LIDAR_DIST_MAX * scale * math.sin(ang))
        ey = int(cy - LIDAR_DIST_MAX * scale * math.cos(ang))
        cv2.line(img, (cx, cy), (ex, ey), (55, 55, 55), 1)

    # 라이다 포인트
    if scan is not None:
        r = scan.ranges
        n = len(r)
        front_set = set(LIDAR_FRONT_DEG)
        for d in range(min(360, n)):
            dist = r[d]
            if not math.isfinite(dist) or dist <= 0.3:
                continue
            ang = math.radians(d)
            px = int(cx - dist * scale * math.sin(ang))
            py = int(cy - dist * scale * math.cos(ang))
            if not (0 <= px < pw and 0 <= py < ph):
                continue
            in_sector = d in front_set
            in_range  = LIDAR_DIST_MIN <= dist <= LIDAR_DIST_MAX
            if in_sector and in_range:
                color = (0, 240, 240)   # 밝은 노랑(청록) — 감지 대상
            elif in_sector:
                color = (80, 80, 80)    # 섹터 안이지만 범위 밖
            else:
                color = (45, 45, 45)    # 섹터 밖
            cv2.circle(img, (px, py), 2, color, -1)

    # 차량 마커
    cv2.rectangle(img, (cx - 5, cy - 3), (cx + 5, cy + 3), (0, 160, 255), -1)

    # 포인트 수 + OK/NG
    pt_color = (0, 255, 0) if lid_ok else (0, 80, 255)
    cv2.putText(img, f'LDR {lidar_pts}pts {"OK" if lid_ok else "NG"}',
                (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.4, pt_color, 1)
    cv2.putText(img, f'{LIDAR_DIST_MIN:.0f}~{LIDAR_DIST_MAX:.0f}m front+-20',
                (4, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (130, 130, 130), 1)
    return img


# ── 내부 헬퍼 ────────────────────────────────────────────────────────────

def _color_ratios(image):
    """ROI 내 흰색 픽셀 비율, 초록 픽셀 비율을 반환한다."""
    if image is None:
        return 0.0, 0.0
    h, w = image.shape[:2]
    roi = image[int(h * ROI_TOP):int(h * ROI_BOT),
                int(w * ROI_LEFT):int(w * ROI_RIGHT)]
    if roi.size == 0:
        return 0.0, 0.0

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    H, S, V = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]

    white_mask = ((H >= WHT_H_MIN) & (H <= WHT_H_MAX)
                  & (S >= WHT_S_MIN) & (S <= WHT_S_MAX)
                  & (V >= WHT_V_MIN) & (V <= WHT_V_MAX))
    green_mask = (H >= GRN_H_LO) & (H <= GRN_H_HI) & (S >= GRN_S_MIN) & (V >= GRN_V_MIN)

    total = roi.shape[0] * roi.shape[1]
    return float(white_mask.sum()) / total, float(green_mask.sum()) / total


def _lidar_front_count(scan):
    """전방 ±20° 섹터에서 LIDAR_DIST_MIN ~ LIDAR_DIST_MAX 구간 포인트 수를 반환한다."""
    if scan is None:
        return 0
    r = scan.ranges
    n = len(r)
    count = 0
    for d in LIDAR_FRONT_DEG:
        if d >= n:
            continue
        dist = r[d]
        if math.isfinite(dist) and LIDAR_DIST_MIN <= dist <= LIDAR_DIST_MAX:
            count += 1
    return count
