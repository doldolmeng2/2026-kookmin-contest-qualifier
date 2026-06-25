#!/usr/bin/env python3
"""straight_test.py - ROI + BEV 시각화 노드.

하단 10%~40% 구간 ROI에서 노란 차선을 검출하고 BEV 변환 결과를 나란히 표시.
    왼쪽 패널: 원본 + ROI 영역 + 노란 픽셀 강조
    오른쪽 패널: BEV 변환 결과 + 슬라이딩 윈도우 + 다항식 피팅 + offset

실행:
    ros2 run driving straight_test
"""

import traceback

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

# ── ROI 설정 (이미지 하단 기준 %) ────────────────────────────────────────────
ROI_BOTTOM_PCT = 0.10   # 하단에서 10% 지점  → y = h * 0.90
ROI_TOP_PCT    = 0.40   # 하단에서 40% 지점  → y = h * 0.60

# ── 노란색 HSV 범위 ───────────────────────────────────────────────────────────
YELLOW_LOWER = np.array([25, 205,  80])
YELLOW_UPPER = np.array([35, 255, 255])

# ── BEV 출력 크기 ─────────────────────────────────────────────────────────────
BEV_W = 400
BEV_H = 300

# ── BEV dst 하단 여백 ────────────────────────────────────────────────────────
DST_X_MARGIN = 0.32   # BEV 하단 좌우 여백 비율 (0=직사각형, 0.5=삼각형)

# ── 슬라이딩 윈도우 파라미터 ─────────────────────────────────────────────────
N_WINDOWS  = 20   # BEV를 세로로 몇 구간으로 나눌지
WIN_MARGIN = 40   # 윈도우 좌우 반폭 (px)
MIN_PIX    = 20   # 유효 판정 최소 노란 픽셀 수
MAX_X_SPAN = 60  # 이보다 x 스팬이 넓으면 노이즈로 무시 (px)

# ── offset 기준값 ─────────────────────────────────────────────────────────────
# 피팅 곡선의 y=0(BEV 상단, 먼 쪽)에서 노란선이 있어야 할 목표 x
# 노란색은 중앙선이므로 실제로는 한쪽으로 치우쳐 있음
#   1차선 주행: 노란선이 오른쪽 → BEV_W/2(=200)보다 큰 값
#   2차선 주행: 노란선이 왼쪽  → BEV_W/2(=200)보다 작은 값
LANE_REF_X = 230   # 튜닝 포인트 (BEV_W/2 = 200)

# ── 곡률 가중치 ───────────────────────────────────────────────────────────────
K_CURVE = 200.0   # 곡률 계수 a → offset 추가 가중치 (튜닝 필요)

# BEV 픽셀 offset → /driving/offset 발행값 스케일
# offset(px) / (BEV_W/2) * OFFSET_SCALE → -100~100 범위
OFFSET_SCALE = 200.0

WIN_NAME = 'straight_test'


def build_bev_matrix(h, w):
    """ROI 전체 직사각형(src) → BEV 사다리꼴(dst) 변환 행렬 반환."""
    y_top    = int(h * (1.0 - ROI_TOP_PCT))
    y_bottom = int(h * (1.0 - ROI_BOTTOM_PCT))

    src = np.float32([
        [0, y_bottom],
        [w, y_bottom],
        [w, y_top   ],
        [0, y_top   ],
    ])

    x_m = int(BEV_W * DST_X_MARGIN)
    dst = np.float32([
        [x_m,         BEV_H],
        [BEV_W - x_m, BEV_H],
        [BEV_W,       0    ],
        [0,           0    ],
    ])

    M = cv2.getPerspectiveTransform(src, dst)
    return M, src, y_top, y_bottom


class StraightTest(Node):

    def __init__(self):
        super().__init__('straight_test')
        self._M        = None
        self._src_pts  = None
        self._y_top    = None
        self._y_bottom = None
        self._offset   = 0.0
        self._frame    = None

        self.create_subscription(
            Image, '/usb_cam/image_raw/front',
            self._image_cb, qos_profile_sensor_data)

        self._offset_pub = self.create_publisher(Float32, '/driving/offset', 10)

        self.create_timer(1.0 / 30.0, self._tick)
        self.get_logger().info('straight_test ready')

    # ── 이미지 콜백: 프레임 저장만 ───────────────────────────────────────────
    def _image_cb(self, msg):
        frame = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
            (msg.height, msg.width, 3)).copy()
        if msg.encoding == 'rgb8':
            frame = frame[:, :, ::-1].copy()
        self._frame = frame

    # ── 30Hz 타이머: 처리 + 시각화 ──────────────────────────────────────────
    def _tick(self):
        if self._frame is None:
            placeholder = np.zeros((300, 600, 3), dtype=np.uint8)
            cv2.putText(placeholder, 'Waiting for camera...',
                        (60, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.imshow(WIN_NAME, placeholder)
            cv2.waitKey(1)
            return
        try:
            frame = self._frame
            h, w  = frame.shape[:2]

            if self._M is None:
                self._M, self._src_pts, self._y_top, self._y_bottom = \
                    build_bev_matrix(h, w)
                self.get_logger().info(
                    f'BEV matrix built: img={w}x{h}  '
                    f'ROI y={self._y_top}~{self._y_bottom}  '
                    f'LANE_REF_X={LANE_REF_X}')

            left_panel  = self._draw_original(frame, h, w)
            right_panel = self._draw_bev(frame)

            rh = left_panel.shape[0]
            right_panel = cv2.resize(right_panel, (int(BEV_W * rh / BEV_H), rh))

            cv2.imshow(WIN_NAME, np.hstack([left_panel, right_panel]))
            cv2.waitKey(1)

            pub_val = self._offset / (BEV_W / 2) * OFFSET_SCALE
            self._offset_pub.publish(Float32(data=float(pub_val)))

        except Exception as e:
            self.get_logger().error(f'tick: {e}\n{traceback.format_exc()}')

    # ── 왼쪽 패널: 원본 오버레이 ──────────────────────────────────────────────
    def _draw_original(self, frame, h, w):
        vis = frame.copy()
        y0, y1 = self._y_top, self._y_bottom

        roi_hsv = cv2.cvtColor(frame[y0:y1, :], cv2.COLOR_BGR2HSV)
        mask    = cv2.inRange(roi_hsv, YELLOW_LOWER, YELLOW_UPPER)

        full_mask = np.zeros((h, w), dtype=np.uint8)
        full_mask[y0:y1] = mask
        vis[full_mask > 0] = (0, 220, 255)

        cv2.rectangle(vis, (0, y0), (w - 1, y1), (0, 255, 0), 2)
        cv2.putText(vis,
                    f'ROI  bottom {int(ROI_BOTTOM_PCT*100)}%~{int(ROI_TOP_PCT*100)}%'
                    f'  y={y0}~{y1}',
                    (6, y0 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)

        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00']) + y0
            cv2.circle(vis, (cx, cy), 8, (0, 0, 255), -1)
            cv2.putText(vis, f'cx={cx}', (cx + 10, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        else:
            cv2.putText(vis, 'NO YELLOW', (10, y0 + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        cv2.line(vis, (w // 2, y0), (w // 2, y1), (100, 100, 100), 1)
        return vis

    # ── 오른쪽 패널: BEV + 슬라이딩 윈도우 + 다항식 피팅 ────────────────────
    def _draw_bev(self, frame):
        bev = cv2.warpPerspective(frame, self._M, (BEV_W, BEV_H))

        bev_hsv  = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
        bev_mask = cv2.inRange(bev_hsv, YELLOW_LOWER, YELLOW_UPPER)

        vis = bev.copy()
        vis[bev_mask > 0] = (0, 220, 255)

        # BEV 중심선 (회색)
        cv2.line(vis, (BEV_W // 2, 0), (BEV_W // 2, BEV_H), (80, 80, 80), 1)

        # 목표 기준선 (하늘색 점선)
        ref_x_clipped = int(np.clip(LANE_REF_X, 0, BEV_W - 1))
        for y_dash in range(0, BEV_H, 12):
            cv2.line(vis, (ref_x_clipped, y_dash),
                     (ref_x_clipped, min(y_dash + 6, BEV_H)), (200, 180, 0), 1)
        cv2.putText(vis, f'REF={LANE_REF_X}',
                    (ref_x_clipped + 4, BEV_H - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)

        # ── 슬라이딩 윈도우 ───────────────────────────────────────────────
        win_h   = BEV_H // N_WINDOWS
        cx_list = []
        cy_list = []

        for i in range(N_WINDOWS):
            y_end   = BEV_H - i * win_h
            y_start = y_end - win_h

            strip = bev_mask[y_start:y_end, :]
            _, xs = np.where(strip > 0)

            if len(xs) < MIN_PIX:
                cv2.rectangle(vis, (0, y_start), (BEV_W, y_end), (50, 50, 50), 1)
                continue

            x_span = int(xs.max()) - int(xs.min())
            if x_span > MAX_X_SPAN:
                cx_noise = int(xs.mean())
                cv2.rectangle(vis,
                               (max(0, cx_noise - WIN_MARGIN), y_start),
                               (min(BEV_W, cx_noise + WIN_MARGIN), y_end),
                               (0, 0, 180), 1)
                cv2.putText(vis, f'span={x_span}', (4, y_start + 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.32, (0, 0, 255), 1)
                continue

            cx = int(xs.mean())
            cy = (y_start + y_end) // 2
            cx_list.append(cx)
            cy_list.append(cy)

            x0 = max(0,     cx - WIN_MARGIN)
            x1 = min(BEV_W, cx + WIN_MARGIN)
            cv2.rectangle(vis, (x0, y_start), (x1, y_end), (0, 230, 0), 2)
            cv2.circle(vis, (cx, cy), 4, (0, 230, 0), -1)

        # ── 다항식 피팅 + offset 계산 ────────────────────────────────────
        self._offset = 0.0

        if len(cx_list) >= 3:
            coeffs = np.polyfit(cy_list, cx_list, 2)
            poly   = np.poly1d(coeffs)
            a      = coeffs[0]

            # 피팅 곡선 그리기
            y_pts = np.linspace(0, BEV_H, 120).astype(int)
            x_pts = np.clip(poly(y_pts).astype(int), 0, BEV_W - 1)
            cv2.polylines(vis, [np.column_stack([x_pts, y_pts])], False, (255, 60, 0), 2)

            # y=0(BEV 상단, 먼 쪽) 기준으로 offset 계산
            x_at_top   = float(poly(0))           # 클리핑 없이 실제 값 사용
            lat_offset = x_at_top - LANE_REF_X    # 기준선 대비 편차
            curve_corr = K_CURVE * abs(a)         # 곡률 크기로만 증폭 (방향은 lat_offset이 결정)
            self._offset = lat_offset * (1.0 + curve_corr)

            # y=0 마커 (클리핑해서 화면에 표시)
            xt = int(np.clip(x_at_top, 0, BEV_W - 1))
            cv2.circle(vis, (xt, 4), 7, (255, 60, 0), -1)
            cv2.line(vis, (ref_x_clipped, 4), (xt, 4), (255, 60, 0), 2)

            cv2.putText(vis, f'a={a:.5f}  {"CURVE" if abs(a) > 0.002 else "STRAIGHT"}',
                        (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 0, 255), 1)
            cv2.putText(vis, f'x@top={x_at_top:.1f}  ref={LANE_REF_X}',
                        (6, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 0, 255), 1)
            cv2.putText(vis, f'lat={lat_offset:+.1f}  factor={1.0+curve_corr:.2f}',
                        (6, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 0, 255), 1)
            cv2.putText(vis, f'OFFSET={self._offset:+.1f}px',
                        (6, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)

        elif len(cx_list) >= 2:
            coeffs = np.polyfit(cy_list, cx_list, 1)
            poly   = np.poly1d(coeffs)
            y_pts  = np.linspace(min(cy_list), max(cy_list), 60).astype(int)
            x_pts  = np.clip(poly(y_pts).astype(int), 0, BEV_W - 1)
            cv2.polylines(vis, [np.column_stack([x_pts, y_pts])], False, (100, 180, 255), 2)

            x_at_top     = float(poly(0))
            self._offset = x_at_top - LANE_REF_X
            cv2.putText(vis, f'OFFSET={self._offset:+.1f}px (linear)',
                        (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

        else:
            cv2.putText(vis, f'windows={len(cx_list)} / need >=2',
                        (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

        cv2.putText(vis, 'BEV', (BEV_W - 38, BEV_H - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
        return vis


def main(args=None):
    rclpy.init(args=args)
    node = StraightTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
