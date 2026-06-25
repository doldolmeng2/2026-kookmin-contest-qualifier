#!/usr/bin/env python3
"""yaw + 노란 차선 블렌딩 offset 테스트 노드.

목표 yaw 기준으로 주행하고, 특정 y 행의 노란 차선 cx 로 보정한다.
  offset = YAW_WEIGHT * yaw_norm + LANE_WEIGHT * lane_norm
  - yaw_norm  : yaw 오차(rad) / (π/2)  →  90도 오차 = 1.0
  - lane_norm : (lane_cx - 이미지중심) / (w/2)  →  -1 ~ 1
노란 차선 미검출 시 이전 lane_cx 값을 유지한다.

실행:
    ros2 run driving lane_detection_test
"""

import math
import traceback

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Float32
from cv_bridge import CvBridge

# ── 파라미터 ──────────────────────────────────────────────────────────────────
TARGET_YAW_DEG = 90.0   # 목표 yaw (imu_visualizer 기준, 도 단위)

YAW_WEIGHT  = 0.8       # yaw 기여 비율 (0~1)
LANE_WEIGHT = 0.2       # 차선 기여 비율 (0~1)

LANE_Y_RATIO = 0.60     # 차선 검출 y (위에서 60% = 아래에서 40%)
LANE_Y_BAND  = 5        # 검출 행 ± BAND 픽셀

# 노란색 HSV 범위 (lane_detection.py 와 동일)
YELLOW_LOWER = np.array([25, 205,  80])
YELLOW_UPPER = np.array([35, 255, 255])

WIN_NAME = 'yaw_lane_test'
COMPASS_SIZE = 300      # 우측 패널 내 나침반 크기

# offset(-1~1) × OFFSET_SCALE → /driving/offset 발행값
# control.py MODE_LANE ratio=0.30 기준:  100 * 0.30 = 30° (최대 조향)
OFFSET_SCALE = 100.0


class YawLaneTest(Node):

    def __init__(self):
        super().__init__('yaw_lane_test')
        self._bridge = CvBridge()
        self._yaw = 0.0
        self._last_lane_cx = None
        self._frame_count = 0

        self.create_subscription(Imu, '/imu', self._imu_cb, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/usb_cam/image_raw/front',
            self._image_cb, qos_profile_sensor_data)

        self._offset_pub = self.create_publisher(Float32, '/driving/offset', 10)

        self.get_logger().info('yaw_lane_test ready')

    # ── 콜백 ──────────────────────────────────────────────────────────────────
    def _imu_cb(self, msg):
        q = msg.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw = math.atan2(siny, cosy)

    def _image_cb(self, msg):
        self._frame_count += 1
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            h, w = frame.shape[:2]

            lane_cx, lane_fresh = self._detect_yellow_cx(frame, h, w)
            yaw_norm, lane_norm, offset = self._compute_offset(lane_cx, w)

            self._draw(frame, h, w, lane_cx, lane_fresh,
                       yaw_norm, lane_norm, offset)

            self._offset_pub.publish(Float32(data=float(offset * OFFSET_SCALE)))

            if self._frame_count % 30 == 0:
                self.get_logger().info(
                    f'frame={self._frame_count}  offset={offset * OFFSET_SCALE:+.2f}')
        except Exception as e:
            self.get_logger().error(f'image_cb error: {e}\n{traceback.format_exc()}')

    # ── 노란 차선 검출 ────────────────────────────────────────────────────────
    def _detect_yellow_cx(self, frame, h, w):
        """지정 y 행 대역에서 노란 픽셀 평균 x 반환. 미검출 시 이전 값."""
        lane_y = int(h * LANE_Y_RATIO)
        y0 = max(0, lane_y - LANE_Y_BAND)
        y1 = min(h, lane_y + LANE_Y_BAND)

        roi = frame[y0:y1, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, YELLOW_LOWER, YELLOW_UPPER)

        _, xs = np.where(mask > 0)
        if len(xs) > 0:
            self._last_lane_cx = int(xs.mean())
            return self._last_lane_cx, True

        return self._last_lane_cx, False   # 이전 값 유지

    # ── offset 계산 ───────────────────────────────────────────────────────────
    def _compute_offset(self, lane_cx, w):
        # yaw 오차 → 정규화 (90° 오차 = 1.0)
        target_rad = math.radians(TARGET_YAW_DEG)
        yaw_err = self._yaw - target_rad
        yaw_err = (yaw_err + math.pi) % (2 * math.pi) - math.pi  # -π ~ π 클램프
        yaw_norm = yaw_err / (math.pi / 2)

        # 차선 오차 → 정규화 (-1 ~ 1)
        if lane_cx is not None:
            lane_norm = (lane_cx - w / 2) / (w / 2)
        else:
            lane_norm = 0.0

        offset = YAW_WEIGHT * yaw_norm + LANE_WEIGHT * lane_norm
        return yaw_norm, lane_norm, offset

    # ── 시각화 ────────────────────────────────────────────────────────────────
    def _draw(self, frame, h, w, lane_cx, lane_fresh,
              yaw_norm, lane_norm, offset):

        lane_y = int(h * LANE_Y_RATIO)
        y0 = max(0, lane_y - LANE_Y_BAND)
        y1 = min(h, lane_y + LANE_Y_BAND)

        # ── 왼쪽: 카메라 오버레이 ────────────────────────────────────────
        vis = frame.copy()

        # 노란 검출 띠
        roi_hsv = cv2.cvtColor(frame[y0:y1, :], cv2.COLOR_BGR2HSV)
        mask_roi = cv2.inRange(roi_hsv, YELLOW_LOWER, YELLOW_UPPER)
        full_mask = np.zeros((h, w), dtype=np.uint8)
        full_mask[y0:y1] = mask_roi
        vis[full_mask > 0] = (0, 220, 255)   # 노랑 강조 (BGR: 노란색)

        band_color = (0, 220, 80) if lane_fresh else (80, 80, 80)
        cv2.rectangle(vis, (0, y0), (w - 1, y1), band_color, 1)
        cv2.putText(vis,
                    f'lane_y={lane_y}px  (bottom {int((1-LANE_Y_RATIO)*100)}%)',
                    (6, y0 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, band_color, 1)

        # 이미지 중심선
        cv2.line(vis, (w // 2, 0), (w // 2, h), (100, 100, 100), 1)

        # 차선 cx 마커
        if lane_cx is not None:
            col = (0, 255, 0) if lane_fresh else (120, 120, 255)
            cv2.circle(vis, (lane_cx, lane_y), 8, col, -1)
            tag = 'FRESH' if lane_fresh else 'PREV'
            cv2.putText(vis, f'{tag} cx={lane_cx}',
                        (lane_cx + 10, lane_y + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)

        # ── 오른쪽: 나침반 + 수치 패널 ──────────────────────────────────
        panel = np.zeros((h, COMPASS_SIZE, 3), dtype=np.uint8)
        cx_p = COMPASS_SIZE // 2
        cy_p = COMPASS_SIZE // 2
        R = COMPASS_SIZE // 2 - 22

        # 외곽 링 + 눈금
        cv2.circle(panel, (cx_p, cy_p), R, (60, 60, 60), 2)
        for deg in range(0, 360, 30):
            r = math.radians(deg)
            x0 = int(cx_p + R * math.sin(r))
            y00 = int(cy_p - R * math.cos(r))
            x1 = int(cx_p + (R - 8) * math.sin(r))
            y11 = int(cy_p - (R - 8) * math.cos(r))
            cv2.line(panel, (x0, y00), (x1, y11), (70, 70, 70), 1)

        # 방위 레이블 (imu_visualizer 와 동일)
        for label, deg in [('N', 0), ('E', 90), ('S', 180), ('W', 270)]:
            r = math.radians(deg)
            lx = int(cx_p + (R + 14) * math.sin(r)) - 5
            ly = int(cy_p - (R + 14) * math.cos(r)) + 5
            cv2.putText(panel, label, (lx, ly),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (130, 130, 130), 1)

        # 목표 yaw 방향 (파란 화살표)
        t_r = math.radians(TARGET_YAW_DEG)
        tx = int(cx_p - R * math.sin(t_r))
        ty = int(cy_p - R * math.cos(t_r))
        cv2.arrowedLine(panel, (cx_p, cy_p), (tx, ty),
                        (255, 100, 0), 2, tipLength=0.15)

        # 현재 yaw 방향 (초록 화살표)
        alen = R - 12
        ax = int(cx_p - alen * math.sin(self._yaw))
        ay = int(cy_p - alen * math.cos(self._yaw))
        cv2.arrowedLine(panel, (cx_p, cy_p), (ax, ay),
                        (0, 230, 0), 3, tipLength=0.18)
        tail_x = int(cx_p + (R // 3) * math.sin(self._yaw))
        tail_y = int(cy_p + (R // 3) * math.cos(self._yaw))
        cv2.line(panel, (cx_p, cy_p), (tail_x, tail_y), (0, 90, 0), 1)
        cv2.circle(panel, (cx_p, cy_p), 5, (0, 200, 200), -1)

        # ── 수치 텍스트 (나침반 아래) ────────────────────────────────────
        yaw_deg = math.degrees(self._yaw)
        yaw_err_deg = math.degrees(self._yaw - math.radians(TARGET_YAW_DEG))
        yaw_err_deg = (yaw_err_deg + 180) % 360 - 180

        if lane_cx is not None:
            lane_err_px = lane_cx - w // 2
            lane_str = f'{lane_err_px:+d}px  norm={lane_norm:+.3f}'
            if not lane_fresh:
                lane_str += ' [PREV]'
        else:
            lane_str = 'not detected'

        lane_col = (0, 220, 80) if lane_fresh else (120, 120, 200)
        info_lines = [
            (f'Yaw   : {yaw_deg:+.1f} deg',                     (200, 200, 200)),
            (f'Target: {TARGET_YAW_DEG:+.1f} deg',              (255, 120,  60)),
            (f'yaw_err: {yaw_err_deg:+.1f}d  norm={yaw_norm:+.3f}', (0, 220, 0)),
            None,
            (f'lane : {lane_str}',                               lane_col),
            (f'lane_norm: {lane_norm:+.3f}',                     lane_col),
            None,
            (f'OFFSET: {offset:+.4f}',                          (0, 220, 255)),
            (f'={YAW_WEIGHT}*{yaw_norm:+.3f}+{LANE_WEIGHT}*{lane_norm:+.3f}',
             (110, 110, 110)),
        ]

        ty_t = COMPASS_SIZE + 14
        for item in info_lines:
            if item is None:
                ty_t += 6          # 구분 여백 (빈 줄 대신 작은 간격)
            else:
                text, color = item
                cv2.putText(panel, text, (6, ty_t),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1)
                ty_t += 17

        # ── offset 게이지 바 (패널 맨 아래) ─────────────────────────────
        bar_margin = 12
        bar_h_px = 12
        bar_y_center = h - bar_margin - bar_h_px // 2
        bar_x0 = bar_margin
        bar_x1 = COMPASS_SIZE - bar_margin
        bar_mid = (bar_x0 + bar_x1) // 2
        bar_half = (bar_x1 - bar_x0) // 2

        cv2.rectangle(panel,
                      (bar_x0, bar_y_center - bar_h_px // 2),
                      (bar_x1, bar_y_center + bar_h_px // 2),
                      (40, 40, 40), -1)

        clipped = max(-1.0, min(1.0, offset))
        fill_end = bar_mid + int(clipped * bar_half)
        bar_col = (0, 210, 255) if abs(offset) < 0.3 else (0, 60, 255)
        if fill_end != bar_mid:
            x0f = min(bar_mid, fill_end)
            x1f = max(bar_mid, fill_end)
            cv2.rectangle(panel,
                          (x0f, bar_y_center - bar_h_px // 2 + 1),
                          (x1f, bar_y_center + bar_h_px // 2 - 1),
                          bar_col, -1)
        cv2.line(panel,
                 (bar_mid, bar_y_center - bar_h_px // 2 - 3),
                 (bar_mid, bar_y_center + bar_h_px // 2 + 3),
                 (180, 180, 180), 1)
        cv2.putText(panel, f'offset={offset:+.3f}',
                    (bar_x0, bar_y_center - bar_h_px // 2 - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, bar_col, 1)

        # ── 합치기 ───────────────────────────────────────────────────────
        combined = np.hstack([vis, panel])
        cv2.imshow(WIN_NAME, combined)
        cv2.waitKey(1) & 0xFF


def main(args=None):
    rclpy.init(args=args)
    node = YawLaneTest()
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
