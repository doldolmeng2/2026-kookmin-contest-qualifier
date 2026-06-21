#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import cv2
import numpy as np


class LidarVisualizer(Node):

    def __init__(self):
        super().__init__('lidar_visualizer')
        self.declare_parameter('max_range', 8.0)

        self._ranges = []
        self._angle_min = 0.0
        self._angle_inc = 0.0

        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_timer(0.05, self._draw)  # 20 Hz

    def _scan_cb(self, msg):
        self._ranges = list(msg.ranges)
        self._angle_min = msg.angle_min
        self._angle_inc = msg.angle_increment

    def _draw(self):
        max_range = float(self.get_parameter('max_range').value)
        max_range = max(max_range, 0.5)  # guard against 0

        SIZE = 600
        cx, cy = SIZE // 2, SIZE // 2
        margin = 20
        draw_r = SIZE // 2 - margin          # pixel radius for max_range
        ppm = draw_r / max_range             # pixels per meter

        img = np.zeros((SIZE, SIZE, 3), dtype=np.uint8)

        # ── concentric distance rings ──────────────────────────────────────
        num_rings = 4
        for i in range(1, num_rings + 1):
            ring_m = max_range * i / num_rings
            ring_px = int(ppm * ring_m)
            cv2.circle(img, (cx, cy), ring_px, (45, 45, 45), 1)
            label = f'{ring_m:.1f}m'
            cv2.putText(img, label,
                        (cx + ring_px + 3, cy - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.32, (90, 90, 90), 1)

        # ── cross lines ───────────────────────────────────────────────────
        cv2.line(img, (cx, margin), (cx, SIZE - margin), (35, 35, 35), 1)
        cv2.line(img, (margin, cy), (SIZE - margin, cy), (35, 35, 35), 1)

        # ── scan points ───────────────────────────────────────────────────
        for i, r in enumerate(self._ranges):
            if not math.isfinite(r) or r < 1.2 or r > max_range:
                continue
            angle = self._angle_min + i * self._angle_inc
            # ROS convention: angle increases counter-clockwise, negate sin for correct left/right
            px = int(cx - ppm * r * math.sin(angle))
            py = int(cy - ppm * r * math.cos(angle))
            if 0 <= px < SIZE and 0 <= py < SIZE:
                cv2.circle(img, (px, py), 2, (0, 255, 60), -1)

        # ── robot marker ──────────────────────────────────────────────────
        cv2.circle(img, (cx, cy), 5, (0, 200, 255), -1)
        cv2.arrowedLine(img, (cx, cy), (cx, cy - 18),
                        (0, 200, 255), 2, tipLength=0.4)

        # ── HUD text ──────────────────────────────────────────────────────
        cv2.putText(img, f'max_range: {max_range:.1f} m',
                    (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (180, 180, 180), 1)
        cv2.putText(img, f'points: {len(self._ranges)}',
                    (8, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (180, 180, 180), 1)

        cv2.imshow('LiDAR View', img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = LidarVisualizer()
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
