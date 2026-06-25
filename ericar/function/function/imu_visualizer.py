#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
import cv2
import numpy as np


class ImuVisualizer(Node):

    def __init__(self):
        super().__init__('imu_visualizer')
        self.yaw = 0.0
        self.create_subscription(Imu, '/imu', self._imu_cb, qos_profile_sensor_data)
        self.create_timer(0.05, self._draw)  # 20 Hz

    def _imu_cb(self, msg):
        q = msg.orientation
        # quaternion → yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def _draw(self):
        SIZE = 420
        cx, cy = SIZE // 2, SIZE // 2
        R = SIZE // 2 - 30

        img = np.zeros((SIZE, SIZE, 3), dtype=np.uint8)

        # outer ring
        cv2.circle(img, (cx, cy), R, (60, 60, 60), 2)

        # tick marks every 30 deg
        for deg in range(0, 360, 30):
            rad = math.radians(deg)
            x0 = int(cx + R * math.sin(rad))
            y0 = int(cy - R * math.cos(rad))
            x1 = int(cx + (R - 10) * math.sin(rad))
            y1 = int(cy - (R - 10) * math.cos(rad))
            cv2.line(img, (x0, y0), (x1, y1), (80, 80, 80), 1)

        # cardinal labels
        for label, deg in [('N', 0), ('E', 90), ('S', 180), ('W', 270)]:
            rad = math.radians(deg)
            lx = int(cx + (R + 16) * math.sin(rad)) - 6
            ly = int(cy - (R + 16) * math.cos(rad)) + 5
            cv2.putText(img, label, (lx, ly),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (140, 140, 140), 1)

        # direction arrow (green)
        # ROS yaw increases counter-clockwise, so negate sin to match compass
        arrow_len = R - 15
        tip_x = int(cx - arrow_len * math.sin(self.yaw))
        tip_y = int(cy - arrow_len * math.cos(self.yaw))
        cv2.arrowedLine(img, (cx, cy), (tip_x, tip_y),
                        (0, 230, 0), 3, tipLength=0.18)

        # tail (opposite direction, thin)
        tail_len = R // 3
        tail_x = int(cx + tail_len * math.sin(self.yaw))
        tail_y = int(cy + tail_len * math.cos(self.yaw))
        cv2.line(img, (cx, cy), (tail_x, tail_y), (0, 100, 0), 1)

        # center dot
        cv2.circle(img, (cx, cy), 5, (0, 200, 200), -1)

        # yaw text
        deg_val = math.degrees(self.yaw)
        cv2.putText(img, f'Yaw: {deg_val:+.1f} deg',
                    (10, SIZE - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (200, 200, 200), 1)

        cv2.imshow('IMU Direction', img)
        cv2.waitKey(1) & 0xFF


def main(args=None):
    rclpy.init(args=args)
    node = ImuVisualizer()
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
