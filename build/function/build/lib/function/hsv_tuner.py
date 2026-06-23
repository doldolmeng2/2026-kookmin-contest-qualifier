#!/usr/bin/env python3
"""HSV 튜너 노드.

카메라 이미지를 구독하고 트랙바로 HSV 최소/최대값을 조절하면
해당 마스크 결과를 실시간으로 보여준다.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


WIN_ORIGINAL = 'HSV Tuner - Original'
WIN_MASK     = 'HSV Tuner - Mask'
WIN_RESULT   = 'HSV Tuner - Result'


class HsvTuner(Node):

    def __init__(self):
        super().__init__('hsv_tuner')
        self._bridge = CvBridge()
        self._img = None

        self._setup_windows()

        self.create_subscription(
            Image, '/usb_cam/image_raw/front',
            self._image_cb, qos_profile_sensor_data)

        self.create_timer(1.0 / 30.0, self._tick)
        self.get_logger().info('HSV tuner ready — adjust trackbars and watch the mask')

    def _setup_windows(self):
        cv2.namedWindow(WIN_ORIGINAL, cv2.WINDOW_NORMAL)
        cv2.namedWindow(WIN_MASK,     cv2.WINDOW_NORMAL)
        cv2.namedWindow(WIN_RESULT,   cv2.WINDOW_NORMAL)

        def nothing(_): pass

        # H: 0~179, S: 0~255, V: 0~255
        cv2.createTrackbar('H min', WIN_MASK,  0,   179, nothing)
        cv2.createTrackbar('H max', WIN_MASK,  179, 179, nothing)
        cv2.createTrackbar('S min', WIN_MASK,  0,   255, nothing)
        cv2.createTrackbar('S max', WIN_MASK,  255, 255, nothing)
        cv2.createTrackbar('V min', WIN_MASK,  0,   255, nothing)
        cv2.createTrackbar('V max', WIN_MASK,  255, 255, nothing)

    def _image_cb(self, msg):
        self._img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _tick(self):
        if self._img is None:
            return

        h_min = cv2.getTrackbarPos('H min', WIN_MASK)
        h_max = cv2.getTrackbarPos('H max', WIN_MASK)
        s_min = cv2.getTrackbarPos('S min', WIN_MASK)
        s_max = cv2.getTrackbarPos('S max', WIN_MASK)
        v_min = cv2.getTrackbarPos('V min', WIN_MASK)
        v_max = cv2.getTrackbarPos('V max', WIN_MASK)

        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])

        hsv  = cv2.cvtColor(self._img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        result = cv2.bitwise_and(self._img, self._img, mask=mask)

        pixel_count = int(np.count_nonzero(mask))
        info = f'H:[{h_min},{h_max}] S:[{s_min},{s_max}] V:[{v_min},{v_max}]  pixels={pixel_count}'
        vis = self._img.copy()
        cv2.putText(vis, info, (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        cv2.imshow(WIN_ORIGINAL, vis)
        cv2.imshow(WIN_MASK,     mask)
        cv2.imshow(WIN_RESULT,   result)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = HsvTuner()
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
