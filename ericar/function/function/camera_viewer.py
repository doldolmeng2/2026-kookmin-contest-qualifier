#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraViewer(Node):

    def __init__(self):
        super().__init__('camera_viewer')
        self._bridge = CvBridge()
        self.create_subscription(
            Image, '/usb_cam/image_raw/front',
            self._image_cb, qos_profile_sensor_data)
        self.get_logger().info('Camera viewer ready — waiting for /usb_cam/image_raw/front')

    def _image_cb(self, msg):
        img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imshow('Front Camera', img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()
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
