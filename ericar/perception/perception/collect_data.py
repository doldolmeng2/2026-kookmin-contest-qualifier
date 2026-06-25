#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""YOLO 학습용 카메라 프레임 수집 노드.

카메라 토픽을 받아 일정 간격으로 .jpg 로 저장한다.
실시간 시뮬(endpoint)에서도, rosbag 재생에서도 동일하게 동작한다.

실행 예:
  # 이미 찍어둔 bag에서 프레임 뽑기 (추천)
  ros2 bag play schoolzone --loop          # 다른 터미널
  ros2 run perception collect_data --ros-args -p out_dir:=/home/gill/sz_dataset -p interval:=0.2

  # 실시간 시뮬에서 수집 (endpoint + 시뮬 + 주행 필요)
  ros2 run perception collect_data

저장 위치 기본값: ~/dataset/  (Windows 탐색기: \\wsl.localhost\Ubuntu-22.04\home\gill\dataset)
"""

import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

import cv2
import numpy as np


# 시뮬/-bag 둘 다 RELIABLE 로 발행되므로 맞춰서 구독 (안 그러면 큰 이미지 드롭)
_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')

        self.declare_parameter('topic', '/usb_cam/image_raw/front')  # 수집할 카메라
        self.declare_parameter('interval', 0.3)                      # 저장 간격(초)
        self.declare_parameter('out_dir', os.path.expanduser('~/dataset'))
        self.topic = self.get_parameter('topic').value
        self.interval = float(self.get_parameter('interval').value)
        self.out_dir = self.get_parameter('out_dir').value
        os.makedirs(self.out_dir, exist_ok=True)

        self.last_save = 0.0
        self.count = 0

        self.create_subscription(Image, self.topic, self.cam_cb, _QOS)
        self.get_logger().info(
            f'수집 시작: {self.topic} → {self.out_dir} (간격 {self.interval}s). Ctrl+C로 종료')

    def cam_cb(self, msg):
        now = time.time()
        if now - self.last_save < self.interval:
            return
        self.last_save = now
        img = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape((msg.height, msg.width, 3))
        if msg.encoding == 'rgb8':
            img = img[:, :, ::-1]
        fn = os.path.join(self.out_dir, f'frame_{int(now * 1000)}.jpg')
        cv2.imwrite(fn, img)
        self.count += 1
        if self.count % 20 == 0:
            self.get_logger().info(f'{self.count} 장 저장됨 ({self.out_dir})')


def main(args=None):
    rclpy.init(args=args)
    node = DataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'총 {node.count} 장 저장 후 종료')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
