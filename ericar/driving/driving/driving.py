#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32, Int32MultiArray, Float32, Bool
from sensor_msgs.msg import Image, LaserScan, Imu
from cv_bridge import CvBridge

from driving.lane_detection import LaneDetector
from driving.rubbercone import ConeDriver
from driving.turn_left import LeftTurner

MODE_WAIT, MODE_CONE, MODE_LANE, MODE_LEFT_TURN, \
    MODE_LANE_CHANGE, MODE_FOLLOW, MODE_SIGNAL_WAIT = range(7)

STAGE_LANE_TARGET = 0
STAGE_TURN_TYPE = 1
LANE_CHANGE_DONE_TOL = 0.1

class Driving(Node):
    def __init__(self):
        super().__init__('driving')
        self._bridge = CvBridge()
        self._lane = LaneDetector()
        self._cone = ConeDriver()
        self._turn = LeftTurner()
	self._turn_done_pub = self.create_publisher(Bool, '/driving/turn_done', 10)
	self._turn_reached = False
        self._img_front = None
        self._scan = None
        self._yaw = 0.0
        self._mode = MODE_WAIT
        self._stage = [0, 0]
        self._lane_change_sent = False

        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(Image, '/usb_cam/image_raw/front', self._front_cb, qos_reliable)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', self._imu_cb, qos_profile_sensor_data)
        self.create_subscription(Int32, '/main/mode', self._mode_cb, 10)
        self.create_subscription(Int32MultiArray, '/main/stage', self._stage_cb, 10)

        self._offset_pub = self.create_publisher(Float32, '/driving/offset', 10)
        self._lane_change_pub = self.create_publisher(Bool, '/driving/lane_change_done', 10)

        self.create_timer(1.0 / 30.0, self._tick)
        self.get_logger().info('driving node ready')

    def _front_cb(self, msg):
        self._img_front = self._bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _scan_cb(self, msg):
        self._scan = msg

    def _imu_cb(self, msg):
        self._yaw = self._quat_to_yaw(msg.orientation)

    def _mode_cb(self, msg):
        if msg.data != self._mode:
            self._on_mode_enter(msg.data)
        self._mode = msg.data

    def _stage_cb(self, msg):
        if msg.data:
            self._stage = list(msg.data)

    def _on_mode_enter(self, new_mode):
        if new_mode == MODE_LANE_CHANGE:
            self._lane_change_sent = False
        if new_mode == MODE_LEFT_TURN:
            self._turn.start(self._stage[STAGE_TURN_TYPE], self._yaw)
            self._turn_reached = False
    def _tick(self):
        offset = None
        if self._mode == MODE_CONE:
            offset = self._cone.compute_offset(self._img_front)
        elif self._mode in (MODE_LANE, MODE_FOLLOW):
            lane_target = self._stage[STAGE_LANE_TARGET]
            offset = self._lane.compute_offset(self._img_front, lane_target)
        elif self._mode == MODE_LANE_CHANGE:
            lane_target = self._stage[STAGE_LANE_TARGET]
            offset = self._lane.compute_offset(self._img_front, lane_target)
            self._check_lane_change_done(offset)
        elif self._mode == MODE_LEFT_TURN:
            offset = self._turn.compute_offset(self._yaw)
	    if self._turn.reached(self._yaw) and not self._turn_reached:
                self._turn_done_pub.publish(Bool(data=True))
                self._turn_reached = True
                self.get_logger().info('left turn done')
        if offset is not None:
            self._publish_offset(offset)

    def _check_lane_change_done(self, offset):
        if self._lane_change_sent or offset is None:
            return
        if abs(offset) < LANE_CHANGE_DONE_TOL:
            self._lane_change_pub.publish(Bool(data=True))
            self._lane_change_sent = True
            self.get_logger().info('lane change done')

    def _publish_offset(self, offset):
        self._offset_pub.publish(Float32(data=float(offset)))

    @staticmethod
    def _quat_to_yaw(q):
        import math
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = Driving()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
