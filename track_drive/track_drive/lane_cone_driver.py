#!/usr/bin/env python3

import math
import sys
import threading
import time
import tty
import termios
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from xycar_msgs.msg import XycarMotor


Point = Tuple[float, float]


class LaneConeDriver(Node):
    def __init__(self):
        super().__init__("lane_cone_driver")

        self.declare_parameter("mode", "lane")
        self.declare_parameter("lane", 1)

        self.declare_parameter("target_x_use_ratio", False)
        self.declare_parameter("lane1_target_x", 400)
        self.declare_parameter("lane2_target_x", 180)
        self.declare_parameter("lane1_target_x_ratio", 0.75)
        self.declare_parameter("lane2_target_x_ratio", 0.28125)

        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("resize_image_enable", False)
        self.declare_parameter("debug_overlay_on_image", True)

        self.declare_parameter("roi_y_ratio", 0.42)
        self.declare_parameter("roi_left_ratio", 0.0)
        self.declare_parameter("roi_right_ratio", 1.0)

        self.declare_parameter("lane_sample_y_use_ratio", True)
        self.declare_parameter("lane_sample_y", 320)
        self.declare_parameter("lane_sample_y_ratio", 0.667)

        self.declare_parameter("roi_trapezoid_enable", True)

        self.declare_parameter("lane_band_half_height", 30)
        self.declare_parameter("max_prediction_frames", 10)

        self.declare_parameter("speed_lane", 8.0)
        self.declare_parameter("speed_cone", 8.0)

        self.declare_parameter("max_angle", 180.0)

        # 기존 P 제어용. lane_pid_enable=False일 때 사용
        self.declare_parameter("lane_kp", 0.35)

        # PID 제어 파라미터
        self.declare_parameter("lane_pid_enable", True)
        self.declare_parameter("lane_pid_kp", 0.35)
        self.declare_parameter("lane_pid_ki", 0.00)
        self.declare_parameter("lane_pid_kd", 0.05)
        self.declare_parameter("lane_pid_integral_limit", 300.0)
        self.declare_parameter("lane_pid_derivative_alpha", 0.70)

        self.declare_parameter("cone_kp", 28.0)
        self.declare_parameter("steer_sign", 3.0)

        self.declare_parameter("cone_enable_min_points", 6)
        self.declare_parameter("cone_range_min", 0.18)
        self.declare_parameter("cone_range_max", 5.0)
        self.declare_parameter("cone_front_angle_deg", 70.0)

        self.declare_parameter("debug_view", True)

        self.declare_parameter("control_period", 0.02)
        self.declare_parameter("yellow_max_ratio", 0.05)

        # ============================================================
        # lane별 ROI 설정
        # x ratio는 0.0~1.0 밖 값도 허용함.
        # 예: -0.5 = 화면 왼쪽 바깥, 1.5 = 화면 오른쪽 바깥
        # ============================================================

        # 1차선 주행 ROI
        self.declare_parameter("lane1_roi_trap_top_y_ratio", 0.33)
        self.declare_parameter("lane1_roi_trap_bottom_y_ratio", 1.00)
        self.declare_parameter("lane1_roi_trap_top_left_ratio", 0.45)
        self.declare_parameter("lane1_roi_trap_top_right_ratio", 0.73)
        self.declare_parameter("lane1_roi_trap_bottom_left_ratio", 0.25)
        self.declare_parameter("lane1_roi_trap_bottom_right_ratio", 1.30)

        # 2차선 주행 ROI
        self.declare_parameter("lane2_roi_trap_top_y_ratio", 0.12)
        self.declare_parameter("lane2_roi_trap_bottom_y_ratio", 1.00)
        self.declare_parameter("lane2_roi_trap_top_left_ratio", 0.20)
        self.declare_parameter("lane2_roi_trap_top_right_ratio", 0.40)
        self.declare_parameter("lane2_roi_trap_bottom_left_ratio", -0.50)
        self.declare_parameter("lane2_roi_trap_bottom_right_ratio", 0.50)

        # lane별 제거 영역
        self.declare_parameter("lane1_ignore_x_start_ratio", 0.00)
        self.declare_parameter("lane1_ignore_x_end_ratio", 0.25)

        self.declare_parameter("lane2_ignore_x_start_ratio", 0.75)
        self.declare_parameter("lane2_ignore_x_end_ratio", 1.00)

        self.bridge = CvBridge()
        self.image: Optional[np.ndarray] = None
        self.scan_msg: Optional[LaserScan] = None

        self.last_lane_angle = 0.0
        self.last_cone_angle: Optional[float] = None
        self.last_valid_angle = 0.0

        self.mode = str(self.get_parameter("mode").value).lower()
        self.lane = int(self.get_parameter("lane").value)
        self.running = True

        self.last_status_time = 0.0
        self.last_publish_angle = 0.0
        self.last_publish_speed = 0.0

        self.last_yellow_x: Optional[float] = None
        self.last_lane_target_x: Optional[float] = None
        self.last_lane_error: Optional[float] = None

        self.prev_detected_yellow_x: Optional[float] = None
        self.last_yellow_dx = 0.0
        self.missing_lane_frames = 0
        self.last_lane_x_source = "none"
        self.last_yellow_ratio = 0.0

        # PID 상태 변수
        self.lane_pid_integral = 0.0
        self.lane_pid_prev_error: Optional[float] = None
        self.lane_pid_prev_time: Optional[float] = None
        self.lane_pid_prev_derivative = 0.0

        self.motor_pub = self.create_publisher(XycarMotor, "/xycar_motor", 10)

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(
            Image,
            "/usb_cam/image_raw/front",
            self.image_callback,
            sensor_qos,
        )

        self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            sensor_qos,
        )

        control_period = float(self.get_parameter("control_period").value)
        self.timer = self.create_timer(control_period, self.control_loop)

        self.keyboard_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.keyboard_thread.start()

        self.get_logger().info(
            f"lane_cone_driver started: /usb_cam/image_raw/front + /scan -> /xycar_motor, "
            f"control_period={control_period:.3f}s, hz={1.0 / control_period:.1f}"
        )

        self.print_help()
        self.print_status(force=True)

    def image_callback(self, msg: Image):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def scan_callback(self, msg: LaserScan):
        self.scan_msg = msg

    def control_loop(self):
        lane_angle = self.compute_lane_angle()
        cone_angle = self.compute_cone_angle()

        angle = self.last_valid_angle
        speed = 0.0

        if self.mode == "stop":
            angle = 0.0
            speed = 0.0
            self.reset_lane_pid()

        elif self.mode == "cone":
            speed = float(self.get_parameter("speed_cone").value)

            if cone_angle is not None:
                angle = cone_angle
            else:
                angle = self.last_valid_angle

        elif self.mode == "lane":
            speed = float(self.get_parameter("speed_lane").value)

            if lane_angle is not None:
                angle = lane_angle
            else:
                angle = self.last_valid_angle

        elif self.mode == "auto":
            if cone_angle is not None:
                angle = cone_angle
                speed = float(self.get_parameter("speed_cone").value)

            elif lane_angle is not None:
                angle = lane_angle
                speed = float(self.get_parameter("speed_lane").value)

            else:
                angle = self.last_valid_angle
                speed = float(self.get_parameter("speed_lane").value)

        else:
            angle = self.last_valid_angle
            speed = 0.0

        angle = self.clamp_angle(angle)

        if self.mode != "stop":
            self.last_valid_angle = angle

        self.publish_motor(angle, speed)
        self.print_status()

    def get_working_frame(self) -> Tuple[np.ndarray, int, int]:
        resize_image_enable = bool(self.get_parameter("resize_image_enable").value)

        if resize_image_enable:
            width = int(self.get_parameter("image_width").value)
            height = int(self.get_parameter("image_height").value)
            frame = cv2.resize(self.image, (width, height))
        else:
            frame = self.image.copy()
            height, width = frame.shape[:2]

        return frame, width, height

    def get_target_x(self, width: int) -> float:
        use_ratio = bool(self.get_parameter("target_x_use_ratio").value)

        if use_ratio:
            if self.lane == 1:
                ratio = float(self.get_parameter("lane1_target_x_ratio").value)
            else:
                ratio = float(self.get_parameter("lane2_target_x_ratio").value)

            ratio = max(0.0, min(1.0, ratio))
            return float(width) * ratio

        if self.lane == 1:
            return float(self.get_parameter("lane1_target_x").value)

        return float(self.get_parameter("lane2_target_x").value)

    def get_sample_y(self, height: int) -> int:
        use_ratio = bool(self.get_parameter("lane_sample_y_use_ratio").value)

        if use_ratio:
            ratio = float(self.get_parameter("lane_sample_y_ratio").value)
            ratio = max(0.0, min(1.0, ratio))
            return int(height * ratio)

        return int(self.get_parameter("lane_sample_y").value)

    def compute_lane_angle(self) -> Optional[float]:
        if self.image is None:
            self.last_lane_x_source = "no_image"
            self.reset_lane_pid()
            return None

        frame, width, height = self.get_working_frame()

        roi_y = int(height * float(self.get_parameter("roi_y_ratio").value))
        sample_y = self.get_sample_y(height)
        band_half_height = int(self.get_parameter("lane_band_half_height").value)

        roi_y = max(0, min(height - 1, roi_y))
        sample_y = max(0, min(height - 1, sample_y))

        band_top = max(roi_y, sample_y - band_half_height)
        band_bottom = min(height, sample_y + band_half_height + 1)

        if band_bottom <= band_top:
            band_top = max(roi_y, min(height - 1, sample_y))
            band_bottom = min(height, band_top + 1)

        roi_left = int(width * float(self.get_parameter("roi_left_ratio").value))
        roi_right = int(width * float(self.get_parameter("roi_right_ratio").value))

        roi_left = max(0, min(width - 1, roi_left))
        roi_right = max(roi_left + 1, min(width, roi_right))

        roi = frame[roi_y:, roi_left:roi_right]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        hls = cv2.cvtColor(roi, cv2.COLOR_BGR2HLS)

        hsv_mask = cv2.inRange(
            hsv,
            np.array([15, 70, 70]),
            np.array([40, 255, 255]),
        )

        hls_mask = cv2.inRange(
            hls,
            np.array([15, 45, 60]),
            np.array([45, 255, 255]),
        )

        mask = cv2.bitwise_and(hsv_mask, hls_mask)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        mask = self.apply_trapezoid_roi_mask(mask)
        mask = self.apply_lane_ignore_mask(mask)

        yellow_ratio = float(np.count_nonzero(mask)) / float(mask.size)
        self.last_yellow_ratio = yellow_ratio

        target_x = self.get_target_x(width)
        self.last_lane_target_x = target_x

        debug = self.make_debug_image(
            frame=frame,
            roi_y=roi_y,
            roi_left=roi_left,
            roi_right=roi_right,
            roi=roi,
            mask=mask,
        )

        yellow_max_ratio = float(self.get_parameter("yellow_max_ratio").value)

        if yellow_ratio > yellow_max_ratio:
            self.last_lane_x_source = "too_many_yellow"
            self.last_yellow_x = None
            self.last_lane_error = None
            self.reset_lane_pid()

            self.show_lane_debug(
                debug=debug,
                roi_left=roi_left,
                roi_right=roi_right,
                roi_y=roi_y,
                height=height,
                width=width,
                sample_y=sample_y,
                band_top=band_top,
                band_bottom=band_bottom,
                mask=mask,
                target_x=target_x,
                yellow_x=None,
                yellow_ratio=yellow_ratio,
                status_text=f"IGNORE: too many yellow {yellow_ratio:.3f}",
            )

            return None

        band_mask = mask[band_top - roi_y:band_bottom - roi_y, :]
        ys, xs = np.where(band_mask > 0)

        yellow_x = self.estimate_lane_x(xs, roi_left, width)

        if yellow_x is None:
            self.last_yellow_x = None
            self.last_lane_error = None
            self.reset_lane_pid()

            self.show_lane_debug(
                debug=debug,
                roi_left=roi_left,
                roi_right=roi_right,
                roi_y=roi_y,
                height=height,
                width=width,
                sample_y=sample_y,
                band_top=band_top,
                band_bottom=band_bottom,
                mask=mask,
                target_x=target_x,
                yellow_x=None,
                yellow_ratio=yellow_ratio,
                status_text=f"MISSING: source={self.last_lane_x_source}",
            )

            return None

        error = yellow_x - target_x

        self.last_yellow_x = yellow_x
        self.last_lane_error = error

        if bool(self.get_parameter("lane_pid_enable").value):
            self.last_lane_angle = self.compute_lane_pid_angle(error)
        else:
            angle = (
                float(self.get_parameter("steer_sign").value)
                * float(self.get_parameter("lane_kp").value)
                * error
            )
            self.last_lane_angle = self.clamp_angle(angle)

        self.show_lane_debug(
            debug=debug,
            roi_left=roi_left,
            roi_right=roi_right,
            roi_y=roi_y,
            height=height,
            width=width,
            sample_y=sample_y,
            band_top=band_top,
            band_bottom=band_bottom,
            mask=mask,
            target_x=target_x,
            yellow_x=yellow_x,
            yellow_ratio=yellow_ratio,
            status_text=f"DETECT: mean_x={yellow_x:.1f}, error={error:.1f}, angle={self.last_lane_angle:.1f}",
        )

        return self.last_lane_angle

    def reset_lane_pid(self):
        self.lane_pid_integral = 0.0
        self.lane_pid_prev_error = None
        self.lane_pid_prev_time = None
        self.lane_pid_prev_derivative = 0.0

    def compute_lane_pid_angle(self, error: float) -> float:
        now = time.monotonic()

        if self.lane_pid_prev_time is None:
            dt = float(self.get_parameter("control_period").value)
        else:
            dt = now - self.lane_pid_prev_time

        if dt <= 1e-4:
            dt = float(self.get_parameter("control_period").value)

        kp = float(self.get_parameter("lane_pid_kp").value)
        ki = float(self.get_parameter("lane_pid_ki").value)
        kd = float(self.get_parameter("lane_pid_kd").value)

        integral_limit = float(self.get_parameter("lane_pid_integral_limit").value)
        derivative_alpha = float(self.get_parameter("lane_pid_derivative_alpha").value)
        derivative_alpha = max(0.0, min(0.99, derivative_alpha))

        p_term = kp * error

        self.lane_pid_integral += error * dt
        self.lane_pid_integral = max(
            -integral_limit,
            min(integral_limit, self.lane_pid_integral),
        )
        i_term = ki * self.lane_pid_integral

        if self.lane_pid_prev_error is None:
            raw_derivative = 0.0
        else:
            raw_derivative = (error - self.lane_pid_prev_error) / dt

        derivative = (
            derivative_alpha * self.lane_pid_prev_derivative
            + (1.0 - derivative_alpha) * raw_derivative
        )
        d_term = kd * derivative

        self.lane_pid_prev_error = error
        self.lane_pid_prev_time = now
        self.lane_pid_prev_derivative = derivative

        steer_sign = float(self.get_parameter("steer_sign").value)

        angle = steer_sign * (p_term + i_term + d_term)
        return self.clamp_angle(angle)

    def show_lane_debug(
        self,
        debug: np.ndarray,
        roi_left: int,
        roi_right: int,
        roi_y: int,
        height: int,
        width: int,
        sample_y: int,
        band_top: int,
        band_bottom: int,
        mask: np.ndarray,
        target_x: Optional[float],
        yellow_x: Optional[float],
        yellow_ratio: float,
        status_text: str,
    ):
        if not bool(self.get_parameter("debug_view").value):
            return

        self.draw_roi_debug(debug, roi_left, roi_right, roi_y, height)
        self.draw_trapezoid_debug(debug, roi_left, roi_y, mask)

        cv2.rectangle(
            debug,
            (roi_left, band_top),
            (roi_right - 1, band_bottom - 1),
            (0, 120, 0),
            1,
        )

        cv2.line(
            debug,
            (0, sample_y),
            (width - 1, sample_y),
            (80, 80, 80),
            1,
        )

        if target_x is not None:
            cv2.line(
                debug,
                (int(target_x), roi_y),
                (int(target_x), height),
                (255, 0, 0),
                2,
            )

        if yellow_x is not None:
            color = (0, 255, 255) if self.last_lane_x_source == "detect" else (0, 128, 255)
            cv2.circle(
                debug,
                (int(yellow_x), sample_y),
                6,
                color,
                -1,
            )

        if yellow_x is None and self.prev_detected_yellow_x is not None:
            cv2.circle(
                debug,
                (int(self.prev_detected_yellow_x), sample_y),
                6,
                (0, 128, 255),
                -1,
            )

        cv2.putText(
            debug,
            status_text,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.70,
            (0, 0, 255) if yellow_x is None else (0, 255, 255),
            2,
        )

        cv2.putText(
            debug,
            f"lane={self.lane}, source={self.last_lane_x_source}, missing={self.missing_lane_frames}, yellow_ratio={yellow_ratio:.3f}",
            (20, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (0, 255, 255),
            2,
        )

        cv2.putText(
            debug,
            f"last_valid_angle={self.last_valid_angle:.1f}, pid={bool(self.get_parameter('lane_pid_enable').value)}",
            (20, 105),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (0, 255, 255),
            2,
        )

        cv2.imshow("yellow_lane_mask", debug)
        cv2.waitKey(1)

    def make_debug_image(
        self,
        frame: np.ndarray,
        roi_y: int,
        roi_left: int,
        roi_right: int,
        roi: np.ndarray,
        mask: np.ndarray,
    ) -> np.ndarray:
        debug_overlay_on_image = bool(self.get_parameter("debug_overlay_on_image").value)

        if debug_overlay_on_image:
            debug = frame.copy()

            yellow_overlay = np.zeros_like(roi)
            yellow_overlay[:, :] = (0, 255, 255)

            yellow_area = cv2.bitwise_and(yellow_overlay, yellow_overlay, mask=mask)

            roi_debug = debug[roi_y:, roi_left:roi_right]
            blended = cv2.addWeighted(roi_debug, 0.65, yellow_area, 0.35, 0.0)
            debug[roi_y:, roi_left:roi_right] = blended

        else:
            debug = np.zeros_like(frame)
            yellow_only_roi = cv2.bitwise_and(roi, roi, mask=mask)
            debug[roi_y:, roi_left:roi_right] = yellow_only_roi

        return debug

    def draw_roi_debug(
        self,
        debug: np.ndarray,
        roi_left: int,
        roi_right: int,
        roi_y: int,
        height: int,
    ):
        if not bool(self.get_parameter("debug_view").value):
            return

        cv2.rectangle(
            debug,
            (roi_left, roi_y),
            (roi_right - 1, height - 1),
            (255, 255, 0),
            1,
        )

    def get_lane_trapezoid_params(self):
        if self.lane == 1:
            prefix = "lane1"
        else:
            prefix = "lane2"

        top_y_ratio = float(self.get_parameter(f"{prefix}_roi_trap_top_y_ratio").value)
        bottom_y_ratio = float(self.get_parameter(f"{prefix}_roi_trap_bottom_y_ratio").value)

        top_left_ratio = float(self.get_parameter(f"{prefix}_roi_trap_top_left_ratio").value)
        top_right_ratio = float(self.get_parameter(f"{prefix}_roi_trap_top_right_ratio").value)

        bottom_left_ratio = float(self.get_parameter(f"{prefix}_roi_trap_bottom_left_ratio").value)
        bottom_right_ratio = float(self.get_parameter(f"{prefix}_roi_trap_bottom_right_ratio").value)

        return (
            top_y_ratio,
            bottom_y_ratio,
            top_left_ratio,
            top_right_ratio,
            bottom_left_ratio,
            bottom_right_ratio,
        )

    def get_trapezoid_roi_points(self, roi_width: int, roi_height: int) -> np.ndarray:
        (
            top_y_ratio,
            bottom_y_ratio,
            top_left_ratio,
            top_right_ratio,
            bottom_left_ratio,
            bottom_right_ratio,
        ) = self.get_lane_trapezoid_params()

        top_y_ratio = max(0.0, min(1.0, top_y_ratio))
        bottom_y_ratio = max(0.0, min(1.0, bottom_y_ratio))

        if bottom_y_ratio < top_y_ratio:
            top_y_ratio, bottom_y_ratio = bottom_y_ratio, top_y_ratio

        # x ratio는 clamp하지 않음. -0.5, 1.5 같은 화면 밖 좌표 허용.
        if top_right_ratio < top_left_ratio:
            top_left_ratio, top_right_ratio = top_right_ratio, top_left_ratio

        if bottom_right_ratio < bottom_left_ratio:
            bottom_left_ratio, bottom_right_ratio = bottom_right_ratio, bottom_left_ratio

        top_y = int(roi_height * top_y_ratio)
        bottom_y = int(roi_height * bottom_y_ratio)

        top_left_x = int(roi_width * top_left_ratio)
        top_right_x = int(roi_width * top_right_ratio)
        bottom_left_x = int(roi_width * bottom_left_ratio)
        bottom_right_x = int(roi_width * bottom_right_ratio)

        top_y = max(0, min(roi_height - 1, top_y))
        bottom_y = max(0, min(roi_height - 1, bottom_y))

        points = np.array(
            [
                [top_left_x, top_y],
                [top_right_x, top_y],
                [bottom_right_x, bottom_y],
                [bottom_left_x, bottom_y],
            ],
            dtype=np.int32,
        )

        return points

    def apply_trapezoid_roi_mask(self, mask: np.ndarray) -> np.ndarray:
        if not bool(self.get_parameter("roi_trapezoid_enable").value):
            return mask

        roi_height, roi_width = mask.shape[:2]
        trapezoid_points = self.get_trapezoid_roi_points(roi_width, roi_height)

        roi_mask = np.zeros_like(mask)
        cv2.fillPoly(roi_mask, [trapezoid_points], 255)

        masked = cv2.bitwise_and(mask, roi_mask)

        return masked

    def draw_trapezoid_debug(
        self,
        debug: np.ndarray,
        roi_left: int,
        roi_y: int,
        mask: np.ndarray,
    ):
        if not bool(self.get_parameter("debug_view").value):
            return

        if not bool(self.get_parameter("roi_trapezoid_enable").value):
            return

        roi_h, roi_w = mask.shape[:2]
        trap_pts = self.get_trapezoid_roi_points(roi_w, roi_h)

        trap_pts_full = trap_pts.copy()
        trap_pts_full[:, 0] += roi_left
        trap_pts_full[:, 1] += roi_y

        cv2.polylines(
            debug,
            [trap_pts_full],
            isClosed=True,
            color=(0, 255, 0),
            thickness=2,
        )

    def apply_lane_ignore_mask(self, mask: np.ndarray) -> np.ndarray:
        h, w = mask.shape[:2]

        if self.lane == 1:
            start_ratio = float(self.get_parameter("lane1_ignore_x_start_ratio").value)
            end_ratio = float(self.get_parameter("lane1_ignore_x_end_ratio").value)
        else:
            start_ratio = float(self.get_parameter("lane2_ignore_x_start_ratio").value)
            end_ratio = float(self.get_parameter("lane2_ignore_x_end_ratio").value)

        start_ratio = max(0.0, min(1.0, start_ratio))
        end_ratio = max(0.0, min(1.0, end_ratio))

        if end_ratio < start_ratio:
            start_ratio, end_ratio = end_ratio, start_ratio

        x_start = int(w * start_ratio)
        x_end = int(w * end_ratio)

        x_start = max(0, min(w, x_start))
        x_end = max(0, min(w, x_end))

        mask[:, x_start:x_end] = 0

        return mask

    def estimate_lane_x(self, band_xs: np.ndarray, roi_left: int, width: int) -> Optional[float]:
        if band_xs.size > 0:
            # 기존: (min_x + max_x) / 2
            # 변경: 밴드 안에 있는 모든 노란색 픽셀 x좌표의 평균
            detected_x = roi_left + float(np.mean(band_xs))

            if self.prev_detected_yellow_x is not None:
                measured_dx = detected_x - self.prev_detected_yellow_x
                self.last_yellow_dx = 0.7 * self.last_yellow_dx + 0.3 * measured_dx

            self.prev_detected_yellow_x = detected_x
            self.missing_lane_frames = 0
            self.last_lane_x_source = "detect"

            return detected_x

        if self.prev_detected_yellow_x is None:
            self.last_lane_x_source = "none"
            return None

        self.missing_lane_frames += 1

        max_prediction_frames = int(self.get_parameter("max_prediction_frames").value)

        if self.missing_lane_frames > max_prediction_frames:
            self.last_lane_x_source = "none"
            return None

        predicted_x = self.prev_detected_yellow_x + self.last_yellow_dx * self.missing_lane_frames
        predicted_x = max(0.0, min(float(width - 1), predicted_x))

        self.last_lane_x_source = "predict"

        return predicted_x

    def compute_cone_angle(self) -> Optional[float]:
        if self.scan_msg is None:
            return None

        points = self.scan_to_front_points(self.scan_msg)

        min_points = int(self.get_parameter("cone_enable_min_points").value)

        if len(points) < min_points:
            self.last_cone_angle = None
            return None

        left = sorted([p for p in points if p[1] > 0.0], key=lambda p: p[0])
        right = sorted([p for p in points if p[1] <= 0.0], key=lambda p: p[0])

        if len(left) < 2 or len(right) < 2:
            self.last_cone_angle = None
            return None

        left_rep = self.average_points(left[: min(4, len(left))])
        right_rep = self.average_points(right[: min(4, len(right))])

        corridor_center_y = (left_rep[1] + right_rep[1]) * 0.5

        angle = (
            -float(self.get_parameter("steer_sign").value)
            * float(self.get_parameter("cone_kp").value)
            * corridor_center_y
        )

        self.last_cone_angle = self.clamp_angle(angle)

        return self.last_cone_angle

    def scan_to_front_points(self, scan: LaserScan) -> List[Point]:
        points: List[Point] = []

        angle_limit = math.radians(float(self.get_parameter("cone_front_angle_deg").value))
        range_min = float(self.get_parameter("cone_range_min").value)
        range_max = float(self.get_parameter("cone_range_max").value)

        angle = scan.angle_min

        for distance in scan.ranges:
            if (
                math.isfinite(distance)
                and range_min <= distance <= range_max
                and abs(angle) <= angle_limit
            ):
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)

                if x > 0.0:
                    points.append((x, y))

            angle += scan.angle_increment

        return points

    @staticmethod
    def average_points(points: List[Point]) -> Point:
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]

        return sum(xs) / len(xs), sum(ys) / len(ys)

    def clamp_angle(self, angle: float) -> float:
        max_angle = float(self.get_parameter("max_angle").value)
        return max(-max_angle, min(max_angle, angle))

    def publish_motor(self, angle: float, speed: float):
        msg = XycarMotor()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.angle = float(angle)
        msg.speed = float(speed)

        self.last_publish_angle = msg.angle
        self.last_publish_speed = msg.speed

        self.motor_pub.publish(msg)

    def keyboard_loop(self):
        if not sys.stdin.isatty():
            self.line_keyboard_loop()
            return

        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        try:
            while self.running and rclpy.ok():
                cmd = sys.stdin.read(1).lower()

                if cmd in ("\n", "\r", " "):
                    continue

                self.handle_key(cmd)

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def line_keyboard_loop(self):
        while self.running and rclpy.ok():
            line = sys.stdin.readline()

            if not line:
                continue

            cmd = line.strip().lower()

            if cmd in ("auto", "lane", "cone", "stop", "help", "quit"):
                self.handle_key(cmd[0])
            else:
                for ch in cmd:
                    self.handle_key(ch)

    def handle_key(self, cmd: str):
        if cmd == "a":
            self.set_mode("auto")

        elif cmd == "l":
            self.set_mode("lane")

        elif cmd == "c":
            self.set_mode("cone")

        elif cmd == "s":
            self.set_mode("stop")

        elif cmd == "1":
            self.lane = 1
            self.reset_lane_prediction()
            self.reset_lane_pid()
            self.get_logger().info("target lane = 1")
            self.print_status(force=True)

        elif cmd == "2":
            self.lane = 2
            self.reset_lane_prediction()
            self.reset_lane_pid()
            self.get_logger().info("target lane = 2")
            self.print_status(force=True)

        elif cmd == "h":
            self.print_help()
            self.print_status(force=True)

        elif cmd == "q":
            self.set_mode("stop")
            self.publish_motor(0.0, 0.0)
            rclpy.shutdown()

        else:
            self.get_logger().info("unknown key. press h for help")

    def reset_lane_prediction(self):
        self.prev_detected_yellow_x = None
        self.last_yellow_dx = 0.0
        self.missing_lane_frames = 0
        self.last_lane_x_source = "reset"

    def set_mode(self, mode: str):
        self.mode = mode
        self.get_logger().info(f"mode = {self.mode}")
        self.print_status(force=True)

    def print_help(self):
        self.get_logger().info(
            "keys: l=yellow lane, c=rubbercone, a=auto, s=stop, "
            "1/2=target lane, h=help, q=quit"
        )

    def print_status(self, force: bool = False):
        now = time.monotonic()

        if not force and now - self.last_status_time < 1.0:
            return

        self.last_status_time = now

        image_state = "ok" if self.image is not None else "wait"
        scan_state = "ok" if self.scan_msg is not None else "wait"

        yellow_x = "none" if self.last_yellow_x is None else f"{self.last_yellow_x:.1f}"
        target_x = "none" if self.last_lane_target_x is None else f"{self.last_lane_target_x:.1f}"
        lane_error = "none" if self.last_lane_error is None else f"{self.last_lane_error:.1f}"

        self.get_logger().info(
            f"status: mode={self.mode}, lane={self.lane}, "
            f"angle={self.last_publish_angle:.1f}, speed={self.last_publish_speed:.1f}, "
            f"last_valid_angle={self.last_valid_angle:.1f}, "
            f"yellow_x={yellow_x}, target_x={target_x}, lane_error={lane_error}, "
            f"x_source={self.last_lane_x_source}, missing={self.missing_lane_frames}, "
            f"yellow_ratio={self.last_yellow_ratio:.3f}, "
            f"camera={image_state}, scan={scan_state}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = LaneConeDriver()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.running = False

        if rclpy.ok():
            node.publish_motor(0.0, 0.0)

        cv2.destroyAllWindows()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()