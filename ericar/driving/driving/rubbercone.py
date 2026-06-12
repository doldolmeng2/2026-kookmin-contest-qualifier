#!/usr/bin/env python3
import math

CONE_FRONT_ANGLE_DEG = 90.0
CONE_RANGE_MIN       = 0.18
CONE_RANGE_MAX       = 2.0
CONE_MIN_POINTS      = 6
CONE_OFFSET_SCALE    = 1.0

class ConeDriver:
    def __init__(self):
        self.last_offset = 0.0

    def compute_offset(self, scan_msg):
        if scan_msg is None:
            return self.last_offset

        angle_limit = math.radians(CONE_FRONT_ANGLE_DEG)
        points = []
        angle = scan_msg.angle_min
        for dist in scan_msg.ranges:
            norm_angle = angle
            if norm_angle > math.pi:
                norm_angle -= 2.0 * math.pi
            if (math.isfinite(dist)
                    and CONE_RANGE_MIN <= dist <= CONE_RANGE_MAX
                    and abs(norm_angle) <= angle_limit):
                x = dist * math.cos(norm_angle)
                y = dist * math.sin(norm_angle)
                if x > 0.0:
                    points.append((x, y))
            angle += scan_msg.angle_increment

        left  = sorted([p for p in points if p[1] > 0.0],  key=lambda p: p[0])
        right = sorted([p for p in points if p[1] <= 0.0], key=lambda p: p[0])

        if len(points) < CONE_MIN_POINTS:
            return self.last_offset
        if len(left) < 2 or len(right) < 2:
            return self.last_offset

        n = 4
        left_rep_y  = sum(p[1] for p in left[:n])  / min(n, len(left))
        right_rep_y = sum(p[1] for p in right[:n]) / min(n, len(right))
        corridor_center_y = (left_rep_y + right_rep_y) * 0.5
        offset = max(-1.0, min(1.0, -corridor_center_y / CONE_OFFSET_SCALE))
        self.last_offset = offset
        return offset
