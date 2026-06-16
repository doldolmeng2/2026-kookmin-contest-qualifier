#!/usr/bin/env python3
import math

CONE_FRONT_ANGLE_DEG = 90.0
CONE_RANGE_MIN       = 0.18
CONE_RANGE_MAX       = 5.0
CLUSTER_DIST_TH      = 0.4
CLUSTER_MIN_PTS      = 2
CLUSTER_MAX_PTS      = 20
CONE_OFFSET_SCALE    = 1.5
CONE_DEAD_ZONE       = 0.20
CONE_CORNER_TH       = 0.45   # 이 이상이면 코너 → 즉시 최대 조향
CONE_SMOOTH_ALPHA    = 0.35
MEMORY_FRAMES        = 8

class ConeDriver:
    def __init__(self):
        self.last_offset  = 0.0
        self.last_left_y  = None
        self.last_right_y = None
        self.left_age     = 999
        self.right_age    = 999

    def _cluster(self, points):
        if not points:
            return []
        pts = sorted(points, key=lambda p: math.atan2(p[1], p[0]))
        clusters, cur = [], [pts[0]]
        for p in pts[1:]:
            dx, dy = p[0]-cur[-1][0], p[1]-cur[-1][1]
            if math.sqrt(dx*dx+dy*dy) < CLUSTER_DIST_TH:
                cur.append(p)
            else:
                clusters.append(cur)
                cur = [p]
        clusters.append(cur)
        centroids = []
        for c in clusters:
            if CLUSTER_MIN_PTS <= len(c) <= CLUSTER_MAX_PTS:
                cx = sum(p[0] for p in c) / len(c)
                cy = sum(p[1] for p in c) / len(c)
                centroids.append((cx, cy))
        return centroids

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
                if x > 0.3:
                    points.append((x, y))
            angle += scan_msg.angle_increment

        centroids = self._cluster(points)
        left_cones  = [c for c in centroids if c[1] > 0.0]
        right_cones = [c for c in centroids if c[1] <= 0.0]

        # 가장 먼(x 최대) 클러스터 선택 → look-ahead
        if left_cones:
            ref = max(left_cones, key=lambda c: c[0])
            self.last_left_y = ref[1]
            self.left_age = 0
        else:
            self.left_age += 1

        if right_cones:
            ref = max(right_cones, key=lambda c: c[0])
            self.last_right_y = ref[1]
            self.right_age = 0
        else:
            self.right_age += 1

        use_left  = self.last_left_y  if (self.last_left_y  is not None and self.left_age  <= MEMORY_FRAMES) else None
        use_right = self.last_right_y if (self.last_right_y is not None and self.right_age <= MEMORY_FRAMES) else None

        import sys

        if use_left is not None and use_right is not None:
            mid_y = (use_left + use_right) / 2.0

            if abs(mid_y) >= CONE_CORNER_TH:
                # 코너 감지 → 즉시 최대 조향 (smoothing 없음)
                smoothed = -1.0 if mid_y > 0 else 1.0
                tag = "CORNER"
            elif abs(mid_y) >= CONE_DEAD_ZONE:
                # 비례 제어
                raw = max(-1.0, min(1.0, -mid_y / CONE_OFFSET_SCALE))
                smoothed = CONE_SMOOTH_ALPHA * raw + (1.0 - CONE_SMOOTH_ALPHA) * self.last_offset
                tag = "CURVE"
            else:
                # dead zone → 직진 유지
                smoothed = CONE_SMOOTH_ALPHA * 0.0 + (1.0 - CONE_SMOOTH_ALPHA) * self.last_offset
                tag = "STRAIGHT"

            print(f"[CONE/{tag}] L={use_left:.2f}(a={self.left_age}) R={use_right:.2f}(a={self.right_age}) mid={mid_y:.2f} out={smoothed:.3f}", file=sys.stderr, flush=True)

        elif use_left is not None:
            raw = max(-1.0, min(1.0, (use_left - 1.5) / CONE_OFFSET_SCALE))
            smoothed = CONE_SMOOTH_ALPHA * raw + (1.0 - CONE_SMOOTH_ALPHA) * self.last_offset
            print(f"[CONE] only-L={use_left:.2f}(a={self.left_age}) out={smoothed:.3f}", file=sys.stderr, flush=True)
        elif use_right is not None:
            raw = max(-1.0, min(1.0, (use_right + 1.5) / CONE_OFFSET_SCALE))
            smoothed = CONE_SMOOTH_ALPHA * raw + (1.0 - CONE_SMOOTH_ALPHA) * self.last_offset
            print(f"[CONE] only-R={use_right:.2f}(a={self.right_age}) out={smoothed:.3f}", file=sys.stderr, flush=True)
        else:
            smoothed = self.last_offset

        self.last_offset = smoothed
        return smoothed
