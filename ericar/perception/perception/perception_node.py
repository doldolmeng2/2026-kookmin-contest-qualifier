#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# perception_node.py  (Team ERICAR - A 파트: 인식 담당)
#
# 역할:
#   - 시뮬레이터 센서(카메라 / 라이다 / IMU)를 구독한다.
#   - main 노드가 보내는 /ericar/active_perceptions 를 구독해
#     "지금 켜야 할 인식기"만 골라서 실행한다.
#   - 모든 인식 결과를 ericar_msgs/Perception 한 개로 묶어
#     /ericar/perception 으로 30Hz 고정 발행한다.
#
# 설계 규칙(중요):
#   1. 매 발행마다 Perception() 을 새로 만들어 전 필드를 0/false 로 초기화한다.
#      active 목록에 있는 인식기만 결과를 덮어쓴다. (비활성 = 항상 0/false)
#   2. 3-state 필드(obstacle_faster_lane, police_car_status)는
#      "확신이 있을 때만" 1/2 를 채우고, 관측 중에는 0 으로 둔다.
#   3. 결과가 변하지 않아도 매 프레임 같은 값을 계속 발행한다.
# =============================================================================

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, LaserScan, Imu
from ericar_msgs.msg import Perception, ActivePerceptions

from cv_bridge import CvBridge

from perception.detectors.traffic_light_start import StartTrafficLightDetector
from perception.detectors.traffic_light_track import TrackTrafficLightDetector


# 활성 인식기 이름 상수 (오타 방지를 위해 한 곳에서 관리)
P_TRAFFIC_START   = "traffic_light_start"
P_TRAFFIC_TRACK   = "traffic_light_track"
P_PEDESTRIAN      = "pedestrian"
P_OBSTACLE        = "obstacle_vehicle"
P_SCHOOL_ENTRY    = "school_zone_entry"
P_SCHOOL_EXIT     = "school_zone_exit"
P_POLICE          = "police_car"
P_LAP_COUNTER     = "lap_counter"
P_SHORTCUT_EXIT   = "shortcut_exit_trigger"


class PerceptionNode(Node):

    # -------------------------------------------------------------------------
    # 초기화: 퍼블리셔 / 서브스크라이버 / 30Hz 타이머 / 센서 버퍼 설정
    # -------------------------------------------------------------------------
    def __init__(self):
        super().__init__('ericar_perception')

        # --- 팀 공용 QoS: 실시간성 우선 (best-effort, depth=10) ---
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # --- 센서 데이터 버퍼 (콜백이 갱신, 인식 함수가 읽음) ---
        self.bridge = CvBridge()
        self.image = None          # 최신 카메라 프레임 (OpenCV BGR)
        self.lidar_ranges = None   # 최신 라이다 거리 배열
        self.imu_yaw = None        # 최신 IMU yaw (필요 시 사용)

        # --- 활성 인식기 목록 (main 이 지정; 초기엔 빈 집합) ---
        self.active = set()

        # --- 인식기 모듈 인스턴스 ---
        #   debug=true 로 켜면 /tmp/tl_debug 에 튜닝용 이미지 저장
        #   ros2 run perception perception_node --ros-args -p tl_debug:=true
        self.declare_parameter('tl_debug', False)
        dbg = self.get_parameter('tl_debug').value
        # 시작등/트랙등은 코드 파일·파라미터·히스토리가 완전히 분리됨
        self.tl_start = StartTrafficLightDetector(debug=dbg, logger=self.get_logger())
        self.tl_track = TrackTrafficLightDetector(debug=dbg, logger=self.get_logger())
        self.get_logger().info(f"tl_debug = {dbg}")

        # --- 센서 구독 ---
        self.create_subscription(
            Image, '/usb_cam/image_raw/front', self.cam_callback, qos)
        self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, qos)
        self.create_subscription(
            Imu, '/imu', self.imu_callback, qos)

        # --- main 으로부터 활성 인식기 목록 구독 ---
        self.create_subscription(
            ActivePerceptions, '/ericar/active_perceptions',
            self.active_callback, qos)

        # --- 인식 결과 발행 ---
        self.perception_pub = self.create_publisher(
            Perception, '/ericar/perception', qos)

        # --- 30Hz 발행 타이머 ---
        self.timer = self.create_timer(1.0 / 30.0, self.publish_perception)

        self.get_logger().info('----- ERICAR perception node started -----')

    # =========================================================================
    # 센서 콜백들 (수신한 데이터를 버퍼에 저장만 한다)
    # =========================================================================

    # 카메라 프레임 수신 → OpenCV 이미지로 변환해 저장
    def cam_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    # 라이다 스캔 수신 → 거리 배열 저장
    def lidar_callback(self, msg):
        self.lidar_ranges = msg.ranges

    # IMU 수신 → (필요 시) yaw 계산해 저장. 현재는 원본 쿼터니언만 보관용.
    def imu_callback(self, msg):
        self.imu_yaw = msg.orientation  # 실제 yaw 변환은 사용 시점에 처리

    # main 의 활성 인식기 목록 수신 → set 으로 보관
    def active_callback(self, msg):
        new_active = set(msg.names)
        if new_active != self.active:
            self.get_logger().info(f'active perceptions -> {sorted(new_active)}')
        self.active = new_active

    # =========================================================================
    # 메인 발행 루프 (30Hz)
    #   - 매번 Perception 을 새로 만들어 전 필드 0/false 로 시작
    #   - active 에 포함된 인식기만 결과를 채운다
    # =========================================================================
    def publish_perception(self):
        msg = Perception()
        msg.header.stamp = self.get_clock().now().to_msg()

        active = self.active  # 지역 변수로 고정 (콜백 변경 대비)

        # --- 신호등 (시작) ---
        if P_TRAFFIC_START in active:
            msg.traffic_light_start = self.detect_traffic_light_start()

        # --- 신호등 (트랙 위 / 경로선택) ---
        if P_TRAFFIC_TRACK in active:
            msg.traffic_light_track = self.detect_traffic_light_track()

        # --- 보행자 ---
        if P_PEDESTRIAN in active:
            msg.pedestrian_detected = self.detect_pedestrian()

        # --- 방해차량 (3개 필드를 함께 채움) ---
        if P_OBSTACLE in active:
            detected, faster_lane, overtake_done = self.detect_obstacle_vehicle()
            msg.obstacle_vehicle_detected = detected
            msg.obstacle_faster_lane = faster_lane
            msg.obstacle_overtake_done = overtake_done

        # --- 어린이 보호구역 시작 표시 ---
        if P_SCHOOL_ENTRY in active:
            msg.school_zone_entry = self.detect_school_zone_entry()

        # --- 어린이 보호구역 종료 표시 ---
        if P_SCHOOL_EXIT in active:
            msg.school_zone_exit = self.detect_school_zone_exit()

        # --- 경찰차 유무 ---
        if P_POLICE in active:
            msg.police_car_status = self.detect_police_car()

        # --- 출발선(체스판) → lap 카운트 ---
        if P_LAP_COUNTER in active:
            msg.start_line_detected = self.detect_start_line()

        # --- 지름길 두 번째 좌회전 트리거 ---
        if P_SHORTCUT_EXIT in active:
            msg.shortcut_exit_trigger = self.detect_shortcut_exit_trigger()

        self.perception_pub.publish(msg)

    # =========================================================================
    # 인식기 함수들 (현재는 stub — 추후 실제 로직으로 채운다)
    #   - 카메라가 필요한 함수는 self.image, 라이다는 self.lidar_ranges 사용
    #   - 데이터가 아직 없으면 안전하게 기본값(0/false) 반환
    # =========================================================================

    # 시작 신호등(3구): 시작등 전용 인스턴스에 위임
    def detect_traffic_light_start(self):
        return self.tl_start.detect(self.image)

    # 트랙 위(경로선택) 신호등(4구, 좌회전 포함): 트랙등 전용 인스턴스에 위임
    def detect_traffic_light_track(self):
        return self.tl_track.detect(self.image)

    # 보행자: 위험 위치에 있으면 True
    def detect_pedestrian(self):
        # TODO: 카메라 또는 라이다 기반 보행자 검출
        return False

    # 방해차량: (검출여부, 더 빠른 차선[0/1/2], 추월완료여부)
    #   - faster_lane 은 확신이 있을 때만 1/2, 관측 중엔 0
    def detect_obstacle_vehicle(self):
        # TODO: 라이다 클러스터링 + 좌우 차선 차량 속도 비교
        detected = False
        faster_lane = Perception.LANE_UNKNOWN
        overtake_done = False
        return detected, faster_lane, overtake_done

    # 어린이 보호구역 시작 표시("어린이보호구역" 노면 글자)
    def detect_school_zone_entry(self):
        # TODO: 노면 글자/색 인식
        return False

    # 어린이 보호구역 종료 표시("해제" 노면 글자)
    def detect_school_zone_exit(self):
        # TODO: 노면 글자/색 인식
        return False

    # 경찰차 유무: 0:미판단 1:있음 2:없음 (확신할 때만 1/2)
    def detect_police_car(self):
        # TODO: 카메라 기반 경찰차 검출
        return Perception.POLICE_UNKNOWN

    # 출발선(체스판 무늬) 인식
    def detect_start_line(self):
        # TODO: 체스판 패턴 검출
        return False

    # 지름길 두 번째 좌회전 트리거(차선 패턴 끝남 등)
    def detect_shortcut_exit_trigger(self):
        # TODO: 차선 패턴/카메라 기반 트리거
        return False


# =============================================================================
# 엔트리 포인트
# =============================================================================
def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
