# Perception.msg
# 인식 담당 → main
# 모든 인식 결과를 통합한 단일 메시지

# === 신호등 enum 값 ===
uint8 SIGNAL_NONE = 0
uint8 SIGNAL_RED = 1
uint8 SIGNAL_YELLOW = 2
uint8 SIGNAL_GREEN = 3
uint8 SIGNAL_LEFT = 4

# === 차선 enum 값 (obstacle_faster_lane용) ===
uint8 LANE_UNKNOWN = 0
uint8 LANE_1 = 1
uint8 LANE_2 = 2

# === 경찰차 enum 값 ===
uint8 POLICE_UNKNOWN = 0
uint8 POLICE_PRESENT = 1
uint8 POLICE_ABSENT = 2

# Header (timestamp 추적용)
std_msgs/Header header

# 신호등 (총 2개)
uint8 traffic_light_start         # 시작 신호등
uint8 traffic_light_track         # 트랙 위 신호등 = 경로선택 신호등

# 보행자
bool pedestrian_detected          # 위험 위치 보행자 감지 시 true

# 방해차량
bool obstacle_vehicle_detected    # 앞 차량 인식 여부
uint8 obstacle_faster_lane        # 어느 차선이 더 빠른지 (LANE_*)
bool obstacle_overtake_done       # 느린 차량 추월 완료 신호

# 어린이 보호구역
bool school_zone_entry            # 시작 표시 인식
bool school_zone_exit             # 종료 표시 인식

# 경찰차
uint8 police_car_status           # POLICE_*

# lap 카운트
bool start_line_detected          # 출발선 인식

# 지름길
bool shortcut_exit_trigger        # 두 번째 좌회전 트리거
