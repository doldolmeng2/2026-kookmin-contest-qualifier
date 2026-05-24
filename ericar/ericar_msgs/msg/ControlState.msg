# ControlState.msg
# main → control
# 모드와 정지 명령을 묶어 전달

# === 모드 enum ===
uint8 MODE_WAITING = 0
uint8 MODE_CONE_DRIVING = 1
uint8 MODE_LANE_DRIVING = 2
uint8 MODE_LANE_DRIVING_2 = 3
uint8 MODE_LANE_OBSTACLE_FOLLOW = 4
uint8 MODE_LANE_CHANGE = 5
uint8 MODE_LEFT_TURN = 6

# Header (선택)
std_msgs/Header header

# 현재 모드
uint8 mode

# 정지 명령 (모드와 무관하게 true면 speed=0)
bool stop_request
