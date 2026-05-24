# DrivingOffset.msg
# ericar_driving → control
# 차선/라바콘/좌회전 offset 통합

# === source enum ===
uint8 SOURCE_LANE = 0       # 차선 인식
uint8 SOURCE_CONE = 1       # 라바콘 인식
uint8 SOURCE_LEFT_TURN = 2  # 좌회전 모듈

# Header (선택)
std_msgs/Header header

# 횡방향 오차 (부호 규칙: 좌-, 우+)
float32 offset

# 어디서 온 offset인지 (디버깅용)
uint8 source
