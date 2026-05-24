# ActivePerceptions.msg
# main → ericar_perception
# 현재 활성화할 인식기 이름 목록
#
# 가능한 값:
#   "traffic_light_start"
#   "traffic_light_track"
#   "pedestrian"
#   "obstacle_vehicle"
#   "school_zone_entry"
#   "school_zone_exit"
#   "police_car"
#   "lap_counter"
#   "shortcut_exit_trigger"

# Header (선택)
std_msgs/Header header

# 활성 인식기 이름들
string[] names
