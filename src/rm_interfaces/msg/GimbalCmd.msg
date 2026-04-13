std_msgs/Header header
float64 yaw
float64 yaw_diff
float64 yaw_v
float64 yaw_a
float64 pitch
float64 pitch_diff
float64 pitch_v
float64 pitch_a
# mode 语义:
#   MODE_BLIND_CAMERA_RESULT(-2): 补盲相机结果（预留）
#   MODE_NO_VALID_MEASUREMENT(-1): 无有效测量
#   MODE_UNKNOWN(0): 未初始化/未知
#   MODE_NORMAL_MEASUREMENT(1): 正常测量
int8 MODE_BLIND_CAMERA_RESULT = -2
int8 MODE_NO_VALID_MEASUREMENT = -1
int8 MODE_UNKNOWN = 0
int8 MODE_NORMAL_MEASUREMENT = 1

# 纯物理距离（米），不再承载状态哨兵语义
float64 distance
bool fire_advice
string target_id
int8 mode