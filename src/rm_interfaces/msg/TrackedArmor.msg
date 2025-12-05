# TrackedArmor.msg
# 单个被跟踪装甲板的状态信息

std_msgs/Header header

# 跟踪ID (由跟踪器分配的唯一ID)
int32 track_id

# 装甲板ID: "1"~"5", "outpost", "base"
string armor_id

# 装甲板类型: "small", "large"
string armor_type

# 3D位置 (在目标坐标系下,如odom)
geometry_msgs/Point position

# 3D速度
geometry_msgs/Vector3 velocity

# yaw角和角速度
float64 yaw
float64 yaw_velocity

# 跟踪置信度 [0,1]
float64 confidence

# ================== 来源标记 ==================
# 用于标识当前状态估计的来源
# DETECT: 融合了上游检测节点的位置估计
# ESTIMATE: 融合了robot_pose_estimator发布的估计装甲板位置
# PREDICT: 无观测输入，仅通过模型预测的位置估计
uint8 SOURCE_DETECT = 0
uint8 SOURCE_ESTIMATE = 1
uint8 SOURCE_PREDICT = 2
uint8 source_type

# ================== 跟踪状态 ==================
# 跟踪状态: LOST=0, DETECTING=1, TRACKING=2, TEMP_LOST=3
uint8 LOST = 0
uint8 DETECTING = 1
uint8 TRACKING = 2
uint8 TEMP_LOST = 3
uint8 tracking_state

# 最后检测到的时间
builtin_interfaces/Time last_detected_time

# 连续跟踪帧数
int32 tracking_count

# 连续丢失帧数
int32 lost_count

# 自上次更新后的帧数 (time_since_update)
int32 time_since_update
