# TrackedRobot.msg
# 单个被跟踪机器人的状态信息

std_msgs/Header header

# 机器人ID: "1"~"5", "outpost", "base"
string robot_id

# 机器人类型
uint8 BALANCE_2 = 0    # 平衡步兵: 2装甲板
uint8 STANDARD_4 = 1   # 标准机器人: 4装甲板
uint8 HERO_4 = 2       # 英雄: 4装甲板
uint8 OUTPOST_3 = 3    # 前哨站: 3装甲板
uint8 SENTRY = 4       # 哨兵
uint8 BASE = 5         # 基地
uint8 UNKNOWN = 255    # 未知类型
uint8 robot_type

# 跟踪状态
uint8 DETECTING = 0    # 正在检测
uint8 TRACKING = 1     # 稳定跟踪
uint8 TEMP_LOST = 2    # 临时丢失
uint8 track_state

# 机器人中心位置 (世界坐标系)
geometry_msgs/Point center_position

# 机器人中心速度
geometry_msgs/Vector3 center_velocity

# 机器人中心加速度
geometry_msgs/Vector3 center_acceleration

# 机器人yaw角、角速度、角加速度
float64 yaw
float64 yaw_velocity
float64 yaw_acceleration

# 旋转半径 (支持双半径，4装甲板机器人)
float64 radius
float64 radius_2

# 装甲板高度差 (4装甲板机器人的高度偏移)
float64 d_za
float64 d_zc

# 装甲板在机器人坐标系中的几何偏移
# 每个Pose表示一个装甲板相对于机器人质心的位置和朝向
# position: 装甲板中心相对于机器人质心的偏移 (机器人坐标系)
# orientation: 装甲板法向量的朝向 (机器人坐标系)
geometry_msgs/Pose[] armors_offset

# 状态协方差矩阵 (行优先存储)
# 维度由 covariance_dim 指定，矩阵大小为 covariance_dim x covariance_dim
# 典型状态: [x, y, z, yaw, vx, vy, vz, v_yaw] 对应 dim=8
float64[] state_covariance
uint8 covariance_dim

# 绑定的装甲板ID列表
string[] bound_armor_ids

# 估计置信度 [0,1]
float64 confidence

# 装甲板数量
int32 num_armors

# 可见性信息
bool is_visible
int32 visible_armor_count
