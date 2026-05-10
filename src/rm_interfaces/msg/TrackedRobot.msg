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

# ========================
# 完整 6DoF 运动学状态（新实现）
# ========================
# 当 full_state_valid=true 时，优先使用以下字段。
# center_pose:     机器人中心位姿 (xyz + roll/pitch/yaw 的四元数表达)
# center_twist:    机器人中心速度 (linear + angular)
# center_accel:    机器人中心加速度 (linear + angular)
bool full_state_valid
geometry_msgs/Pose center_pose
geometry_msgs/Twist center_twist
geometry_msgs/Accel center_accel

# ========================
# 兼容旧实现字段（保留）
# ========================
# 为兼容历史代码，以下字段继续保留并由新实现同步维护：
# center_position      <-> center_pose.position
# center_velocity      <-> center_twist.linear
# center_acceleration  <-> center_accel.linear
# yaw                  <-> center_pose.orientation(z-y-x 中的 yaw)
# yaw_velocity         <-> center_twist.angular.z
# yaw_acceleration     <-> center_accel.angular.z

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

# 表示模式：区分完整机器人 vs 单装甲板降级表示
uint8 REP_STRUCTURED_ROBOT = 0
uint8 REP_AMBIGUOUS_SINGLE_ARMOR = 1
uint8 representation_mode

# 可见性信息
bool is_visible
int32 visible_armor_count

# 模块可击打掩码（LSB 对应 index=0，bit=1 表示可击打）
# 例如 5 扇叶目标中，mask=0b00101 表示 index 0 和 2 可击打
uint32 engageable_mask

# 可击打模块数量（建议与 engageable_mask 置位个数一致）
int32 engageable_count

# 目标来源相机 frame_id (首次检测到该机器人的相机)
string source_frame
