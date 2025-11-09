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

# 机器人中心位置
geometry_msgs/Point center_position

# 机器人中心速度
geometry_msgs/Vector3 center_velocity

# 机器人yaw角和角速度
float64 yaw
float64 yaw_velocity

# 旋转半径
float64 radius

# 绑定的装甲板ID列表
string[] bound_armor_ids

# 估计置信度 [0,1]
float64 confidence

# 装甲板数量
int32 num_armors
