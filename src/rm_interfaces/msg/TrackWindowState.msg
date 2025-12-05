# TrackWindowState.msg
# 单个时刻的跟踪状态快照

# 迭代序号 (用于标识帧顺序)
uint64 iteration

# 时间戳
builtin_interfaces/Time timestamp

# 3D位置
geometry_msgs/Point position

# 3D速度
geometry_msgs/Vector3 velocity

# yaw角和角速度
float64 yaw
float64 yaw_velocity

# 跟踪置信度 [0,1]
float64 confidence

# 来源类型: DETECT=0, ESTIMATE=1, PREDICT=2
uint8 source_type

# 跟踪状态: LOST=0, DETECTING=1, TRACKING=2, TEMP_LOST=3
uint8 tracking_state
