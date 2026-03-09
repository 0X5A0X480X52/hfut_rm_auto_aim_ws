# ManeuverState.msg
# 单个被跟踪机器人的机动检测指标（每帧发布，便于图表监控）

# 机器人ID
string robot_id

# 是否判定为机动状态
bool is_maneuvering

# 归一化新息平方 (NIS)；-1 表示本帧无 UKF 更新
float64 nis

# 新息 xyz 向量的 L2 范数 (单位: m)
float64 innov_norm

# yaw 方向新息绝对值 (单位: rad)
float64 innov_yaw_abs

# UKF 更新类型: 0=无更新, 1=单目标板观测, 2=双目标板观测
int32 update_type
