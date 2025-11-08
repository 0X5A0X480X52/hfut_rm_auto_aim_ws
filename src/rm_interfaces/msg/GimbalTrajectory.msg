# GimbalTrajectory.msg
# 云台运动轨迹 (MPC规划的结果)

std_msgs/Header header

# 轨迹点序列
GimbalState[] trajectory

# 时间步长 (秒)
float64 dt

# 轨迹总时长 (秒)
float64 total_time
