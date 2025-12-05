# TrackPredictionWindow.msg
# 单个跟踪对象的预测状态窗口

std_msgs/Header header

# 跟踪ID (由PointTracker分配的唯一ID)
int32 track_id

# 装甲板ID: "1"~"5", "outpost", "base"
string armor_id

# 装甲板类型: "small", "large"
string armor_type

# 预测状态数组 (按时间顺序,从当前到未来)
TrackWindowState[] predictions

# 窗口配置信息
uint32 prediction_steps       # 预测步数
uint32 predict_interval       # 预测间隔 (每多少次迭代记录一次)
uint64 current_iteration      # 当前迭代序号
