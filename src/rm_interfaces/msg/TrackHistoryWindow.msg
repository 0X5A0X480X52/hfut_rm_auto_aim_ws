# TrackHistoryWindow.msg
# 单个跟踪对象的历史状态窗口

std_msgs/Header header

# 跟踪ID (由PointTracker分配的唯一ID)
int32 track_id

# 装甲板ID: "1"~"5", "outpost", "base"
string armor_id

# 装甲板类型: "small", "large"
string armor_type

# 历史状态数组 (按时间顺序,最早到最近)
TrackWindowState[] history

# 窗口配置信息
uint32 max_window_size        # 窗口最大容量
uint32 record_interval        # 记录间隔 (每多少次迭代记录一次)
uint64 current_iteration      # 当前迭代序号
