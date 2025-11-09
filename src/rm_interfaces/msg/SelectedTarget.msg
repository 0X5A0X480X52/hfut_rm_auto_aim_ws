# SelectedTarget.msg
# 选中的击打目标

std_msgs/Header header

# 选中的机器人ID
string robot_id

# 选择置信度 [0,1]
float64 confidence

# 选择策略
string selection_strategy
