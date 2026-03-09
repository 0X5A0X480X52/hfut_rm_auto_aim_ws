# ManeuverStates.msg
# 所有被跟踪机器人的机动检测指标数组（每帧发布）

std_msgs/Header header

# 各被跟踪机器人的机动指标，每个元素对应一个 robot_id
ManeuverState[] states
