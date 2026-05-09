std_msgs/Header header
Point2d[5] pts
bool is_lost
bool is_big_rune

# 扇叶语义（与 buff_tracker::BuffBladeType 对齐）
uint8 BLADE_INACTIVATED = 0
uint8 BLADE_ACTIVATED = 1
uint8 BLADE_UNKNOWN = 255
uint8 blade_type

# 扇叶槽位提示 [0..4]，未知为 -1
int32 blade_slot_hint

# 当前观测置信度 [0,1]
float32 confidence

# 观测/跟踪 id（同帧内唯一，跨帧可选延续），未知为 0
uint32 track_id
