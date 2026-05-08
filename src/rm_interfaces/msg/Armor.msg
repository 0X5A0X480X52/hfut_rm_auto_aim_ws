string number
string type
float32 distance_to_image_center
geometry_msgs/Pose pose

# 来源相机 frame_id (用于区分主相机和补盲相机)
string source_frame

# PnP covariance propagation — 3x3 position covariance matrix (column-major)
float64[9] pos_covariance
# Yaw variance from PnP rotation uncertainty
float64 yaw_variance