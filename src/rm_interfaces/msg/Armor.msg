string number
string type
float32 distance_to_image_center
geometry_msgs/Pose pose

# ── 2D image-domain metadata (Phase 1: append-only, optional) ──
float32 detection_confidence
bool has_image_geometry
float32[4] bbox_xywh
geometry_msgs/Point32[4] image_corners
uint8 corners_ordering

# 来源相机 frame_id (用于区分主相机和补盲相机)
string source_frame

# PnP covariance propagation — 3x3 position covariance matrix (column-major)
float64[9] pos_covariance
# Yaw variance from PnP rotation uncertainty
float64 yaw_variance