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

# ── Pose/refiner quality metadata (Phase 1: append-only, optional) ──
uint8 pose_estimate_mode
float32 pose_quality_score
float32 reproj_error_raw
float32 reproj_error_refined
float32 pose_condition_number
uint16 pose_num_points
uint16 pose_num_inliers

# Covariance of [x, y, z, yaw] in the same frame as Armor.pose.
# Phase 1: always false until Hessian covariance is validated.
bool pose_covariance_valid
float64[16] pose_covariance_xyz_yaw