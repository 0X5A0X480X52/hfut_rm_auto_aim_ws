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