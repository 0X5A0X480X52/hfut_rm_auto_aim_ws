#ifndef ARMOR_DETECTOR_NN_DETECTOR_CONFIG_HPP_
#define ARMOR_DETECTOR_NN_DETECTOR_CONFIG_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace fyt::auto_aim {

enum class Precision { FP32, FP16, INT8 };
enum class ColorFilterSource { MODEL, IMAGE, DISABLED };
enum class DetectMode { RED, BLUE, DISABLED };
enum class CopyPolicy { NEVER_COPY, COPY_ON_WRITE_DEBUG, ALWAYS_COPY };

struct BackendConfig {
  std::string device{"CPU"};
  Precision precision{Precision::FP32};
  std::string model_path;
  int warmup_iterations{10};
  int num_threads{2};
};

struct PreprocessConfig {
  int input_width{640};
  int input_height{640};
  std::string input_layout{"nchw"};
  std::string input_color{"rgb"};
  std::string resize_mode{"letterbox"};
  bool normalize{true};
  std::vector<double> mean{0.0, 0.0, 0.0};
  std::vector<double> std{255.0, 255.0, 255.0};
  float pad_value{114.0F};
};

struct PostprocessConfig {
  std::string output_layout{"candidates_first"};
  int num_classes{14};
  int num_keypoints{4};
  int keypoint_dims{2};
  int bbox_offset{0};
  int class_offset{4};
  int keypoint_offset{18};
  std::string box_format{"xyxy_from_kpts"};
  float conf_threshold{0.35F};
  float nms_threshold{0.45F};
  int max_detections{32};
  bool class_agnostic_nms{false};
  std::vector<int> keypoint_remap{0, 1, 2, 3};
  bool head_already_applied{true};
  bool keypoint_auto_reorder{false};
};

struct LabelMapConfig {
  std::string path;
};

struct SingleYawConfig {
  int max_iterations{15};
  double huber_delta{3.0};
  double pitch_deg_default{15.0};
  double roll_deg_default{0.0};
  bool outpost_pitch_sign{true};
};

struct SlidingWindowConfig {
  int window_size{8};
  int min_frames{4};
  double max_time_span_ms{300};
  double max_solver_time_ms{2.0};
  int max_opt_iters{20};
  double sigma_prior_xy{0.08};
  double sigma_prior_z{0.15};
  double sigma_prior_yaw{0.35};
  double sigma_smooth_xy{0.05};
  double sigma_smooth_z{0.10};
  double sigma_smooth_yaw{0.10};
  double sigma_kp_min{1.0};
  double sigma_kp_scale{5.0};
  double huber_delta{3.0};
};

struct RefinerConfig {
  std::string mode{"single_yaw"};  // none | single_yaw | sliding_window
};

struct GateConfig {
  double max_reproj_error{3.0};
  double max_pose_delta_m{0.20};
  double max_yaw_delta_deg{20.0};
  bool require_finite{true};
};

struct TrackerConfig {
  double iou_threshold{0.30};
  int max_missed{15};
  int min_hits{2};
  int max_center_dist_px{120};
};

struct CornerRefineConfig {
  bool enabled{false};
  bool apply_on_confirmed_only{true};
  int max_targets_per_frame{1};
  double time_budget_ms{2.0};
  double roi_expand_ratio{1.2};
  int min_bright_points{30};
  double pca_stability_threshold{0.7};
  double max_aspect_ratio{5.0};
  double min_aspect_ratio{1.5};
};

struct PoseConfig {
  bool use_ba{true};
  std::string pnp_method{"ippe"};
  double small_armor_width{0.133};
  double small_armor_height{0.050};
  double large_armor_width{0.225};
  double large_armor_height{0.050};

  RefinerConfig refiner;
  SingleYawConfig single_yaw;
  SlidingWindowConfig sliding;
  GateConfig gate;
  bool force_pnp_rotate_180{false};
};

struct RuntimeConfig {
  ColorFilterSource color_filter_source{ColorFilterSource::MODEL};
  bool publish_empty{true};
  CopyPolicy copy_policy{CopyPolicy::COPY_ON_WRITE_DEBUG};
  bool profile{true};
};

struct DetectorConfig {
  bool debug{false};
  std::string target_frame{"odom"};

  BackendConfig backend;
  PreprocessConfig preprocess;
  PostprocessConfig postprocess;
  LabelMapConfig label_map;
  PoseConfig pose;
  RuntimeConfig runtime;
  TrackerConfig tracker;
  CornerRefineConfig corner_refine;
};

struct BackendInfo {
  std::string backend_name;
  std::string precision;
  int min_batch_size{1};
  int max_batch_size{1};
  bool dynamic_batch{false};
};

}  // namespace fyt::auto_aim

#endif
