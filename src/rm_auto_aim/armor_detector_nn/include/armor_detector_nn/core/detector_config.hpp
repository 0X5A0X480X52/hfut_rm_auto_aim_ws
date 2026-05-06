#ifndef ARMOR_DETECTOR_NN_DETECTOR_CONFIG_HPP_
#define ARMOR_DETECTOR_NN_DETECTOR_CONFIG_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace fyt::auto_aim {

enum class BackendType { ONNX_RUNTIME, OPENVINO, TENSORRT };
enum class Precision { FP32, FP16, INT8 };
enum class SchedulingMode { SYNC, ASYNC_LATEST, ASYNC_BATCH };
enum class ColorFilterSource { MODEL, IMAGE, DISABLED };
enum class DetectMode { RED, BLUE, DISABLED };
enum class CopyPolicy { NEVER_COPY, COPY_ON_WRITE_DEBUG, ALWAYS_COPY };
enum class PlatformProfile { JETSON, NUC_CPUONLY, NUC_WITH_GPU, CUSTOM };

inline std::string backendTypeToString(BackendType type) {
  switch (type) {
    case BackendType::ONNX_RUNTIME: return "onnxruntime";
    case BackendType::OPENVINO:     return "openvino";
    case BackendType::TENSORRT:     return "tensorrt";
    default:                        return "unknown";
  }
}

struct BackendConfig {
  BackendType type{BackendType::ONNX_RUNTIME};
  std::string device{"cpu"};
  Precision precision{Precision::FP32};
  std::string model_path;
  std::string engine_path;
  std::string openvino_xml_path;
  std::string openvino_bin_path;
  std::string calibration_cache;
  std::string input_name{"images"};
  std::vector<std::string> output_names{"output0"};
  int warmup_iterations{10};
  int num_threads{2};
  bool preallocate_buffers{true};
  bool use_pinned_memory{true};
  int cuda_stream_count{1};
  bool gpu_preprocess{false};
  bool gpu_decode{false};

  // OpenVINO extension options (interface reserved)
  bool openvino_use_native_preprocess{false};
  std::string openvino_cache_dir;
  bool openvino_hybrid_affinity{false};
  int openvino_num_requests{1};
  std::string openvino_device_config;

  bool allow_fallback{false};
  BackendType fallback_type{BackendType::ONNX_RUNTIME};
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
  std::string strategy{"ultralytics_pose"};
  std::string output_layout{"channels_first"};
  int num_classes{14};
  int num_keypoints{4};
  int keypoint_dims{2};
  int bbox_offset{0};
  int class_offset{4};
  int keypoint_offset{18};
  std::string box_format{"cxcywh"};
  float conf_threshold{0.35F};
  float nms_threshold{0.45F};
  int max_detections{32};
  bool class_agnostic_nms{false};
  std::vector<int> keypoint_remap{1, 0, 3, 2};
  bool head_already_applied{true};
  bool keypoint_auto_reorder{false};
};

struct NumberClassifierConfig {
  bool enabled{false};
  std::string model_path;
  std::string label_path;
  double threshold{0.7};
  std::vector<std::string> ignore_classes{"negative"};
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
  std::string strategy{"internal_iou"};  // internal_iou | muit_sort
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

struct AsyncConfig {
  bool enabled{false};
  double max_wait_ms{2.0};
  bool drop_if_busy{true};
  double max_observation_age_ms{100.0};
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
};

struct RuntimeConfig {
  PlatformProfile platform_profile{PlatformProfile::CUSTOM};
  ColorFilterSource color_filter_source{ColorFilterSource::MODEL};
  bool publish_empty{true};
  bool drop_frame_when_busy{true};
  CopyPolicy copy_policy{CopyPolicy::COPY_ON_WRITE_DEBUG};
  SchedulingMode scheduling_mode{SchedulingMode::SYNC};
  int frame_queue_size{2};
  int batch_min_size{1};
  int batch_max_size{1};
  double batch_timeout_ms{2.0};
  double max_observation_age_ms{50.0};
  bool publish_out_of_order{false};
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
  NumberClassifierConfig number_classifier;

  TrackerConfig tracker;
  CornerRefineConfig corner_refine;
  AsyncConfig async;
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
