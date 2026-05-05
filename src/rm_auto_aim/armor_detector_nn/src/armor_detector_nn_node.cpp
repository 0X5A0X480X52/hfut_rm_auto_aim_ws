#include "armor_detector_nn/armor_detector_nn_node.hpp"

#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "rm_utils/assert.hpp"
#include "rm_utils/logger/log.hpp"

namespace fyt::auto_aim {

ArmorDetectorNNNode::ArmorDetectorNNNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("armor_detector", options)
{
  FYT_REGISTER_LOGGER("armor_detector", "~/fyt2024-log", INFO);
  FYT_REGISTER_LOGGER("armor_detector_nn", "~/fyt2024-log", INFO);
  FYT_INFO("armor_detector", "Starting ArmorDetectorNNNode (neural-network detector)");

  initializeParameters();
  validateParameters();

  // --- detector ---
  detector_ = std::make_unique<ArmorDetectorNN>(config_);
  if (!detector_->initialize()) {
    FYT_ERROR("armor_detector", "Failed to initialize detector. "
              "Node will start but detection is disabled.");
  }

  // --- pose estimator ---
  pose_estimator_adapter_ = std::make_unique<ArmorPoseEstimatorAdapter>(config_.pose);

  // --- debug drawer ---
  debug_drawer_ = std::make_unique<DebugDrawer>();

  // --- subscriptions ---
  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "image_raw", rclcpp::SensorDataQoS(),
    std::bind(&ArmorDetectorNNNode::imageCallback, this, std::placeholders::_1));

  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera_info", rclcpp::SensorDataQoS(),
    std::bind(&ArmorDetectorNNNode::cameraInfoCallback, this, std::placeholders::_1));

  // --- publishers ---
  armors_pub_ = this->create_publisher<rm_interfaces::msg::Armors>(
    "armor_detector/armors", rclcpp::SensorDataQoS());

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "armor_detector/marker", rclcpp::SensorDataQoS());

  // --- service ---
  set_mode_srv_ = this->create_service<rm_interfaces::srv::SetMode>(
    "armor_detector/set_mode",
    std::bind(&ArmorDetectorNNNode::setModeCallback, this,
              std::placeholders::_1, std::placeholders::_2));

  // --- debug ---
  if (debug_) {
    createDebugPublishers();
  }

  // --- profiler ---
  if (config_.runtime.profile) {
    profiler_ = std::make_unique<Profiler>();
  }

  // --- parameter callback ---
  on_set_parameters_callback_handle_ =
    this->add_on_set_parameters_callback(
      std::bind(&ArmorDetectorNNNode::onSetParameters, this, std::placeholders::_1));

  // --- heartbeat ---
  heartbeat_ = HeartBeatPublisher::create(this);

  FYT_INFO("armor_detector", "ArmorDetectorNNNode initialized. Mode: DISABLED.");
}

void ArmorDetectorNNNode::initializeParameters() {
  debug_ = this->declare_parameter("debug", true);
  config_.target_frame = this->declare_parameter("target_frame", "odom");

  // backend
  {
    std::string type_str = this->declare_parameter("backend.type", "onnxruntime");
    config_.backend.device     = this->declare_parameter("backend.device", "cpu");
    std::string prec_str       = this->declare_parameter("backend.precision", "fp32");
    config_.backend.model_path      = this->declare_parameter("backend.model_path", "");
    config_.backend.engine_path     = this->declare_parameter("backend.engine_path", "");
    config_.backend.openvino_xml_path = this->declare_parameter("backend.openvino_model_xml", "");
    config_.backend.openvino_bin_path = this->declare_parameter("backend.openvino_model_bin", "");
    config_.backend.calibration_cache = this->declare_parameter("backend.calibration_cache", "");
    config_.backend.input_name    = this->declare_parameter("backend.input_name", "images");
    config_.backend.output_names  = this->declare_parameter("backend.output_names",
                                        std::vector<std::string>{"output0"});
    config_.backend.warmup_iterations = this->declare_parameter("backend.warmup_iterations", 10);
    config_.backend.num_threads    = this->declare_parameter("backend.num_threads", 2);
    config_.backend.preallocate_buffers = this->declare_parameter("backend.preallocate_buffers", true);
    config_.backend.use_pinned_memory = this->declare_parameter("backend.use_pinned_memory", true);
    config_.backend.cuda_stream_count = this->declare_parameter("backend.cuda_stream_count", 1);
    config_.backend.gpu_preprocess = this->declare_parameter("backend.gpu_preprocess", false);
    config_.backend.gpu_decode = this->declare_parameter("backend.gpu_decode", false);
    config_.backend.allow_fallback = this->declare_parameter("backend.allow_fallback", false);
    config_.backend.openvino_use_native_preprocess =
      this->declare_parameter("backend.openvino_use_native_preprocess", false);
    config_.backend.openvino_cache_dir =
      this->declare_parameter("backend.openvino_cache_dir", "");
    config_.backend.openvino_hybrid_affinity =
      this->declare_parameter("backend.openvino_hybrid_affinity", false);
    config_.backend.openvino_num_requests =
      this->declare_parameter("backend.openvino_num_requests", 1);
    config_.backend.openvino_device_config =
      this->declare_parameter("backend.openvino_device_config", "");
    std::string fallback_str       = this->declare_parameter("backend.fallback_type", "onnxruntime");

    if (type_str == "openvino") config_.backend.type = BackendType::OPENVINO;
    else if (type_str == "tensorrt") config_.backend.type = BackendType::TENSORRT;
    else config_.backend.type = BackendType::ONNX_RUNTIME;

    if (prec_str == "fp16") config_.backend.precision = Precision::FP16;
    else if (prec_str == "int8") config_.backend.precision = Precision::INT8;
    else config_.backend.precision = Precision::FP32;

    if (fallback_str == "openvino") config_.backend.fallback_type = BackendType::OPENVINO;
    else if (fallback_str == "tensorrt") config_.backend.fallback_type = BackendType::TENSORRT;
    else config_.backend.fallback_type = BackendType::ONNX_RUNTIME;
  }

  // preprocess
  {
    config_.preprocess.input_width  = this->declare_parameter("preprocess.input_width", 640);
    config_.preprocess.input_height = this->declare_parameter("preprocess.input_height", 640);
    config_.preprocess.input_layout = this->declare_parameter("preprocess.input_layout", "nchw");
    config_.preprocess.input_color  = this->declare_parameter("preprocess.input_color", "rgb");
    config_.preprocess.resize_mode  = this->declare_parameter("preprocess.resize_mode", "letterbox");
    config_.preprocess.normalize    = this->declare_parameter("preprocess.normalize", true);
    config_.preprocess.mean = this->declare_parameter("preprocess.mean",
                                  std::vector<double>{0.0, 0.0, 0.0});
    config_.preprocess.std  = this->declare_parameter("preprocess.std",
                                  std::vector<double>{255.0, 255.0, 255.0});
    config_.preprocess.pad_value = static_cast<float>(
      this->declare_parameter("preprocess.pad_value", 114.0));

    // mean/std already stored with correct type
  }

  // postprocess
  {
    config_.postprocess.strategy = this->declare_parameter("postprocess.strategy", "ultralytics_pose");
    config_.postprocess.output_layout = this->declare_parameter("postprocess.output_layout", "channels_first");
    config_.postprocess.num_classes     = this->declare_parameter("postprocess.num_classes", 14);
    config_.postprocess.num_keypoints   = this->declare_parameter("postprocess.num_keypoints", 4);
    config_.postprocess.keypoint_dims   = this->declare_parameter("postprocess.keypoint_dims", 2);
    config_.postprocess.bbox_offset     = this->declare_parameter("postprocess.bbox_offset", 0);
    config_.postprocess.class_offset    = this->declare_parameter("postprocess.class_offset", 4);
    config_.postprocess.keypoint_offset = this->declare_parameter("postprocess.keypoint_offset", 18);
    config_.postprocess.box_format      = this->declare_parameter("postprocess.box_format", "cxcywh");
    config_.postprocess.conf_threshold  = this->declare_parameter("postprocess.conf_threshold", 0.35);
    config_.postprocess.nms_threshold   = this->declare_parameter("postprocess.nms_threshold", 0.45);
    config_.postprocess.max_detections   = this->declare_parameter("postprocess.max_detections", 32);
    config_.postprocess.class_agnostic_nms = this->declare_parameter("postprocess.class_agnostic_nms", false);
    {
      auto remap = this->declare_parameter("postprocess.keypoint_remap",
                    std::vector<int64_t>{1, 0, 3, 2});
      config_.postprocess.keypoint_remap.assign(remap.begin(), remap.end());
    }
    config_.postprocess.head_already_applied = this->declare_parameter("postprocess.head_already_applied", true);
    config_.postprocess.keypoint_auto_reorder = this->declare_parameter("postprocess.keypoint_auto_reorder", false);
  }

  // label_map
  {
    config_.label_map.path = this->declare_parameter("label_map.path", "");
  }

  // number_classifier (optional ID postprocessor)
  {
    config_.number_classifier.enabled =
      this->declare_parameter("number_classifier.enabled", false);
    config_.number_classifier.model_path =
      this->declare_parameter("number_classifier.model_path", "");
    config_.number_classifier.label_path =
      this->declare_parameter("number_classifier.label_path", "");
    config_.number_classifier.threshold =
      this->declare_parameter("number_classifier.threshold", 0.7);
    config_.number_classifier.ignore_classes =
      this->declare_parameter("number_classifier.ignore_classes",
                              std::vector<std::string>{"negative"});
  }

  // pose
  {
    config_.pose.use_ba = this->declare_parameter("pose.use_ba", true);
    config_.pose.pnp_method = this->declare_parameter("pose.pnp_method", "ippe");
    config_.pose.small_armor_width  = this->declare_parameter("pose.small_armor_width", 0.133);
    config_.pose.small_armor_height = this->declare_parameter("pose.small_armor_height", 0.050);
    config_.pose.large_armor_width  = this->declare_parameter("pose.large_armor_width", 0.225);
    config_.pose.large_armor_height = this->declare_parameter("pose.large_armor_height", 0.050);
  }

  // runtime
  {
    std::string profile_str = this->declare_parameter("runtime.platform_profile", "custom");
    if (profile_str == "jetson") config_.runtime.platform_profile = PlatformProfile::JETSON;
    else if (profile_str == "nuc_cpuonly") config_.runtime.platform_profile = PlatformProfile::NUC_CPUONLY;
    else if (profile_str == "nuc_with_gpu") config_.runtime.platform_profile = PlatformProfile::NUC_WITH_GPU;
    else config_.runtime.platform_profile = PlatformProfile::CUSTOM;

    std::string cfs = this->declare_parameter("runtime.color_filter_source", "model");
    if (cfs == "image") config_.runtime.color_filter_source = ColorFilterSource::IMAGE;
    else if (cfs == "disabled") config_.runtime.color_filter_source = ColorFilterSource::DISABLED;
    else config_.runtime.color_filter_source = ColorFilterSource::MODEL;
    config_.runtime.publish_empty = this->declare_parameter("runtime.publish_empty", true);
    config_.runtime.drop_frame_when_busy = this->declare_parameter("runtime.drop_frame_when_busy", true);
    std::string copy_policy_str = this->declare_parameter("runtime.copy_policy", "copy_on_write_debug");
    std::string sched = this->declare_parameter("runtime.scheduling_mode", "sync");
    config_.runtime.frame_queue_size = this->declare_parameter("runtime.frame_queue_size", 2);
    config_.runtime.batch_min_size   = this->declare_parameter("runtime.batch_min_size", 1);
    config_.runtime.batch_max_size   = this->declare_parameter("runtime.batch_max_size", 2);
    config_.runtime.batch_timeout_ms  = this->declare_parameter("runtime.batch_timeout_ms", 2.0);
    config_.runtime.max_observation_age_ms = this->declare_parameter("runtime.max_observation_age_ms", 50.0);
    config_.runtime.publish_out_of_order = this->declare_parameter("runtime.publish_out_of_order", false);
    config_.runtime.profile = this->declare_parameter("runtime.profile", true);

    if (sched == "async_latest") config_.runtime.scheduling_mode = SchedulingMode::ASYNC_LATEST;
    else if (sched == "async_batch") config_.runtime.scheduling_mode = SchedulingMode::ASYNC_BATCH;
    else config_.runtime.scheduling_mode = SchedulingMode::SYNC;

    if (copy_policy_str == "never_copy") config_.runtime.copy_policy = CopyPolicy::NEVER_COPY;
    else if (copy_policy_str == "always_copy") config_.runtime.copy_policy = CopyPolicy::ALWAYS_COPY;
    else config_.runtime.copy_policy = CopyPolicy::COPY_ON_WRITE_DEBUG;
  }
}

void ArmorDetectorNNNode::validateParameters() {
  if (config_.postprocess.conf_threshold < 0.0 || config_.postprocess.conf_threshold > 1.0) {
    FYT_ERROR("armor_detector", "conf_threshold out of range, using default 0.35");
    config_.postprocess.conf_threshold = 0.35F;
  }
  if (config_.postprocess.nms_threshold < 0.0 || config_.postprocess.nms_threshold > 1.0) {
    FYT_ERROR("armor_detector", "nms_threshold out of range, using default 0.45");
    config_.postprocess.nms_threshold = 0.45F;
  }
  if (config_.runtime.batch_min_size < 1) {
    config_.runtime.batch_min_size = 1;
  }
  if (config_.runtime.batch_max_size < config_.runtime.batch_min_size) {
    config_.runtime.batch_max_size = config_.runtime.batch_min_size;
  }
  if (config_.backend.cuda_stream_count < 1) {
    config_.backend.cuda_stream_count = 1;
  }
  if (config_.backend.openvino_num_requests < 1) {
    config_.backend.openvino_num_requests = 1;
  }
}

void ArmorDetectorNNNode::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& img_msg)
{
  if (current_mode_ == DetectMode::DISABLED) {
    return;
  }
  if (!detector_ || !detector_->isInitialized()) {
    return;
  }

  FYT_INFO("armor_detector", "Received image frame (timestamp: {}.{})", img_msg->header.stamp.sec, img_msg->header.stamp.nanosec);

  auto t_start = std::chrono::steady_clock::now();

  // cv_bridge — apply copy policy
  cv::Mat frame;
  try {
    bool need_copy = (config_.runtime.copy_policy == CopyPolicy::ALWAYS_COPY) ||
                     (config_.runtime.copy_policy == CopyPolicy::COPY_ON_WRITE_DEBUG && debug_);
    if (need_copy) {
      frame = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
    } else {
      frame = cv_bridge::toCvShare(img_msg, "bgr8")->image;
    }
  } catch (const cv_bridge::Exception& e) {
    FYT_ERROR("armor_detector", "cv_bridge error: {}", e.what());
    return;
  }

  auto t_preprocess_end = std::chrono::steady_clock::now();

  // Detect
  auto results = detector_->detectBatch({frame}, {img_msg->header});
  if (results.empty()) {
    if (config_.runtime.publish_empty) {
      publishEmptyArmors(img_msg->header);
    }
    return;
  }

  auto& fd = results[0];

  auto t_detect_end = std::chrono::steady_clock::now();

  // PnP pose estimation
  std::vector<PoseEstimate> poses;
  if (cam_info_ && pose_estimator_adapter_) {
    poses = pose_estimator_adapter_->estimateBatch(fd.detections, *cam_info_);
  } else {
    poses.resize(fd.detections.size());
  }

  auto t_pose_end = std::chrono::steady_clock::now();

  // Build and publish Armors message
  {
    rm_interfaces::msg::Armors armors_msg;
    armors_msg.header = img_msg->header;

    for (size_t i = 0; i < fd.detections.size(); ++i) {
      rm_interfaces::msg::Armor armor;
      armor.number = fd.detections[i].publish_number;
      armor.type   = fd.detections[i].publish_type;
      armor.distance_to_image_center =
        ArmorPoseEstimatorAdapter::distanceToImageCenter(
          fd.detections[i].center, cam_center_);

      if (poses[i].valid) {
        armor.pose.position.x = poses[i].translation.x();
        armor.pose.position.y = poses[i].translation.y();
        armor.pose.position.z = poses[i].translation.z();
        armor.pose.orientation.x = poses[i].rotation.x();
        armor.pose.orientation.y = poses[i].rotation.y();
        armor.pose.orientation.z = poses[i].rotation.z();
        armor.pose.orientation.w = poses[i].rotation.w();
      }

      armors_msg.armors.push_back(armor);
    }

    armors_pub_->publish(armors_msg);
  }

  // Markers
  if (debug_ && marker_pub_) {
    publishMarkers(fd.detections, poses, img_msg->header);
  }

  // Debug image
  if (debug_) {
    publishDebugImage(frame, fd, poses);
  }

  // Profiler
  if (profiler_) {
    auto t_total = std::chrono::steady_clock::now();
    ProfilerEntry entry = detector_->lastProfiler();
    entry.pose_ms  = std::chrono::duration<double, std::milli>(t_pose_end - t_start).count();
    entry.total_ms = std::chrono::duration<double, std::milli>(t_total - t_start).count();
    profiler_->record(entry);
  }

  FYT_INFO("armor_detector", "Frame processed. Preprocess: {:.2f} ms, Detect: {:.2f} ms, Pose: {:.2f} ms",
           std::chrono::duration<double, std::milli>(t_preprocess_end - t_start).count(),
           std::chrono::duration<double, std::milli>(t_detect_end - t_preprocess_end).count(),
           std::chrono::duration<double, std::milli>(t_pose_end - t_detect_end).count());

}

void ArmorDetectorNNNode::cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& ci_msg)
{
  cam_center_ = cv::Point2f(ci_msg->k[2], ci_msg->k[5]);
  cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*ci_msg);
  FYT_INFO("armor_detector", "Camera info received: {}x{}",
           ci_msg->width, ci_msg->height);
  cam_info_sub_.reset();
}

void ArmorDetectorNNNode::setModeCallback(
    const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
    std::shared_ptr<rm_interfaces::srv::SetMode::Response> response)
{
  response->success = true;
  response->message = "0";

  VisionMode mode = static_cast<VisionMode>(request->mode);

  switch (mode) {
    case VisionMode::AUTO_AIM_RED:
      current_mode_ = DetectMode::RED;
      if (detector_) detector_->setTargetColor(fyt::EnemyColor::RED);
      FYT_INFO("armor_detector", "Mode set to RED");
      break;
    case VisionMode::AUTO_AIM_BLUE:
      current_mode_ = DetectMode::BLUE;
      if (detector_) detector_->setTargetColor(fyt::EnemyColor::BLUE);
      FYT_INFO("armor_detector", "Mode set to BLUE");
      break;
    default:
      current_mode_ = DetectMode::DISABLED;
      FYT_INFO("armor_detector", "Mode set to DISABLED");
      break;
  }
}

void ArmorDetectorNNNode::publishEmptyArmors(const std_msgs::msg::Header& header) {
  rm_interfaces::msg::Armors msg;
  msg.header = header;
  armors_pub_->publish(msg);
}

void ArmorDetectorNNNode::publishMarkers(
    const std::vector<ArmorDetection>& detections,
    const std::vector<PoseEstimate>& poses,
    const std_msgs::msg::Header& header)
{
  visualization_msgs::msg::MarkerArray marker_array;

  for (size_t i = 0; i < detections.size(); ++i) {
    // Armor cube marker
    visualization_msgs::msg::Marker m;
    m.header = header;
    m.ns = "armors_nn";
    m.id = static_cast<int>(i);
    m.action = visualization_msgs::msg::Marker::ADD;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.scale.x = 0.03;
    m.scale.y = 0.15;
    m.scale.z = 0.12;
    m.color.a = 1.0;
    m.lifetime = rclcpp::Duration::from_seconds(0.1);

    if (detections[i].color == fyt::EnemyColor::RED) {
      m.color.r = 1.0;
      m.color.g = 0.0;
      m.color.b = 0.0;
    } else {
      m.color.r = 0.0;
      m.color.g = 0.0;
      m.color.b = 1.0;
    }

    if (poses[i].valid) {
      m.pose.position.x = poses[i].translation.x();
      m.pose.position.y = poses[i].translation.y();
      m.pose.position.z = poses[i].translation.z();
      m.pose.orientation.x = poses[i].rotation.x();
      m.pose.orientation.y = poses[i].rotation.y();
      m.pose.orientation.z = poses[i].rotation.z();
      m.pose.orientation.w = poses[i].rotation.w();
    }

    marker_array.markers.push_back(m);

    // Armor plate outline: 4 line segments connecting keypoints
    if (detections[i].keypoints[0].x > 0 && detections[i].keypoints[3].x > 0) {
      visualization_msgs::msg::Marker outline;
      outline.header = header;
      outline.ns = "armors_nn_outline";
      outline.id = static_cast<int>(i);
      outline.action = visualization_msgs::msg::Marker::ADD;
      outline.type = visualization_msgs::msg::Marker::LINE_STRIP;
      outline.scale.x = 0.005;  // line width
      outline.color.a = 0.8;
      outline.lifetime = rclcpp::Duration::from_seconds(0.1);
      if (detections[i].color == fyt::EnemyColor::RED) {
        outline.color.r = 1.0;
      } else {
        outline.color.b = 1.0;
      }
      // kpt0→kpt1→kpt2→kpt3→kpt0 (xy image coords → 3D at z=1 for RViz)
      for (int k = 0; k <= 4; ++k) {
        const auto& kp = detections[i].keypoints[k % 4];
        geometry_msgs::msg::Point pt;
        pt.x = kp.x;
        pt.y = kp.y;
        pt.z = 0.0;
        outline.points.push_back(pt);
      }
      marker_array.markers.push_back(outline);
    }

    // Text marker
    visualization_msgs::msg::Marker text;
    text.header = header;
    text.ns = "armors_nn_text";
    text.id = static_cast<int>(i);
    text.action = visualization_msgs::msg::Marker::ADD;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.scale.z = 0.1;
    text.color.a = 1.0;
    text.color.r = 1.0;
    text.color.g = 1.0;
    text.color.b = 1.0;
    text.lifetime = rclcpp::Duration::from_seconds(0.1);
    text.text = detections[i].publish_number;
    if (poses[i].valid) {
      text.pose.position.x = poses[i].translation.x();
      text.pose.position.y = poses[i].translation.y() - 0.1;
      text.pose.position.z = poses[i].translation.z();
    }
    marker_array.markers.push_back(text);
  }

  marker_pub_->publish(marker_array);
}

void ArmorDetectorNNNode::publishDebugImage(
    const cv::Mat& frame,
    const FrameDetections& fd,
    const std::vector<PoseEstimate>& /*poses*/)
{
  cv::Mat debug_img = frame.clone();

  debug_drawer_->drawDetections(debug_img, fd.detections, true);

  if (profiler_) {
    double fps = profiler_->avgFPS();
    double latency = profiler_->avgTotalMs();
    auto bi = detector_->backendInfo();
    debug_drawer_->drawProfiler(debug_img, fps, latency,
                                bi.backend_name, bi.precision);
    debug_drawer_->drawArmorsCount(debug_img, static_cast<int>(fd.detections.size()));
  }

  auto msg = cv_bridge::CvImage(fd.header, "bgr8", debug_img).toImageMsg();
  result_img_pub_.publish(*msg);
}

void ArmorDetectorNNNode::createDebugPublishers() {
  result_img_pub_ = image_transport::create_publisher(this, "armor_detector/result_img");
}

void ArmorDetectorNNNode::destroyDebugPublishers() {
  result_img_pub_.shutdown();
}

rcl_interfaces::msg::SetParametersResult
ArmorDetectorNNNode::onSetParameters(const std::vector<rclcpp::Parameter>& params) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto& p : params) {
    if (p.get_name() == "debug") {
      debug_ = p.as_bool();
      debug_ ? createDebugPublishers() : destroyDebugPublishers();
    }
  }

  return result;
}

}  // namespace fyt::auto_aim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::auto_aim::ArmorDetectorNNNode)
