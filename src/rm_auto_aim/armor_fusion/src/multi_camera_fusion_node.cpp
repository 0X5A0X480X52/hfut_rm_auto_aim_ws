#include "armor_fusion/multi_camera_fusion_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <functional>
#include <unordered_set>

#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/exceptions.h>

#include "armor_fusion/clustering_utils.hpp"
#include "armor_fusion/fusion_utils.hpp"
#include "armor_fusion/transform_utils.hpp"
#include "armor_fusion/visualization_utils.hpp"

namespace fyt::auto_aim {

namespace {
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kMinDepth = 1e-6;
constexpr char kArmorTopicSuffix[] = "/armor_detector/armors";
constexpr char kArmorDetectorNamespaceSuffix[] = "/armor_detector";
}  // namespace

std::string MultiCameraFusionNode::normalizeFrameId(const std::string & frame_id)
{
  if (!frame_id.empty() && frame_id.front() == '/') {
    return frame_id.substr(1);
  }
  return frame_id;
}

std::string MultiCameraFusionNode::inferCameraInfoTopic(const std::string & camera_topic)
{
  if (camera_topic.empty()) {
    return "/camera_info";
  }

  std::string inferred = camera_topic;
  const std::size_t armor_pos = inferred.rfind(kArmorTopicSuffix);
  if (armor_pos != std::string::npos) {
    inferred.erase(armor_pos);
  } else {
    const std::size_t slash_pos = inferred.rfind('/');
    inferred = (slash_pos == std::string::npos) ? std::string() : inferred.substr(0, slash_pos);
    const std::size_t detector_pos = inferred.rfind(kArmorDetectorNamespaceSuffix);
    if (detector_pos != std::string::npos) {
      inferred.erase(detector_pos);
    }
  }

  if (inferred.empty()) {
    return "/camera_info";
  }

  if (inferred.front() != '/') {
    inferred = "/" + inferred;
  }
  return inferred + "/camera_info";
}

MultiCameraFusionNode::MultiCameraFusionNode(const rclcpp::NodeOptions & options)
: Node("multi_camera_fusion", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  // Topics and frame settings.
  camera_topics_ = this->declare_parameter<std::vector<std::string>>(
    "camera_topics",
    std::vector<std::string>{"/camera1/armor_detector/armors", "/camera2/armor_detector/armors"});
  camera_info_topics_ = this->declare_parameter<std::vector<std::string>>(
    "camera_info_topics",
    std::vector<std::string>{});
  target_frame_ = this->declare_parameter<std::string>("target_frame", "base_link");
  output_topic_ = this->declare_parameter<std::string>("output_topic", "/armor_detector/armors");
  marker_topic_ = this->declare_parameter<std::string>("marker_topic", "armor_fusion/markers");

  // Fusion behavior settings.
  dbscan_eps_ = this->declare_parameter<double>("dbscan_eps", 0.3);
  dbscan_min_samples_ = this->declare_parameter<int>("dbscan_min_samples", 1);
  max_cluster_noise_ = this->declare_parameter<double>("max_cluster_noise", 0.5);
  publish_rate_ = this->declare_parameter<double>("publish_rate", 100.0);
  enable_visualization_ = this->declare_parameter<bool>("enable_visualization", true);
  console_debug_ = this->declare_parameter<bool>("console_debug", false);
  measurement_buffer_size_ = this->declare_parameter<int>("measurement_buffer_size", 10);
  sync_timeout_ = this->declare_parameter<double>("sync_timeout", 0.05);
  tf_lookup_timeout_ = this->declare_parameter<double>("tf_lookup_timeout", 0.05);
  enable_strict_fov_gate_ = this->declare_parameter<bool>("enable_strict_fov_gate", true);
  use_camera_info_fov_ = this->declare_parameter<bool>("use_camera_info_fov", true);
  const double camera_horizontal_fov_deg = this->declare_parameter<double>(
    "camera_horizontal_fov_deg", 80.0);
  const double camera_vertical_fov_deg = this->declare_parameter<double>(
    "camera_vertical_fov_deg", 60.0);
  const double fov_margin_deg = this->declare_parameter<double>("fov_margin_deg", 2.0);

  camera_half_horizontal_fov_rad_ =
    0.5 * std::max(1.0, camera_horizontal_fov_deg) * kDegToRad;
  camera_half_vertical_fov_rad_ =
    0.5 * std::max(1.0, camera_vertical_fov_deg) * kDegToRad;
  fov_margin_rad_ = std::max(0.0, fov_margin_deg) * kDegToRad;

  if (camera_topics_.empty()) {
    RCLCPP_WARN(this->get_logger(), "camera_topics is empty, using default detector topics.");
    camera_topics_ = {"/camera1/armor_detector/armors", "/camera2/armor_detector/armors"};
  }

  if (std::find(camera_topics_.begin(), camera_topics_.end(), output_topic_) != camera_topics_.end()) {
    RCLCPP_WARN(
      this->get_logger(),
      "output_topic [%s] is also configured as input, this can create a feedback loop.",
      output_topic_.c_str());
  }

  std::vector<std::string> resolved_camera_info_topics;
  resolved_camera_info_topics.reserve(camera_topics_.size());
  if (!camera_info_topics_.empty() && camera_info_topics_.size() != camera_topics_.size()) {
    RCLCPP_WARN(
      this->get_logger(),
      "camera_info_topics size (%zu) does not match camera_topics size (%zu), missing entries will be inferred.",
      camera_info_topics_.size(),
      camera_topics_.size());
  }
  for (std::size_t i = 0; i < camera_topics_.size(); ++i) {
    if (i < camera_info_topics_.size() && !camera_info_topics_[i].empty()) {
      resolved_camera_info_topics.push_back(camera_info_topics_[i]);
    } else {
      resolved_camera_info_topics.push_back(inferCameraInfoTopic(camera_topics_[i]));
    }
  }
  camera_info_topics_ = resolved_camera_info_topics;

  // Subscribers and per-camera buffers.
  const auto qos = rclcpp::SensorDataQoS();
  for (const auto & topic : camera_topics_) {
    auto sub = this->create_subscription<ArmorsMsg>(
      topic,
      qos,
      [this, topic](const ArmorsMsg::SharedPtr msg) { this->armorsCallback(msg, topic); });
    subscribers_.push_back(sub);
    measurement_buffers_[topic] = {};
    RCLCPP_INFO(this->get_logger(), "Subscribed camera topic: %s", topic.c_str());
  }

  if (use_camera_info_fov_) {
    camera_info_subscribers_.reserve(camera_info_topics_.size());
    for (const auto & topic : camera_info_topics_) {
      auto sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        topic,
        qos,
        [this, topic](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
          this->cameraInfoCallback(msg, topic);
        });
      camera_info_subscribers_.push_back(sub);
      RCLCPP_INFO(this->get_logger(), "Subscribed camera_info topic: %s", topic.c_str());
    }
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "Camera-info-driven FOV disabled, always using fallback HFOV/VFOV parameters.");
  }

  fused_armors_pub_ = this->create_publisher<ArmorsMsg>(output_topic_, qos);
  if (enable_visualization_) {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
  }

  // Timer-driven processing decouples input jitter from output cadence.
  const double safe_rate = std::max(1.0, publish_rate_);
  const auto period = std::chrono::duration<double>(1.0 / safe_rate);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&MultiCameraFusionNode::processAndPublish, this));

  last_log_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "MultiCameraFusionNode (C++) started.");
  RCLCPP_INFO(this->get_logger(), "Output topic fixed to detector-compatible topic: %s", output_topic_.c_str());
  RCLCPP_INFO(
    this->get_logger(),
    "Strict FOV gate: %s (HFOV=%.1f deg, VFOV=%.1f deg, margin=%.1f deg)",
    enable_strict_fov_gate_ ? "enabled" : "disabled",
    camera_half_horizontal_fov_rad_ * 2.0 / kDegToRad,
    camera_half_vertical_fov_rad_ * 2.0 / kDegToRad,
    fov_margin_rad_ / kDegToRad);
  RCLCPP_INFO(
    this->get_logger(),
    "FOV source mode: %s (fallback to configured HFOV/VFOV when camera_info is unavailable)",
    use_camera_info_fov_ ? "camera_info" : "fixed-parameter");
}

void MultiCameraFusionNode::armorsCallback(const ArmorsMsg::SharedPtr msg, const std::string & topic)
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  auto & queue = measurement_buffers_[topic];
  queue.push_back(msg);
  while (static_cast<int>(queue.size()) > measurement_buffer_size_) {
    queue.pop_front();
  }
}

std::vector<MultiCameraFusionNode::ArmorsMsg::SharedPtr> MultiCameraFusionNode::selectFreshInputs(
  const rclcpp::Time & now,
  builtin_interfaces::msg::Time & fusion_stamp,
  bool & has_fusion_stamp)
{
  std::vector<ArmorsMsg::SharedPtr> selected_msgs;

  std::lock_guard<std::mutex> lock(buffer_mutex_);
  selected_msgs.reserve(measurement_buffers_.size());
  for (const auto & item : measurement_buffers_) {
    const auto & queue = item.second;
    if (queue.empty()) {
      continue;
    }

    const auto & latest_msg = queue.back();
    const double age = (now - rclcpp::Time(latest_msg->header.stamp)).seconds();
    if (age > sync_timeout_) {
      continue;
    }

    selected_msgs.push_back(latest_msg);
    if (!has_fusion_stamp || rclcpp::Time(latest_msg->header.stamp) > rclcpp::Time(fusion_stamp)) {
      fusion_stamp = latest_msg->header.stamp;
      has_fusion_stamp = true;
    }
  }

  return selected_msgs;
}

std::unordered_map<std::string, MultiCameraFusionNode::CameraPoseInTarget>
MultiCameraFusionNode::buildCameraPoseMap(const std::vector<ArmorsMsg::SharedPtr> & selected_msgs)
{
  std::unordered_map<std::string, builtin_interfaces::msg::Time> latest_frame_stamp;
  latest_frame_stamp.reserve(selected_msgs.size());
  for (const auto & msg : selected_msgs) {
    const std::string & frame = msg->header.frame_id;
    const auto stamp_it = latest_frame_stamp.find(frame);
    if (stamp_it == latest_frame_stamp.end() ||
      rclcpp::Time(msg->header.stamp) > rclcpp::Time(stamp_it->second))
    {
      latest_frame_stamp[frame] = msg->header.stamp;
    }
  }

  std::unordered_map<std::string, CameraPoseInTarget> camera_poses;
  camera_poses.reserve(latest_frame_stamp.size());

  for (const auto & item : latest_frame_stamp) {
    try {
      const auto tf_msg = tf_buffer_.lookupTransform(
        target_frame_,
        item.first,
        rclcpp::Time(item.second),
        rclcpp::Duration::from_seconds(tf_lookup_timeout_));

      CameraPoseInTarget pose;
      pose.position = Eigen::Vector3d(
        tf_msg.transform.translation.x,
        tf_msg.transform.translation.y,
        tf_msg.transform.translation.z);
      pose.orientation_target_from_camera = Eigen::Quaterniond(
        tf_msg.transform.rotation.w,
        tf_msg.transform.rotation.x,
        tf_msg.transform.rotation.y,
        tf_msg.transform.rotation.z);
      pose.orientation_target_from_camera.normalize();
      camera_poses[item.first] = pose;
    } catch (const tf2::TransformException & ex) {
      if (console_debug_) {
        RCLCPP_WARN(
          this->get_logger(),
          "FOV gate pose lookup failed for [%s] -> [%s]: %s",
          item.first.c_str(),
          target_frame_.c_str(),
          ex.what());
      }
    }
  }

  return camera_poses;
}

void MultiCameraFusionNode::cameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::SharedPtr msg,
  const std::string & camera_info_topic)
{
  const std::string frame_key = normalizeFrameId(msg->header.frame_id);
  const double fx = msg->k[0];
  const double fy = msg->k[4];
  if (frame_key.empty() || msg->width == 0U || msg->height == 0U || fx <= kMinDepth || fy <= kMinDepth) {
    if (console_debug_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid camera_info for topic [%s] (frame=[%s], width=%u, height=%u, fx=%.6f, fy=%.6f), fallback FOV will be used.",
        camera_info_topic.c_str(),
        msg->header.frame_id.c_str(),
        msg->width,
        msg->height,
        fx,
        fy);
    }
    return;
  }

  CameraFovInFrame fov;
  fov.half_horizontal_rad = std::atan(static_cast<double>(msg->width) / (2.0 * fx));
  fov.half_vertical_rad = std::atan(static_cast<double>(msg->height) / (2.0 * fy));

  {
    std::lock_guard<std::mutex> lock(fov_mutex_);
    camera_fov_by_frame_[frame_key] = fov;
  }

  if (console_debug_) {
    RCLCPP_INFO(
      this->get_logger(),
      "Updated FOV from [%s], frame [%s]: HFOV=%.2f deg, VFOV=%.2f deg",
      camera_info_topic.c_str(),
      frame_key.c_str(),
      (2.0 * fov.half_horizontal_rad) / kDegToRad,
      (2.0 * fov.half_vertical_rad) / kDegToRad);
  }
}

MultiCameraFusionNode::CameraFovInFrame MultiCameraFusionNode::getCameraFovForFrame(
  const std::string & source_frame) const
{
  CameraFovInFrame fallback;
  fallback.half_horizontal_rad = camera_half_horizontal_fov_rad_;
  fallback.half_vertical_rad = camera_half_vertical_fov_rad_;

  if (!use_camera_info_fov_) {
    return fallback;
  }

  const std::string frame_key = normalizeFrameId(source_frame);
  if (frame_key.empty()) {
    return fallback;
  }

  std::lock_guard<std::mutex> lock(fov_mutex_);
  const auto it = camera_fov_by_frame_.find(frame_key);
  if (it != camera_fov_by_frame_.end()) {
    return it->second;
  }

  return fallback;
}

bool MultiCameraFusionNode::isTargetVisibleFromCamera(
  const Eigen::Vector3d & target_position,
  const std::string & source_frame,
  const std::unordered_map<std::string, CameraPoseInTarget> & camera_poses) const
{
  const auto pose_it = camera_poses.find(source_frame);
  if (pose_it == camera_poses.end()) {
    // Strict mode: missing camera pose means visibility is unknown -> reject cross-camera merge.
    return false;
  }

  const CameraPoseInTarget & pose = pose_it->second;
  const Eigen::Vector3d target_vector_in_target = target_position - pose.position;
  const Eigen::Vector3d target_vector_in_camera =
    pose.orientation_target_from_camera.inverse() * target_vector_in_target;

  if (target_vector_in_camera.z() <= kMinDepth) {
    return false;
  }

  const double horizontal_angle = std::atan2(target_vector_in_camera.x(), target_vector_in_camera.z());
  const double vertical_angle = std::atan2(target_vector_in_camera.y(), target_vector_in_camera.z());
  const auto camera_fov = getCameraFovForFrame(source_frame);
  const double horizontal_limit = camera_fov.half_horizontal_rad + fov_margin_rad_;
  const double vertical_limit = camera_fov.half_vertical_rad + fov_margin_rad_;

  return std::abs(horizontal_angle) <= horizontal_limit &&
         std::abs(vertical_angle) <= vertical_limit;
}

builtin_interfaces::msg::Time MultiCameraFusionNode::toBuiltinTime(const rclcpp::Time & time)
{
  const int64_t ns = time.nanoseconds();
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
  stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return stamp;
}

void MultiCameraFusionNode::processAndPublish()
{
  const rclcpp::Time now = this->now();
  const builtin_interfaces::msg::Time now_stamp = toBuiltinTime(now);
  builtin_interfaces::msg::Time fusion_stamp = now_stamp;
  bool has_fusion_stamp = false;

  // Step 1: pick one fresh message from each camera buffer.
  const auto selected_msgs = selectFreshInputs(now, fusion_stamp, has_fusion_stamp);

  // Step 2: transform all detections to target_frame.
  std::vector<ArmorMeasurement> all_measurements;
  all_measurements.reserve(selected_msgs.size() * 4U);
  for (const auto & msg : selected_msgs) {
    for (const auto & armor : msg->armors) {
      auto measurement = transformMeasurement(
        tf_buffer_,
        armor,
        msg->header.frame_id,
        target_frame_,
        msg->header.stamp,
        tf_lookup_timeout_,
        console_debug_,
        this->get_logger());
      if (measurement.has_value()) {
        all_measurements.push_back(measurement.value());
      }
    }
  }

  // Step 3: prepare output header using synchronized stamp.
  ArmorsMsg output_msg;
  output_msg.header.frame_id = target_frame_;
  output_msg.header.stamp = has_fusion_stamp ? fusion_stamp : now_stamp;

  if (all_measurements.empty()) {
    fused_armors_pub_->publish(output_msg);
    return;
  }

  // Step 4: cluster close measurements and merge split clusters.
  const auto camera_poses = enable_strict_fov_gate_
    ? buildCameraPoseMap(selected_msgs)
    : std::unordered_map<std::string, CameraPoseInTarget>{};

  const ClusterMergeConstraint merge_constraint =
    [this, &camera_poses](
    const ArmorMeasurement & candidate,
    const std::vector<ArmorMeasurement> & cluster,
    const Eigen::Vector3d & candidate_center) {
      if (!enable_strict_fov_gate_) {
        return true;
      }

      for (const auto & existing : cluster) {
        if (existing.source_frame == candidate.source_frame) {
          continue;
        }

        if (!isTargetVisibleFromCamera(candidate_center, candidate.source_frame, camera_poses) ||
          !isTargetVisibleFromCamera(candidate_center, existing.source_frame, camera_poses))
        {
          return false;
        }
      }
      return true;
    };

  auto clusters = clusterMeasurements(
    all_measurements,
    dbscan_eps_,
    dbscan_min_samples_,
    merge_constraint);

  const ClusterPairMergeConstraint pair_merge_constraint =
    [this, &camera_poses](
    const std::vector<ArmorMeasurement> & cluster_a,
    const std::vector<ArmorMeasurement> & cluster_b,
    const Eigen::Vector3d & merged_center) {
      if (!enable_strict_fov_gate_) {
        return true;
      }

      std::unordered_set<std::string> source_frames;
      source_frames.reserve(cluster_a.size() + cluster_b.size());
      for (const auto & m : cluster_a) {
        source_frames.insert(m.source_frame);
      }
      for (const auto & m : cluster_b) {
        source_frames.insert(m.source_frame);
      }

      if (source_frames.size() <= 1U) {
        return true;
      }

      for (const auto & frame : source_frames) {
        if (!isTargetVisibleFromCamera(merged_center, frame, camera_poses)) {
          return false;
        }
      }
      return true;
    };

  clusters = mergeCloseClusters(clusters, max_cluster_noise_, pair_merge_constraint);

  // Step 5: fuse each cluster into one armor output.
  output_msg.armors.reserve(clusters.size());
  for (const auto & cluster : clusters) {
    output_msg.armors.push_back(fuseCluster(cluster));
  }

  fused_armors_pub_->publish(output_msg);

  // Step 6: optional RViz markers for debugging.
  if (enable_visualization_ && marker_pub_) {
    const auto marker_array = buildVisualizationMarkers(
      clusters,
      output_msg.armors,
      target_frame_,
      output_msg.header.stamp);
    marker_pub_->publish(marker_array);
  }

  frame_count_++;
  if ((now - last_log_time_).seconds() > 5.0) {
    RCLCPP_INFO(
      this->get_logger(),
      "Processed %zu frames, measurements: %zu, fused targets: %zu",
      frame_count_,
      all_measurements.size(),
      output_msg.armors.size());
    frame_count_ = 0;
    last_log_time_ = now;
  }
}

}  // namespace fyt::auto_aim

RCLCPP_COMPONENTS_REGISTER_NODE(fyt::auto_aim::MultiCameraFusionNode)
