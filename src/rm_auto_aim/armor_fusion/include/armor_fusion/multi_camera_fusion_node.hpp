#ifndef ARMOR_FUSION__MULTI_CAMERA_FUSION_NODE_HPP_
#define ARMOR_FUSION__MULTI_CAMERA_FUSION_NODE_HPP_

#include <deque>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "armor_fusion/measurement_types.hpp"
#include "rm_interfaces/msg/armors.hpp"

namespace fyt::auto_aim {

class MultiCameraFusionNode : public rclcpp::Node {
public:
  explicit MultiCameraFusionNode(const rclcpp::NodeOptions & options);

private:
  using ArmorsMsg = rm_interfaces::msg::Armors;

  struct CameraPoseInTarget {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation_target_from_camera{Eigen::Quaterniond::Identity()};
  };

  struct CameraFovInFrame {
    double half_horizontal_rad{0.6981317007977318};
    double half_vertical_rad{0.5235987755982988};
  };

  // Subscriber callback only does bounded buffering.
  void armorsCallback(const ArmorsMsg::SharedPtr msg, const std::string & topic);

  // Timer callback: select fresh inputs -> transform -> cluster/fuse -> publish.
  void processAndPublish();

  // Select the latest fresh message from each camera topic.
  std::vector<ArmorsMsg::SharedPtr> selectFreshInputs(
    const rclcpp::Time & now,
    builtin_interfaces::msg::Time & fusion_stamp,
    bool & has_fusion_stamp);

  // Build per-camera poses at current fusion cycle for FOV visibility checks.
  std::unordered_map<std::string, CameraPoseInTarget> buildCameraPoseMap(
    const std::vector<ArmorsMsg::SharedPtr> & selected_msgs);

  // CameraInfo callback for dynamic FOV estimation.
  void cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg,
    const std::string & camera_info_topic);

  // Infer a camera_info topic from one detector topic.
  static std::string inferCameraInfoTopic(const std::string & camera_topic);

  // Normalize frame IDs so both "/camera_optical" and "camera_optical" map to one key.
  static std::string normalizeFrameId(const std::string & frame_id);

  // Fetch per-frame FOV from camera_info, or fallback to configured defaults.
  CameraFovInFrame getCameraFovForFrame(const std::string & source_frame) const;

  // Check whether a target-frame point is inside one camera frustum.
  bool isTargetVisibleFromCamera(
    const Eigen::Vector3d & target_position,
    const std::string & source_frame,
    const std::unordered_map<std::string, CameraPoseInTarget> & camera_poses) const;

  static builtin_interfaces::msg::Time toBuiltinTime(const rclcpp::Time & time);

  // Input/output topics.
  std::vector<std::string> camera_topics_;
  std::string target_frame_;
  std::string output_topic_;
  std::string marker_topic_;

  // Fusion parameters.
  double dbscan_eps_{0.3};
  int dbscan_min_samples_{1};
  double max_cluster_noise_{0.5};
  double publish_rate_{100.0};
  bool enable_visualization_{true};
  bool console_debug_{false};
  int measurement_buffer_size_{10};
  double sync_timeout_{0.05};
  double tf_lookup_timeout_{0.05};
  bool enable_strict_fov_gate_{true};
  bool use_camera_info_fov_{true};
  double camera_half_horizontal_fov_rad_{0.6981317007977318};
  double camera_half_vertical_fov_rad_{0.5235987755982988};
  double fov_margin_rad_{0.0};

  std::vector<std::string> camera_info_topics_;

  // TF resources.
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ROS interfaces.
  rclcpp::Publisher<ArmorsMsg>::SharedPtr fused_armors_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::vector<rclcpp::Subscription<ArmorsMsg>::SharedPtr> subscribers_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_subscribers_;

  std::unordered_map<std::string, CameraFovInFrame> camera_fov_by_frame_;
  mutable std::mutex fov_mutex_;

  // Per-topic measurement buffers.
  std::unordered_map<std::string, std::deque<ArmorsMsg::SharedPtr>> measurement_buffers_;
  mutable std::mutex buffer_mutex_;

  // Runtime stats.
  rclcpp::TimerBase::SharedPtr timer_;
  size_t frame_count_{0};
  rclcpp::Time last_log_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace fyt::auto_aim

#endif  // ARMOR_FUSION__MULTI_CAMERA_FUSION_NODE_HPP_
