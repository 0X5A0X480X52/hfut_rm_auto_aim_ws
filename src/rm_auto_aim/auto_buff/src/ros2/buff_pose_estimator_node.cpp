#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "opencv2/calib3d.hpp"
#include "opencv2/core.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/rune_target_array.hpp"
#include "rm_interfaces/msg/rune_target.hpp"
#include "rm_interfaces/msg/serial_receive_data.hpp"
#include "rm_interfaces/msg/tracked_robot.hpp"
#include "rm_interfaces/srv/set_mode.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace fyt::auto_aim::auto_buff
{

class BuffPoseEstimatorNode : public rclcpp::Node
{
public:
  BuffPoseEstimatorNode()
  : Node("buff_pose_estimator_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/rune_target");
    input_array_topic_ = declare_parameter<std::string>("input_array_topic", "/rune_targets");
    output_topic_ = declare_parameter<std::string>("output_topic", "/auto_buff/tracked_robot");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/camera_info");
    serial_state_topic_ = declare_parameter<std::string>("serial_state_topic", "/serial_receive_data");
    target_frame_ = declare_parameter<std::string>("target_frame", "odom");
    big_buff_id_ = declare_parameter<std::string>("big_buff_id", "big_buff");
    small_buff_id_ = declare_parameter<std::string>("small_buff_id", "small_buff");
    tf_timeout_s_ = declare_parameter<double>("tf_timeout_s", 0.02);
    min_dt_s_ = declare_parameter<double>("min_dt_s", 1e-3);
    pnp_fallback_distance_m_ = declare_parameter<double>("pnp_fallback_distance_m", 6.0);
    mode_override_enable_ = declare_parameter<bool>("mode_override_enable", true);
    rune_radius_m_ = declare_parameter<double>("rune_radius_m", 0.7);
    debug_markers_enable_ = declare_parameter<bool>("debug_markers_enable", false);
    marker_topic_ = declare_parameter<std::string>("marker_topic", "/auto_buff/debug/markers");
    marker_publish_rate_hz_ = declare_parameter<double>("marker_publish_rate_hz", 10.0);

    object_points_ = {
      cv::Point3f(0.0f, 0.0f, 0.0f),
      cv::Point3f(0.0f, -0.186f, 0.5415f),
      cv::Point3f(0.0f, -0.160f, 0.8585f),
      cv::Point3f(0.0f, 0.160f, 0.8585f),
      cv::Point3f(0.0f, 0.186f, 0.5415f),
    };
    hit_point_obj_ = cv::Point3f(0.0f, 0.0f, static_cast<float>(rune_radius_m_));

    pub_ = create_publisher<rm_interfaces::msg::TrackedRobot>(output_topic_, rclcpp::SensorDataQoS());
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_, rclcpp::SensorDataQoS());
    marker_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(1.0, marker_publish_rate_hz_))),
      std::bind(&BuffPoseEstimatorNode::onMarkerTimer, this));
    sub_ = create_subscription<rm_interfaces::msg::RuneTarget>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&BuffPoseEstimatorNode::onRuneTarget, this, std::placeholders::_1));
    sub_array_ = create_subscription<rm_interfaces::msg::RuneTargetArray>(
      input_array_topic_, rclcpp::SensorDataQoS(),
      std::bind(&BuffPoseEstimatorNode::onRuneTargets, this, std::placeholders::_1));
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      std::bind(&BuffPoseEstimatorNode::onCameraInfo, this, std::placeholders::_1));
    serial_state_sub_ = create_subscription<rm_interfaces::msg::SerialReceiveData>(
      serial_state_topic_, rclcpp::SensorDataQoS(),
      std::bind(&BuffPoseEstimatorNode::onSerialState, this, std::placeholders::_1));
    set_mode_srv_ = create_service<rm_interfaces::srv::SetMode>(
      "~/set_mode",
      std::bind(&BuffPoseEstimatorNode::onSetMode, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (!msg || msg->k.size() != 9 || msg->d.empty()) {
      return;
    }
    cv::Mat K = cv::Mat::zeros(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i) {
      K.at<double>(i / 3, i % 3) = msg->k[static_cast<size_t>(i)];
    }
    cv::Mat D = cv::Mat::zeros(static_cast<int>(msg->d.size()), 1, CV_64F);
    for (size_t i = 0; i < msg->d.size(); ++i) {
      D.at<double>(static_cast<int>(i), 0) = msg->d[i];
    }
    cam_matrix_ = K;
    dist_coeffs_ = D;
    has_camera_info_ = true;
  }

  void onSerialState(const rm_interfaces::msg::SerialReceiveData::SharedPtr msg)
  {
    if (msg) {
      last_mode_ = static_cast<int>(msg->mode);
    }
  }

  void onSetMode(
    const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
    std::shared_ptr<rm_interfaces::srv::SetMode::Response> response)
  {
    response->success = true;
    if (!request) {
      response->success = false;
      response->message = "null request";
      return;
    }
    last_mode_ = static_cast<int>(request->mode);
    response->message = "mode cached for rune id resolution";
  }

  bool solvePnPInCamera(
    const rm_interfaces::msg::RuneTarget & msg,
    cv::Point3d & hit_cam) const
  {
    if (!has_camera_info_ || msg.pts.size() != 5) {
      return false;
    }
    std::vector<cv::Point2f> image_points;
    image_points.reserve(5);
    for (const auto & p : msg.pts) {
      image_points.emplace_back(static_cast<float>(p.x), static_cast<float>(p.y));
    }

    cv::Mat rvec, tvec;
    const bool ok = cv::solvePnP(
      object_points_, image_points, cam_matrix_, dist_coeffs_, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    if (!ok || tvec.total() < 3) {
      return false;
    }

    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Mat hit_obj = (cv::Mat_<double>(3, 1) << hit_point_obj_.x, hit_point_obj_.y, hit_point_obj_.z);
    cv::Mat hit = R * hit_obj + tvec;
    hit_cam = cv::Point3d(hit.at<double>(0, 0), hit.at<double>(1, 0), hit.at<double>(2, 0));
    return std::isfinite(hit_cam.x) && std::isfinite(hit_cam.y) && std::isfinite(hit_cam.z);
  }

  std::string resolveRuneIdFromMode(bool is_big_rune) const
  {
    if (mode_override_enable_) {
      if (last_mode_ == 2 || last_mode_ == 3) return small_buff_id_;
      if (last_mode_ == 4 || last_mode_ == 5) return big_buff_id_;
    }
    return is_big_rune ? big_buff_id_ : small_buff_id_;
  }

  void onRuneTarget(const rm_interfaces::msg::RuneTarget::SharedPtr msg)
  {
    if (!msg || msg->is_lost) {
      return;
    }
    rm_interfaces::msg::RuneTargetArray wrapped;
    wrapped.header = msg->header;
    wrapped.targets.push_back(*msg);
    processRuneTargets(wrapped);
  }

  void onRuneTargets(const rm_interfaces::msg::RuneTargetArray::SharedPtr msg)
  {
    if (!msg || msg->targets.empty()) {
      return;
    }
    processRuneTargets(*msg);
  }

  bool runeToWorld(
    const rm_interfaces::msg::RuneTarget & msg,
    geometry_msgs::msg::PointStamped & p_world)
  {
    geometry_msgs::msg::PointStamped p_cam;
    p_cam.header = msg.header;
    cv::Point3d hit_cam;
    if (solvePnPInCamera(msg, hit_cam)) {
      p_cam.point.x = hit_cam.x;
      p_cam.point.y = hit_cam.y;
      p_cam.point.z = hit_cam.z;
    } else {
      double u = 0.0, v = 0.0;
      for (const auto & p : msg.pts) {
        u += p.x;
        v += p.y;
      }
      u /= static_cast<double>(msg.pts.size());
      v /= static_cast<double>(msg.pts.size());
      const double fx = cam_matrix_.empty() ? 1200.0 : cam_matrix_.at<double>(0, 0);
      const double fy = cam_matrix_.empty() ? 1200.0 : cam_matrix_.at<double>(1, 1);
      const double cx = cam_matrix_.empty() ? 720.0 : cam_matrix_.at<double>(0, 2);
      const double cy = cam_matrix_.empty() ? 540.0 : cam_matrix_.at<double>(1, 2);
      const double z_cam = pnp_fallback_distance_m_;
      p_cam.point.x = z_cam;
      p_cam.point.y = (u - cx) * z_cam / std::max(1.0, fx);
      p_cam.point.z = (v - cy) * z_cam / std::max(1.0, fy);
    }

    try {
      p_world = tf_buffer_.transform(p_cam, target_frame_, tf2::durationFromSec(tf_timeout_s_));
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "buff pose tf failed: %s", e.what());
      return false;
    }
    return true;
  }

  void processRuneTargets(const rm_interfaces::msg::RuneTargetArray & msg)
  {
    std::vector<geometry_msgs::msg::PointStamped> world_points;
    std::vector<rm_interfaces::msg::RuneTarget> valid_targets;
    world_points.reserve(msg.targets.size());
    valid_targets.reserve(msg.targets.size());
    for (const auto & t : msg.targets) {
      if (t.is_lost) {
        continue;
      }
      geometry_msgs::msg::PointStamped p_world;
      if (!runeToWorld(t, p_world)) {
        continue;
      }
      valid_targets.push_back(t);
      world_points.push_back(p_world);
    }
    if (world_points.empty()) {
      return;
    }

    size_t selected_idx = 0;
    if (last_stamp_.nanoseconds() > 0) {
      double best_dist = std::numeric_limits<double>::max();
      for (size_t i = 0; i < world_points.size(); ++i) {
        const auto & p = world_points[i].point;
        const double d2 =
          (p.x - last_pos_.x) * (p.x - last_pos_.x) +
          (p.y - last_pos_.y) * (p.y - last_pos_.y) +
          (p.z - last_pos_.z) * (p.z - last_pos_.z);
        if (d2 < best_dist) {
          best_dist = d2;
          selected_idx = i;
        }
      }
    }
    const auto & p_world = world_points[selected_idx];
    const auto & selected_target = valid_targets[selected_idx];

    rm_interfaces::msg::TrackedRobot out;
    out.header = msg.header;
    out.header.frame_id = target_frame_;
    out.robot_id = resolveRuneIdFromMode(selected_target.is_big_rune);
    out.robot_type = rm_interfaces::msg::TrackedRobot::UNKNOWN;
    out.track_state = rm_interfaces::msg::TrackedRobot::TRACKING;
    out.full_state_valid = true;
    out.center_pose.position = p_world.point;
    out.center_pose.orientation.w = 1.0;
    out.center_twist.linear.x = 0.0;
    out.center_twist.linear.y = 0.0;
    out.center_twist.linear.z = 0.0;
    out.center_accel.linear.x = 0.0;
    out.center_accel.linear.y = 0.0;
    out.center_accel.linear.z = 0.0;

    if (last_stamp_.nanoseconds() > 0 && (selected_target.is_big_rune == last_is_big_rune_)) {
      const double dt = std::max(min_dt_s_, (rclcpp::Time(msg.header.stamp) - last_stamp_).seconds());
      out.center_twist.linear.x = (p_world.point.x - last_pos_.x) / dt;
      out.center_twist.linear.y = (p_world.point.y - last_pos_.y) / dt;
      out.center_twist.linear.z = (p_world.point.z - last_pos_.z) / dt;
    }

    out.center_position = out.center_pose.position;
    out.center_velocity = out.center_twist.linear;
    out.center_acceleration = out.center_accel.linear;
    out.yaw = 0.0;
    out.yaw_velocity = 0.0;
    out.yaw_acceleration = 0.0;
    out.representation_mode = rm_interfaces::msg::TrackedRobot::REP_AMBIGUOUS_SINGLE_ARMOR;
    out.num_armors = 1;
    out.confidence = 0.8;
    out.is_visible = true;
    out.visible_armor_count = 1;
    geometry_msgs::msg::Pose zero_offset;
    zero_offset.orientation.w = 1.0;
    out.armors_offset.push_back(zero_offset);

    pub_->publish(out);
    last_world_points_ = world_points;
    last_selected_world_index_ = selected_idx;
    last_debug_target_ = out;
    has_last_debug_target_ = true;
    publishDebugMarkers(last_debug_target_);
    last_stamp_ = rclcpp::Time(msg.header.stamp);
    last_pos_ = p_world.point;
    last_is_big_rune_ = selected_target.is_big_rune;
  }

  void publishDebugMarkers(const rm_interfaces::msg::TrackedRobot & out)
  {
    if (!debug_markers_enable_) {
      return;
    }
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker pos_marker;
    pos_marker.header = out.header;
    pos_marker.ns = "auto_buff";
    pos_marker.id = 0;
    pos_marker.type = visualization_msgs::msg::Marker::SPHERE;
    pos_marker.action = visualization_msgs::msg::Marker::ADD;
    pos_marker.pose = out.center_pose;
    pos_marker.scale.x = 0.15;
    pos_marker.scale.y = 0.15;
    pos_marker.scale.z = 0.15;
    pos_marker.color.a = 1.0;
    pos_marker.color.r = 0.2F;
    pos_marker.color.g = 1.0F;
    pos_marker.color.b = 0.2F;

    visualization_msgs::msg::Marker vel_marker;
    vel_marker.header = out.header;
    vel_marker.ns = "auto_buff";
    vel_marker.id = 1;
    vel_marker.type = visualization_msgs::msg::Marker::ARROW;
    vel_marker.action = visualization_msgs::msg::Marker::ADD;
    vel_marker.scale.x = 0.04;
    vel_marker.scale.y = 0.08;
    vel_marker.scale.z = 0.08;
    vel_marker.color.a = 1.0;
    vel_marker.color.r = 1.0F;
    vel_marker.color.g = 0.8F;
    vel_marker.color.b = 0.2F;
    geometry_msgs::msg::Point p0 = out.center_pose.position;
    geometry_msgs::msg::Point p1 = p0;
    p1.x += out.center_twist.linear.x * 0.2;
    p1.y += out.center_twist.linear.y * 0.2;
    p1.z += out.center_twist.linear.z * 0.2;
    vel_marker.points = {p0, p1};

    visualization_msgs::msg::Marker text_marker;
    text_marker.header = out.header;
    text_marker.ns = "auto_buff";
    text_marker.id = 2;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position = out.center_pose.position;
    text_marker.pose.position.z += 0.25;
    text_marker.pose.orientation.w = 1.0;
    text_marker.scale.z = 0.15;
    text_marker.color.a = 1.0;
    text_marker.color.r = 0.3F;
    text_marker.color.g = 0.9F;
    text_marker.color.b = 1.0F;
    text_marker.text = out.robot_id;

    marker_array.markers.push_back(pos_marker);
    marker_array.markers.push_back(vel_marker);
    marker_array.markers.push_back(text_marker);
    for (size_t i = 0; i < last_world_points_.size(); ++i) {
      visualization_msgs::msg::Marker blade;
      blade.header = out.header;
      blade.ns = "auto_buff_blades";
      blade.id = static_cast<int>(100 + i);
      blade.type = visualization_msgs::msg::Marker::SPHERE;
      blade.action = visualization_msgs::msg::Marker::ADD;
      blade.pose.position = last_world_points_[i].point;
      blade.pose.orientation.w = 1.0;
      blade.scale.x = 0.08;
      blade.scale.y = 0.08;
      blade.scale.z = 0.08;
      blade.color.a = 0.9;
      if (i == last_selected_world_index_) {
        blade.color.r = 1.0F;
        blade.color.g = 0.2F;
        blade.color.b = 0.2F;
      } else {
        blade.color.r = 0.2F;
        blade.color.g = 0.4F;
        blade.color.b = 1.0F;
      }
      marker_array.markers.push_back(blade);
    }
    marker_pub_->publish(marker_array);
  }

  void onMarkerTimer()
  {
    if (!debug_markers_enable_ || !has_last_debug_target_) {
      return;
    }
    last_debug_target_.header.stamp = now();
    publishDebugMarkers(last_debug_target_);
  }

  std::string input_topic_;
  std::string input_array_topic_;
  std::string output_topic_;
  std::string camera_info_topic_;
  std::string serial_state_topic_;
  std::string target_frame_;
  std::string big_buff_id_;
  std::string small_buff_id_;
  double tf_timeout_s_{0.02};
  double min_dt_s_{1e-3};
  double pnp_fallback_distance_m_{6.0};
  bool mode_override_enable_{true};
  double rune_radius_m_{0.7};
  bool debug_markers_enable_{false};
  std::string marker_topic_;
  double marker_publish_rate_hz_{10.0};

  rclcpp::Subscription<rm_interfaces::msg::RuneTarget>::SharedPtr sub_;
  rclcpp::Subscription<rm_interfaces::msg::RuneTargetArray>::SharedPtr sub_array_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<rm_interfaces::msg::SerialReceiveData>::SharedPtr serial_state_sub_;
  rclcpp::Publisher<rm_interfaces::msg::TrackedRobot>::SharedPtr pub_;
  rclcpp::Service<rm_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr marker_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  cv::Mat cam_matrix_;
  cv::Mat dist_coeffs_;
  bool has_camera_info_{false};
  std::vector<cv::Point3f> object_points_;
  cv::Point3f hit_point_obj_;
  int last_mode_{0};
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::Point last_pos_;
  bool last_is_big_rune_{false};
  rm_interfaces::msg::TrackedRobot last_debug_target_;
  bool has_last_debug_target_{false};
  std::vector<geometry_msgs::msg::PointStamped> last_world_points_;
  size_t last_selected_world_index_{0};
};

}  // namespace fyt::auto_aim::auto_buff

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fyt::auto_aim::auto_buff::BuffPoseEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
