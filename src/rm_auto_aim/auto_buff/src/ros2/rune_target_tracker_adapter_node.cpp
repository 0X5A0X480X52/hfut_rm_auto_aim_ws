#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "opencv2/calib3d.hpp"
#include "opencv2/core.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/rune_target.hpp"
#include "rm_interfaces/msg/serial_receive_data.hpp"
#include "rm_interfaces/msg/tracked_robot.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace fyt::auto_aim::auto_buff
{

class RuneTargetTrackerAdapterNode : public rclcpp::Node
{
public:
  RuneTargetTrackerAdapterNode()
  : Node("rune_target_tracker_adapter_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/rune_target");
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

    // JLU object points (meters): center, br, tr, tl, bl
    object_points_ = {
      cv::Point3f(0.0f, 0.0f, 0.0f),
      cv::Point3f(0.0f, -0.186f, 0.5415f),
      cv::Point3f(0.0f, -0.160f, 0.8585f),
      cv::Point3f(0.0f, 0.160f, 0.8585f),
      cv::Point3f(0.0f, 0.186f, 0.5415f),
    };
    hit_point_obj_ = cv::Point3f(0.0f, 0.0f, static_cast<float>(rune_radius_m_));

    pub_ = create_publisher<rm_interfaces::msg::TrackedRobot>(output_topic_, rclcpp::SensorDataQoS());
    sub_ = create_subscription<rm_interfaces::msg::RuneTarget>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RuneTargetTrackerAdapterNode::onRuneTarget, this, std::placeholders::_1));
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RuneTargetTrackerAdapterNode::onCameraInfo, this, std::placeholders::_1));
    serial_state_sub_ = create_subscription<rm_interfaces::msg::SerialReceiveData>(
      serial_state_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RuneTargetTrackerAdapterNode::onSerialState, this, std::placeholders::_1));
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
    if (!msg) {
      return;
    }
    last_mode_ = static_cast<int>(msg->mode);
  }

  bool solvePnPInCamera(
    const rm_interfaces::msg::RuneTarget & msg,
    cv::Point3d & center_cam,
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
      object_points_,
      image_points,
      cam_matrix_,
      dist_coeffs_,
      rvec,
      tvec,
      false,
      cv::SOLVEPNP_ITERATIVE);
    if (!ok || tvec.total() < 3) {
      return false;
    }

    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Mat hit_obj = (cv::Mat_<double>(3, 1) << hit_point_obj_.x, hit_point_obj_.y, hit_point_obj_.z);
    cv::Mat hit = R * hit_obj + tvec;

    center_cam = cv::Point3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0));
    hit_cam = cv::Point3d(hit.at<double>(0, 0), hit.at<double>(1, 0), hit.at<double>(2, 0));
    return std::isfinite(center_cam.x) && std::isfinite(center_cam.y) && std::isfinite(center_cam.z) &&
           std::isfinite(hit_cam.x) && std::isfinite(hit_cam.y) && std::isfinite(hit_cam.z);
  }

  std::string resolveRuneIdFromMode(bool is_big_rune) const
  {
    if (mode_override_enable_) {
      if (last_mode_ == 2 || last_mode_ == 3) {
        return small_buff_id_;
      }
      if (last_mode_ == 4 || last_mode_ == 5) {
        return big_buff_id_;
      }
    }
    return is_big_rune ? big_buff_id_ : small_buff_id_;
  }

  void onRuneTarget(const rm_interfaces::msg::RuneTarget::SharedPtr msg)
  {
    if (!msg || msg->is_lost) {
      return;
    }

    geometry_msgs::msg::PointStamped p_cam;
    p_cam.header = msg->header;
    cv::Point3d center_cam, hit_cam;
    const bool pnp_ok = solvePnPInCamera(*msg, center_cam, hit_cam);
    if (pnp_ok) {
      p_cam.point.x = hit_cam.x;
      p_cam.point.y = hit_cam.y;
      p_cam.point.z = hit_cam.z;
    } else {
      // Fallback to principal-point ray + fixed range if camera model unavailable.
      double u = 0.0;
      double v = 0.0;
      for (const auto & p : msg->pts) {
        u += p.x;
        v += p.y;
      }
      u /= static_cast<double>(msg->pts.size());
      v /= static_cast<double>(msg->pts.size());
      const double fx = cam_matrix_.empty() ? 1200.0 : cam_matrix_.at<double>(0, 0);
      const double fy = cam_matrix_.empty() ? 1200.0 : cam_matrix_.at<double>(1, 1);
      const double cx = cam_matrix_.empty() ? 720.0 : cam_matrix_.at<double>(0, 2);
      const double cy = cam_matrix_.empty() ? 540.0 : cam_matrix_.at<double>(1, 2);
      const double z_cam = pnp_fallback_distance_m_;
      p_cam.point.x = z_cam;
      p_cam.point.y = (u - cx) * z_cam / std::max(1.0, fx);
      p_cam.point.z = (v - cy) * z_cam / std::max(1.0, fy);
      center_cam = cv::Point3d(p_cam.point.x, p_cam.point.y, p_cam.point.z);
    }

    geometry_msgs::msg::PointStamped p_world;
    try {
      p_world = tf_buffer_.transform(
        p_cam, target_frame_, tf2::durationFromSec(tf_timeout_s_));
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "rune target tf transform failed: %s", e.what());
      return;
    }

    rm_interfaces::msg::TrackedRobot out;
    out.header = msg->header;
    out.header.frame_id = target_frame_;
    out.robot_id = resolveRuneIdFromMode(msg->is_big_rune);
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

    if (last_stamp_.nanoseconds() > 0 &&
      (msg->is_big_rune == last_is_big_rune_))
    {
      const double dt = std::max(min_dt_s_, (rclcpp::Time(msg->header.stamp) - last_stamp_).seconds());
      const double vx = (p_world.point.x - last_pos_.x) / dt;
      const double vy = (p_world.point.y - last_pos_.y) / dt;
      const double vz = (p_world.point.z - last_pos_.z) / dt;
      out.center_twist.linear.x = vx;
      out.center_twist.linear.y = vy;
      out.center_twist.linear.z = vz;
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
    last_stamp_ = rclcpp::Time(msg->header.stamp);
    last_pos_ = p_world.point;
    last_is_big_rune_ = msg->is_big_rune;
  }

private:
  std::string input_topic_;
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

  rclcpp::Subscription<rm_interfaces::msg::RuneTarget>::SharedPtr sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<rm_interfaces::msg::SerialReceiveData>::SharedPtr serial_state_sub_;
  rclcpp::Publisher<rm_interfaces::msg::TrackedRobot>::SharedPtr pub_;

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
};

}  // namespace fyt::auto_aim::auto_buff

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fyt::auto_aim::auto_buff::RuneTargetTrackerAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
