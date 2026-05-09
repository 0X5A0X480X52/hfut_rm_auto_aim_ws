#include "buff_fitter.hpp"
#include "configs.hpp"
#include "targets.hpp"
#include "types.hpp"

#include "opencv2/core.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/buff_tracker_state.hpp"
#include "rm_interfaces/msg/rune_target_array.hpp"
#include "rm_interfaces/msg/rune_target.hpp"
#include "rm_interfaces/msg/serial_receive_data.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace fyt::auto_aim::auto_buff {

namespace {

Eigen::Isometry3d transformToIsometry(const geometry_msgs::msg::Transform &t) {
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.translation() =
      Eigen::Vector3d(t.translation.x, t.translation.y, t.translation.z);
  Eigen::Quaterniond q(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
  iso.linear() = q.toRotationMatrix();
  return iso;
}

} // namespace

class BuffTrackerNode : public rclcpp::Node {
public:
  BuffTrackerNode()
      : Node("buff_tracker_node"), tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {
    // Topic parameters
    rune_target_topic_ =
        declare_parameter<std::string>("rune_target_topic", "/rune_targets");
    camera_info_topic_ =
        declare_parameter<std::string>("camera_info_topic", "/camera_info");
    serial_state_topic_ = declare_parameter<std::string>(
        "serial_state_topic", "/serial_receive_data");
    output_topic_ = declare_parameter<std::string>(
        "output_topic", "/auto_buff/buff_tracker_state");
    odom_frame_id_ =
        declare_parameter<std::string>("odom_frame_id", "odom");

    // General parameters
    tf_timeout_s_ = declare_parameter<double>("tf_timeout_s", 0.02);
    enable_small_buff_ = declare_parameter<bool>("enable_small_buff", true);
    enable_big_buff_ = declare_parameter<bool>("enable_big_buff", true);
    force_task_mode_ = declare_parameter<int>("force_task_mode", 0);
    publish_rate_hz_ =
        declare_parameter<double>("publish_rate_hz", 200.0);
    auto publish_period_ms = static_cast<int>(1000.0 / std::max(1.0, publish_rate_hz_));

    // --- Buff configs (read inline from parameters) ---
    small_buff_config_.match_conf.max_match_distance_m =
        declare_parameter<double>("small.match.max_match_distance_m", 1.0);
    small_buff_config_.match_conf.max_match_roll_diff_degree =
        declare_parameter<double>("small.match.max_match_roll_diff_degree", 30.0);
    small_buff_config_.blade_conf.pixel_error.x =
        declare_parameter<double>("small.blade.pixel_error_x", 5.0);
    small_buff_config_.blade_conf.pixel_error.y =
        declare_parameter<double>("small.blade.pixel_error_y", 5.0);
    small_buff_config_.blade_conf.position_noise_m.x =
        declare_parameter<double>("small.blade.position_noise_x", 0.03);
    small_buff_config_.blade_conf.position_noise_m.y =
        declare_parameter<double>("small.blade.position_noise_y", 0.03);
    small_buff_config_.blade_conf.position_noise_m.z =
        declare_parameter<double>("small.blade.position_noise_z", 0.03);
    small_buff_config_.blade_conf.roll_noise_degree =
        declare_parameter<double>("small.blade.roll_noise_degree", 2.0);
    small_buff_config_.center_conf.position_consistency_noise_m.x =
        declare_parameter<double>("small.center.position_consistency_noise_x", 0.01);
    small_buff_config_.center_conf.position_consistency_noise_m.y =
        declare_parameter<double>("small.center.position_consistency_noise_y", 0.01);
    small_buff_config_.center_conf.position_consistency_noise_m.z =
        declare_parameter<double>("small.center.position_consistency_noise_z", 0.01);
    small_buff_config_.center_conf.roll_noise_degree =
        declare_parameter<double>("small.center.roll_noise_degree", 3.0);
    small_buff_config_.center_conf.vroll_noise_rad =
        declare_parameter<double>("small.center.vroll_noise_rad", 0.10);
    small_buff_config_.center_conf.position_prior_noise_m.x =
        declare_parameter<double>("small.center.position_prior_noise_x", 0.10);
    small_buff_config_.center_conf.position_prior_noise_m.y =
        declare_parameter<double>("small.center.position_prior_noise_y", 0.10);
    small_buff_config_.center_conf.position_prior_noise_m.z =
        declare_parameter<double>("small.center.position_prior_noise_z", 0.10);
    small_buff_config_.center_conf.roll_prior_noise_degree =
        declare_parameter<double>("small.center.roll_prior_noise_degree", 30.0);
    small_buff_config_.center_conf.vroll_prior_noise_rad =
        declare_parameter<double>("small.center.vroll_prior_noise_rad", 6.0);
    small_buff_config_.lost_threshold_sec =
        declare_parameter<double>("small.lost_threshold_sec", 0.8);

    big_buff_config_.fitter_conf.queue_upper_limit =
        declare_parameter<int>("big.fitter.queue_upper_limit", 200);
    big_buff_config_.fitter_conf.queue_lower_limit =
        declare_parameter<int>("big.fitter.queue_lower_limit", 100);
    big_buff_config_.fitter_conf.param_lower_bound_scale =
        declare_parameter<double>("big.fitter.param_lower_bound_scale", 1.0);
    big_buff_config_.fitter_conf.param_upper_bound_scale =
        declare_parameter<double>("big.fitter.param_upper_bound_scale", 1.0);
    big_buff_config_.fitter_conf.curve_fitting_interval_time_ms =
        declare_parameter<int>("big.fitter.curve_fitting_interval_time_ms", 20);
    big_buff_config_.match_conf.max_match_distance_m =
        declare_parameter<double>("big.match.max_match_distance_m", 1.0);
    big_buff_config_.match_conf.max_match_roll_diff_degree =
        declare_parameter<double>("big.match.max_match_roll_diff_degree", 30.0);
    big_buff_config_.blade_conf.pixel_error.x =
        declare_parameter<double>("big.blade.pixel_error_x", 5.0);
    big_buff_config_.blade_conf.pixel_error.y =
        declare_parameter<double>("big.blade.pixel_error_y", 5.0);
    big_buff_config_.blade_conf.position_noise_m.x =
        declare_parameter<double>("big.blade.position_noise_x", 0.03);
    big_buff_config_.blade_conf.position_noise_m.y =
        declare_parameter<double>("big.blade.position_noise_y", 0.03);
    big_buff_config_.blade_conf.position_noise_m.z =
        declare_parameter<double>("big.blade.position_noise_z", 0.03);
    big_buff_config_.blade_conf.roll_noise_degree =
        declare_parameter<double>("big.blade.roll_noise_degree", 2.0);
    big_buff_config_.center_conf.position_consistency_noise_m.x =
        declare_parameter<double>("big.center.position_consistency_noise_x", 0.01);
    big_buff_config_.center_conf.position_consistency_noise_m.y =
        declare_parameter<double>("big.center.position_consistency_noise_y", 0.01);
    big_buff_config_.center_conf.position_consistency_noise_m.z =
        declare_parameter<double>("big.center.position_consistency_noise_z", 0.01);
    big_buff_config_.center_conf.roll_noise_degree =
        declare_parameter<double>("big.center.roll_noise_degree", 3.0);
    big_buff_config_.center_conf.vroll_noise_rad =
        declare_parameter<double>("big.center.vroll_noise_rad", 0.10);
    big_buff_config_.center_conf.position_prior_noise_m.x =
        declare_parameter<double>("big.center.position_prior_noise_x", 0.10);
    big_buff_config_.center_conf.position_prior_noise_m.y =
        declare_parameter<double>("big.center.position_prior_noise_y", 0.10);
    big_buff_config_.center_conf.position_prior_noise_m.z =
        declare_parameter<double>("big.center.position_prior_noise_z", 0.10);
    big_buff_config_.center_conf.roll_prior_noise_degree =
        declare_parameter<double>("big.center.roll_prior_noise_degree", 30.0);
    big_buff_config_.center_conf.vroll_prior_noise_rad =
        declare_parameter<double>("big.center.vroll_prior_noise_rad", 6.0);
    big_buff_config_.lost_threshold_sec =
        declare_parameter<double>("big.lost_threshold_sec", 1.0);

    // Subscriptions
    rune_target_sub_ = create_subscription<rm_interfaces::msg::RuneTargetArray>(
        rune_target_topic_, rclcpp::SensorDataQoS(),
        std::bind(&BuffTrackerNode::onRuneTargets, this, std::placeholders::_1));
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, rclcpp::SensorDataQoS(),
        std::bind(&BuffTrackerNode::onCameraInfo, this, std::placeholders::_1));
    serial_state_sub_ =
        create_subscription<rm_interfaces::msg::SerialReceiveData>(
            serial_state_topic_, rclcpp::SensorDataQoS(),
            std::bind(&BuffTrackerNode::onSerialState, this,
                      std::placeholders::_1));

    // Publisher
    state_pub_ = create_publisher<rm_interfaces::msg::BuffTrackerState>(
        output_topic_, rclcpp::SensorDataQoS());

    // Periodic publish timer
    publish_timer_ = create_wall_timer(
        std::chrono::milliseconds(publish_period_ms),
        std::bind(&BuffTrackerNode::onPublishTimer, this));

    RCLCPP_INFO(get_logger(),
                "BuffTrackerNode started. Publishing at %.1f Hz to %s",
                publish_rate_hz_, output_topic_.c_str());
  }

  ~BuffTrackerNode() override {
    if (small_buff_target_) small_buff_target_.reset();
    if (big_buff_target_) big_buff_target_.reset();
  }

private:
  void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (!msg || msg->k.size() != 9 || msg->d.empty()) return;

    cv::Mat K = cv::Mat::zeros(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i)
      K.at<double>(i / 3, i % 3) = msg->k[static_cast<size_t>(i)];
    cv::Mat D =
        cv::Mat::zeros(static_cast<int>(msg->d.size()), 1, CV_64F);
    for (size_t i = 0; i < msg->d.size(); ++i)
      D.at<double>(static_cast<int>(i), 0) = msg->d[i];

    camera_matrix_ = K;
    distortion_coefficients_ = D;
    has_camera_info_ = true;

    // Lazy-init tracking targets
    if (!small_buff_target_ && enable_small_buff_) {
      small_buff_target_ = std::make_unique<::auto_buff::SmallBuffTarget>(
          get_logger(), small_buff_config_, camera_matrix_,
          distortion_coefficients_);
      RCLCPP_INFO(get_logger(), "SmallBuffTarget initialized.");
    }
    if (!big_buff_target_ && enable_big_buff_) {
      big_buff_target_ = std::make_unique<::auto_buff::BigBuffTarget>(
          get_logger(), big_buff_config_, camera_matrix_,
          distortion_coefficients_);
      RCLCPP_INFO(get_logger(), "BigBuffTarget initialized.");
    }
  }

  void onSerialState(
      const rm_interfaces::msg::SerialReceiveData::SharedPtr msg) {
    if (msg) last_mode_ = static_cast<int>(msg->mode);
  }

  bool onSmallBuffTask() const {
    if (force_task_mode_ == 2 || force_task_mode_ == 3) return true;
    if (force_task_mode_ != 0) return false;
    return last_mode_ == 2 || last_mode_ == 3;
  }

  bool onBigBuffTask() const {
    if (force_task_mode_ == 4 || force_task_mode_ == 5) return true;
    if (force_task_mode_ != 0) return false;
    return last_mode_ == 4 || last_mode_ == 5;
  }

  void onRuneTargets(
      const rm_interfaces::msg::RuneTargetArray::SharedPtr msg) {
    if (!msg || msg->targets.empty() || !has_camera_info_) return;

    // Convert RuneTarget to BuffBlade
    std::vector<::auto_buff::BuffBlade> blades;
    blades.reserve(msg->targets.size());
    for (const auto &t : msg->targets) {
      if (t.is_lost) continue;
      blades.emplace_back(t);
    }
    if (blades.empty()) return;

    // TF: camera -> odom
    Eigen::Isometry3d T_camera_to_odom = Eigen::Isometry3d::Identity();
    try {
      auto tf = tf_buffer_.lookupTransform(
          odom_frame_id_, msg->header.frame_id,
          tf2::TimePoint(std::chrono::nanoseconds{
              static_cast<int64_t>(msg->header.stamp.sec) * 1'000'000'000LL +
              static_cast<int64_t>(msg->header.stamp.nanosec)}),
          tf2::durationFromSec(tf_timeout_s_));
      T_camera_to_odom = transformToIsometry(tf.transform);
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "TF lookup failed: %s", e.what());
      return;
    }

    auto stamp_chrono = std::chrono::system_clock::time_point{
        std::chrono::nanoseconds{
            static_cast<int64_t>(msg->header.stamp.sec) * 1'000'000'000LL +
            static_cast<int64_t>(msg->header.stamp.nanosec)}};

    // Feed to active tracker(s)
    if (onSmallBuffTask() && small_buff_target_) {
      small_buff_target_->track(blades, stamp_chrono, T_camera_to_odom);
    }
    if (onBigBuffTask() && big_buff_target_) {
      big_buff_target_->track(blades, stamp_chrono, T_camera_to_odom);
    }
  }

  void onPublishTimer() {
    bool small_active = onSmallBuffTask();
    bool big_active = onBigBuffTask();

    // Prefer the active task mode. If both are active (force), prefer big.
    bool use_big = big_active;
    bool use_small = small_active && !big_active;
    if (!use_big && !use_small) {
      // No active buff task — publish empty state
      rm_interfaces::msg::BuffTrackerState msg;
      msg.header.stamp = now();
      msg.header.frame_id = odom_frame_id_;
      msg.robot_id = "";
      msg.track_state = rm_interfaces::msg::BuffTrackerState::LOST;
      msg.state_seq = state_seq_++;
      state_pub_->publish(msg);
      return;
    }

    ::auto_buff::BuffState buff_state;
    ::auto_buff::TrackState track_state;

    if (use_big && big_buff_target_) {
      std::tie(buff_state, track_state) = big_buff_target_->getTargetTrackState();
    } else if (use_small && small_buff_target_) {
      std::tie(buff_state, track_state) = small_buff_target_->getTargetTrackState();
    } else {
      return;
    }

    rm_interfaces::msg::BuffTrackerState msg;
    msg.header.stamp = now();
    msg.header.frame_id = odom_frame_id_;
    msg.robot_id = use_big ? "big_buff" : "small_buff";

    switch (track_state.state) {
    case ::auto_buff::TrackState::State::TRACKING:
      msg.track_state = rm_interfaces::msg::BuffTrackerState::TRACKING;
      break;
    case ::auto_buff::TrackState::State::TEMPLOST:
      msg.track_state = rm_interfaces::msg::BuffTrackerState::TEMP_LOST;
      break;
    default:
      msg.track_state = rm_interfaces::msg::BuffTrackerState::LOST;
      break;
    }

    msg.center_position.x = buff_state.center_position.x();
    msg.center_position.y = buff_state.center_position.y();
    msg.center_position.z = buff_state.center_position.z();
    msg.center_roll = buff_state.center_roll;

    msg.center_vroll = 0.0;
    if (use_small && small_buff_target_) {
      msg.center_vroll = small_buff_target_->get("vroll");
    } else if (use_big && big_buff_target_) {
      msg.center_vroll = big_buff_target_->get("vroll");
    }

    msg.num_blades = 5;
    msg.engageable_mask = 0u;
    for (int i = 0; i < 5; ++i) {
      msg.blade_inactivated[i] = buff_state.inactivated_flag.at(i);
      if (buff_state.inactivated_flag.at(i))
        msg.engageable_mask |= (1u << static_cast<uint32_t>(i));
    }

    msg.confidence =
        (track_state.state == ::auto_buff::TrackState::State::TRACKING) ? 0.9
        : 0.5;

    // Big buff curve-fit fields
    if (use_big) {
      msg.big_a = big_buff_target_->get("a");
      msg.big_omega = big_buff_target_->get("omega");
      msg.big_c = big_buff_target_->get("c");
      msg.big_d = big_buff_target_->get("d");
      msg.big_dt_from_start = big_buff_target_->get("dt_from_start");
    } else {
      msg.big_a = std::numeric_limits<double>::quiet_NaN();
      msg.big_omega = std::numeric_limits<double>::quiet_NaN();
      msg.big_c = std::numeric_limits<double>::quiet_NaN();
      msg.big_d = std::numeric_limits<double>::quiet_NaN();
      msg.big_dt_from_start = std::numeric_limits<double>::quiet_NaN();
    }

    msg.state_seq = state_seq_++;
    state_pub_->publish(msg);
  }

  // --- Parameters ---
  std::string rune_target_topic_;
  std::string camera_info_topic_;
  std::string serial_state_topic_;
  std::string output_topic_;
  std::string odom_frame_id_;
  double tf_timeout_s_{0.02};
  bool enable_small_buff_{true};
  bool enable_big_buff_{true};
  int force_task_mode_{0};
  double publish_rate_hz_{200.0};

  ::auto_buff::SmallBuffConfig small_buff_config_;
  ::auto_buff::BigBuffConfig big_buff_config_;

  // --- TF ---
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // --- Camera ---
  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;
  bool has_camera_info_{false};

  // --- State ---
  int last_mode_{0};
  std::unique_ptr<::auto_buff::SmallBuffTarget> small_buff_target_;
  std::unique_ptr<::auto_buff::BigBuffTarget> big_buff_target_;
  uint32_t state_seq_{0};

  // --- ROS2 ---
  rclcpp::Subscription<rm_interfaces::msg::RuneTargetArray>::SharedPtr
      rune_target_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
      camera_info_sub_;
  rclcpp::Subscription<rm_interfaces::msg::SerialReceiveData>::SharedPtr
      serial_state_sub_;
  rclcpp::Publisher<rm_interfaces::msg::BuffTrackerState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

} // namespace fyt::auto_aim::auto_buff

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<fyt::auto_aim::auto_buff::BuffTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
