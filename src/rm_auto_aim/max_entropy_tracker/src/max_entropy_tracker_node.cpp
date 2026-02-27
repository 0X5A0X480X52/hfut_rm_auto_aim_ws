// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/max_entropy_tracker_node.hpp"

#include <cmath>
#include <sstream>

#include "max_entropy_tracker/msg_converter.hpp"

namespace fyt::auto_aim {

/* ================================================================ */
/*  Construction                                                     */
/* ================================================================ */

MaxEntropyTrackerNode::MaxEntropyTrackerNode(const rclcpp::NodeOptions &options)
    : Node("max_entropy_tracker_node", options) {
  declare_parameters();

  // Read basic params
  target_frame_ = get_parameter("target_frame").as_string();
  source_frame_ = get_parameter("source_frame").as_string();
  predict_rate_ = get_parameter("predict_rate").as_double();
  debug_mode_ = get_parameter("debug_mode").as_bool();
  visualization_frame_ = get_parameter("visualization_frame").as_string();

  config_ = UnifiedConfig::create_default();
  apply_parameters_to_config();

  // TF handler
  tf_handler_ = std::make_unique<TFHandler>(this, target_frame_);

  // Tracker manager
  double dt = (predict_rate_ > 0) ? (1.0 / predict_rate_) : 0.01;
  tracker_manager_ = std::make_unique<TrackerManager>(
      config_, dt,
      get_parameter("default_r1").as_double(),
      get_parameter("default_r2").as_double(),
      get_parameter("default_dza").as_double(),
      get_parameter("tracker_timeout").as_double(),
      get_parameter("enable_oscillation_detection").as_bool());

  // QoS – BEST_EFFORT for sensor data
  rclcpp::QoS sensor_qos(10);
  sensor_qos.best_effort();

  // Subscriptions & publishers
  armors_sub_ = create_subscription<rm_interfaces::msg::Armors>(
      "/armor_detector/armors", sensor_qos,
      std::bind(&MaxEntropyTrackerNode::armors_callback, this,
                std::placeholders::_1));

  target_pub_ = create_publisher<rm_interfaces::msg::Target>(
      "/max_entropy_tracker/target", sensor_qos);

  tracked_robots_pub_ = create_publisher<rm_interfaces::msg::TrackedRobots>(
      "/max_entropy_tracker/tracked_robots", sensor_qos);

  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/max_entropy_tracker/markers", 10);

  RCLCPP_INFO(get_logger(),
              "MaxEntropyTrackerNode initialized: target_frame=%s, "
              "update_mode=triggered_on_observation",
              target_frame_.c_str());
}

/* ================================================================ */
/*  Parameter declaration                                            */
/* ================================================================ */

void MaxEntropyTrackerNode::declare_parameters() {
  // Basic
  declare_parameter("target_frame", "odom");
  declare_parameter("source_frame", "camera_optical_frame");
  declare_parameter("predict_rate", 100.0);
  declare_parameter("default_r1", 0.15);
  declare_parameter("default_r2", 0.20);
  declare_parameter("default_dza", 0.0);
  declare_parameter("tracker_timeout", 3.0);
  declare_parameter("debug_mode", false);
  declare_parameter("enable_oscillation_detection", false);
  declare_parameter("visualization_frame", "odom");

  // UKF
  declare_parameter("ukf.alpha", 0.001);
  declare_parameter("ukf.beta", 2.0);
  declare_parameter("ukf.kappa", 0.0);
  declare_parameter("ukf.obs_noise_pos", 0.05);
  declare_parameter("ukf.obs_noise_yaw", 0.05);
  declare_parameter("ukf.dual_obs_noise_pos", 0.01);
  declare_parameter("ukf.dual_obs_noise_yaw", 0.03);
  declare_parameter("ukf.dual_obs_geometry_noise_scale", 0.2);
  declare_parameter("ukf.single_obs_update_weight_pos", 0.05);
  declare_parameter("ukf.enable_innovation_gating", false);
  declare_parameter("ukf.innovation_gate_chi2_threshold", 9.49);

  // Motion
  declare_parameter("motion.translation_model", "CA");
  declare_parameter("motion.cv_process_noise_vel", 0.5);
  declare_parameter("motion.ca_process_noise_acc", 1.0);
  declare_parameter("motion.singer_alpha", 0.5);
  declare_parameter("motion.singer_sigma", 2.0);
  declare_parameter("motion.process_noise_r", 0.02);
  declare_parameter("motion.process_noise_dz", 0.005);

  // Spin
  declare_parameter("spin.spin_process_noise_yaw_rate", 0.3);
  declare_parameter("spin.spin_process_noise_yaw_acc", 1.0);
  declare_parameter("spin.spin_process_noise_delta_rate", 0.3);
  declare_parameter("spin.spin_process_noise_delta_acc", 3.0);

  // Entropy
  declare_parameter("entropy.temperature", 2.0);
  declare_parameter("entropy.use_adaptive", true);
  declare_parameter("entropy.k_prior_weight", 0.7);

  // Tracker
  declare_parameter("tracker.tracking_thres", 2);
  declare_parameter("tracker.lost_thres", 8);
  declare_parameter("tracker.temp_lost_thres", 3);
  declare_parameter("tracker.max_match_distance", 2.0);
  declare_parameter("tracker.max_match_yaw_diff", 1.0);
  declare_parameter("tracker.n_panels", 4);
  declare_parameter("tracker.panel_angle_step", M_PI / 2.0);

  // Constraints
  declare_parameter("constraints.min_radius", 0.12);
  declare_parameter("constraints.max_radius", 0.5);
  declare_parameter("constraints.min_dz", -1.0);
  declare_parameter("constraints.max_dz", 1.0);
}

/* ================================================================ */
/*  Apply parameters to config                                       */
/* ================================================================ */

void MaxEntropyTrackerNode::apply_parameters_to_config() {
  auto &c = config_;

  c.ukf.alpha = get_parameter("ukf.alpha").as_double();
  c.ukf.beta = get_parameter("ukf.beta").as_double();
  c.ukf.kappa = get_parameter("ukf.kappa").as_double();
  c.ukf.obs_noise_pos = get_parameter("ukf.obs_noise_pos").as_double();
  c.ukf.obs_noise_yaw = get_parameter("ukf.obs_noise_yaw").as_double();
  c.ukf.dual_obs_noise_pos = get_parameter("ukf.dual_obs_noise_pos").as_double();
  c.ukf.dual_obs_noise_yaw = get_parameter("ukf.dual_obs_noise_yaw").as_double();
  c.ukf.dual_obs_geometry_noise_scale =
      get_parameter("ukf.dual_obs_geometry_noise_scale").as_double();
  c.ukf.single_obs_update_weight_pos =
      get_parameter("ukf.single_obs_update_weight_pos").as_double();
  c.ukf.enable_innovation_gating =
      get_parameter("ukf.enable_innovation_gating").as_bool();
  c.ukf.innovation_gate_chi2_threshold =
      get_parameter("ukf.innovation_gate_chi2_threshold").as_double();

  auto tm_str = get_parameter("motion.translation_model").as_string();
  c.motion.translation_model = translation_model_from_string(tm_str);
  c.motion.cv_process_noise_vel =
      get_parameter("motion.cv_process_noise_vel").as_double();
  c.motion.ca_process_noise_acc =
      get_parameter("motion.ca_process_noise_acc").as_double();
  c.motion.singer_alpha = get_parameter("motion.singer_alpha").as_double();
  c.motion.singer_sigma = get_parameter("motion.singer_sigma").as_double();
  c.motion.process_noise_r = get_parameter("motion.process_noise_r").as_double();
  c.motion.process_noise_dz = get_parameter("motion.process_noise_dz").as_double();

  c.spin.spin_process_noise_yaw_rate =
      get_parameter("spin.spin_process_noise_yaw_rate").as_double();
  c.spin.spin_process_noise_yaw_acc =
      get_parameter("spin.spin_process_noise_yaw_acc").as_double();
  c.spin.spin_process_noise_delta_rate =
      get_parameter("spin.spin_process_noise_delta_rate").as_double();
  c.spin.spin_process_noise_delta_acc =
      get_parameter("spin.spin_process_noise_delta_acc").as_double();

  c.entropy.temperature = get_parameter("entropy.temperature").as_double();
  c.entropy.use_adaptive = get_parameter("entropy.use_adaptive").as_bool();
  c.entropy.k_prior_weight = get_parameter("entropy.k_prior_weight").as_double();

  c.tracker.tracking_thres = get_parameter("tracker.tracking_thres").as_int();
  c.tracker.lost_thres = get_parameter("tracker.lost_thres").as_int();
  c.tracker.temp_lost_thres = get_parameter("tracker.temp_lost_thres").as_int();
  c.tracker.max_match_distance =
      get_parameter("tracker.max_match_distance").as_double();
  c.tracker.max_match_yaw_diff =
      get_parameter("tracker.max_match_yaw_diff").as_double();
  c.tracker.n_panels = get_parameter("tracker.n_panels").as_int();
  c.tracker.panel_angle_step =
      get_parameter("tracker.panel_angle_step").as_double();

  c.constraints.min_radius = get_parameter("constraints.min_radius").as_double();
  c.constraints.max_radius = get_parameter("constraints.max_radius").as_double();
  c.constraints.min_dz = get_parameter("constraints.min_dz").as_double();
  c.constraints.max_dz = get_parameter("constraints.max_dz").as_double();

  RCLCPP_INFO(get_logger(), "Parameters applied to UnifiedConfig");
}

/* ================================================================ */
/*  Armors callback                                                  */
/* ================================================================ */

void MaxEntropyTrackerNode::armors_callback(
    const rm_interfaces::msg::Armors::SharedPtr msg) {
  if (msg->armors.empty()) return;

  rclcpp::Time msg_time(msg->header.stamp);
  double current_time = msg_time.seconds();

  // Predict all existing trackers up to observation time
  tracker_manager_->predict_all(current_time);
  auto removed = tracker_manager_->remove_stale(current_time);
  if (!removed.empty() && debug_mode_) {
    RCLCPP_INFO(get_logger(), "Removed %zu stale trackers", removed.size());
  }

  // Group observations by robot ID
  std::unordered_map<std::string, std::vector<ObservationData>> obs_by_robot;

  std::string sf =
      msg->header.frame_id.empty() ? source_frame_ : msg->header.frame_id;

  for (const auto &armor : msg->armors) {
    auto obs = tf_handler_->transform_armor_to_observation(armor, sf, msg_time);
    if (!obs.has_value()) {
      if (debug_mode_)
        RCLCPP_WARN(get_logger(), "TF transform failed for armor %s",
                    armor.number.c_str());
      continue;
    }
    obs_by_robot[armor.number].push_back(obs.value());
  }

  for (auto &[rid, obs_list] : obs_by_robot) {
    last_obs_counts_[rid] = static_cast<int>(obs_list.size());
    tracker_manager_->update(rid, obs_list, current_time);
  }

  publish_results(msg->header);
}

/* ================================================================ */
/*  Publishing                                                       */
/* ================================================================ */

void MaxEntropyTrackerNode::publish_results(
    const std_msgs::msg::Header &header) {
  auto tracking_ids = tracker_manager_->tracking_robot_ids();
  if (tracking_ids.empty()) return;

  rm_interfaces::msg::TrackedRobots tracked_msg;
  tracked_msg.header = header;
  tracked_msg.header.frame_id = target_frame_;

  for (const auto &rid : tracking_ids) {
    auto *tracker = tracker_manager_->get(rid);
    if (!tracker || !tracker->is_tracking()) continue;

    auto target = build_target_message(header, rid, *tracker);
    target_pub_->publish(target);

    auto robot = build_tracked_robot_message(header, rid, *tracker);
    tracked_msg.robots.push_back(robot);
  }

  if (!tracked_msg.robots.empty())
    tracked_robots_pub_->publish(tracked_msg);
}

/* ================================================================ */
/*  Message builders                                                 */
/* ================================================================ */

rm_interfaces::msg::Target MaxEntropyTrackerNode::build_target_message(
    const std_msgs::msg::Header &header, const std::string &robot_id,
    AdaptiveArmorTracker &tracker) {
  rm_interfaces::msg::Target target;
  target.header = header;
  target.header.frame_id = target_frame_;
  target.tracking = true;
  target.id = robot_id;
  target.armors_num = 4;

  auto pos = tracker.get_center_position();
  target.position.x = pos.x();
  target.position.y = pos.y();
  target.position.z = pos.z();

  auto idx = tracker.ukf().state_idx();
  const auto &x = tracker.ukf().x();
  target.velocity.x = x(idx.VX());
  target.velocity.y = x(idx.VY());
  target.velocity.z = x(idx.VZ());

  target.yaw = tracker.get_yaw();
  target.v_yaw = x(idx.DELTA_RATE());

  auto [r1, r2] = tracker.get_radii();
  target.radius_1 = r1;
  target.radius_2 = r2;
  target.d_za = tracker.get_dza();

  return target;
}

rm_interfaces::msg::TrackedRobot
MaxEntropyTrackerNode::build_tracked_robot_message(
    const std_msgs::msg::Header &header, const std::string &robot_id,
    AdaptiveArmorTracker &tracker) {
  rm_interfaces::msg::TrackedRobot msg;
  msg.header = header;
  msg.header.frame_id = target_frame_;
  msg.robot_id = robot_id;
  msg.robot_type = static_cast<uint8_t>(infer_robot_type(robot_id));

  auto pos = tracker.get_center_position();
  msg.center_position.x = pos.x();
  msg.center_position.y = pos.y();
  msg.center_position.z = pos.z();

  auto idx = tracker.ukf().state_idx();
  const auto &x = tracker.ukf().x();
  msg.center_velocity.x = x(idx.VX());
  msg.center_velocity.y = x(idx.VY());
  msg.center_velocity.z = x(idx.VZ());

  msg.yaw = tracker.get_yaw();
  msg.yaw_velocity = x(idx.DELTA_RATE());

  auto [r1, r2] = tracker.get_radii();
  msg.radius = r1;

  msg.num_armors = infer_num_armors(robot_id, msg.robot_type);
  msg.bound_armor_ids = {robot_id};
  msg.confidence = tracker.is_tracking() ? 1.0 : 0.3;

  return msg;
}

/* ================================================================ */
/*  Helpers                                                          */
/* ================================================================ */

uint8_t MaxEntropyTrackerNode::infer_robot_type(
    const std::string &robot_id) const {
  if (robot_id == "outpost")
    return rm_interfaces::msg::TrackedRobot::OUTPOST_3;
  if (robot_id == "base")
    return rm_interfaces::msg::TrackedRobot::BASE;
  if (robot_id == "sentry")
    return rm_interfaces::msg::TrackedRobot::SENTRY;
  if (robot_id == "1")
    return rm_interfaces::msg::TrackedRobot::HERO_4;
  if (robot_id == "2" || robot_id == "3" || robot_id == "4" ||
      robot_id == "5")
    return rm_interfaces::msg::TrackedRobot::STANDARD_4;
  return rm_interfaces::msg::TrackedRobot::UNKNOWN;
}

int MaxEntropyTrackerNode::infer_num_armors(const std::string & /*robot_id*/,
                                            int robot_type) const {
  using T = rm_interfaces::msg::TrackedRobot;
  if (robot_type == T::OUTPOST_3 || robot_type == T::BASE) return 3;
  if (robot_type == T::BALANCE_2) return 2;
  return 4;
}

}  // namespace fyt::auto_aim

// Register as composable node
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::auto_aim::MaxEntropyTrackerNode)
