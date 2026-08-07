// Copyright (C) FYT Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0
//
// Unified gimbal pipeline node — merges MaxEntropyTracker + TargetSelector +
// GimbalController into a single node to eliminate inter-node ROS2 latency.

#ifndef GIMBAL_PIPELINE__GIMBAL_PIPELINE_NODE_HPP_
#define GIMBAL_PIPELINE__GIMBAL_PIPELINE_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// ─── message_filters + TF2 filter ──────────────────────────────
#include <message_filters/subscriber.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>

// ─── rm_interfaces ─────────────────────────────────────────────
#include <rm_interfaces/msg/armor.hpp>
#include <rm_interfaces/msg/armors.hpp>
#include <rm_interfaces/msg/delay_audit.hpp>
#include <rm_interfaces/msg/fire_advice_debug.hpp>
#include <rm_interfaces/msg/gimbal_cmd.hpp>
#include <rm_interfaces/msg/maneuver_state.hpp>
#include <rm_interfaces/msg/maneuver_states.hpp>
#include <rm_interfaces/msg/selected_target.hpp>
#include <rm_interfaces/msg/target.hpp>
#include <rm_interfaces/msg/tracked_robot.hpp>
#include <rm_interfaces/msg/tracked_robots.hpp>
#include <rm_interfaces/srv/set_mode.hpp>

// ─── max_entropy_tracker internals ────────────────────────────
#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/tf_handler.hpp"
#include "max_entropy_tracker/tracker_manager.hpp"
#include "max_entropy_tracker/utils/output_smoother.hpp"

// ─── target_selector internals ────────────────────────────────
#include "target_selector/selection_strategy.hpp"
#include "target_selector/strategies/priority_list_strategy.hpp"

// ─── prediction logger ────────────────────────────────────────
#include "gimbal_pipeline/prediction_logger.hpp"
#include "gimbal_pipeline/adapters/buff_target_adapter.hpp"
#include "gimbal_pipeline/common/robot_description/robot_description_facade.hpp"

// ─── gimbal_controller internals ──────────────────────────────
#include "gimbal_controller/armor_position_calculator.hpp"
#include "gimbal_controller/armor_selector.hpp"
#include "gimbal_controller/fire_advice_engine.hpp"
#include "gimbal_controller/fire_advisor.hpp"
#include "gimbal_controller/gimbal_control_core.hpp"
#include "gimbal_controller/gimbal_control_strategy.hpp"
#include "gimbal_controller/local_trajectory_compensator.hpp"

// ─── heartbeat 
#include "rm_utils/heartbeat.hpp"

namespace fyt::auto_aim {

using tf2_armor_filter = tf2_ros::MessageFilter<rm_interfaces::msg::Armors>;

class GimbalPipelineNode : public rclcpp::Node {
 public:
  explicit GimbalPipelineNode(const rclcpp::NodeOptions &options);
  ~GimbalPipelineNode() override = default;

 private:
  /* ================================================================ */
  /*  Parameter declaration (merged from three nodes)                 */
  /* ================================================================ */
  void declareTrackerParameters();
  void declareTargetSelectorParameters();
  void declareGimbalControllerParameters();
  void applyTrackerParamsToConfig();

  /* ================================================================ */
  /*  Tracker logic (from MaxEntropyTrackerNode)                      */
  /* ================================================================ */
  void armorsCallback(const rm_interfaces::msg::Armors::SharedPtr msg);
  rm_interfaces::msg::TrackedRobots buildTrackedRobotsMsg(
      const std_msgs::msg::Header &header);
  void mergeExternalTargets(
      rm_interfaces::msg::TrackedRobots & tracked_msg,
      const std_msgs::msg::Header &header);
  void refreshExternalTargetAllowlist(int mode);
  rm_interfaces::msg::Target buildTargetMessage(
      const std_msgs::msg::Header &header, const std::string &robot_id,
      BaseTracker &tracker, const SmoothedOutput *smoothed = nullptr);
  rm_interfaces::msg::TrackedRobot buildTrackedRobotMessage(
      const std_msgs::msg::Header &header, const std::string &robot_id,
      BaseTracker &tracker, const SmoothedOutput *smoothed = nullptr,
      int visible_armor_count = 0);

  /* ================================================================ */
  /*  Target selection logic (from TargetSelectorNode)                */
  /* ================================================================ */
  void initSelectionStrategy();
  SelectionResult selectTargetInternal(
      const rm_interfaces::msg::TrackedRobots &robots);

  /* ================================================================ */
  /*  Gimbal controller logic (from GimbalControllerNode)             */
  /* ================================================================ */
  void initGimbalComponents();
  void initGimbalStrategies();
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void updateGimbalState();
  void buildControlContextFromCache(
      gimbal_controller::GimbalControlContext &context,
      std::string &selected_id);
  void publishDelayAuditDebug(
      const gimbal_controller::GimbalControlContext &context,
      const gimbal_controller::DelayAuditSnapshot &audit,
      const std::string &strategy_name);
  void publishFireAdviceDebug(
      const gimbal_controller::GimbalControlContext &context,
      const rm_interfaces::msg::GimbalCmd &cmd,
      const gimbal_controller::FireAdviceDebugSnapshot &snapshot);
  void publishArmorSelectionDebug(
      const gimbal_controller::GimbalControlContext &context,
      const std::string &strategy_name);
  void timerCallback();
  void applyPendingRuntimeUpdates();
  void setModeCallback(
      const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
      std::shared_ptr<rm_interfaces::srv::SetMode::Response> response);
  rcl_interfaces::msg::SetParametersResult onSetParameters(
      const std::vector<rclcpp::Parameter> &params);
  bool isValidGimbalStrategyName(const std::string &name) const;
  gimbal_controller::GimbalControlStrategy::SharedPtr getGimbalStrategy(
      const std::string &name) const;

  /* ================================================================ */
  /*  Debug visualization                                             */
  /* ================================================================ */
  void initMarkers();
  void publishGimbalMarkers(
      const rm_interfaces::msg::TrackedRobot &target_robot,
      const rm_interfaces::msg::GimbalCmd &cmd,
      const gimbal_controller::FireAdviceDebugSnapshot & fire_snapshot);
  void publishManeuverMarkers(const std_msgs::msg::Header &header);
  void publishFireProbabilityMarkers(
      const std_msgs::msg::Header &header,
      const gimbal_controller::FireAdviceDebugSnapshot & fire_snapshot,
      visualization_msgs::msg::MarkerArray & marker_array);
  void publishFireProbabilityDebugImages(
      const std_msgs::msg::Header &header,
      const gimbal_controller::FireAdviceDebugSnapshot & fire_snapshot);
  void publish2DTrackerDebugImage(
      const std_msgs::msg::Header &header,
      const std::vector<TrackerManager::TrackerConstView> &tracker_views);
  void publishEvidenceFrameDebug(
      const std_msgs::msg::Header &header,
      const std::vector<TrackerManager::TrackerConstView> &tracker_views);
  void logNorm4V3TrackerDebug(
      const std::vector<TrackerManager::TrackerConstView> &tracker_views);
  std::array<float, 4> hsvToRgb(float h, float s, float v);

  /* ================================================================ */
  /*  Tracker state (from MaxEntropyTrackerNode)                      */
  /* ================================================================ */
  UnifiedConfig tracker_config_;
  std::string target_frame_;
  std::string source_frame_;
  double predict_rate_;
  bool debug_mode_;
  std::string visualization_frame_;
  double tracker_timeout_s_{0.5};

  std::unique_ptr<TFHandler> tf_handler_;
  std::unique_ptr<TrackerManager> tracker_manager_;
  std::unique_ptr<robot_description::RobotDescriptionFacade>
      robot_description_facade_;
  std::unique_ptr<adapters::BuffTargetAdapter> buff_target_adapter_;

  bool external_targets_enable_{false};
  bool external_targets_buff_enable_{false};
  std::string external_targets_buff_topic_{"auto_buff/tracked_robot"};
  double external_targets_buff_timeout_s_{0.3};
  int current_mode_{0};
  std::unordered_map<int, std::unordered_set<std::string>> allowed_ids_by_mode_;
  std::unordered_set<std::string> active_external_allowed_ids_;

  SmootherConfig smoother_config_;

  /* ================================================================ */
  /*  Target selector state (from TargetSelectorNode)                 */
  /* ================================================================ */
  SelectionStrategyPtr selection_strategy_;
  SelectionConfig selection_config_;
  std::string selector_strategy_name_;
  std::string current_target_id_;

  /* ================================================================ */
  /*  Gimbal controller state (from GimbalControllerNode)             */
  /* ================================================================ */
  std::shared_ptr<gimbal_controller::ArmorPositionCalculator> position_calculator_;
  std::shared_ptr<gimbal_controller::ArmorSelector> armor_selector_;
  std::shared_ptr<gimbal_controller::LocalTrajectoryCompensator> local_compensator_;
  std::shared_ptr<gimbal_controller::FireAdvisor> fire_advisor_;
    std::shared_ptr<gimbal_controller::FireAdviceEngine> fire_advice_engine_;
    std::shared_ptr<gimbal_controller::GimbalControlCore> gimbal_control_core_;
  std::unordered_map<std::string,
                     gimbal_controller::GimbalControlStrategy::SharedPtr>
      gimbal_strategies_;
  std::string current_gimbal_strategy_name_{"current"};

  double current_yaw_{0.0};
  double current_pitch_{0.0};
  double bullet_speed_{20.0};
  double control_rate_{250.0};
  bool enable_{true};
    bool radial_selection_enabled_{false};

    // Cached selector parameters for marker visualization
    double facing_enter_angle_deg_{40.0};
    double facing_exit_angle_deg_{55.0};
    bool radial_dynamic_enable_{false};
    double radial_dynamic_v_yaw_ref_{8.0};
    double radial_dynamic_shrink_ratio_{0.6};
    double radial_dynamic_min_angle_deg_{5.0};
    double radial_dynamic_bias_gain_deg_{0.0};
    double radial_dynamic_max_bias_deg_{0.0};
    bool virtual_auto_switch_enable_{false};
    double mpc_dt_debug_{0.01};

  /* ================================================================ */
  /*  Shared pipeline state (protected by mutex)                      */
  /* ================================================================ */
  std::mutex pipeline_mutex_;
  rm_interfaces::msg::TrackedRobots::SharedPtr latest_tracked_robots_;
  std::string latest_selected_target_id_;
  double latest_selected_confidence_{0.0};
  rclcpp::Time latest_update_time_{0, 0, RCL_ROS_TIME};  // local clock when data was cached

  /* ================================================================ */
  /*  ROS2 external interfaces (kept)                                 */
  /* ================================================================ */
  // Subscriptions
  message_filters::Subscriber<rm_interfaces::msg::Armors> armors_sub_;
  std::shared_ptr<tf2_armor_filter> tf2_filter_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // Publishers
  rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_pub_;

  // Maneuver states publisher (always-on, for chart monitoring)
  rclcpp::Publisher<rm_interfaces::msg::ManeuverStates>::SharedPtr
      maneuver_states_pub_;

  // Debug publishers (only when debug_mode_ == true)
  rclcpp::Publisher<rm_interfaces::msg::TrackedRobots>::SharedPtr
      debug_tracked_robots_pub_;
  rclcpp::Publisher<rm_interfaces::msg::SelectedTarget>::SharedPtr
      debug_selected_target_pub_;
  rclcpp::Publisher<rm_interfaces::msg::Target>::SharedPtr debug_target_pub_;
  rclcpp::Publisher<rm_interfaces::msg::DelayAudit>::SharedPtr
      debug_delay_audit_pub_;
  rclcpp::Publisher<rm_interfaces::msg::FireAdviceDebug>::SharedPtr
      debug_fire_advice_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      debug_armor_selection_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      debug_evidence_frame_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      debug_tracker_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      debug_gimbal_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      debug_maneuver_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      debug_fire_plane_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      debug_fire_normal_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr
      debug_tracker_2d_image_pub_;

  // Services
  rclcpp::Service<rm_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;

  // Timer
  rclcpp::TimerBase::SharedPtr control_timer_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  // Heartbeat monitor for critical components
  HeartBeatPublisher::SharedPtr heartbeat_;

  /* ================================================================ */
  /*  Prediction logger (optional, controlled by logging.enable)   */
  /* ================================================================ */
  std::unique_ptr<PredictionLogger> prediction_logger_;

  // Markers (gimbal visualization)
  visualization_msgs::msg::Marker position_marker_;
  visualization_msgs::msg::Marker target_velocity_marker_;
  visualization_msgs::msg::Marker armors_marker_;
  visualization_msgs::msg::Marker selection_marker_;
  visualization_msgs::msg::Marker predicted_marker_;
  visualization_msgs::msg::Marker trajectory_marker_;
    visualization_msgs::msg::Marker radial_allowed_arc_marker_;
    visualization_msgs::msg::Marker radial_allowed_bounds_marker_;
    visualization_msgs::msg::Marker virtual_armor_marker_;
        visualization_msgs::msg::Marker virtual_armor_text_marker_;
  std::vector<std::array<float, 4>> color_palette_;
  bool fire_prob_vis_enable_{true};
  int fire_prob_vis_ellipse_samples_{64};
  int fire_prob_vis_max_impact_points_{120};
  bool fire_prob_image_debug_enable_{false};
  bool fire_prob_image_debug_show_text_{true};
  bool fire_prob_image_debug_show_sigma_ellipse_{true};
  bool fire_prob_image_debug_show_velocity_fan_{true};
  int fire_prob_image_debug_width_{960};
  int fire_prob_image_debug_height_{540};
  double fire_prob_image_debug_publish_rate_hz_{10.0};
  rclcpp::Time last_fire_prob_image_pub_time_{0, 0, RCL_ROS_TIME};
  bool tracker_2d_image_debug_enable_{false};
  int tracker_2d_image_debug_width_{960};
  int tracker_2d_image_debug_height_{540};
  int tracker_2d_image_debug_jpeg_quality_{70};
};

}  // namespace fyt::auto_aim

#endif  // GIMBAL_PIPELINE__GIMBAL_PIPELINE_NODE_HPP_
