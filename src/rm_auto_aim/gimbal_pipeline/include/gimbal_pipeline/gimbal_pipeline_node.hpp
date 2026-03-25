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
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// ─── message_filters + TF2 filter ──────────────────────────────
#include <message_filters/subscriber.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>

// ─── rm_interfaces ─────────────────────────────────────────────
#include <rm_interfaces/msg/armor.hpp>
#include <rm_interfaces/msg/armors.hpp>
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
#include "max_entropy_tracker/utils/observation_outlier_filter.hpp"
#include "max_entropy_tracker/utils/output_smoother.hpp"

// ─── target_selector internals ────────────────────────────────
#include "target_selector/selection_strategy.hpp"
#include "target_selector/strategies/min_yaw_deviation_strategy.hpp"
#include "target_selector/strategies/priority_list_strategy.hpp"
#include "target_selector/strategies/sticky_min_yaw_deviation_strategy.hpp"
#include "target_selector/strategies/priority_list_strategy.hpp"
#include "target_selector/strategies/sticky_min_yaw_deviation_strategy.hpp"

// ─── prediction logger ────────────────────────────────────────
#include "gimbal_pipeline/prediction_logger.hpp"

// ─── gimbal_controller internals ──────────────────────────────
#include "gimbal_controller/armor_position_calculator.hpp"
#include "gimbal_controller/armor_selector.hpp"
#include "gimbal_controller/ballistic_solver_client.hpp"
#include "gimbal_controller/fire_advisor.hpp"
#include "gimbal_controller/gimbal_cmd_filter.hpp"
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
  rm_interfaces::msg::Target buildTargetMessage(
      const std_msgs::msg::Header &header, const std::string &robot_id,
      AdaptiveArmorTracker &tracker, const SmoothedOutput *smoothed = nullptr);
  rm_interfaces::msg::TrackedRobot buildTrackedRobotMessage(
      const std_msgs::msg::Header &header, const std::string &robot_id,
      AdaptiveArmorTracker &tracker, const SmoothedOutput *smoothed = nullptr);
  uint8_t inferRobotType(const std::string &robot_id) const;
  int inferNumArmors(const std::string &robot_id, int robot_type) const;
  std::vector<geometry_msgs::msg::Pose> generateArmorsOffset(
      int num_armors, double r1, double r2, double d_za, double d_zc) const;

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
  void timerCallback();
  void setModeCallback(
      const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
      std::shared_ptr<rm_interfaces::srv::SetMode::Response> response);
  gimbal_controller::GimbalControlStrategy::SharedPtr getGimbalStrategy(
      const std::string &name) const;

  /* ================================================================ */
  /*  Debug visualization                                             */
  /* ================================================================ */
  void initMarkers();
  void publishGimbalMarkers(
      const rm_interfaces::msg::TrackedRobot &target_robot,
      const rm_interfaces::msg::GimbalCmd &cmd);
  void publishManeuverMarkers(const std_msgs::msg::Header &header);
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

  std::unique_ptr<TFHandler> tf_handler_;
  std::unique_ptr<TrackerManager> tracker_manager_;

  SmootherConfig smoother_config_;
  std::unordered_map<std::string, OutputSmoother> smoothers_;
  std::unordered_map<std::string, int> last_obs_counts_;
  std::unordered_map<std::string, bool> last_dual_obs_;

  // Outlier filter (per-robot, pre-smoother; independent of smoother.enable)
  std::unordered_map<std::string, ObservationOutlierFilter> outlier_filters_;
  // Last valid smoothed output cache — used by hold strategy on outlier frames
  std::unordered_map<std::string, SmoothedOutput> last_smoothed_outputs_;

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
  std::shared_ptr<gimbal_controller::BallisticSolverClient> ballistic_client_;
  std::shared_ptr<gimbal_controller::LocalTrajectoryCompensator> local_compensator_;
  std::shared_ptr<gimbal_controller::FireAdvisor> fire_advisor_;
  std::unordered_map<std::string,
                     gimbal_controller::GimbalControlStrategy::SharedPtr>
      gimbal_strategies_;
  std::string current_gimbal_strategy_name_{"current"};

  double current_yaw_{0.0};
  double current_pitch_{0.0};
  double bullet_speed_{20.0};
  double control_rate_{250.0};
  std::string ballistic_mode_{"service"};
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

  // GimbalCmd 输出端保护滤波器
  gimbal_controller::GimbalCmdFilter cmd_filter_;
  // 记录上一帧跟踪的目标 ID，用于检测目标切换并 reset 滤波器
  std::string prev_tracking_target_id_;

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
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      debug_tracker_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      debug_gimbal_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      debug_maneuver_pub_;

  // Services
  rclcpp::Service<rm_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;

  // Timer
  rclcpp::TimerBase::SharedPtr control_timer_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

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
  std::vector<std::array<float, 4>> color_palette_;
};

}  // namespace fyt::auto_aim

#endif  // GIMBAL_PIPELINE__GIMBAL_PIPELINE_NODE_HPP_
