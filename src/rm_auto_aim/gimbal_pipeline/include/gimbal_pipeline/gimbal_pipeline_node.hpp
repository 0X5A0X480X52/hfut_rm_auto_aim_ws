// Copyright (C) FYT Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0
//
// Unified gimbal pipeline node — merges MaxEntropyTracker + TargetSelector +
// GimbalController into a single node to eliminate inter-node ROS2 latency.

#ifndef GIMBAL_PIPELINE__GIMBAL_PIPELINE_NODE_HPP_
#define GIMBAL_PIPELINE__GIMBAL_PIPELINE_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
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
#include <rm_interfaces/msg/blind.hpp>
#include <rm_interfaces/msg/blinds.hpp>
#include <rm_interfaces/msg/delay_audit.hpp>
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
#include "target_selector/strategies/min_yaw_deviation_strategy.hpp"
#include "target_selector/strategies/priority_list_strategy.hpp"
#include "target_selector/strategies/sticky_min_yaw_deviation_strategy.hpp"

// ─── prediction logger ────────────────────────────────────────
#include "gimbal_pipeline/prediction_logger.hpp"
#include "gimbal_pipeline/common/robot_description/robot_description_facade.hpp"

// ─── gimbal_controller internals ──────────────────────────────
#include "gimbal_controller/armor_position_calculator.hpp"
#include "gimbal_controller/armor_selector.hpp"
#include "gimbal_controller/ballistic_solver_client.hpp"
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
  void blindCallback(const rm_interfaces::msg::Blinds::SharedPtr msg,
                      const std::string &topic);
  void updateGimbalState();
  void buildControlContextFromCache(
      gimbal_controller::GimbalControlContext &context,
      std::string &selected_id,
      SelectionResult::ControlMode &control_mode);
  void publishDelayAuditDebug(
      const gimbal_controller::GimbalControlContext &context,
      const gimbal_controller::DelayAuditSnapshot &audit,
      const std::string &strategy_name);

  // ── blind selection strategy ──────────────────────────────────────────
  using BlindSelectionFunc = std::function<rm_interfaces::msg::Blind::SharedPtr(
      const std::vector<rm_interfaces::msg::Blind::SharedPtr> &candidates,
      double current_yaw)>;
  void initBlindSelectionStrategies();

  // ── timerCallback helper functions ─────────────────────────────────────
  void publishIdleCommand();
  rm_interfaces::msg::Blind::SharedPtr collectBlindCandidates();
  uint8_t computeTargetSources(bool main_camera_has_target);
  rm_interfaces::msg::GimbalCmd buildBlindGuidanceCommand();
  void applyGuidanceVelocitySmoothing(
      double yaw_diff_rad, double pitch_diff_rad,
      rm_interfaces::msg::GimbalCmd &cmd);
  rm_interfaces::msg::GimbalCmd buildNoTargetCommand();
  rm_interfaces::msg::GimbalCmd buildNormalCommand(
      const gimbal_controller::GimbalControlContext &context,
      const std::string &selected_id);
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
  double tracker_timeout_s_{0.5};

  std::unique_ptr<TFHandler> tf_handler_;
  std::unique_ptr<TrackerManager> tracker_manager_;
  std::unique_ptr<robot_description::RobotDescriptionFacade>
      robot_description_facade_;

  SmootherConfig smoother_config_;

  /* ================================================================ */
  /*  Target selector state (from TargetSelectorNode)                 */
  /* ================================================================ */
  SelectionStrategyPtr selection_strategy_;
  SelectionConfig selection_config_;
  std::string selector_strategy_name_;
  std::string current_target_id_;

  rclcpp::Time guidance_start_time_;
  static constexpr double GUIDANCE_TIMEOUT{3.0};  // 引导超时（秒），超时后重置计时器
  double guidance_end_yaw_threshold_deg_{5.0};    // 引导结束的 yaw deviation 阈值（度），目标进入主相机视野中心时结束引导
  bool enable_guidance_timeout_{false};           // 是否启用引导超时检测，默认关闭

  // 引导速度平滑控制
  double current_yaw_v_measured_{0.0};            // 从 TF 差分估计的当前角速度 (rad/s)
  double current_pitch_v_measured_{0.0};
  double last_guidance_cmd_yaw_v_{0.0};           // 上一周期引导模式输出的速度指令 (rad/s)
  double last_guidance_cmd_pitch_v_{0.0};
  bool guidance_vel_initialized_{false};          // 是否已从实测速度初始化引导速度指令
  bool blind_guidance_active_{false};             // blind guidance 是否激活（用于状态切换）
  double guidance_accel_limit_{10.0};             // 引导模式角加速度限幅 (rad/s²)
  bool enable_guidance_velocity_smoothing_{true}; // 是否启用引导速度平滑，默认开启
  double guidance_constant_yaw_v_{M_PI};          // 速度平滑关闭时的恒定 yaw 角速度 (rad/s)，默认 π rad/s = 180°/s

  // 引导模式目标角度锁定（防止 cmd.yaw/pitch 频繁跳变）
  // 原子类型：由 TF 回调线程和 timer 线程并发访问
  std::atomic<double> guidance_locked_yaw_deg_{0.0};   // 锁定的目标 yaw (度，绝对坐标系)
  std::atomic<double> guidance_locked_pitch_deg_{0.0}; // 锁定的目标 pitch (度，绝对坐标系)
  std::atomic<bool> guidance_target_locked_{false};    // 是否已锁定引导目标角度

  // 补盲目标超时机制：若某补盲相机在 blind_target_timeout_ 秒内未收到有效检测，
  // 则认为该目标已"消失"，用于避免引导持续向无效目标旋转
  double blind_target_timeout_{0.4};
  // 每个补盲相机 topic 最近一次收到非空 Blinds 消息的时间戳（受 blind_buffer_mutex_ 保护）
  std::unordered_map<std::string, rclcpp::Time> blind_last_nonempty_time_;

  /* ================================================================ */
  /*  Gimbal controller state (from GimbalControllerNode)             */
  /* ================================================================ */
  std::shared_ptr<gimbal_controller::ArmorPositionCalculator> position_calculator_;
  std::shared_ptr<gimbal_controller::ArmorSelector> armor_selector_;
  std::shared_ptr<gimbal_controller::BallisticSolverClient> ballistic_client_;
  std::shared_ptr<gimbal_controller::LocalTrajectoryCompensator> local_compensator_;
  std::shared_ptr<gimbal_controller::FireAdvisor> fire_advisor_;
    std::shared_ptr<gimbal_controller::FireAdviceEngine> fire_advice_engine_;
    std::shared_ptr<gimbal_controller::GimbalControlCore> gimbal_control_core_;
  std::unordered_map<std::string,
                     gimbal_controller::GimbalControlStrategy::SharedPtr>
      gimbal_strategies_;
  std::string current_gimbal_strategy_name_{"current"};

  bool enable_blind_{true};

  double current_yaw_{0.0};
  double current_pitch_{0.0};
  double bullet_speed_{20.0};
  double max_yaw_v_{540.0};
  double max_pitch_v_{360.0};
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

  /* ================================================================ */
  /*  Shared pipeline state (protected by mutex)                      */
  /* ================================================================ */
  std::mutex pipeline_mutex_;
  rm_interfaces::msg::TrackedRobots::SharedPtr latest_tracked_robots_;
  std::string latest_selected_target_id_;
  double latest_selected_confidence_{0.0};
  SelectionResult::ControlMode latest_control_mode_{SelectionResult::MODE_NO_TARGET};
  rclcpp::Time latest_update_time_{0, 0, RCL_ROS_TIME};  // local clock when data was cached

  /* ================================================================ */
  /*  ROS2 external interfaces (kept)                                 */
  /* ================================================================ */
  // Subscriptions
  message_filters::Subscriber<rm_interfaces::msg::Armors> armors_sub_;
  std::shared_ptr<tf2_armor_filter> tf2_filter_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // Blind detector subscriptions — supports multi-camera (one sub per configured topic)
  std::vector<std::string> blind_topics_;
  double blind_sync_timeout_{0.05};
  std::vector<rclcpp::Subscription<rm_interfaces::msg::Blinds>::SharedPtr> blind_subs_;
  // Per-topic latest message buffer (protected by blind_buffer_mutex_)
  std::unordered_map<std::string, rm_interfaces::msg::Blinds::SharedPtr> blind_latest_per_topic_;
  std::mutex blind_buffer_mutex_;
  // Best candidate selected each control cycle (used by buildBlindGuidanceCommand)
  rm_interfaces::msg::Blind::SharedPtr latest_blind_msg_;
  // Blind candidate selection strategy (configurable, keyed by name)
  std::unordered_map<std::string, BlindSelectionFunc> blind_selection_strategies_;
  std::string blind_selection_strategy_name_{"min_yaw"};

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

  // Blind target debug publisher (always-on)
  rclcpp::Publisher<rm_interfaces::msg::Blind>::SharedPtr
      debug_blind_target_pub_;
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
  std::vector<std::array<float, 4>> color_palette_;
};

}  // namespace fyt::auto_aim

#endif  // GIMBAL_PIPELINE__GIMBAL_PIPELINE_NODE_HPP_
