// Copyright (C) FYT Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0
//
// GimbalPipelineNode — unified node merging MaxEntropyTracker +
// TargetSelector + GimbalController. Inter-node ROS2 topics are replaced
// by direct C++ function calls to eliminate serialization / scheduling
// latency.

#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rm_utils/heartbeat.hpp>
#include <sstream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unordered_set>

#include "gimbal_pipeline/gimbal_pipeline_node.hpp"
#include "max_entropy_tracker/msg_converter.hpp"
#include "max_entropy_tracker/trackers/norm4_baseline/tracker/norm4_tracker_baseline.hpp"
#include "max_entropy_tracker/visualization.hpp"
#include "rm_utils/logger/log.hpp"

// Gimbal strategies
#include "gimbal_controller/fire_advisor.hpp"
#include "gimbal_controller/strategies/mpc_control_strategy.hpp"

namespace fyt::auto_aim {

void GimbalPipelineNode::initSelectionStrategy() {
  selection_strategy_ = std::make_unique<PriorityListStrategy>();
  RCLCPP_INFO(get_logger(), "Selection strategy: %s", selection_strategy_->getName().c_str());
}

SelectionResult GimbalPipelineNode::selectTargetInternal(
  const rm_interfaces::msg::TrackedRobots &robots) {
  selection_config_.current_target_id = current_target_id_;

  auto result = selection_strategy_->selectTarget(robots, selection_config_);

  if (!result.has_value()) {
    current_target_id_ = "";
    return SelectionResult();
  }

  bool target_changed = (result->robot_id != current_target_id_);
  if (target_changed) {
    RCLCPP_INFO(get_logger(),
                "Target changed: %s -> %s (yaw_dev=%.3f, dist=%.2f)",
                current_target_id_.empty() ? "none" : current_target_id_.c_str(),
                result->robot_id.c_str(),
                result->yaw_deviation,
                result->distance);
    current_target_id_ = result->robot_id;
  }

  return *result;
}

/* ================================================================ */
/*  Gimbal controller initialization                                 */
/* ================================================================ */

}  // namespace fyt::auto_aim
