// Copyright (C) FYT Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#include "gimbal_pipeline/adapters/pipeline_config_loader.hpp"

#include <algorithm>
#include <string>

namespace fyt::auto_aim::pipeline::ros_adapter
{

PipelineConfig PipelineConfigLoader::load(const rclcpp::Node & node)
{
  PipelineConfig config;
  config.target_selection.reference_yaw =
    node.get_parameter("selector.reference_yaw").as_double();
  config.target_selection.max_yaw_deviation =
    node.get_parameter("selector.max_yaw_deviation").as_double();
  config.target_selection.max_distance =
    node.get_parameter("selector.max_distance").as_double();
  config.target_selection.min_confidence =
    node.get_parameter("selector.min_confidence").as_double();
  config.target_selection.priority_robot_ids =
    node.get_parameter("selector.priority_robot_ids").as_string_array();

  const bool external_enabled = node.get_parameter("external_targets.enable").as_bool();
  const bool buff_enabled = node.get_parameter("external_targets.buff.enable").as_bool();
  config.external_targets.enabled = external_enabled && buff_enabled;
  config.external_targets.timeout_ns = static_cast<TimestampNs>(
    std::max(node.get_parameter("external_targets.buff.timeout_s").as_double(), 0.01) * 1e9);
  for (int mode = 0; mode <= 5; ++mode) {
    const auto values = node.get_parameter(
      "external_targets.allowed_ids_by_mode.mode_" + std::to_string(mode)).as_string_array();
    config.external_targets.allowed_ids_by_mode[mode] = {
      values.begin(), values.end()};
  }

  config.tracker_timeout_ns = static_cast<TimestampNs>(
    std::max(node.get_parameter("tracker_timeout").as_double(), 1e-3) * 1e9);
  return config;
}

}  // namespace fyt::auto_aim::pipeline::ros_adapter
