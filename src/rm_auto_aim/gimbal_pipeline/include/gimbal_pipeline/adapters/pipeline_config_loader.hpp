// Copyright (C) FYT Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#ifndef GIMBAL_PIPELINE__ADAPTERS__PIPELINE_CONFIG_LOADER_HPP_
#define GIMBAL_PIPELINE__ADAPTERS__PIPELINE_CONFIG_LOADER_HPP_

#include <rclcpp/node.hpp>

#include "gimbal_pipeline/core/auto_aim_pipeline.hpp"

namespace fyt::auto_aim::pipeline::ros_adapter
{

class PipelineConfigLoader
{
public:
  static PipelineConfig load(const rclcpp::Node & node);
};

}  // namespace fyt::auto_aim::pipeline::ros_adapter

#endif  // GIMBAL_PIPELINE__ADAPTERS__PIPELINE_CONFIG_LOADER_HPP_
