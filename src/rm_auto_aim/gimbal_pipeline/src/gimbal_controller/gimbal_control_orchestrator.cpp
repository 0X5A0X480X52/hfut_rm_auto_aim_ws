// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gimbal_controller/gimbal_control_orchestrator.hpp"

#include <algorithm>
#include <cmath>

#include "gimbal_controller/fire_advice_engine.hpp"
#include "gimbal_controller/fire_advisor.hpp"
#include "gimbal_pipeline/common/robot_description/robot_description_facade.hpp"

namespace gimbal_controller
{

namespace
{

constexpr double kDeg2Rad = M_PI / 180.0;

bool hasTargetHeader(const rm_interfaces::msg::TrackedRobot & target_robot)
{
  return target_robot.header.stamp.sec != 0 || target_robot.header.stamp.nanosec != 0 ||
         !target_robot.header.frame_id.empty();
}

}  // namespace

void GimbalControlOrchestrator::setFireModules(
  std::shared_ptr<FireAdviceEngine> fire_advice_engine,
  std::shared_ptr<FireAdvisor> fire_advisor)
{
  fire_advice_engine_ = fire_advice_engine;
  fire_advisor_ = fire_advisor;
}

rm_interfaces::msg::GimbalCmd GimbalControlOrchestrator::buildIdleCmd(
  const GimbalControlContext & context) const
{
  rm_interfaces::msg::GimbalCmd cmd;
  cmd.yaw_diff = 0.0;
  cmd.pitch_diff = 0.0;
  cmd.distance = 0.0;
  cmd.fire_advice = false;
  cmd.mode = rm_interfaces::msg::GimbalCmd::MODE_NO_VALID_MEASUREMENT;
  if (hasTargetHeader(context.target_robot)) {
    cmd.header = context.target_robot.header;
  }
  if (!context.target_robot.robot_id.empty()) {
    cmd.target_id = context.target_robot.robot_id;
  }
  return cmd;
}

rm_interfaces::msg::GimbalCmd GimbalControlOrchestrator::finalize(
  const GimbalControlContext & context,
  const rm_interfaces::msg::GimbalCmd & control_cmd) const
{
  rm_interfaces::msg::GimbalCmd cmd = control_cmd;

  if (hasTargetHeader(context.target_robot)) {
    cmd.header = context.target_robot.header;
  }
  if (!context.target_robot.robot_id.empty() && cmd.target_id.empty()) {
    cmd.target_id = context.target_robot.robot_id;
  }

  cmd.mode = decideMode(context);

  if (cmd.mode != rm_interfaces::msg::GimbalCmd::MODE_NORMAL_MEASUREMENT) {
    cmd.distance = 0.0;
    cmd.fire_advice = false;
    return cmd;
  }

  const auto target_robot =
    fyt::auto_aim::robot_description::TrackedRobotUsage::normalizeState(context.target_robot);
  const double fallback_distance =
    fyt::auto_aim::robot_description::TrackedRobotUsage::centerDistance(target_robot);

  cmd.distance = std::max(control_cmd.distance, 0.0);
  if (cmd.distance <= 1e-6) {
    cmd.distance = std::max(fallback_distance, 0.0);
  }

  cmd.fire_advice = evaluateFireAdvice(context, cmd, cmd.distance);
  return cmd;
}

int8_t GimbalControlOrchestrator::decideMode(const GimbalControlContext & context) const
{
  if (context.is_tracking) {
    return rm_interfaces::msg::GimbalCmd::MODE_NORMAL_MEASUREMENT;
  }
  if (context.is_temp_lost) {
    return rm_interfaces::msg::GimbalCmd::MODE_NO_VALID_MEASUREMENT;
  }
  return rm_interfaces::msg::GimbalCmd::MODE_NO_VALID_MEASUREMENT;
}

bool GimbalControlOrchestrator::evaluateFireAdvice(
  const GimbalControlContext & context,
  const rm_interfaces::msg::GimbalCmd & cmd,
  double distance) const
{
  if (fire_advice_engine_) {
    FireAdviceEngineRequest request;
    request.target_robot = context.target_robot;
    request.current_time = context.current_time;
    request.observation_stamp = context.target_stamp;
    request.current_yaw = context.current_yaw;
    request.current_pitch = context.current_pitch;
    request.current_yaw_rate = cmd.yaw_v * kDeg2Rad;
    request.current_pitch_rate = cmd.pitch_v * kDeg2Rad;
    request.current_yaw_accel = cmd.yaw_a * kDeg2Rad;
    request.current_pitch_accel = cmd.pitch_a * kDeg2Rad;
    request.bullet_speed = context.bullet_speed;
    request.timing.prediction_delay_s = std::max(fire_cfg_.prediction_delay_s, 0.0);
    request.timing.control_latency_s = std::max(fire_cfg_.control_latency_s, 0.0);
    request.timing.trigger_to_muzzle_s = std::max(fire_cfg_.trigger_to_muzzle_s, 0.0);
    request.timing.max_processing_delay_s = std::max(fire_cfg_.max_processing_delay_s, 0.0);
    request.timing.include_processing_delay = fire_cfg_.include_processing_delay;
    request.timing.include_control_latency_in_target_prediction =
      fire_cfg_.include_control_latency_in_target_prediction;

    const auto result = fire_advice_engine_->evaluate(request);
    if (result.valid) {
      return result.fire_advice;
    }
  }

  if (fire_advisor_) {
    return fire_advisor_->shouldFireWithDelay(
      context.current_yaw,
      context.current_pitch,
      cmd.yaw * kDeg2Rad,
      cmd.pitch * kDeg2Rad,
      std::max(distance, 1e-3),
      std::max(fire_cfg_.trigger_to_muzzle_s, 0.0),
      cmd.yaw_v * kDeg2Rad,
      cmd.pitch_v * kDeg2Rad,
      cmd.yaw_a * kDeg2Rad,
      cmd.pitch_a * kDeg2Rad);
  }

  return false;
}

}  // namespace gimbal_controller
