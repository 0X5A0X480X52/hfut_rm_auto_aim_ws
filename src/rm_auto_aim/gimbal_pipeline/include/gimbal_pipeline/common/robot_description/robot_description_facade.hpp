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

#ifndef GIMBAL_PIPELINE__COMMON__ROBOT_DESCRIPTION__ROBOT_DESCRIPTION_FACADE_HPP_
#define GIMBAL_PIPELINE__COMMON__ROBOT_DESCRIPTION__ROBOT_DESCRIPTION_FACADE_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include <geometry_msgs/msg/pose.hpp>
#include <rm_interfaces/msg/tracked_robot.hpp>
#include <std_msgs/msg/header.hpp>

namespace fyt::auto_aim
{

class AdaptiveArmorTracker;
struct SmoothedOutput;

namespace robot_description
{

enum class TrackedRobotBuildStatus
{
  SUCCESS = 0,
  REJECTED_UNSUPPORTED_ID = 1,
  INTERNAL_ERROR = 2,
};

struct TrackedRobotBuildResult
{
  TrackedRobotBuildStatus status{TrackedRobotBuildStatus::INTERNAL_ERROR};
  rm_interfaces::msg::TrackedRobot robot{};
  std::string reason{};

  bool ok() const
  {
    return status == TrackedRobotBuildStatus::SUCCESS;
  }
};

struct TrackedRobotBuildInput
{
  const std_msgs::msg::Header & header;
  const std::string & target_frame;
  const std::string & robot_id;
  AdaptiveArmorTracker & tracker;
  const SmoothedOutput * smoothed{nullptr};
  int visible_armor_count{0};
};

class ITrackedRobotBuilderStrategy
{
public:
  virtual ~ITrackedRobotBuilderStrategy() = default;

  virtual std::string strategyName() const = 0;
  virtual uint8_t robotType() const = 0;
  virtual int numArmors() const = 0;

  virtual rm_interfaces::msg::TrackedRobot buildTrackedRobot(
    const TrackedRobotBuildInput & input) const = 0;
};

class TrackedRobotBuilderRegistry
{
public:
  void registerBuilder(
    const std::string & robot_id,
    const std::shared_ptr<ITrackedRobotBuilderStrategy> & builder);

  std::shared_ptr<const ITrackedRobotBuilderStrategy> findBuilder(
    const std::string & robot_id) const;

  bool supports(const std::string & robot_id) const;

  std::vector<std::string> supportedRobotIds() const;

private:
  std::unordered_map<std::string, std::shared_ptr<ITrackedRobotBuilderStrategy>> builders_;
};

class RobotDescriptionFacade
{
public:
  RobotDescriptionFacade();

  void setStrictUnknownReject(bool strict_unknown_reject);
  bool strictUnknownReject() const;

  void registerBuilder(
    const std::string & robot_id,
    const std::shared_ptr<ITrackedRobotBuilderStrategy> & builder);

  bool isSupportedRobotId(const std::string & robot_id) const;

  std::vector<std::string> supportedRobotIds() const;

  TrackedRobotBuildResult tryBuildTrackedRobot(
    const TrackedRobotBuildInput & input) const;

private:
  void registerDefaultBuilders();

  TrackedRobotBuilderRegistry builder_registry_;
  bool strict_unknown_reject_{true};
};

class TrackedRobotUsage
{
public:
  using OffsetFallbackGenerator = std::function<std::vector<Eigen::Vector3d>(
    const rm_interfaces::msg::TrackedRobot &)>;

  static std::vector<Eigen::Vector3d> resolveOffsets(
    const rm_interfaces::msg::TrackedRobot & robot,
    const OffsetFallbackGenerator & fallback_generator);

  static std::vector<geometry_msgs::msg::Pose> generateArmorsOffsetFromProfile(
    int num_armors,
    double r1,
    double r2,
    double d_za,
    double d_zc);
};

}  // namespace robot_description

}  // namespace fyt::auto_aim

#endif  // GIMBAL_PIPELINE__COMMON__ROBOT_DESCRIPTION__ROBOT_DESCRIPTION_FACADE_HPP_
