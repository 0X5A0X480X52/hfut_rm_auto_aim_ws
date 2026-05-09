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
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rm_interfaces/msg/tracked_robot.hpp>
#include <std_msgs/msg/header.hpp>

namespace fyt::auto_aim
{

class BaseTracker;
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
  BaseTracker & tracker;
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
  enum class MotionModel
  {
    CONSTANT_VELOCITY = 0,
    CONSTANT_ACCELERATION = 1,
  };

  enum class ProjectionMode
  {
    AUTO = 0,
    YAW_PLANE = 1,
    FULL_SE3 = 2,
  };

  /// Discriminates between full-robot and degraded single-armor representations.
  enum class RepresentationMode
  {
    STRUCTURED_ROBOT = 0,         // full center + N-armor geometry
    AMBIGUOUS_SINGLE_ARMOR = 1,   // single-armor degraded representation
    UNKNOWN = 2,
  };

  using OffsetFallbackGenerator = std::function<std::vector<Eigen::Vector3d>(
    const rm_interfaces::msg::TrackedRobot &)>;

  static Eigen::Vector3d toEigen(const geometry_msgs::msg::Point & point);

  static Eigen::Vector3d toEigen(const geometry_msgs::msg::Vector3 & vector);

  static geometry_msgs::msg::Point toPoint(const Eigen::Vector3d & vector);

  static geometry_msgs::msg::Vector3 toVector3(const Eigen::Vector3d & vector);

  static std::vector<Eigen::Vector3d> resolveOffsets(
    const rm_interfaces::msg::TrackedRobot & robot,
    const OffsetFallbackGenerator & fallback_generator);

  static rm_interfaces::msg::TrackedRobot predict(
    const rm_interfaces::msg::TrackedRobot & robot,
    double dt,
    MotionModel model);

  static Eigen::Vector3d predictCenter(
    const rm_interfaces::msg::TrackedRobot & robot,
    double dt,
    MotionModel model);

  static double predictYaw(
    const rm_interfaces::msg::TrackedRobot & robot,
    double dt,
    MotionModel model);

  static std::vector<Eigen::Vector3d> calculateArmorWorldPositionsEigen(
    const rm_interfaces::msg::TrackedRobot & robot,
    double dt,
    MotionModel model,
    ProjectionMode projection_mode = ProjectionMode::AUTO,
    const OffsetFallbackGenerator & fallback_generator = OffsetFallbackGenerator{});

  static std::vector<Eigen::Vector3d> calculateArmorWorldPositionsEigen(
    const rm_interfaces::msg::TrackedRobot & robot,
    double dt,
    MotionModel model,
    const OffsetFallbackGenerator & fallback_generator);

  static std::vector<geometry_msgs::msg::Point> calculateArmorWorldPositionsPoints(
    const rm_interfaces::msg::TrackedRobot & robot,
    double dt,
    MotionModel model,
    ProjectionMode projection_mode = ProjectionMode::AUTO,
    const OffsetFallbackGenerator & fallback_generator = OffsetFallbackGenerator{});

  static std::vector<geometry_msgs::msg::Point> calculateArmorWorldPositionsPoints(
    const rm_interfaces::msg::TrackedRobot & robot,
    double dt,
    MotionModel model,
    const OffsetFallbackGenerator & fallback_generator);

  static ProjectionMode resolveProjectionMode(const rm_interfaces::msg::TrackedRobot & robot);

  static void setProjectionModePolicy(
    ProjectionMode default_mode,
    const std::unordered_set<std::string> & full_se3_ids,
    const std::unordered_set<uint8_t> & full_se3_robot_types);

  static Eigen::Vector3d calculateArmorWorldNormal(
    const rm_interfaces::msg::TrackedRobot & robot,
    int armor_index,
    double dt,
    MotionModel model,
    ProjectionMode projection_mode);

  static double computeFacingCos(
    const Eigen::Vector3d & center,
    const Eigen::Vector3d & armor,
    const Eigen::Vector3d & observer = Eigen::Vector3d::Zero());

  static void syncFullStateFromLegacy(rm_interfaces::msg::TrackedRobot & robot);

  static void syncLegacyStateFromFull(rm_interfaces::msg::TrackedRobot & robot);

  static rm_interfaces::msg::TrackedRobot normalizeState(
    const rm_interfaces::msg::TrackedRobot & robot);

  static Eigen::Vector3d centerPosition(
    const rm_interfaces::msg::TrackedRobot & robot);

  static Eigen::Vector3d linearVelocity(
    const rm_interfaces::msg::TrackedRobot & robot);

  static Eigen::Vector3d linearAcceleration(
    const rm_interfaces::msg::TrackedRobot & robot);

  static Eigen::Vector3d angularVelocity(
    const rm_interfaces::msg::TrackedRobot & robot);

  static Eigen::Vector3d angularAcceleration(
    const rm_interfaces::msg::TrackedRobot & robot);

  static double yaw(const rm_interfaces::msg::TrackedRobot & robot);

  static double yawVelocity(const rm_interfaces::msg::TrackedRobot & robot);

  static double yawAcceleration(const rm_interfaces::msg::TrackedRobot & robot);

  static double centerDistance(const rm_interfaces::msg::TrackedRobot & robot);

  // ── Representation mode ──

  /// Infer representation mode from TrackedRobot fields.
  /// - num_armors == 1 && armors_offset.size() <= 1 → AMBIGUOUS_SINGLE_ARMOR
  /// - num_armors >= 3 → STRUCTURED_ROBOT
  /// - otherwise: fall back to robot_type-based heuristic.
  static RepresentationMode inferRepresentationMode(
      const rm_interfaces::msg::TrackedRobot &robot);

  static bool isSingleArmorRepresentation(
      const rm_interfaces::msg::TrackedRobot &robot);

  /// Accessors that interpret center_* fields as single-armor state when
  /// isSingleArmorRepresentation() is true. They are identity wrappers over
  /// centerPosition/linearVelocity/yaw but carry semantic intent.
  static Eigen::Vector3d singleArmorPosition(
      const rm_interfaces::msg::TrackedRobot &robot);
  static Eigen::Vector3d singleArmorVelocity(
      const rm_interfaces::msg::TrackedRobot &robot);
  static double singleArmorYaw(
      const rm_interfaces::msg::TrackedRobot &robot);
  static Eigen::Vector3d singleArmorNormal(
      const rm_interfaces::msg::TrackedRobot &robot);

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
