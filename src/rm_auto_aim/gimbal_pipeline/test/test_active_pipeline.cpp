#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "gimbal_controller/local_trajectory_compensator.hpp"
#include "gimbal_pipeline/adapters/buff_target_adapter.hpp"
#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/trackers/norm4_baseline/backends/norm4_backend_factory.hpp"
#include "max_entropy_tracker/trackers/outpost_tracker_baseline.hpp"
#include "gimbal_pipeline/core/priority_target_selector.hpp"

namespace
{

using fyt::auto_aim::ObservationData;
using fyt::auto_aim::OutpostTrackerBaseline;
using fyt::auto_aim::UnifiedConfig;
using fyt::auto_aim::adapters::BuffTargetAdapter;
using fyt::auto_aim::norm4_baseline::BackendType;

ObservationData makeObservation(double timestamp)
{
  ObservationData observation;
  observation.x = 3.0;
  observation.y = 0.2;
  observation.z = 0.1;
  observation.yaw = 0.0;
  observation.timestamp = timestamp;
  observation.panel_id = 1;
  return observation;
}

fyt::auto_aim::pipeline::RobotTrack makeRobot(
  const std::string & id, double x, double y, double confidence)
{
  fyt::auto_aim::pipeline::RobotTrack robot;
  robot.robot_id = id;
  robot.center_position = Eigen::Vector3d(x, y, 0.0);
  robot.confidence = confidence;
  robot.track_state = fyt::auto_aim::pipeline::TrackState::TRACKING;
  robot.is_visible = true;
  return robot;
}

}  // namespace

TEST(Norm4BackendFactory, CreatesAndAdvancesRetainedBackends)
{
  for (const auto type : {BackendType::UKF_BASELINE, BackendType::INEKF}) {
    auto config = UnifiedConfig::create_default();
    config.norm4_baseline.inekf.enabled = true;
    auto backend = fyt::auto_aim::norm4_baseline::create_backend(type, config, 0.05);

    ASSERT_NE(backend, nullptr);
    backend->reset(makeObservation(1.0), 0, 0.15, 0.20, 0.0);
    ASSERT_TRUE(backend->initialized());
    backend->predict(0.05);

    const auto snapshot = backend->snapshot();
    EXPECT_GT(snapshot.x.size(), 0);
    EXPECT_TRUE(snapshot.x.allFinite());
    EXPECT_TRUE(snapshot.P.allFinite());
  }
}

TEST(OutpostBaseline, InitializesAndUpdatesSingleObservationMode)
{
  auto config = UnifiedConfig::create_default();
  config.outpost.baseline_warmup_enable = false;
  OutpostTrackerBaseline tracker(config, 0.05, false);

  tracker.initialize({makeObservation(1.0)});
  EXPECT_TRUE(tracker.is_initialized());
  EXPECT_TRUE(tracker.is_ambiguous_single_mode());
  EXPECT_EQ(tracker.effective_num_armors(), 1);
  EXPECT_TRUE(tracker.get_center_position().allFinite());
  EXPECT_EQ(tracker.selected_panel_id(), 1);

  tracker.predict(1.05);
  EXPECT_TRUE(tracker.update({makeObservation(1.05)}));
  EXPECT_TRUE(tracker.get_center_position().allFinite());
  EXPECT_FALSE(tracker.build_armors_offset_for_message().empty());
}

TEST(PriorityList, UsesConfiguredPriorityThenYawFallback)
{
  fyt::auto_aim::pipeline::RobotTrackSet robots;
  robots.robots.push_back(makeRobot("3", 5.0, 0.0, 0.9));
  robots.robots.push_back(makeRobot("1", 4.0, 1.0, 0.9));

  fyt::auto_aim::pipeline::TargetSelectionConfig config;
  config.min_confidence = 0.5;
  config.max_distance = 10.0;
  config.priority_robot_ids = {"1", "3"};

  fyt::auto_aim::pipeline::PriorityTargetSelector strategy(config);
  const auto prioritized = strategy.select(robots);
  ASSERT_TRUE(prioritized.has_value());
  EXPECT_EQ(prioritized->robot_id, "1");

  config.priority_robot_ids.clear();
  fyt::auto_aim::pipeline::PriorityTargetSelector fallback_strategy(config);
  const auto yaw_fallback = fallback_strategy.select(robots);
  ASSERT_TRUE(yaw_fallback.has_value());
  EXPECT_EQ(yaw_fallback->robot_id, "3");
}

TEST(LocalTrajectoryCompensator, ProducesFiniteBallisticSolution)
{
  gimbal_controller::LocalTrajectoryCompensator compensator;
  compensator.setParameters(20.0, 9.8, 0.0, 30);

  const auto result = compensator.compensate(Eigen::Vector3d(5.0, 1.0, 0.2));
  EXPECT_TRUE(result.success);
  EXPECT_TRUE(std::isfinite(result.pitch));
  EXPECT_TRUE(std::isfinite(result.yaw));
  EXPECT_GT(result.flight_time, 0.0);
  EXPECT_EQ(compensator.getTrajectory(5.0, result.pitch).size(), 51U);
}

TEST(BuffTargetAdapter, NormalizesFreshTargetAndHonorsDisable)
{
  if (!rclcpp::ok()) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  auto adapter_node = std::make_shared<rclcpp::Node>("buff_adapter_test");
  auto publisher_node = std::make_shared<rclcpp::Node>("buff_publisher_test");

  BuffTargetAdapter::Config config;
  config.enable = true;
  config.topic = "test/buff_target";
  config.timeout_s = 0.5;
  config.target_frame = "odom";
  BuffTargetAdapter adapter(*adapter_node, config);

  auto publisher = publisher_node->create_publisher<rm_interfaces::msg::TrackedRobot>(
    config.topic, rclcpp::SensorDataQoS());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(adapter_node);
  executor.add_node(publisher_node);

  for (int i = 0; i < 50 && publisher->get_subscription_count() == 0; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_GT(publisher->get_subscription_count(), 0U);

  rm_interfaces::msg::TrackedRobot message;
  message.header.stamp = adapter_node->now();
  publisher->publish(message);

  std::optional<rm_interfaces::msg::TrackedRobot> received;
  for (int i = 0; i < 50 && !received.has_value(); ++i) {
    executor.spin_some();
    received = adapter.latestValid(adapter_node->now());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_TRUE(received.has_value());
  EXPECT_EQ(received->robot_id, "big_buff");
  EXPECT_EQ(received->header.frame_id, "odom");
  EXPECT_EQ(received->track_state, rm_interfaces::msg::TrackedRobot::TRACKING);
  EXPECT_EQ(received->num_armors, 1);
  EXPECT_EQ(received->armors_offset.size(), 1U);

  adapter.setEnabled(false);
  EXPECT_FALSE(adapter.latestValid(adapter_node->now()).has_value());

  executor.remove_node(publisher_node);
  executor.remove_node(adapter_node);
}
