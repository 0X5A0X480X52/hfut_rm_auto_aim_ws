#include <gtest/gtest.h>

#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "gimbal_pipeline/core/auto_aim_pipeline.hpp"
#include "gimbal_pipeline/core/pipeline_inbox.hpp"

namespace pipeline = fyt::auto_aim::pipeline;

namespace
{

pipeline::RobotTrack makeTrack(
  const std::string & id, pipeline::TimestampNs stamp, double x, double y,
  double confidence = 1.0)
{
  pipeline::RobotTrack track;
  track.timestamp_ns = stamp;
  track.robot_id = id;
  track.track_state = pipeline::TrackState::TRACKING;
  track.center_position = Eigen::Vector3d(x, y, 0.0);
  track.confidence = confidence;
  track.is_visible = true;
  return track;
}

pipeline::AutoAimPipeline makePipeline(
  int & tracking_calls, std::vector<pipeline::TimestampNs> & processed_stamps)
{
  pipeline::TargetSelectionConfig selection;
  selection.max_distance = 20.0;
  selection.priority_robot_ids = {"hero", "infantry"};

  pipeline::ExternalTargetConfig external;
  external.enabled = true;
  external.timeout_ns = 100;
  external.allowed_ids_by_mode[1] = {"big_buff"};

  return pipeline::AutoAimPipeline(
    [&](const pipeline::ObservationFrame & frame) {
      ++tracking_calls;
      processed_stamps.push_back(frame.timestamp_ns);
      pipeline::RobotTrackSet tracks;
      tracks.timestamp_ns = frame.timestamp_ns;
      tracks.robots.push_back(makeTrack("infantry", frame.timestamp_ns, 2.0, 0.1));
      return tracks;
    },
    pipeline::PriorityTargetSelector(std::move(selection)),
    pipeline::ExternalTargetMerger(std::move(external)),
    [](const pipeline::ControlRequest & request) {
      pipeline::GimbalCommand command;
      command.timestamp_ns = request.now_ns;
      command.fire = request.enabled && request.target.has_value();
      command.target_id = request.target ? request.target->robot_id : std::string();
      return command;
    },
    1000);
}

}  // namespace

TEST(PipelineInbox, KeepsNewestFramesWithinBound)
{
  pipeline::PipelineInbox inbox(2);
  EXPECT_FALSE(inbox.push({1, {}}).dropped_oldest);
  EXPECT_FALSE(inbox.push({2, {}}).dropped_oldest);
  EXPECT_TRUE(inbox.push({3, {}}).dropped_oldest);

  const auto frames = inbox.takeAll();
  ASSERT_EQ(frames.size(), 2U);
  EXPECT_EQ(frames[0].timestamp_ns, 2);
  EXPECT_EQ(frames[1].timestamp_ns, 3);
  EXPECT_EQ(inbox.size(), 0U);
}

TEST(PipelineInbox, SupportsConcurrentProducers)
{
  pipeline::PipelineInbox inbox(200);
  auto producer = [&](pipeline::TimestampNs base) {
      for (pipeline::TimestampNs i = 0; i < 100; ++i) {
        inbox.push({base + i, {}});
      }
    };
  std::thread first(producer, 0);
  std::thread second(producer, 1000);
  first.join();
  second.join();
  EXPECT_EQ(inbox.takeAll().size(), 200U);
}

TEST(AutoAimPipeline, ConsumesEachDetectionFrameOnce)
{
  int tracking_calls = 0;
  std::vector<pipeline::TimestampNs> stamps;
  auto pipeline_core = makePipeline(tracking_calls, stamps);

  pipeline::PipelineCycleInput first;
  first.now_ns = 20;
  first.detection_frames = {{10, {}}, {20, {}}};
  auto result = pipeline_core.runCycle(first);

  EXPECT_EQ(tracking_calls, 2);
  EXPECT_TRUE(result.tracking_updated);
  ASSERT_TRUE(result.selected_target);
  EXPECT_EQ(result.selected_target->robot_id, "infantry");
  EXPECT_TRUE(result.command.fire);

  pipeline::PipelineCycleInput next;
  next.now_ns = 21;
  result = pipeline_core.runCycle(next);
  EXPECT_EQ(tracking_calls, 2);
  EXPECT_FALSE(result.tracking_updated);
  EXPECT_TRUE(result.command.fire);
}

TEST(AutoAimPipeline, RejectsOlderFramesAndIdlesForStaleTarget)
{
  int tracking_calls = 0;
  std::vector<pipeline::TimestampNs> stamps;
  auto pipeline_core = makePipeline(tracking_calls, stamps);

  pipeline::PipelineCycleInput first;
  first.now_ns = 100;
  first.detection_frames = {{100, {}}};
  pipeline_core.runCycle(first);

  pipeline::PipelineCycleInput second;
  second.now_ns = 1200;
  second.detection_frames = {{90, {}}};
  const auto result = pipeline_core.runCycle(second);

  EXPECT_EQ(tracking_calls, 1);
  EXPECT_FALSE(result.command.fire);
  EXPECT_TRUE(result.command.target_id.empty());
  EXPECT_EQ(result.events.front().type, pipeline::PipelineEvent::Type::OUT_OF_ORDER_FRAME);
}

TEST(AutoAimPipeline, MergesAllowedFreshExternalTarget)
{
  int tracking_calls = 0;
  std::vector<pipeline::TimestampNs> stamps;
  auto pipeline_core = makePipeline(tracking_calls, stamps);

  pipeline::PipelineCycleInput input;
  input.now_ns = 50;
  input.mode = 1;
  input.detection_frames = {{50, {}}};
  input.external_targets.timestamp_ns = 50;
  input.external_targets.robots.push_back(makeTrack("big_buff", 50, 1.0, 0.0));
  const auto result = pipeline_core.runCycle(input);

  EXPECT_EQ(result.tracks.robots.size(), 2U);
}

TEST(AutoAimPipeline, InvalidBulletSpeedForcesSafeFireOutput)
{
  int tracking_calls = 0;
  std::vector<pipeline::TimestampNs> stamps;
  auto pipeline_core = makePipeline(tracking_calls, stamps);

  pipeline::PipelineCycleInput input;
  input.now_ns = 50;
  input.gimbal.bullet_speed = 0.0;
  input.detection_frames = {{50, {}}};
  const auto result = pipeline_core.runCycle(input);
  EXPECT_FALSE(result.command.fire);
}
