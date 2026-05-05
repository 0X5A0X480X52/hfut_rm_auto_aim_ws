#ifndef ARMOR_DETECTOR_NN_FRAME_SCHEDULER_HPP_
#define ARMOR_DETECTOR_NN_FRAME_SCHEDULER_HPP_

#include <cstdint>
#include <optional>
#include <vector>

#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

namespace fyt::auto_aim {

struct FramePacket {
  std_msgs::msg::Header header;
  sensor_msgs::msg::Image::ConstSharedPtr image_msg;
  rclcpp::Time receive_time;
  uint64_t sequence_id{0};
};

struct FrameBatch {
  std::vector<FramePacket> frames;
};

struct SchedulerStats {
  uint64_t received_frames{0};
  uint64_t dropped_old_frames{0};
  uint64_t dropped_stale_frames{0};
  uint64_t inferred_frames{0};
  double queue_age_ms{0.0};
};

class FrameScheduler {
public:
  virtual ~FrameScheduler() = default;
  virtual void submit(FramePacket frame) = 0;
  virtual std::optional<FrameBatch> waitBatch() = 0;
  virtual SchedulerStats stats() const = 0;
};

// Reserved scheduler skeletons.
class LatestFrameScheduler : public FrameScheduler {
public:
  void submit(FramePacket frame) override;
  std::optional<FrameBatch> waitBatch() override;
  SchedulerStats stats() const override;
};

class DynamicBatchScheduler : public FrameScheduler {
public:
  void submit(FramePacket frame) override;
  std::optional<FrameBatch> waitBatch() override;
  SchedulerStats stats() const override;
};

}  // namespace fyt::auto_aim

#endif
