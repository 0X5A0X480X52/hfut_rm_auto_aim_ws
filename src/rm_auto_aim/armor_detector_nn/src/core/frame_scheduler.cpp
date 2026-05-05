#include "armor_detector_nn/core/frame_scheduler.hpp"

namespace fyt::auto_aim {

void LatestFrameScheduler::submit(FramePacket frame) {
  (void)frame;
}

std::optional<FrameBatch> LatestFrameScheduler::waitBatch() {
  return std::nullopt;
}

SchedulerStats LatestFrameScheduler::stats() const {
  return SchedulerStats{};
}

void DynamicBatchScheduler::submit(FramePacket frame) {
  (void)frame;
}

std::optional<FrameBatch> DynamicBatchScheduler::waitBatch() {
  return std::nullopt;
}

SchedulerStats DynamicBatchScheduler::stats() const {
  return SchedulerStats{};
}

}  // namespace fyt::auto_aim
