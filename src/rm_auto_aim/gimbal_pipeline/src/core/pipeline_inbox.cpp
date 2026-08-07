// Copyright (C) FYT Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#include "gimbal_pipeline/core/pipeline_inbox.hpp"

#include <utility>

namespace fyt::auto_aim::pipeline
{

PipelineInbox::PushResult PipelineInbox::push(ObservationFrame frame)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const bool dropped = frames_.size() >= capacity_;
  if (dropped) {
    frames_.pop_front();
  }
  frames_.push_back(std::move(frame));
  return PushResult{dropped, frames_.size()};
}

std::vector<ObservationFrame> PipelineInbox::takeAll()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<ObservationFrame> result;
  result.reserve(frames_.size());
  while (!frames_.empty()) {
    result.push_back(std::move(frames_.front()));
    frames_.pop_front();
  }
  return result;
}

std::size_t PipelineInbox::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return frames_.size();
}

}  // namespace fyt::auto_aim::pipeline
