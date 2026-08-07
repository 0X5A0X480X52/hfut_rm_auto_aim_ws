// Copyright (C) FYT Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#ifndef GIMBAL_PIPELINE__CORE__PIPELINE_INBOX_HPP_
#define GIMBAL_PIPELINE__CORE__PIPELINE_INBOX_HPP_

#include <cstddef>
#include <deque>
#include <mutex>
#include <vector>

#include "gimbal_pipeline/core/pipeline_types.hpp"

namespace fyt::auto_aim::pipeline
{

class PipelineInbox
{
public:
  struct PushResult
  {
    bool dropped_oldest{false};
    std::size_t size{0};
  };

  explicit PipelineInbox(std::size_t capacity = 10)
  : capacity_(capacity == 0 ? 1 : capacity) {}

  PushResult push(ObservationFrame frame);
  std::vector<ObservationFrame> takeAll();
  std::size_t size() const;

private:
  const std::size_t capacity_;
  mutable std::mutex mutex_;
  std::deque<ObservationFrame> frames_;
};

}  // namespace fyt::auto_aim::pipeline

#endif  // GIMBAL_PIPELINE__CORE__PIPELINE_INBOX_HPP_
