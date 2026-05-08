// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_PIPELINE_DEBUG_TRACE_HPP_
#define MAX_ENTROPY_TRACKER_PIPELINE_DEBUG_TRACE_HPP_

#include <string>
#include <vector>

namespace fyt::auto_aim::pipeline {

/// Snapshot of one pipeline stage's inputs/outputs for a single frame.
struct StageTrace {
  std::string stage_name;
  bool active = false;

  double duration_us = 0.0;
  bool ok = true;
  std::string error;

  // Stage-specific scalar outputs (generic key-value for debug tooling).
  struct KeyValue {
    std::string key;
    double value;
  };
  std::vector<KeyValue> scalars;

  void add(const std::string &k, double v) {
    scalars.push_back({k, v});
  }
};

/// Full per-frame pipeline debug trace.
struct PipelineDebugTrace {
  double timestamp = 0.0;
  bool valid = false;

  std::vector<StageTrace> stages;

  void reset() {
    valid = false;
    timestamp = 0.0;
    stages.clear();
  }

  StageTrace &add_stage(const std::string &name) {
    stages.push_back(StageTrace{});
    stages.back().stage_name = name;
    return stages.back();
  }
};

}  // namespace fyt::auto_aim::pipeline

#endif  // MAX_ENTROPY_TRACKER_PIPELINE_DEBUG_TRACE_HPP_
