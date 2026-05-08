// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V2_NORM4_PHASE_SEQUENCE_MEMORY_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V2_NORM4_PHASE_SEQUENCE_MEMORY_HPP_

#include <deque>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/pose_tracker_backend/single/single_tracker_types.hpp"
#include "max_entropy_tracker/trackers/norm4_v2/norm4_observation_frontend.hpp"

namespace fyt::auto_aim::norm4_v2 {

enum class PingPongReason {
  NONE = 0,
  ABAB_PATTERN = 1,
  OPPOSITE_JUMP = 2,
  KINEMATIC_INCONSISTENCY = 3,
};

struct PingPongRisk {
  double risk_score = 0.0;
  bool pending = false;
  bool should_hold = false;
  PingPongReason reason = PingPongReason::NONE;
  int consistent_frames_needed = 0;
};

/// Sequence memory window for detecting ping-pong and opposite-jump patterns.
class PhaseMemoryWindow {
 public:
  struct Entry {
    int panel_id = -1;
    double confidence = 0.0;
    double timestamp = 0.0;
  };

  explicit PhaseMemoryWindow(int window_size);

  void push(int panel_id, double confidence, double timestamp);
  int size() const { return static_cast<int>(buffer_.size()); }
  void clear();

  /// Detect ABAB or ABA (partial) ping-pong pattern.
  /// Returns true and sets pattern_length if a ping-pong is found.
  bool detect_ping_pong(int &pattern_length) const;

  /// Detect opposite-height jump (0↔2 or 1↔3) in the most recent transition.
  bool detect_opposite_jump() const;

  /// Returns the most recent panel_id, or -1 if empty.
  int last_panel() const;

  const std::deque<Entry> &buffer() const { return buffer_; }

 private:
  int window_size_;
  std::deque<Entry> buffer_;
};

/// Joint sequence + kinematic consistency discriminator.
///
/// Used by Norm4BinderBridge to suppress 0101 ping-pong false cycles.
class PhaseSequenceMemory {
 public:
  explicit PhaseSequenceMemory(const PhaseMemoryConfig &config);

  /// Push the current panel candidate and assess risk.
  /// candidate_panel_id: the panel currently proposed by the binder.
  /// kin_summary: kinematic summary from the corresponding proxy (Phase 3).
  PingPongRisk assess(int candidate_panel_id, double candidate_confidence,
                      double timestamp,
                      const KinematicSummary *kin_summary);

  /// Manual reset (e.g. after hard rebind).
  void reset();

  const PhaseMemoryWindow &window() const { return window_; }
  int consistent_counter() const { return consistent_counter_; }
  int hold_counter() const { return hold_counter_; }

 private:
  static bool are_opposite_panels(int a, int b);

  PhaseMemoryConfig config_;
  PhaseMemoryWindow window_;
  int consistent_counter_ = 0;
  int hold_counter_ = 0;
  int last_panel_ = -1;
};

}  // namespace fyt::auto_aim::norm4_v2

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V2_NORM4_PHASE_SEQUENCE_MEMORY_HPP_
