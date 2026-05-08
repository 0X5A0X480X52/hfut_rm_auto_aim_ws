// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/trackers/norm4_v2/norm4_phase_sequence_memory.hpp"

#include <algorithm>
#include <cmath>

namespace fyt::auto_aim::norm4_v2 {

// ──── PhaseMemoryWindow ────

PhaseMemoryWindow::PhaseMemoryWindow(int window_size)
    : window_size_(std::max(3, window_size)) {}

void PhaseMemoryWindow::push(int panel_id, double confidence,
                             double timestamp) {
  buffer_.push_back({panel_id, confidence, timestamp});
  if (static_cast<int>(buffer_.size()) > window_size_) {
    buffer_.pop_front();
  }
}

void PhaseMemoryWindow::clear() { buffer_.clear(); }

int PhaseMemoryWindow::last_panel() const {
  return buffer_.empty() ? -1 : buffer_.back().panel_id;
}

bool PhaseMemoryWindow::detect_ping_pong(int &pattern_length) const {
  if (buffer_.size() < 4) return false;

  // Look for ABAB pattern starting from the most recent frames.
  // Check the last 4 entries: A B A B
  const int n = static_cast<int>(buffer_.size());
  for (int start = n - 4; start >= 0; --start) {
    int a = buffer_[start].panel_id;
    int b = buffer_[start + 1].panel_id;
    int c = buffer_[start + 2].panel_id;
    int d = buffer_[start + 3].panel_id;

    if (a < 0 || b < 0 || c < 0 || d < 0) continue;
    if (a != b && a == c && b == d) {
      pattern_length = 4;
      return true;
    }
  }

  // Also check ABA (3-frame partial ping-pong).
  if (buffer_.size() >= 3) {
    int a = buffer_[n - 3].panel_id;
    int b = buffer_[n - 2].panel_id;
    int c = buffer_[n - 1].panel_id;
    if (a >= 0 && b >= 0 && c >= 0 && a != b && a == c) {
      pattern_length = 3;
      return true;
    }
  }

  return false;
}

bool PhaseMemoryWindow::detect_opposite_jump() const {
  if (buffer_.size() < 2) return false;
  int prev = buffer_[buffer_.size() - 2].panel_id;
  int curr = buffer_.back().panel_id;
  if (prev < 0 || curr < 0) return false;
  return std::abs(prev - curr) == 2;
}

// ──── PhaseSequenceMemory ────

bool PhaseSequenceMemory::are_opposite_panels(int a, int b) {
  if (a < 0 || b < 0) return false;
  return (a == 0 && b == 2) || (a == 2 && b == 0) ||
         (a == 1 && b == 3) || (a == 3 && b == 1);
}

PhaseSequenceMemory::PhaseSequenceMemory(const PhaseMemoryConfig &config)
    : config_(config), window_(config.sequence_window_size) {}

PingPongRisk PhaseSequenceMemory::assess(
    int candidate_panel_id, double candidate_confidence, double timestamp,
    const KinematicSummary *kin_summary) {
  PingPongRisk risk;

  if (!config_.enable_phase_memory || candidate_panel_id < 0) {
    return risk;
  }

  window_.push(candidate_panel_id, candidate_confidence, timestamp);

  // ── Sequence checks ──
  int pattern_len = 0;
  bool has_ping_pong = window_.detect_ping_pong(pattern_len);
  bool has_opposite_jump =
      config_.enable_opposite_jump_detect && window_.detect_opposite_jump();

  double seq_risk = 0.0;
  if (has_ping_pong) seq_risk = pattern_len >= 4 ? 0.8 : 0.5;
  if (has_opposite_jump) seq_risk = std::max(seq_risk, 0.6);

  // ── Kinematic checks ──
  double kin_risk = 0.0;
  if (kin_summary && config_.enable_kinematic_anti_pingpong) {
    bool kin_trigger = false;

    // Velocity direction sudden reversal.
    if (kin_summary->velocity_dir_cos <
        config_.anti_pingpong.velocity_dir_cos_min) {
      kin_risk = std::max(kin_risk, 0.7);
      kin_trigger = true;
    }

    // High jerk estimate (via acc_norm_window as proxy).
    if (kin_summary->acc_norm_window > config_.anti_pingpong.jerk_gate) {
      kin_risk = std::max(kin_risk, 0.6);
      kin_trigger = true;
    }

    // Yaw rate discontinuity.
    if (kin_summary->yaw_rate_continuity_score < 0.3) {
      kin_risk = std::max(kin_risk, 0.6);
      kin_trigger = true;
    }

    if (!kin_trigger) kin_risk = 0.0;
  }

  // ── Joint assessment ──
  risk.risk_score = std::max(seq_risk, kin_risk);

  if (seq_risk > 0.5 && kin_risk > 0.5) {
    risk.reason = PingPongReason::ABAB_PATTERN;
    if (has_opposite_jump)
      risk.reason = PingPongReason::OPPOSITE_JUMP;
    if (kin_risk > 0.6)
      risk.reason = PingPongReason::KINEMATIC_INCONSISTENCY;

    risk.pending = true;
    risk.should_hold = true;
    risk.consistent_frames_needed =
        config_.anti_pingpong.min_consistent_frames_to_commit;
    consistent_counter_ = 0;
  } else if (seq_risk > 0.3 && kin_risk > 0.3) {
    risk.risk_score = std::max(seq_risk, kin_risk);
    risk.pending = true;
    risk.consistent_frames_needed =
        config_.anti_pingpong.min_consistent_frames_to_commit;
  } else {
    // Consistent frame: count toward commit.
    if (risk.pending || consistent_counter_ > 0) {
      consistent_counter_++;
      if (consistent_counter_ >=
          config_.anti_pingpong.min_consistent_frames_to_commit) {
        risk.pending = false;
        risk.should_hold = false;
        risk.risk_score = 0.0;
        consistent_counter_ = 0;
      }
    }
  }

  // Hold counter management.
  if (risk.should_hold) {
    hold_counter_++;
  } else {
    hold_counter_ = 0;
  }

  // Pending timeout: force release after too many hold frames.
  if (hold_counter_ > config_.anti_pingpong.pending_timeout_frames) {
    risk.should_hold = false;
    risk.pending = false;
    hold_counter_ = 0;
  }

  last_panel_ = candidate_panel_id;
  return risk;
}

void PhaseSequenceMemory::reset() {
  window_.clear();
  consistent_counter_ = 0;
  hold_counter_ = 0;
  last_panel_ = -1;
}

}  // namespace fyt::auto_aim::norm4_v2
