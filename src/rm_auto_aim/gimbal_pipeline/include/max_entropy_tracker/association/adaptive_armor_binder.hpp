// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_ASSOCIATION_ADAPTIVE_ARMOR_BINDER_HPP_
#define MAX_ENTROPY_TRACKER_ASSOCIATION_ADAPTIVE_ARMOR_BINDER_HPP_

#include <array>
#include <deque>
#include <limits>
#include <optional>
#include <vector>

namespace fyt::auto_aim {

/**
 * Unified adaptive armor binder.
 *
 * Given per-frame panel-association results, this class maintains a "bound"
 * panel identity that resists spurious per-frame flips. A transition is only
 * confirmed when consistent evidence (controlled by the configured strategy)
 * has been observed for a minimum number of consecutive frames.
 *
 * Supported strategies:
 *  - PROXIMITY: jump detection via panel adjacency + z-jump magnitude matching
 *    the EMA estimate (used by the standard 4-panel AdaptiveArmorTracker).
 *  - COST: jump detection via hypothesis cost comparison (used by the 3-panel
 *    OutpostArmorTracker).
 *
 * The binder is panel-count agnostic — N and z_offsets are configured at
 * construction. An optional periodic-evidence module can be enabled for
 * rotating-target scenarios where the z-jump pattern is structured.
 */
class AdaptiveArmorBinder {
 public:
  enum class Strategy { PROXIMITY = 0, COST = 1 };

  /// Per-panel height semantics label.
  /// For 2-layer systems: EVEN=LOWER, ODD=UPPER.
  /// For 3-layer systems: 0=HIGH, 1=MIDDLE, 2=LOW.
  enum class HeightLabel { UNKNOWN = -1, LOWER = 0, UPPER = 1 };

  struct Config {
    // ── Core parameters ──
    int n_panels = 4;                    ///< total number of armor panels
    Strategy strategy = Strategy::PROXIMITY;

    /// z-offset of each panel relative to the robot center.
    /// Index must match panel id: z_offsets[panel_id] = z_armor - z_center.
    std::vector<double> z_offsets;

    // ── State machine ──
    int confirm_frames = 3;              ///< consecutive frames before commit
    int cooldown_frames = 20;            ///< cooldown after a switch (PROXIMITY)

    // ── PROXIMITY gates ──
    double z_jump_min = 0.025;           ///< minimum |dz| to consider a jump (m)
    double yaw_err_gate = 0.17;          ///< max yaw error (rad)
    double cost_margin_min = 0.05;       ///< min association cost margin
    double dz_match_tolerance = 0.015;   ///< matching tolerance for |dz| vs EMA (m)
    double dz_gate = 0.01;              ///< z-jump direction decision gate (m)
    double dz_ema_alpha = 0.20;          ///< EMA learning rate
    double confidence_floor = 0.30;      ///< minimum binding confidence

    // ── COST gates ──
    double min_candidate_prob = 0.40;    ///< minimum candidate ML probability
    double min_candidate_margin = 0.12;  ///< minimum cost margin (best vs 2nd)
    double switch_strong_score = 0.60;   ///< score to confirm a switch
    double same_panel_yaw_gate = 0.35;
    double same_panel_z_gate = 0.08;
    double same_panel_xy_gate = 0.18;

    // ── Periodic evidence (optional) ──
    bool periodic_enable = false;
    int periodic_window = 12;
    double periodic_weight = 0.60;
    double periodic_min_spin_rate = 0.8;
    double periodic_update_min_jump = 0.015;
  };

  struct AssociationDiagnostics {
    int selected_id = -1;
    double selected_yaw_err = std::numeric_limits<double>::quiet_NaN();
    double cost_margin = std::numeric_limits<double>::quiet_NaN();
  };

  /// Per-hypothesis cost detail (COST strategy).
  struct HypothesisScore {
    int panel_id = 0;
    double cost = std::numeric_limits<double>::quiet_NaN();
    double probability = std::numeric_limits<double>::quiet_NaN();
    double center_yaw = std::numeric_limits<double>::quiet_NaN();
    double center_z = std::numeric_limits<double>::quiet_NaN();
    double yaw_err = std::numeric_limits<double>::quiet_NaN();
    double z_state_err = std::numeric_limits<double>::quiet_NaN();
    double xy_residual = std::numeric_limits<double>::quiet_NaN();
  };

  struct BindResult {
    int bound_panel_id = -1;
    HeightLabel bound_height_label = HeightLabel::UNKNOWN;
    double bound_confidence = 0.0;
    bool switch_occurred = false;
    int switch_reason = 0;
  };

  struct DebugSnapshot {
    bool valid = false;
    int bound_panel_id = -1;
    HeightLabel bound_height_label = HeightLabel::UNKNOWN;
    double bound_confidence = std::numeric_limits<double>::quiet_NaN();
    int transition_state = 0;   // 0=LOCKED, 1=TRANSITION_CANDIDATE
    int transition_candidate = -1;
    int transition_confirm_count = 0;
    int cooldown_remaining = 0;
    double dz_jump_est = std::numeric_limits<double>::quiet_NaN();
    double period_confidence = std::numeric_limits<double>::quiet_NaN();
    int period_phase = -1;
    int spin_direction = 0;
  };

  explicit AdaptiveArmorBinder(const Config &config);

  /// Reset all internal state. Must be called on tracker (re-)initialization.
  void reset(int init_panel_id, HeightLabel init_label,
             std::optional<double> obs_z = std::nullopt,
             std::optional<double> obs_time = std::nullopt);

  // ──────────── PROXIMITY-strategy interface ────────────

  /**
   * Bind using proximity-based jump detection (AdaptiveArmorTracker style).
   *
   * @param obs_z                current observation Z
   * @param obs_yaw              current observation yaw
   * @param candidate_panel      raw panel id from PanelAssociator
   * @param diag                 association diagnostics (yaw_err, cost_margin)
   * @param candidate_label      height label from HeightIdentifier
   * @param candidate_h_conf     height-identifier confidence
   * @param result               [out] binding result
   * @return true if a switch was confirmed this frame
   */
  bool bind_proximity(double obs_z, double obs_yaw, int candidate_panel,
                      const AssociationDiagnostics &diag,
                      HeightLabel candidate_label, double candidate_h_conf,
                      BindResult *result);

  // ──────────── COST-strategy interface ────────────

  /**
   * Bind using hypothesis-cost comparison (OutpostArmorTracker style).
   *
   * @param hypotheses           scored panel hypotheses
   * @param predicted_center_z   predicted center z from filter
   * @param center_yaw_est       current center yaw estimate
   * @param result               [out] binding result
   */
  bool bind_cost(const std::vector<HypothesisScore> &hypotheses,
                 double predicted_center_z, double center_yaw_est,
                 BindResult *result);

  // ──────────── Periodic evidence (common) ────────────

  /**
   * Update periodic z-jump evidence model.
   *
   * @param z_jump              latest observed z-jump
   * @param yaw_rate_est        estimated yaw rate (for spin direction)
   * @param allow_model_update  whether to update dz EMA this frame
   */
  void update_periodic_evidence(double z_jump, double yaw_rate_est,
                                bool allow_model_update);

  /**
   * Apply periodic prior cost to a set of hypothesis scores.
   * Adds a weighted penalty to each hypothesis proportional to how much the
   * observed jump deviates from the expected jump for that panel transition.
   */
  void apply_periodic_prior(std::vector<HypothesisScore> &hyps,
                            double z_jump) const;

  // ──────────── Convenience helpers ────────────

  /// Map panel id to default height label for the configured z-offset layout.
  HeightLabel default_label_for_panel(int panel_id) const;

  /// Current bound panel.
  int bound_panel_id() const { return bound_panel_id_; }

  /// Current binding confidence.
  double bound_confidence() const { return bound_confidence_; }

  /// Current height label of the bound panel.
  HeightLabel bound_height_label() const { return bound_height_label_; }

  /// Full debug snapshot.
  DebugSnapshot debug_snapshot() const;

  /// Update the z-jump EMA with a confirmed jump magnitude (PROXIMITY only).
  void update_jump_statistics(double z_jump, bool switch_confirmed);

  /// Compute binding confidence (PROXIMITY only).
  double compute_proximity_confidence(
      const AssociationDiagnostics &diag, bool jump_gate_passed) const;

  /// Compute binding confidence (COST only).
  double compute_cost_confidence(double candidate_prob, double candidate_margin,
                                 double same_panel_score,
                                 double switch_score) const;

  /// Compute same-panel retention score (COST only).
  double compute_same_panel_score(const HypothesisScore &hyp,
                                  double predicted_center_z,
                                  double center_yaw_est) const;

 private:
  enum class BindingState { LOCKED = 0, TRANSITION_CANDIDATE = 1 };

  /// Core state machine: attempt to transition from current bound state.
  /// Returns true if a switch was confirmed.
  bool attempt_transition(int candidate_panel, bool gate_passed,
                          int confirm_required);

  /// PROXIMITY-specific: evaluate all five jump gates.
  bool evaluate_proximity_gates(int candidate_panel, double z_jump,
                                bool has_z_jump,
                                const AssociationDiagnostics &diag) const;

  Config config_;

  // ── Bound state ──
  int bound_panel_id_ = -1;
  HeightLabel bound_height_label_ = HeightLabel::UNKNOWN;
  double bound_confidence_ = 0.0;

  // ── Transition state machine ──
  BindingState binding_state_ = BindingState::LOCKED;
  int transition_candidate_ = -1;
  int transition_confirm_count_ = 0;
  int cooldown_remaining_ = 0;

  // ── Z-jump history ──
  std::optional<double> last_obs_z_;
  std::optional<double> last_obs_time_;
  int last_panel_id_ = -1;
  std::deque<double> z_jump_history_;
  double dz_jump_est_ = std::numeric_limits<double>::quiet_NaN();  // PROXIMITY

  // ── Periodic evidence state ──
  std::deque<double> dz_periodic_history_;
  double dz_small_est_ = std::numeric_limits<double>::quiet_NaN();
  double dz_large_est_ = std::numeric_limits<double>::quiet_NaN();
  int period_update_applied_ = 0;
  int period_phase_index_ = -1;
  double period_confidence_ = std::numeric_limits<double>::quiet_NaN();
  int spin_direction_ = 0;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_ASSOCIATION_ADAPTIVE_ARMOR_BINDER_HPP_
