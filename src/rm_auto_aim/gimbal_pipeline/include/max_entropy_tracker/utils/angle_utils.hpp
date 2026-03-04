// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_UTILS_ANGLE_UTILS_HPP_
#define MAX_ENTROPY_TRACKER_UTILS_ANGLE_UTILS_HPP_

#include <cmath>
#include <utility>

namespace fyt::auto_aim {

/// Normalize angle to [-pi, pi]
inline double normalize_angle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

/// Minimal signed angle difference (a - b), result in [-pi, pi]
inline double angle_difference(double a, double b) {
  return normalize_angle(a - b);
}

/// Decompose yaw = k*pi + delta,  k in {0,1}, delta in [-pi/2, pi/2]
inline std::pair<int, double> decompose_yaw(double yaw) {
  yaw = normalize_angle(yaw);
  int k;
  double delta;
  if (yaw > M_PI / 2.0) {
    k = 1;
    delta = yaw - M_PI;
  } else if (yaw <= -M_PI / 2.0) {
    k = 1;
    delta = yaw + M_PI;
  } else {
    k = 0;
    delta = yaw;
  }
  return {k, delta};
}

/// Reconstruct yaw from k and delta
inline double compose_yaw(int k, double delta) {
  return normalize_angle(k * M_PI + delta);
}

/// Delta-angle difference mapped to [-pi/2, pi/2]
inline double delta_angle_diff(double a, double b) {
  double diff = a - b;
  while (diff > M_PI / 2.0) diff -= M_PI;
  while (diff < -M_PI / 2.0) diff += M_PI;
  return diff;
}

/// Posterior probability of k given delta_pred and delta_obs
inline double yaw_k_probability(double delta_pred, double delta_obs, int k,
                                double sigma = 0.1,
                                double /*prior_weight*/ = 0.7) {
  double diff;
  if (k == 0) {
    diff = angle_difference(delta_obs, delta_pred);
  } else {
    diff = angle_difference(delta_obs + M_PI, delta_pred);
  }
  double likelihood = std::exp(-0.5 * (diff / sigma) * (diff / sigma));
  return likelihood * 0.5;  // prior = 0.5
}

/// Select best k from delta prediction and observation (for single obs update)
inline int select_best_k(double delta_pred, double delta_obs, int current_k,
                         double sigma = 0.1, double prior_weight = 0.7) {
  double prob_k0 = yaw_k_probability(delta_pred, delta_obs, 0, sigma, prior_weight);
  double prob_k1 = yaw_k_probability(delta_pred, delta_obs, 1, sigma, prior_weight);

  if (current_k == 0)
    prob_k0 *= (1.0 + prior_weight);
  else
    prob_k1 *= (1.0 + prior_weight);

  return (prob_k0 > prob_k1) ? 0 : 1;
}

/// Select best k from center_yaw observation (corrected version)
inline int select_best_k_from_center_yaw(double delta_pred,
                                         double center_yaw_obs,
                                         int current_k, double sigma = 0.1,
                                         double prior_weight = 0.7) {
  // k=0: center_yaw = delta_pred
  double diff_k0 = angle_difference(center_yaw_obs, delta_pred);
  double likelihood_k0 = std::exp(-0.5 * (diff_k0 / sigma) * (diff_k0 / sigma));

  // k=1: center_yaw = pi + delta_pred
  double center_yaw_pred_k1 = M_PI + delta_pred;
  double diff_k1 = angle_difference(center_yaw_obs, center_yaw_pred_k1);
  double likelihood_k1 = std::exp(-0.5 * (diff_k1 / sigma) * (diff_k1 / sigma));

  double prob_k0 = likelihood_k0 * 0.5;
  double prob_k1 = likelihood_k1 * 0.5;

  if (current_k == 0)
    prob_k0 *= (1.0 + prior_weight);
  else
    prob_k1 *= (1.0 + prior_weight);

  return (prob_k0 > prob_k1) ? 0 : 1;
}

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_UTILS_ANGLE_UTILS_HPP_
