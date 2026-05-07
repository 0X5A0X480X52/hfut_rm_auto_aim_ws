from __future__ import annotations

import math
import time

import numpy as np

from .ballistics import bullet_covariance_b, bullet_mean, solve_flight_time
from .geometry import clamp, look_at_barrel_rotation, transform_point_to_b, transform_rotation_to_b
from .models import ArmorState, ArmorStateB, CandidateDebug, FireResult
from .probability import hit_probability_independent
from .sigma_points import fixed_five_points, unscented_sigma_points


class FireProbabilityEstimator:
    def __init__(self, config):
        self.config = config
        self.fire_probability = config.section("fire_probability")
        self.ballistic = config.section("ballistic")
        self.ballistic_uncertainty = config.section("ballistic_uncertainty")
        self.tracker_uncertainty = config.section("tracker_uncertainty")
        self.normal_velocity_weight = config.section("normal_velocity_weight")
        self.fire_gate = config.section("fire_gate")
        self.sigma_cfg = config.section("sigma_point_extension")
        self.score = 0.0
        self.fire_state = False
        self.system_delay_s = self.fire_probability["system_delay_ms"] / 1000.0
        self.fire_delay_s = self.fire_probability["fire_delay_ms"] / 1000.0
        self.v0 = float(self.ballistic["bullet_speed"])
        self.gravity = float(self.ballistic["gravity"])
        self.muzzle_pos_b = np.asarray(self.ballistic["muzzle_pos_b"], dtype=float)
        self.solve_with_gx = bool(self.ballistic["solve_with_gx"])
        self.use_normal_velocity_weight = bool(self.normal_velocity_weight.get("enable", False))
        self.normal_v_ref = float(self.normal_velocity_weight["v_ref"])
        self.normal_w_min = float(self.normal_velocity_weight["w_min"])
        self.use_sigma_points = bool(self.sigma_cfg.get("enable", False))
        self.min_sigma_bullet_speed = float(self.sigma_cfg.get("min_bullet_speed", 5.0))
        self.sigma_points, self.sigma_wm, self.sigma_wc = self._build_sigma_points()
        window = self.fire_probability["future_window_ms"] / 1000.0
        step = self.fire_probability["future_step_ms"] / 1000.0
        self.future_offsets = np.arange(0.0, window + 0.5 * step, step)

    def update(self, tracker, now: float, dt: float) -> FireResult:
        start = time.perf_counter()
        candidates = [self._evaluate_candidate(tracker, now, float(tau)) for tau in self.future_offsets]
        if self.fire_probability.get("window_fusion", "max") == "softmax":
            beta = float(self.fire_probability["softmax_beta"])
            weights = np.exp(beta * np.array([c.p_hit for c in candidates]))
            p_window = float(np.dot([c.p_hit for c in candidates], weights) / np.sum(weights))
            best = max(candidates, key=lambda c: c.p_hit)
        else:
            best = max(candidates, key=lambda c: c.p_hit)
            p_window = best.p_hit
        self._update_gate(p_window, dt)
        elapsed_us = (time.perf_counter() - start) * 1e6
        return FireResult(p_window, self.score, self.fire_state, best, candidates, elapsed_us)

    def _update_gate(self, p_window: float, dt: float) -> None:
        if self.fire_gate.get("mode", "lowpass") == "integrator":
            base = float(self.fire_gate["integrator_base_probability"])
            rise = float(self.fire_gate["integrator_rise"])
            fall = float(self.fire_gate["integrator_fall"])
            self.score = clamp(self.score + dt * (rise * max(0.0, p_window - base) - fall * max(0.0, base - p_window)), 0.0, 1.0)
        else:
            alpha = float(self.fire_gate["alpha"])
            self.score = alpha * self.score + (1.0 - alpha) * p_window
        if not self.fire_state and self.score > self.fire_gate["fire_on_th"]:
            self.fire_state = True
        elif self.fire_state and self.score < self.fire_gate["fire_off_th"]:
            self.fire_state = False

    def _evaluate_candidate(self, tracker, now: float, tau: float) -> CandidateDebug:
        fire_time = now + tau
        guess = tracker.predict(fire_time + self.system_delay_s + self.fire_delay_s)
        aim_point = self._gravity_compensated_aim_point(guess.center_odom)
        r_o_b = look_at_barrel_rotation(aim_point)
        r_b_o = r_o_b.T
        p_b_odom = np.zeros(3)
        gravity_b = r_b_o @ np.array([0.0, 0.0, -self.gravity])
        armor_guess_b = self._armor_to_b(guess, r_b_o, p_b_odom)
        tf = solve_flight_time(armor_guess_b.center[0], self.v0, gravity_b[0], self.solve_with_gx)
        impact_time = fire_time + self.system_delay_s + self.fire_delay_s + tf

        if self.use_sigma_points:
            e_uv, sigma_sigma = self._sigma_point_error(tracker, fire_time, impact_time, tf, r_b_o, p_b_odom, gravity_b)
            armor = tracker.predict(impact_time)
            armor_b = self._armor_to_b(armor, r_b_o, p_b_odom)
        else:
            armor = tracker.predict(impact_time)
            armor_b = self._armor_to_b(armor, r_b_o, p_b_odom)
            bullet_pos, _ = bullet_mean(self.muzzle_pos_b, self.v0, gravity_b, tf)
            e_uv = self._project_error(bullet_pos, armor_b)
            sigma_sigma = np.zeros((2, 2))

        bullet_pos, bullet_vel = bullet_mean(self.muzzle_pos_b, self.v0, gravity_b, tf)
        j = np.vstack((armor_b.right, armor_b.up))
        sigma_bullet_uv = j @ bullet_covariance_b(tf, self.ballistic_uncertainty) @ j.T
        sigma_tracker_uv = j @ self._tracker_covariance_b(armor_b.covariance_b) @ j.T
        sigma_total = sigma_bullet_uv + sigma_tracker_uv + sigma_sigma
        sigma_u = math.sqrt(max(float(sigma_total[0, 0]), 1e-12))
        sigma_v = math.sqrt(max(float(sigma_total[1, 1]), 1e-12))
        p_hit = hit_probability_independent(float(e_uv[0]), float(e_uv[1]), sigma_u, sigma_v, armor_b.width, armor_b.height)
        normal_velocity = max(0.0, -float(np.dot(bullet_vel, armor_b.normal)))
        normal_weight = 1.0
        if self.use_normal_velocity_weight:
            normal_weight = clamp(normal_velocity / self.normal_v_ref, self.normal_w_min, 1.0)
            p_hit *= normal_weight
        return CandidateDebug(
            tau=tau,
            p_hit=p_hit,
            e_uv=e_uv,
            sigma_uv=np.array([sigma_u, sigma_v]),
            flight_time=tf,
            target_distance=float(armor_b.center[0]),
            bullet_pos_b=bullet_pos,
            bullet_vel_b=bullet_vel,
            armor_center_b=armor_b.center,
            armor_normal_b=armor_b.normal,
            normal_velocity=normal_velocity,
            normal_velocity_weight=normal_weight,
        )

    def _sigma_point_error(self, tracker, fire_time, impact_time, tf_nominal, r_b_o, p_b_odom, gravity_b):
        errors = []
        for dv0, dtd in self.sigma_points:
            v0 = max(self.v0 + float(dv0), self.min_sigma_bullet_speed)
            guess = tracker.predict(impact_time)
            guess_b = self._armor_to_b(guess, r_b_o, p_b_odom)
            tf = solve_flight_time(guess_b.center[0], v0, gravity_b[0], self.solve_with_gx)
            impact_j = fire_time + self.system_delay_s + self.fire_delay_s + float(dtd) + tf
            armor_j = self._armor_to_b(tracker.predict(impact_j), r_b_o, p_b_odom)
            bullet_pos, _ = bullet_mean(self.muzzle_pos_b, v0, gravity_b, tf)
            errors.append(self._project_error(bullet_pos, armor_j))
        errors = np.asarray(errors)
        mean = np.sum(errors * self.sigma_wm[:, None], axis=0)
        cov_uv = np.zeros((2, 2))
        for err, weight in zip(errors, self.sigma_wc):
            d = (err - mean).reshape(2, 1)
            cov_uv += weight * (d @ d.T)
        cov_uv = 0.5 * (cov_uv + cov_uv.T)
        return mean, cov_uv

    def _build_sigma_points(self):
        sigma_v0 = float(self.sigma_cfg.get("sigma_v0", 0.0))
        sigma_delay = float(self.sigma_cfg.get("sigma_delay", 0.0))
        rho = float(self.sigma_cfg.get("rho", 0.0))
        cov = np.array([[sigma_v0 * sigma_v0, rho * sigma_v0 * sigma_delay], [rho * sigma_v0 * sigma_delay, sigma_delay * sigma_delay]])
        if self.sigma_cfg.get("method", "unscented") == "fixed":
            return fixed_five_points(sigma_v0, sigma_delay)
        return unscented_sigma_points(cov, float(self.sigma_cfg.get("alpha", 0.7)), float(self.sigma_cfg.get("beta", 2.0)), float(self.sigma_cfg.get("kappa", 0.0)))

    def _armor_to_b(self, armor: ArmorState, r_b_o: np.ndarray, p_b_odom: np.ndarray) -> ArmorStateB:
        center = transform_point_to_b(armor.center_odom, r_b_o, p_b_odom)
        r_a_b = transform_rotation_to_b(armor.rotation_odom, r_b_o)
        cov_b = r_b_o @ armor.covariance_odom @ r_b_o.T
        return ArmorStateB(center, r_a_b[:, 0], r_a_b[:, 1], r_a_b[:, 2], cov_b, armor.width, armor.height)

    def _tracker_covariance_b(self, covariance_b: np.ndarray) -> np.ndarray:
        if self.tracker_uncertainty.get("use_tracker_covariance", True):
            return covariance_b
        sigmas = np.array([
            self.tracker_uncertainty["fallback_sigma_x"],
            self.tracker_uncertainty["fallback_sigma_y"],
            self.tracker_uncertainty["fallback_sigma_z"],
        ])
        return np.diag(sigmas * sigmas)

    @staticmethod
    def _project_error(bullet_pos_b: np.ndarray, armor_b: ArmorStateB) -> np.ndarray:
        delta = bullet_pos_b - armor_b.center
        return np.array([float(np.dot(armor_b.right, delta)), float(np.dot(armor_b.up, delta))])

    def _gravity_compensated_aim_point(self, target_odom: np.ndarray) -> np.ndarray:
        distance = float(np.linalg.norm(target_odom))
        tf = distance / max(self.v0, 1e-6)
        compensated = target_odom.copy()
        compensated[2] += 0.5 * self.gravity * tf * tf
        return compensated

    def simulate_armor_plane_projection(
        self,
        tracker,
        now: float,
        tau: float,
        n_steps: int = 90,
        n_mc: int = 160,
        random_seed: int = 7,
    ) -> dict:
        fire_time = now + tau
        guess = tracker.predict(fire_time + self.system_delay_s + self.fire_delay_s)
        aim_point = self._gravity_compensated_aim_point(guess.center_odom)
        r_o_b = look_at_barrel_rotation(aim_point)
        r_b_o = r_o_b.T
        p_b_odom = np.zeros(3)
        gravity_b = r_b_o @ np.array([0.0, 0.0, -self.gravity])

        armor_guess_b = self._armor_to_b(guess, r_b_o, p_b_odom)
        tf = solve_flight_time(armor_guess_b.center[0], self.v0, gravity_b[0], self.solve_with_gx)
        impact_time = fire_time + self.system_delay_s + self.fire_delay_s + tf
        armor_impact_b = self._armor_to_b(tracker.predict(impact_time), r_b_o, p_b_odom)
        j = np.vstack((armor_impact_b.right, armor_impact_b.up))

        ts = np.linspace(0.0, tf, n_steps)
        mean_uv = np.zeros((n_steps, 2))
        sigma_uv = np.zeros((n_steps, 2))
        for i, t in enumerate(ts):
            bullet_pos, _ = bullet_mean(self.muzzle_pos_b, self.v0, gravity_b, float(t))
            mean_uv[i] = self._project_error(bullet_pos, armor_impact_b)
            sigma_bullet_uv = j @ bullet_covariance_b(float(t), self.ballistic_uncertainty) @ j.T
            sigma_uv[i, 0] = math.sqrt(max(float(sigma_bullet_uv[0, 0]), 1e-12))
            sigma_uv[i, 1] = math.sqrt(max(float(sigma_bullet_uv[1, 1]), 1e-12))

        rng = np.random.default_rng(random_seed)
        samples = np.zeros((n_mc, n_steps, 2))
        for i in range(n_steps):
            su = sigma_uv[i, 0]
            sv = sigma_uv[i, 1]
            samples[:, i, 0] = rng.normal(mean_uv[i, 0], su, size=n_mc)
            samples[:, i, 1] = rng.normal(mean_uv[i, 1], sv, size=n_mc)

        return {
            "t_rel": ts,
            "mean_uv": mean_uv,
            "sigma_uv": sigma_uv,
            "samples_uv": samples,
            "armor_width": armor_impact_b.width,
            "armor_height": armor_impact_b.height,
            "flight_time": tf,
            "impact_time": impact_time,
            "tau": tau,
            "now": now,
        }
