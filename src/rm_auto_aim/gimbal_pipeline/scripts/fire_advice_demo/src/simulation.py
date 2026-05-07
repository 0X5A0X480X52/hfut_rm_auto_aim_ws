from __future__ import annotations

import numpy as np

from .estimator import FireProbabilityEstimator
from .tracker import DemoTracker


def run_simulation(config):
    sim_cfg = config.section("simulation")
    tracker = DemoTracker(sim_cfg, config.section("armor"))
    estimator = FireProbabilityEstimator(config)
    fps = float(sim_cfg["fps"])
    dt = 1.0 / fps
    frames = int(float(sim_cfg["duration_s"]) * fps)
    rows = []
    for k in range(frames):
        now = k * dt
        result = estimator.update(tracker, now, dt)
        best = result.best_candidate
        rows.append({
            "t": now,
            "p_hit_window": result.p_hit_window,
            "fire_score": result.fire_score,
            "fire_advice": float(result.fire_advice),
            "elapsed_us": result.elapsed_us,
            "best_tau_ms": best.tau * 1000.0,
            "e_u": best.e_uv[0],
            "e_v": best.e_uv[1],
            "sigma_u": best.sigma_uv[0],
            "sigma_v": best.sigma_uv[1],
            "flight_time_ms": best.flight_time * 1000.0,
            "target_distance": best.target_distance,
            "normal_velocity": best.normal_velocity,
            "armor_width": tracker.width,
            "armor_height": tracker.height,
        })
    return rows


def run_simulation_with_projection(config):
    sim_cfg = config.section("simulation")
    tracker = DemoTracker(sim_cfg, config.section("armor"))
    estimator = FireProbabilityEstimator(config)
    fps = float(sim_cfg["fps"])
    dt = 1.0 / fps
    frames = int(float(sim_cfg["duration_s"]) * fps)
    rows = []
    for k in range(frames):
        now = k * dt
        result = estimator.update(tracker, now, dt)
        best = result.best_candidate
        rows.append({
            "t": now,
            "p_hit_window": result.p_hit_window,
            "fire_score": result.fire_score,
            "fire_advice": float(result.fire_advice),
            "elapsed_us": result.elapsed_us,
            "best_tau_ms": best.tau * 1000.0,
            "e_u": best.e_uv[0],
            "e_v": best.e_uv[1],
            "sigma_u": best.sigma_uv[0],
            "sigma_v": best.sigma_uv[1],
            "flight_time_ms": best.flight_time * 1000.0,
            "target_distance": best.target_distance,
            "normal_velocity": best.normal_velocity,
            "armor_width": tracker.width,
            "armor_height": tracker.height,
        })
    best_row = max(rows, key=lambda r: r["p_hit_window"])
    projection = estimator.simulate_armor_plane_projection(
        tracker,
        now=float(best_row["t"]),
        tau=float(best_row["best_tau_ms"]) / 1000.0,
        random_seed=int(sim_cfg.get("random_seed", 7)),
    )
    return rows, projection


def summarize(rows: list[dict]) -> dict:
    elapsed = np.array([r["elapsed_us"] for r in rows])
    p = np.array([r["p_hit_window"] for r in rows])
    advice = np.array([r["fire_advice"] for r in rows])
    return {
        "frames": len(rows),
        "elapsed_mean_us": float(np.mean(elapsed)),
        "elapsed_p95_us": float(np.percentile(elapsed, 95)),
        "elapsed_max_us": float(np.max(elapsed)),
        "p_hit_mean": float(np.mean(p)),
        "p_hit_max": float(np.max(p)),
        "fire_advice_ratio": float(np.mean(advice)),
    }
