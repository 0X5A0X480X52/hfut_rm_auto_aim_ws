from __future__ import annotations

import math

import numpy as np

from .geometry import clamp


def solve_flight_time(x: float, v0: float, gx: float, solve_with_gx: bool = True) -> float:
    if v0 <= 0.0 or x <= 0.0:
        return 0.0
    if not solve_with_gx or abs(gx) < 1e-7:
        return x / v0
    disc = v0 * v0 + 2.0 * gx * x
    if disc <= 0.0:
        return x / v0
    root = (-v0 + math.sqrt(disc)) / gx
    if root <= 0.0 or not math.isfinite(root):
        return x / v0
    return root


def bullet_mean(muzzle_pos_b: np.ndarray, v0: float, gravity_b: np.ndarray, flight_time: float) -> tuple[np.ndarray, np.ndarray]:
    t = flight_time
    pos = muzzle_pos_b + np.array([v0 * t, 0.0, 0.0]) + 0.5 * gravity_b * t * t
    vel = np.array([v0, 0.0, 0.0]) + gravity_b * t
    return pos, vel


def bullet_covariance_b(flight_time: float, params: dict) -> np.ndarray:
    model = params.get("model", "linear_std")
    min_sigma = float(params["min_sigma"])
    max_sigma = float(params["max_sigma"])
    if model == "quadratic_variance":
        variances = np.array([
            params["sigma_x0"] ** 2 + params["q_x"] * flight_time * flight_time,
            params["sigma_y0"] ** 2 + params["q_y"] * flight_time * flight_time,
            params["sigma_z0"] ** 2 + params["q_z"] * flight_time * flight_time,
        ])
        sigmas = np.sqrt(np.maximum(variances, min_sigma * min_sigma))
    else:
        sigmas = np.array([
            params["sigma_x0"] + params["growth_x"] * flight_time,
            params["sigma_y0"] + params["growth_y"] * flight_time,
            params["sigma_z0"] + params["growth_z"] * flight_time,
        ])
    sigmas = np.array([clamp(float(s), min_sigma, max_sigma) for s in sigmas])
    return np.diag(sigmas * sigmas)

