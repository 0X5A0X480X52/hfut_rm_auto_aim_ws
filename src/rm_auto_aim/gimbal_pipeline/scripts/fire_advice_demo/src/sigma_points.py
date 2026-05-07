from __future__ import annotations

import numpy as np


def unscented_sigma_points(cov: np.ndarray, alpha: float, beta: float, kappa: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    n = cov.shape[0]
    lam = alpha * alpha * (n + kappa) - n
    scale = n + lam
    jitter = 1e-12 * np.eye(n)
    root = np.linalg.cholesky(scale * cov + jitter)
    points = [np.zeros(n)]
    for i in range(n):
        points.append(root[:, i])
    for i in range(n):
        points.append(-root[:, i])
    wm = np.full(2 * n + 1, 1.0 / (2.0 * scale))
    wc = wm.copy()
    wm[0] = lam / scale
    wc[0] = lam / scale + 1.0 - alpha * alpha + beta
    return np.asarray(points), wm, wc


def fixed_five_points(sigma_v0: float, sigma_delay: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    points = np.array([
        [0.0, 0.0],
        [sigma_v0, 0.0],
        [-sigma_v0, 0.0],
        [0.0, sigma_delay],
        [0.0, -sigma_delay],
    ])
    weights = np.full(5, 0.2)
    return points, weights, weights.copy()

