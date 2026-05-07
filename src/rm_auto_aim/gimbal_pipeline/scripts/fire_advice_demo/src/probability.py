from __future__ import annotations

import math

from .geometry import clamp


def normal_cdf(x: float) -> float:
    return 0.5 * (1.0 + math.erf(x / math.sqrt(2.0)))


def probability_inside_1d(mean: float, sigma: float, half_size: float) -> float:
    sigma = max(float(sigma), 1e-6)
    upper = (half_size - mean) / sigma
    lower = (-half_size - mean) / sigma
    return clamp(normal_cdf(upper) - normal_cdf(lower), 0.0, 1.0)


def hit_probability_independent(e_u: float, e_v: float, sigma_u: float, sigma_v: float, width: float, height: float) -> float:
    p_u = probability_inside_1d(e_u, sigma_u, width * 0.5)
    p_v = probability_inside_1d(e_v, sigma_v, height * 0.5)
    return clamp(p_u * p_v, 0.0, 1.0)

