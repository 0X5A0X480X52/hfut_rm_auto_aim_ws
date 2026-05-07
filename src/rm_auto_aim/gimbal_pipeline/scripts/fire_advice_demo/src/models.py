from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ArmorState:
    center_odom: np.ndarray
    rotation_odom: np.ndarray
    covariance_odom: np.ndarray
    width: float
    height: float


@dataclass(frozen=True)
class ArmorStateB:
    center: np.ndarray
    right: np.ndarray
    up: np.ndarray
    normal: np.ndarray
    covariance_b: np.ndarray
    width: float
    height: float


@dataclass(frozen=True)
class CandidateDebug:
    tau: float
    p_hit: float
    e_uv: np.ndarray
    sigma_uv: np.ndarray
    flight_time: float
    target_distance: float
    bullet_pos_b: np.ndarray
    bullet_vel_b: np.ndarray
    armor_center_b: np.ndarray
    armor_normal_b: np.ndarray
    normal_velocity: float
    normal_velocity_weight: float


@dataclass(frozen=True)
class FireResult:
    p_hit_window: float
    fire_score: float
    fire_advice: bool
    best_candidate: CandidateDebug
    candidates: list[CandidateDebug]
    elapsed_us: float

