from __future__ import annotations

import math

import numpy as np


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def rot_y(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def look_at_barrel_rotation(target_odom: np.ndarray) -> np.ndarray:
    yaw = math.atan2(target_odom[1], target_odom[0])
    flat = math.hypot(target_odom[0], target_odom[1])
    pitch = math.atan2(target_odom[2], flat)
    return rot_z(yaw) @ rot_y(-pitch)


def armor_rotation_odom(yaw: float) -> np.ndarray:
    right = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    up = np.array([0.0, 0.0, 1.0])
    normal = np.cross(right, up)
    normal /= np.linalg.norm(normal)
    return np.column_stack((right, up, normal))


def transform_point_to_b(point_odom: np.ndarray, r_b_o: np.ndarray, p_b_odom: np.ndarray) -> np.ndarray:
    return r_b_o @ (point_odom - p_b_odom)


def transform_rotation_to_b(r_a_o: np.ndarray, r_b_o: np.ndarray) -> np.ndarray:
    return r_b_o @ r_a_o

