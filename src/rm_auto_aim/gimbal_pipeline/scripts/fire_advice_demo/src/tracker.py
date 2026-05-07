from __future__ import annotations

import math

import numpy as np

from .geometry import armor_rotation_odom
from .models import ArmorState


class DemoTracker:
    def __init__(self, config: dict, armor_config: dict):
        self.config = config
        self.width = float(armor_config["width"])
        self.height = float(armor_config["height"])
        self.initial = np.asarray(config["target_initial_odom"], dtype=float)
        self.velocity = np.asarray(config["target_velocity_odom"], dtype=float)
        sigmas = np.asarray(config["tracker_covariance_sigma"], dtype=float)
        self.covariance = np.diag(sigmas * sigmas)

    def predict(self, t: float) -> ArmorState:
        center = self.initial + self.velocity * t
        center[1] += self.config["target_motion_amplitude_y"] * math.sin(2.0 * math.pi * self.config["target_motion_frequency_hz"] * t)
        center[2] += self.config["target_motion_amplitude_z"] * math.sin(2.0 * math.pi * self.config["target_motion_frequency_z_hz"] * t)
        yaw = math.pi + self.config["armor_yaw_rate_rad_s"] * t
        return ArmorState(center, armor_rotation_odom(yaw), self.covariance.copy(), self.width, self.height)

