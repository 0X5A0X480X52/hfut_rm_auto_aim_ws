import numpy as np
from typing import List, Tuple
from scipy.optimize import least_squares
from .types import ArmorMeasurement

class BundleAdjustment:
    """小型Bundle Adjustment优化器"""
    
    @staticmethod
    def optimize_position(measurements: List[ArmorMeasurement]) -> Tuple[np.ndarray, float]:
        if len(measurements) == 0:
            return np.zeros(3), float('inf')
        if len(measurements) == 1:
            return measurements[0].position, 0.0

        positions = np.array([m.position for m in measurements])
        weights = np.array([1.0 / np.trace(m.covariance) for m in measurements])
        weights /= weights.sum()
        x0 = np.average(positions, axis=0, weights=weights)

        def residuals(x):
            res = []
            for m in measurements:
                diff = x - m.position
                try:
                    info_matrix = np.linalg.inv(m.covariance)
                    weighted = float(np.sqrt(diff @ info_matrix @ diff))
                    res.append(weighted)
                except np.linalg.LinAlgError:
                    res.append(float(np.linalg.norm(diff)))
            return np.array(res)

        result = least_squares(residuals, x0, method='lm')
        return result.x, result.cost

    @staticmethod
    def fuse_orientation(measurements: List[ArmorMeasurement]) -> np.ndarray:
        if len(measurements) == 0:
            return np.array([0.0, 0.0, 0.0, 1.0])
        if len(measurements) == 1:
            return measurements[0].orientation
        quaternions = np.array([m.orientation for m in measurements])
        for i in range(1, len(quaternions)):
            if np.dot(quaternions[0], quaternions[i]) < 0:
                quaternions[i] = -quaternions[i]
        avg_quat = np.mean(quaternions, axis=0)
        avg_quat /= np.linalg.norm(avg_quat)
        return avg_quat
