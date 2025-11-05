from dataclasses import dataclass
import numpy as np

@dataclass
class ArmorMeasurement:
    """单个装甲板测量数据"""
    position: np.ndarray  # 3D position in base_link
    orientation: np.ndarray  # quaternion [x, y, z, w]
    number: str  # armor number
    armor_type: str  # armor type
    camera_frame: str  # source camera frame
    timestamp: float  # measurement timestamp
    covariance: np.ndarray  # measurement uncertainty 3x3
