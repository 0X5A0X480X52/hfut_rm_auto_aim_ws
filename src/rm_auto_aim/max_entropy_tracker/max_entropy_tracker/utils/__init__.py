"""
工具模块

角度处理、Sigma点生成、约束应用、虚拟坐标系变换等通用工具
"""

from .angle_utils import (
    normalize_angle,
    angle_difference,
    decompose_yaw,
    compose_yaw,
    select_best_k,
    select_best_k_from_center_yaw,
)
from .sigma_points import SigmaPointGenerator
from .constraints import apply_state_constraints, apply_radius_constraints
from .virtual_frame import (
    VirtualFrameTransform,
    quaternion_to_euler,
    euler_to_quaternion,
    yaw_only_quaternion,
    quaternion_multiply,
    quaternion_inverse,
    quaternion_to_rotation_matrix,
    rotation_matrix_to_quaternion,
)

__all__ = [
    'normalize_angle',
    'angle_difference',
    'decompose_yaw',
    'compose_yaw',
    'select_best_k',
    'select_best_k_from_center_yaw',
    'SigmaPointGenerator',
    'apply_state_constraints',
    'apply_radius_constraints',
    'VirtualFrameTransform',
    'quaternion_to_euler',
    'euler_to_quaternion',
    'yaw_only_quaternion',
    'quaternion_multiply',
    'quaternion_inverse',
    'quaternion_to_rotation_matrix',
    'rotation_matrix_to_quaternion',
]
