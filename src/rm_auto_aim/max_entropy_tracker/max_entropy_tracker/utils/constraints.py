"""
状态约束工具

应用物理约束到状态向量
"""

import numpy as np
from typing import Optional


def apply_radius_constraints(
    radius: float,
    min_radius: float = 0.12,
    max_radius: float = 0.5
) -> float:
    """
    应用半径约束
    
    Args:
        radius: 半径值
        min_radius: 最小半径
        max_radius: 最大半径
        
    Returns:
        约束后的半径
    """
    return np.clip(radius, min_radius, max_radius)


def apply_dz_constraints(
    dz: float,
    min_dz: float = 0.0,
    max_dz: float = 1.0
) -> float:
    """
    应用高度差约束
    
    Args:
        dz: 高度差值
        min_dz: 最小高度差
        max_dz: 最大高度差
        
    Returns:
        约束后的高度差
    """
    return np.clip(dz, min_dz, max_dz)


def apply_state_constraints(
    x: np.ndarray,
    r1_idx: int,
    r2_idx: int,
    dza_idx: int,
    min_radius: float = 0.12,
    max_radius: float = 0.5,
    min_dz: float = 0.0,
    max_dz: float = 1.0
) -> np.ndarray:
    """
    应用状态约束（通用版本，需要提供索引）
    
    Args:
        x: 状态向量
        r1_idx: r1在状态向量中的索引
        r2_idx: r2在状态向量中的索引
        dza_idx: dza在状态向量中的索引
        min_radius: 最小半径
        max_radius: 最大半径
        min_dz: 最小高度差
        max_dz: 最大高度差
        
    Returns:
        约束后的状态向量
    """
    x_constrained = x.copy()
    
    # 半径约束
    x_constrained[r1_idx] = apply_radius_constraints(
        x_constrained[r1_idx], min_radius, max_radius
    )
    x_constrained[r2_idx] = apply_radius_constraints(
        x_constrained[r2_idx], min_radius, max_radius
    )
    
    # 高度差约束
    x_constrained[dza_idx] = apply_dz_constraints(
        x_constrained[dza_idx], min_dz, max_dz
    )
    
    return x_constrained


def ensure_positive_definite(P: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    """
    确保协方差矩阵正定
    
    Args:
        P: 协方差矩阵
        eps: 最小特征值
        
    Returns:
        正定的协方差矩阵
    """
    # 确保对称
    P = (P + P.T) / 2
    
    # 添加小扰动确保正定
    P += np.eye(P.shape[0]) * eps
    
    return P
