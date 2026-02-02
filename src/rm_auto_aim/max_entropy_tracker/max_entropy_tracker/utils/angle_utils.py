"""
角度处理工具函数
"""

import numpy as np
from typing import Tuple


def normalize_angle(angle: float) -> float:
    """
    将角度归一化到[-π, π]
    
    Args:
        angle: 输入角度
        
    Returns:
        归一化后的角度
    """
    return np.arctan2(np.sin(angle), np.cos(angle))


def angle_difference(angle1: float, angle2: float) -> float:
    """
    计算两个角度的最小差异
    
    Args:
        angle1: 角度1
        angle2: 角度2
        
    Returns:
        最小角度差 (在[-π, π]范围内)
    """
    diff = angle1 - angle2
    return normalize_angle(diff)


def decompose_yaw(yaw: float) -> Tuple[int, float]:
    """
    将yaw分解为离散k和连续delta
    
    yaw = k * π + delta
    其中 k ∈ {0, 1}, delta ∈ [-π/2, π/2]
    
    Args:
        yaw: 原始yaw角
        
    Returns:
        (k, delta)
    """
    # 先归一化到 [-π, π]
    yaw = normalize_angle(yaw)
    
    # 分解
    if yaw > np.pi / 2:
        k = 1
        delta = yaw - np.pi
    elif yaw <= -np.pi / 2:
        k = 1
        delta = yaw + np.pi
    else:
        k = 0
        delta = yaw
    
    return k, delta


def compose_yaw(k: int, delta: float) -> float:
    """
    从k和delta重建yaw
    
    Args:
        k: 离散模态 {0, 1}
        delta: 连续小角度 [-π/2, π/2]
        
    Returns:
        yaw角度
    """
    yaw = k * np.pi + delta
    return normalize_angle(yaw)


def delta_angle_diff(a: float, b: float) -> float:
    """
    计算delta角度差，映射到[-π/2, π/2]
    
    这是分解UKF特有的角度差计算方式
    
    Args:
        a: 角度a
        b: 角度b
        
    Returns:
        角度差，范围[-π/2, π/2]
    """
    diff = a - b
    while diff > np.pi / 2:
        diff -= np.pi
    while diff < -np.pi / 2:
        diff += np.pi
    return diff


def yaw_k_probability(
    delta_pred: float,
    delta_obs: float,
    k: int,
    sigma: float = 0.1,
    prior_weight: float = 0.7
) -> float:
    """
    计算k的后验概率
    
    使用贝叶斯更新:
    p(k|z) ∝ p(z|k) * p(k)
    
    Args:
        delta_pred: 预测的delta
        delta_obs: 观测的delta
        k: 要计算概率的k值
        sigma: 观测噪声标准差
        prior_weight: 先验权重
        
    Returns:
        k的后验概率
    """
    prior = 0.5
    
    if k == 0:
        diff = angle_difference(delta_obs, delta_pred)
    else:
        diff = angle_difference(delta_obs + np.pi, delta_pred)
    
    # 高斯似然
    likelihood = np.exp(-0.5 * (diff / sigma) ** 2)
    
    # 后验
    posterior = likelihood * prior
    
    return posterior


def select_best_k(
    delta_pred: float,
    delta_obs: float,
    current_k: int,
    sigma: float = 0.1,
    prior_weight: float = 0.7
) -> int:
    """
    选择最优的k值
    
    Args:
        delta_pred: 预测的delta
        delta_obs: 观测的delta
        current_k: 当前的k值
        sigma: 观测噪声
        prior_weight: 先验权重
        
    Returns:
        最优的k值
    """
    prob_k0 = yaw_k_probability(delta_pred, delta_obs, 0, sigma, prior_weight)
    prob_k1 = yaw_k_probability(delta_pred, delta_obs, 1, sigma, prior_weight)
    
    # 添加先验偏向当前k
    if current_k == 0:
        prob_k0 *= (1.0 + prior_weight)
    else:
        prob_k1 *= (1.0 + prior_weight)
    
    # 选择概率更大的
    if prob_k0 > prob_k1:
        return 0
    else:
        return 1
