"""
结构参数估计器 (Structural Parameter Estimator)

基于 Robbins-Monro 随机逼近算法，替代原有的递推贝叶斯方法。

专门用于 r1, r2, dza 等结构参数的收敛估计。

动机:
  UKF 中 r1/r2/dza 在单观测模式下被冻结（Kalman增益清零），
  在双观测模式下通过几何约束更新，但更新值噪声较大。
  需要一个独立的估计器将这些参数逐渐收敛到稳定值。

算法:
  Robbins-Monro 随机逼近:
    θ_{n+1} = θ_n + a_n * (z_n - θ_n)
    a_n = c / (n + n0)^γ

  步长衰减保证:
    - Σ a_n = ∞    (能到达任意目标)
    - Σ a_n² < ∞   (噪声消亡)

  优势 (相比递推贝叶斯):
    - 无需维护方差状态
    - 步长衰减策略更直观可调
    - 严格的随机逼近收敛性保证
"""

import numpy as np
from typing import Optional, Dict, Tuple
import logging

from .robbins_monro_estimator import RobbinsMonroEstimator, RobbinsMonroConfig, StructuralRMEstimator

logger = logging.getLogger(__name__)


# Re-export StructuralRMEstimator as StructuralParameterEstimator
# to maintain backward compatibility with existing imports
StructuralParameterEstimator = StructuralRMEstimator
