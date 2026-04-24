"""
Robbins-Monro 随机逼近估计器 (Stochastic Approximation)

用于结构参数 r1, r2, dza 的渐进收敛估计。

核心迭代:
    θ_{n+1} = θ_n + a_n * (z_n - θ_n)

步长衰减 (Polyak-Ruppert):
    a_n = c / (n + n0)^γ

性质:
    - Σ a_n = ∞    (能到达任意目标)  当 γ ≤ 1
    - Σ a_n² < ∞   (噪声消亡)        当 γ > 0.5
    - γ ∈ (0.5, 1] 同时满足两个条件

双观测增强:
    双观测时 UKF 的几何约束使 r1/r2/dza 更可信,
    步长乘以 dual_obs_boost 系数加速收敛。
"""

import numpy as np
from typing import Tuple, Optional, Dict, Any
from dataclasses import dataclass


@dataclass
class RobbinsMonroConfig:
    """Robbins-Monro 估计器配置"""
    initial_step: float = 0.5       # c — 初始步长
    gamma: float = 0.75             # γ — 衰减指数 ∈ (0.5, 1]
    n0: int = 5                     # 偏移量 (避免初始步长过大)
    dual_obs_boost: float = 3.0     # 双观测步长放大倍数
    min_value: float = -1e9         # 下限约束
    max_value: float = 1e9          # 上限约束
    convergence_eps: float = 1e-4   # 收敛判定阈值


class RobbinsMonroEstimator:
    """
    标量 Robbins-Monro 随机逼近估计器

    用法:
        est = RobbinsMonroEstimator(RobbinsMonroConfig(min_value=0.12, max_value=0.5))
        est.reset(0.15)  # 初始值
        for z in observations:
            est.update(z, is_dual_obs=False)
        print(est.estimate)
    """

    def __init__(self, config: Optional[RobbinsMonroConfig] = None):
        self.cfg = config or RobbinsMonroConfig()
        self._theta: float = 0.0
        self._n: int = 0

    def reset(self, initial_value: float):
        """重置到新的初始值, 步计数归零"""
        self._theta = np.clip(initial_value, self.cfg.min_value, self.cfg.max_value)
        self._n = 0

    def update(self, z_obs: float, is_dual_obs: bool = False) -> float:
        """
        单次 Robbins-Monro 更新

        Args:
            z_obs: 观测值 (来自 UKF 状态)
            is_dual_obs: 是否来自双观测更新 (更可信)

        Returns:
            更新后的估计值
        """
        a_n = self._step_size(is_dual_obs)
        innovation = z_obs - self._theta
        self._theta += a_n * innovation
        self._theta = np.clip(self._theta, self.cfg.min_value, self.cfg.max_value)
        self._n += 1
        return self._theta

    @property
    def estimate(self) -> float:
        return self._theta

    @property
    def step_count(self) -> int:
        return self._n

    def current_step_size(self, is_dual: bool = False) -> float:
        return self._step_size(is_dual)

    @property
    def is_converged(self) -> bool:
        return self._step_size(False) < self.cfg.convergence_eps

    def _step_size(self, is_dual: bool) -> float:
        base = self.cfg.initial_step / ((self._n + self.cfg.n0) ** self.cfg.gamma)
        return base * self.cfg.dual_obs_boost if is_dual else base


class StructuralRMEstimator:
    """
    结构参数 Robbins-Monro 估计器

    管理 r1, r2, dza 三个参数的独立随机逼近。

    替代原有的 StructuralParameterEstimator (递推贝叶斯),
    保持相同的外部接口: initialize / predict / update / get_smoothed_params。
    """

    def __init__(
        self,
        initial_step: float = 0.5,
        gamma: float = 0.75,
        n0: int = 5,
        dual_obs_boost: float = 3.0,
        min_radius: float = 0.12,
        max_radius: float = 0.5,
        min_dz: float = 0.0,
        max_dz: float = 1.0,
        convergence_eps: float = 1e-4,
    ):
        r_cfg = RobbinsMonroConfig(
            initial_step=initial_step,
            gamma=gamma,
            n0=n0,
            dual_obs_boost=dual_obs_boost,
            min_value=min_radius,
            max_value=max_radius,
            convergence_eps=convergence_eps,
        )
        dz_cfg = RobbinsMonroConfig(
            initial_step=initial_step,
            gamma=gamma,
            n0=n0,
            dual_obs_boost=dual_obs_boost,
            min_value=min_dz,
            max_value=max_dz,
            convergence_eps=convergence_eps,
        )
        self.r1_est = RobbinsMonroEstimator(r_cfg)
        self.r2_est = RobbinsMonroEstimator(r_cfg)
        self.dza_est = RobbinsMonroEstimator(dz_cfg)
        self._initialized = False

    def initialize(self, r1: float, r2: float, dza: float,
                   r1_var: float = 0.05, r2_var: float = 0.05,
                   dza_var: float = 0.02):
        """
        初始化估计器

        Args:
            r1, r2, dza: 初始参数值
            r1_var, r2_var, dza_var: 保留参数 (RM 不使用方差, 接口兼容)
        """
        self.r1_est.reset(r1)
        self.r2_est.reset(r2)
        self.dza_est.reset(dza)
        self._initialized = True

    def predict(self):
        """
        预测步 (Robbins-Monro 无预测模型, 空操作)

        保留此接口以兼容 OutputSmoother 调用模式。
        """
        pass

    def update_from_ukf(
        self,
        r1_ukf: float, r2_ukf: float, dza_ukf: float,
        P: Optional[np.ndarray] = None,
        r1_idx: Optional[int] = None,
        r2_idx: Optional[int] = None,
        dza_idx: Optional[int] = None,
        is_dual_obs: bool = False,
        noise_scale: float = 1.0,
    ):
        """
        从 UKF 状态更新结构参数估计

        Args:
            r1_ukf, r2_ukf, dza_ukf: UKF 当前估计值
            P: UKF 协方差 (RM 不直接使用, 接口兼容)
            r1_idx, r2_idx, dza_idx: 参数索引 (RM 不使用)
            is_dual_obs: 是否双观测 (影响步长)
            noise_scale: 保留参数 (RM 不使用)
        """
        if not self._initialized:
            return
        self.r1_est.update(r1_ukf, is_dual_obs)
        self.r2_est.update(r2_ukf, is_dual_obs)
        self.dza_est.update(dza_ukf, is_dual_obs)

    def get_smoothed_params(self) -> Tuple[float, float, float]:
        """返回当前收敛后的参数估计"""
        return (self.r1_est.estimate, self.r2_est.estimate, self.dza_est.estimate)

    def is_converged(self) -> bool:
        return (self.r1_est.is_converged and
                self.r2_est.is_converged and
                self.dza_est.is_converged)

    def reset(self):
        self.r1_est.reset(0.0)
        self.r2_est.reset(0.0)
        self.dza_est.reset(0.0)
        self._initialized = False

    def get_diagnostics(self) -> Dict[str, Any]:
        """诊断信息"""
        return {
            'initialized': self._initialized,
            'r1': {
                'estimate': self.r1_est.estimate,
                'step_count': self.r1_est.step_count,
                'converged': self.r1_est.is_converged,
                'current_step': self.r1_est.current_step_size(),
            },
            'r2': {
                'estimate': self.r2_est.estimate,
                'step_count': self.r2_est.step_count,
                'converged': self.r2_est.is_converged,
                'current_step': self.r2_est.current_step_size(),
            },
            'dza': {
                'estimate': self.dza_est.estimate,
                'step_count': self.dza_est.step_count,
                'converged': self.dza_est.is_converged,
                'current_step': self.dza_est.current_step_size(),
            },
        }
