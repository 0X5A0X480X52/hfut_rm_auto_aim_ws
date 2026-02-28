"""
结构参数递推贝叶斯估计器 (Recursive Bayesian Estimator)

专门用于 r1, r2, dza 等结构参数的收敛估计。

动机:
  UKF 中 r1/r2/dza 在单观测模式下被冻结（Kalman增益清零），
  在双观测模式下通过几何约束更新，但更新值噪声较大。
  需要一个独立的估计器将这些参数逐渐收敛到稳定值。

方法:
  将每个参数视为一个标量高斯随机变量:
    x ~ N(mu, sigma^2)
  
  每次收到新的观测值 z ~ N(z_obs, R) 时:
    mu_new = (R * mu_old + sigma^2 * z_obs) / (R + sigma^2)
    sigma_new^2 = (R * sigma^2) / (R + sigma^2)

  此外引入过程噪声 Q 防止完全收敛（保持一定适应性），
  但 Q 可以随时间衰减，使参数在观测足够多后近似收敛。

特性:
  - 每个参数独立估计
  - 支持用 UKF 协方差作为观测噪声 R
  - 支持双观测来源给不同权重
  - 参数最终收敛但非完全锁死（process noise 保底）
"""

import numpy as np
from typing import Optional, Dict, Tuple
from dataclasses import dataclass, field
import logging

logger = logging.getLogger(__name__)


@dataclass
class ScalarBayesianEstimator:
    """
    标量递推贝叶斯估计器 (1D Kalman Filter)
    
    状态: x ~ N(mu, variance)
    观测: z ~ N(z_obs, R)
    """
    
    mu: float = 0.0           # 当前估计均值
    variance: float = 1.0     # 当前估计方差
    
    # 过程噪声（防止完全收敛）
    base_process_noise: float = 1e-5
    
    # 过程噪声衰减参数
    # Q(n) = base_Q + decay_Q * decay_rate^n
    decay_process_noise: float = 0.01   # 初始额外过程噪声
    decay_rate: float = 0.95           # 每步衰减率 (越接近1越慢衰减)
    
    # 约束范围
    min_value: float = -np.inf
    max_value: float = np.inf
    
    # 统计
    update_count: int = 0
    
    def predict(self):
        """
        预测步（只增加不确定性）
        
        在结构参数场景下，参数在predict阶段不变化(无过程模型),
        只是增加少量过程噪声以保持适应性。
        """
        Q = self.base_process_noise + self.decay_process_noise * (
            self.decay_rate ** self.update_count
        )
        self.variance += Q
    
    def update(self, z_obs: float, R: float) -> float:
        """
        更新步
        
        Args:
            z_obs: 观测值
            R: 观测噪声方差，越大越不信任观测
            
        Returns:
            更新后的估计值
        """
        if R <= 0:
            R = 1e-6  # 防止除零
        
        # Kalman增益
        K = self.variance / (self.variance + R)
        
        # 更新
        innovation = z_obs - self.mu
        self.mu = self.mu + K * innovation
        self.variance = (1.0 - K) * self.variance
        
        # 约束
        self.mu = np.clip(self.mu, self.min_value, self.max_value)
        
        # 防止方差退化到0
        self.variance = max(self.variance, self.base_process_noise)
        
        self.update_count += 1
        
        return self.mu
    
    @property
    def std(self) -> float:
        return np.sqrt(self.variance)
    
    @property
    def converged(self) -> bool:
        """判断是否已收敛（方差足够小）"""
        return self.variance < 10 * self.base_process_noise
    
    def reset(self, mu: float, variance: float = 1.0):
        """重置估计器"""
        self.mu = mu
        self.variance = variance
        self.update_count = 0


class StructuralParameterEstimator:
    """
    结构参数收敛估计器
    
    管理 r1, r2, dza 三个参数的独立递推估计。
    
    工作流程:
    1. 初始化: 从 UKF 的初始估计值开始
    2. 每帧 predict: 微量增加不确定性
    3. 双观测时 update: 用 UKF 的双观测更新值和协方差作为观测噪声进行更新
    4. 输出: get_smoothed_params() 返回收敛后的参数值
    
    使用建议:
    - 跟踪器初始化时调用 initialize()
    - 每帧调用 predict()
    - 双观测更新成功后调用 update_from_ukf()
    - 获取输出时使用 get_smoothed_params() 替代 UKF 原始值
    """
    
    def __init__(
        self,
        base_process_noise: float = 1e-5,
        decay_process_noise: float = 0.01,
        decay_rate: float = 0.95,
        min_radius: float = 0.12,
        max_radius: float = 0.5,
        min_dz: float = 0.0,
        max_dz: float = 1.0
    ):
        """
        Args:
            base_process_noise: 最低过程噪声（防止完全锁死）
            decay_process_noise: 初始额外过程噪声
            decay_rate: 过程噪声衰减率，∈(0,1)，越大收敛越慢
            min_radius: 最小半径约束
            max_radius: 最大半径约束
            min_dz: 最小高度差约束
            max_dz: 最大高度差约束
        """
        self.r1_est = ScalarBayesianEstimator(
            base_process_noise=base_process_noise,
            decay_process_noise=decay_process_noise,
            decay_rate=decay_rate,
            min_value=min_radius,
            max_value=max_radius,
        )
        self.r2_est = ScalarBayesianEstimator(
            base_process_noise=base_process_noise,
            decay_process_noise=decay_process_noise,
            decay_rate=decay_rate,
            min_value=min_radius,
            max_value=max_radius,
        )
        self.dza_est = ScalarBayesianEstimator(
            base_process_noise=base_process_noise,
            decay_process_noise=decay_process_noise * 0.5,  # dza噪声可以更小
            decay_rate=decay_rate,
            min_value=min_dz,
            max_value=max_dz,
        )
        
        self._initialized = False
    
    def initialize(self, r1: float, r2: float, dza: float,
                   r1_var: float = 0.05, r2_var: float = 0.05,
                   dza_var: float = 0.02):
        """
        初始化估计器
        
        Args:
            r1, r2, dza: 初始参数值（通常来自UKF初始化）
            r1_var, r2_var, dza_var: 初始方差
        """
        self.r1_est.reset(r1, r1_var)
        self.r2_est.reset(r2, r2_var)
        self.dza_est.reset(dza, dza_var)
        self._initialized = True
        
        logger.debug(f"StructuralEstimator initialized: r1={r1:.3f}, r2={r2:.3f}, dza={dza:.3f}")
    
    def predict(self):
        """预测步：微量增加不确定性"""
        if not self._initialized:
            return
        self.r1_est.predict()
        self.r2_est.predict()
        self.dza_est.predict()
    
    def update_from_ukf(
        self,
        r1_ukf: float, r2_ukf: float, dza_ukf: float,
        P: np.ndarray,
        r1_idx: int, r2_idx: int, dza_idx: int,
        is_dual_obs: bool = False,
        noise_scale: float = 1.0
    ):
        """
        从 UKF 状态更新结构参数估计
        
        Args:
            r1_ukf, r2_ukf, dza_ukf: UKF当前估计的参数值
            P: UKF协方差矩阵
            r1_idx, r2_idx, dza_idx: 参数在状态向量中的索引
            is_dual_obs: 是否来自双观测更新（双观测更可信）
            noise_scale: 观测噪声缩放因子，>1 表示更不信任
        """
        if not self._initialized:
            return
        
        # 使用 UKF 的参数协方差作为观测噪声
        # 双观测模式下 UKF 实际更新了参数，其协方差反映了更新质量
        r1_R = P[r1_idx, r1_idx] * noise_scale
        r2_R = P[r2_idx, r2_idx] * noise_scale
        dza_R = P[dza_idx, dza_idx] * noise_scale
        
        if is_dual_obs:
            # 双观测：UKF的参数估计更可信，使用较小的噪声
            r1_R *= 0.5
            r2_R *= 0.5
            dza_R *= 0.5
        else:
            # 单观测：UKF冻结了参数Kalman增益，参数值不变
            # 仍然可以做轻微更新，但使用大噪声
            r1_R *= 10.0
            r2_R *= 10.0
            dza_R *= 10.0
        
        self.r1_est.update(r1_ukf, r1_R)
        self.r2_est.update(r2_ukf, r2_R)
        self.dza_est.update(dza_ukf, dza_R)
    
    def get_smoothed_params(self) -> Tuple[float, float, float]:
        """
        获取平滑后的参数值
        
        Returns:
            (r1, r2, dza) 平滑值
        """
        return (self.r1_est.mu, self.r2_est.mu, self.dza_est.mu)
    
    def get_variances(self) -> Tuple[float, float, float]:
        """获取当前方差"""
        return (self.r1_est.variance, self.r2_est.variance, self.dza_est.variance)
    
    def is_converged(self, threshold_factor: float = 10.0) -> bool:
        """
        判断所有参数是否收敛
        
        Args:
            threshold_factor: 方差收敛阈值因子
        """
        return (
            self.r1_est.converged and 
            self.r2_est.converged and 
            self.dza_est.converged
        )
    
    def get_diagnostics(self) -> Dict:
        """获取诊断信息"""
        return {
            'r1': {'mu': self.r1_est.mu, 'std': self.r1_est.std, 
                   'count': self.r1_est.update_count, 'converged': self.r1_est.converged},
            'r2': {'mu': self.r2_est.mu, 'std': self.r2_est.std,
                   'count': self.r2_est.update_count, 'converged': self.r2_est.converged},
            'dza': {'mu': self.dza_est.mu, 'std': self.dza_est.std,
                    'count': self.dza_est.update_count, 'converged': self.dza_est.converged},
        }
    
    def reset(self):
        """完全重置"""
        self._initialized = False
