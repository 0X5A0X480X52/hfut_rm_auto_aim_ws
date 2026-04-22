"""
统一配置类

整合UKF和Tracker的配置，使用策略模式支持不同版本
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from ..utils.output_smoother import SmootherConfig


class TranslationModel(Enum):
    """平移运动模型类型"""
    CV = "CV"       # 常速度模型
    CA = "CA"       # 常加速度模型
    Singer = "Singer"  # Singer机动模型


class FilterType(Enum):
    """滤波器类型"""
    STANDARD = "standard"      # 标准UKF
    DECOMPOSED = "decomposed"  # 分解UKF (yaw = k*π + delta)


@dataclass
class UKFParameters:
    """UKF核心参数"""
    
    # Sigma点采样参数
    alpha: float = 0.001
    beta: float = 2.0
    kappa: float = 0.0
    
    # 标准单次更新观测噪声
    obs_noise_pos: float = 0.05
    obs_noise_yaw: float = 0.05
    
    # 双观测更新观测噪声
    dual_obs_noise_pos: float = 0.01
    dual_obs_noise_yaw: float = 0.03
    dual_obs_geometry_noise_scale: float = 0.2
    
    # 单观测更新权重（针对单装甲板观测时的位置更新）
    single_obs_update_weight_pos: float = 0.05
    
    # 创新向量gating（异常值检测）
    enable_innovation_gating: bool = False
    innovation_gate_chi2_threshold: float = 9.49


@dataclass
class MotionModelParameters:
    """运动模型参数"""
    
    # 平移运动模型
    translation_model: TranslationModel = TranslationModel.CA
    
    # CV模型参数
    cv_process_noise_vel: float = 0.5
    
    # CA模型参数
    ca_process_noise_acc: float = 1.0
    
    # Singer模型参数
    singer_alpha: float = 0.5
    singer_sigma: float = 2.0
    
    # 结构参数过程噪声
    process_noise_r: float = 0.02
    process_noise_dz: float = 0.005


@dataclass
class SpinModelParameters:
    """自旋模型参数"""
    
    # 角速度和角加速度噪声
    spin_process_noise_yaw_rate: float = 0.3
    spin_process_noise_yaw_acc: float = 1.0
    
    # delta角速度和角加速度噪声（分解UKF专用）
    spin_process_noise_delta_rate: float = 0.3
    spin_process_noise_delta_acc: float = 3.0


@dataclass
class MaxEntropyParameters:
    """最大熵配置"""
    
    temperature: float = 2.0
    use_adaptive: bool = True
    
    # k先验权重（分解UKF专用）
    k_prior_weight: float = 0.7


@dataclass
class TrackerParameters:
    """跟踪器参数"""
    
    # 状态机阈值
    tracking_thres: int = 2
    lost_thres: int = 8
    temp_lost_thres: int = 3
    
    # 数据关联
    max_match_distance: float = 2.0
    max_match_yaw_diff: float = 1.0
    
    # 装甲板配置（4面固定结构）
    n_panels: int = 4
    panel_angle_step: float = 1.5707963267948966  # π/2


@dataclass
class ConstraintParameters:
    """约束参数"""
    
    min_radius: float = 0.12
    max_radius: float = 0.5
    min_dz: float = -1.0
    max_dz: float = 1.0


@dataclass
class UnifiedConfig:
    """
    统一配置类
    
    设计原则:
    1. 模块化: 每类参数独立分组
    2. 通用性: 支持不同类型的UKF
    3. 可扩展: 易于添加新参数
    """
    
    # 滤波器类型
    filter_type: FilterType = FilterType.DECOMPOSED

    # 基础时间步长 (当未在 Tracker/UKF 初始化时传入 dt，将使用此值)
    dt: float = 0.05
    
    # 各模块参数
    ukf: UKFParameters = field(default_factory=UKFParameters)
    motion: MotionModelParameters = field(default_factory=MotionModelParameters)
    spin: SpinModelParameters = field(default_factory=SpinModelParameters)
    entropy: MaxEntropyParameters = field(default_factory=MaxEntropyParameters)
    tracker: TrackerParameters = field(default_factory=TrackerParameters)
    constraints: ConstraintParameters = field(default_factory=ConstraintParameters)
    smoother: SmootherConfig = field(default_factory=SmootherConfig)
    
    @classmethod
    def create_default(cls) -> 'UnifiedConfig':
        """创建默认配置"""
        return cls(filter_type=FilterType.DECOMPOSED)
    
    @classmethod
    def create_optimized(cls) -> 'UnifiedConfig':
        """创建优化后的配置"""
        config = cls(filter_type=FilterType.DECOMPOSED)
        config.motion.ca_process_noise_acc = 1.5
        config.spin.spin_process_noise_delta_rate = 1.2
        config.spin.spin_process_noise_delta_acc = 12.0
        config.ukf.obs_noise_pos = 0.008
        config.ukf.obs_noise_yaw = 0.015
        config.motion.process_noise_r = 0.008
        config.motion.process_noise_dz = 0.003
        config.entropy.temperature = 1.5
        config.entropy.k_prior_weight = 0.6
        return config
