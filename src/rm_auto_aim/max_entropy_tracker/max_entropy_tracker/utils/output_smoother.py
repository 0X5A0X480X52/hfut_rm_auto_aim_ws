"""
输出平滑器 (Output Smoother)

在 UKF + Tracker 的原始输出之上，增加一层后处理滤波，
实现对输出目标的消抖平滑。

设计架构:
┌─────────────────────────────────────────────────────────┐
│                    OutputSmoother                        │
│                                                         │
│  ┌─────────────────────┐  ┌─────────────────────────┐  │
│  │ StructuralEstimator │  │  OneEuro Position (3D)   │  │
│  │  r1, r2, dza        │  │  center_x, y, z          │  │
│  │  (递推贝叶斯收敛)    │  └─────────────────────────┘  │
│  └─────────────────────┘                                │
│                           ┌─────────────────────────┐  │
│                           │  OneEuro Yaw (Angle)     │  │
│                           │  center_yaw              │  │
│                           └─────────────────────────┘  │
│                                                         │
│  ┌─────────────────────────────────────────────────┐   │
│  │           Velocity Smoother (3D)                 │   │
│  │           vx, vy, vz 速度平滑                    │   │
│  └─────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────┘

工作流程:
1. Tracker 完成 predict + update 后，将原始输出传入 smoother
2. smoother 对各分量分别滤波
3. 下游使用 smoother 的输出而非 tracker 的原始输出

注意:
- smoother 不修改 UKF 内部状态（非侵入式）
- smoother 的输出仅用于下游发送（发布 ROS topic 或串口通信）
- UKF 内部的 predict/update 循环仍使用原始状态
"""

import numpy as np
from typing import Optional, Dict, Any, Tuple
from dataclasses import dataclass, field
import logging

from .one_euro_filter import OneEuroFilter, OneEuroFilter3D, OneEuroFilterAngle
from .structural_estimator import StructuralParameterEstimator

logger = logging.getLogger(__name__)


@dataclass
class SmootherConfig:
    """
    输出平滑器配置
    
    参数调节指南:
    - 位置 min_cutoff 越小越平滑，但延迟越大
    - 位置 beta 越大，对快速运动的跟随越好
    - 角度 min_cutoff 通常比位置小（角度抖动更明显）
    - 结构参数 decay_rate 越接近1，收敛越慢但越稳定
    """
    
    # 是否启用各组件
    enable_position_smooth: bool = True
    enable_yaw_smooth: bool = True
    enable_velocity_smooth: bool = True
    enable_structural_convergence: bool = True
    
    # ---- 位置平滑 (OneEuro 3D) ----
    pos_min_cutoff: float = 1.5     # 最小截止频率(Hz), 越小越平滑
    pos_beta: float = 0.01          # 速度响应系数
    pos_d_cutoff: float = 1.0       # 导数滤波截止
    
    # ---- Yaw平滑 (OneEuro Angle) ----
    yaw_min_cutoff: float = 1.0     # 角度比位置更平滑
    yaw_beta: float = 0.005
    yaw_d_cutoff: float = 1.0
    
    # ---- 速度平滑 (OneEuro 3D) ----
    vel_min_cutoff: float = 2.0     # 速度可以稍微灵敏
    vel_beta: float = 0.01
    vel_d_cutoff: float = 1.0
    
    # ---- 结构参数收敛 ----
    struct_base_process_noise: float = 1e-5
    struct_decay_process_noise: float = 0.01
    struct_decay_rate: float = 0.95
    struct_min_radius: float = 0.12
    struct_max_radius: float = 0.5
    struct_min_dz: float = 0.0
    struct_max_dz: float = 1.0
    
    # ---- 初始采样频率估计 ----
    default_freq: float = 30.0       # 默认采样频率(Hz), 会根据实际dt自动调节


@dataclass
class SmoothedOutput:
    """平滑后的输出数据"""
    
    # 位置
    center_position: np.ndarray    # [x, y, z]
    
    # 速度
    velocity: np.ndarray           # [vx, vy, vz]
    
    # 角度
    yaw: float
    yaw_velocity: float            # 不在 smoother 内滤波，直接透传
    
    # 结构参数
    r1: float
    r2: float
    dza: float
    
    # 原始值 (用于调试对比)
    raw_center_position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    raw_yaw: float = 0.0
    raw_r1: float = 0.0
    raw_r2: float = 0.0
    raw_dza: float = 0.0
    
    # 收敛状态
    structural_converged: bool = False


class OutputSmoother:
    """
    输出平滑器
    
    非侵入式地对 Tracker 输出进行消抖平滑。
    
    使用示例:
    ```python
    smoother = OutputSmoother(SmootherConfig())
    smoother.initialize(r1=0.15, r2=0.20, dza=0.0)
    
    # 在每帧 tracker 更新后:
    result = smoother.smooth(
        center_pos=tracker.get_center_position(),
        yaw=tracker.get_yaw(),
        velocity=np.array([vx, vy, vz]),
        yaw_velocity=delta_rate,
        r1=r1, r2=r2, dza=dza,
        P=tracker.ukf.P,
        r1_idx=..., r2_idx=..., dza_idx=...,
        is_dual_obs=False,
        timestamp=current_time
    )
    
    # 使用 result.center_position, result.yaw 等
    ```
    """
    
    def __init__(self, config: Optional[SmootherConfig] = None):
        self.config = config or SmootherConfig()
        
        freq = self.config.default_freq
        
        # 位置平滑器
        self._pos_filter = OneEuroFilter3D(
            freq=freq,
            min_cutoff=self.config.pos_min_cutoff,
            beta=self.config.pos_beta,
            d_cutoff=self.config.pos_d_cutoff,
        )
        
        # Yaw平滑器
        self._yaw_filter = OneEuroFilterAngle(
            freq=freq,
            min_cutoff=self.config.yaw_min_cutoff,
            beta=self.config.yaw_beta,
            d_cutoff=self.config.yaw_d_cutoff,
        )
        
        # 速度平滑器
        self._vel_filter = OneEuroFilter3D(
            freq=freq,
            min_cutoff=self.config.vel_min_cutoff,
            beta=self.config.vel_beta,
            d_cutoff=self.config.vel_d_cutoff,
        )
        
        # 结构参数收敛器
        self._struct_est = StructuralParameterEstimator(
            base_process_noise=self.config.struct_base_process_noise,
            decay_process_noise=self.config.struct_decay_process_noise,
            decay_rate=self.config.struct_decay_rate,
            min_radius=self.config.struct_min_radius,
            max_radius=self.config.struct_max_radius,
            min_dz=self.config.struct_min_dz,
            max_dz=self.config.struct_max_dz,
        )
        
        self._initialized = False
        self._frame_count = 0
    
    def initialize(self, r1: float, r2: float, dza: float,
                   r1_var: float = 0.05, r2_var: float = 0.05,
                   dza_var: float = 0.02):
        """
        初始化平滑器
        
        应在 Tracker 初始化后立即调用。
        
        Args:
            r1, r2, dza: 初始结构参数值
            r1_var, r2_var, dza_var: 初始方差
        """
        # 重置所有滤波器
        self._pos_filter.reset()
        self._yaw_filter.reset()
        self._vel_filter.reset()
        
        # 初始化结构参数估计
        if self.config.enable_structural_convergence:
            self._struct_est.initialize(r1, r2, dza, r1_var, r2_var, dza_var)
        
        self._initialized = True
        self._frame_count = 0
        
        logger.debug(f"OutputSmoother initialized: r1={r1:.3f}, r2={r2:.3f}, dza={dza:.3f}")
    
    def smooth(
        self,
        center_pos: np.ndarray,
        yaw: float,
        velocity: np.ndarray,
        yaw_velocity: float,
        r1: float,
        r2: float,
        dza: float,
        P: Optional[np.ndarray] = None,
        r1_idx: Optional[int] = None,
        r2_idx: Optional[int] = None,
        dza_idx: Optional[int] = None,
        is_dual_obs: bool = False,
        timestamp: Optional[float] = None
    ) -> SmoothedOutput:
        """
        对一帧输出进行平滑
        
        Args:
            center_pos: 中心位置 [x, y, z]
            yaw: 中心yaw角 (弧度)
            velocity: 速度 [vx, vy, vz]
            yaw_velocity: yaw角速度 (弧度/秒)
            r1, r2, dza: 当前UKF结构参数估计
            P: UKF协方差矩阵（用于结构参数更新, 可选）
            r1_idx, r2_idx, dza_idx: 参数在状态向量中的索引（可选）
            is_dual_obs: 本帧是否是双观测更新
            timestamp: 时间戳 (秒)
            
        Returns:
            SmoothedOutput 包含平滑后的所有输出
        """
        if not self._initialized:
            # 未初始化时直接透传
            return SmoothedOutput(
                center_position=center_pos.copy(),
                velocity=velocity.copy(),
                yaw=yaw,
                yaw_velocity=yaw_velocity,
                r1=r1, r2=r2, dza=dza,
                raw_center_position=center_pos.copy(),
                raw_yaw=yaw,
                raw_r1=r1, raw_r2=r2, raw_dza=dza,
            )
        
        self._frame_count += 1
        
        # ---- 位置平滑 ----
        if self.config.enable_position_smooth:
            smooth_pos = self._pos_filter.filter(center_pos, timestamp)
        else:
            smooth_pos = center_pos.copy()
        
        # ---- Yaw平滑 ----
        if self.config.enable_yaw_smooth:
            smooth_yaw = self._yaw_filter.filter(yaw, timestamp)
        else:
            smooth_yaw = yaw
        
        # ---- 速度平滑 ----
        if self.config.enable_velocity_smooth:
            smooth_vel = self._vel_filter.filter(velocity, timestamp)
        else:
            smooth_vel = velocity.copy()
        
        # ---- 结构参数收敛 ----
        smooth_r1, smooth_r2, smooth_dza = r1, r2, dza
        struct_converged = False
        
        if self.config.enable_structural_convergence:
            # 预测步
            self._struct_est.predict()
            
            # 更新步
            if P is not None and r1_idx is not None:
                self._struct_est.update_from_ukf(
                    r1_ukf=r1, r2_ukf=r2, dza_ukf=dza,
                    P=P,
                    r1_idx=r1_idx, r2_idx=r2_idx, dza_idx=dza_idx,
                    is_dual_obs=is_dual_obs,
                )
            
            smooth_r1, smooth_r2, smooth_dza = self._struct_est.get_smoothed_params()
            struct_converged = self._struct_est.is_converged()
        
        return SmoothedOutput(
            center_position=smooth_pos,
            velocity=smooth_vel,
            yaw=smooth_yaw,
            yaw_velocity=yaw_velocity,  # 直接透传（来自UKF状态）
            r1=smooth_r1,
            r2=smooth_r2,
            dza=smooth_dza,
            raw_center_position=center_pos.copy(),
            raw_yaw=yaw,
            raw_r1=r1,
            raw_r2=r2,
            raw_dza=dza,
            structural_converged=struct_converged,
        )
    
    def predict_only(self, timestamp: Optional[float] = None):
        """
        仅预测/时间推进（无观测时调用）
        
        在 tracker 只做 predict 没有 update 时调用，
        保持 smoother 的时间同步。
        """
        if self.config.enable_structural_convergence:
            self._struct_est.predict()
    
    def reset(self):
        """完全重置"""
        self._pos_filter.reset()
        self._yaw_filter.reset()
        self._vel_filter.reset()
        self._struct_est.reset()
        self._initialized = False
        self._frame_count = 0
    
    @property
    def structural_estimator(self) -> StructuralParameterEstimator:
        """获取结构参数估计器（用于诊断）"""
        return self._struct_est
    
    def get_diagnostics(self) -> Dict[str, Any]:
        """获取诊断信息"""
        diag = {
            'frame_count': self._frame_count,
            'initialized': self._initialized,
        }
        
        if self.config.enable_structural_convergence:
            diag['structural'] = self._struct_est.get_diagnostics()
        
        return diag
