"""
结构参数模型组件

用于估计机器人的结构参数，使用随机游走模型
包括:
- DualRadiusModel: 双半径参数 (r1, r2)
- HeightModel: 高度参数 (dza)
- StructuralModel: 组合结构参数
"""

import numpy as np
from typing import Optional, List
from dataclasses import dataclass

from .base import ProcessModelComponent, ComponentStateSpec


@dataclass
class StructuralConfig:
    """结构参数模型配置"""
    # 半径过程噪声
    process_noise_r: float = 0.02
    
    # 高度差过程噪声
    process_noise_dz: float = 0.005
    
    # 初始半径值
    initial_r1: float = 0.15
    initial_r2: float = 0.20
    
    # 初始高度差
    initial_dza: float = 0.0


class DualRadiusModel(ProcessModelComponent):
    """
    双半径参数模型
    
    状态: [r1, r2] (2维)
    
    物理含义:
    - r1: 偶数panel (0, 2) 使用的半径
    - r2: 奇数panel (1, 3) 使用的半径
    
    运动模型: 随机游走
        r1_k+1 = r1_k + w_r1
        r2_k+1 = r2_k + w_r2
    """
    
    STATE_NAMES = ['R1', 'R2']
    
    def __init__(self, config: Optional[StructuralConfig] = None):
        super().__init__(config)
        self._config = config or StructuralConfig()
    
    def get_state_spec(self) -> ComponentStateSpec:
        return ComponentStateSpec.from_names(self.STATE_NAMES)
    
    def predict(
        self, 
        x_component: np.ndarray, 
        dt: float,
        full_state: Optional[np.ndarray] = None
    ) -> np.ndarray:
        """随机游走: 状态保持不变（噪声在Q中体现）"""
        return x_component.copy()
    
    def build_Q(self, dt: float) -> np.ndarray:
        """构建随机游走过程噪声矩阵"""
        q_r = self._config.process_noise_r
        
        # 随机游走噪声与dt成正比
        Q = np.diag([q_r**2 * dt, q_r**2 * dt])
        
        return Q
    
    def get_initial_state(self) -> np.ndarray:
        """初始状态"""
        return np.array([self._config.initial_r1, self._config.initial_r2])
    
    def get_initial_covariance(self) -> np.ndarray:
        """初始协方差"""
        return np.diag([0.01, 0.01])  # 半径初始不确定性较小


class HeightModel(ProcessModelComponent):
    """
    高度参数模型
    
    状态: [dza] (1维)
    
    物理含义:
    - dza: 装甲板完整高度差（upper层+dza, lower层-dza）
    
    运动模型: 随机游走
        dza_k+1 = dza_k + w_dza
    """
    
    STATE_NAMES = ['DZA']
    
    def __init__(self, config: Optional[StructuralConfig] = None):
        super().__init__(config)
        self._config = config or StructuralConfig()
    
    def get_state_spec(self) -> ComponentStateSpec:
        return ComponentStateSpec.from_names(self.STATE_NAMES)
    
    def predict(
        self, 
        x_component: np.ndarray, 
        dt: float,
        full_state: Optional[np.ndarray] = None
    ) -> np.ndarray:
        """随机游走: 状态保持不变"""
        return x_component.copy()
    
    def build_Q(self, dt: float) -> np.ndarray:
        """构建随机游走过程噪声矩阵"""
        q_dz = self._config.process_noise_dz
        
        # 高度差变化较慢，噪声较小
        Q = np.array([[q_dz**2 * dt * 0.1]])
        
        return Q
    
    def get_initial_state(self) -> np.ndarray:
        """初始状态"""
        return np.array([self._config.initial_dza])
    
    def get_initial_covariance(self) -> np.ndarray:
        """初始协方差"""
        return np.array([[0.05]])  # dza初始不确定性


class StructuralModel(ProcessModelComponent):
    """
    组合结构参数模型
    
    状态: [r1, r2, dza] (3维)
    
    将双半径和高度参数合并为一个组件
    """
    
    STATE_NAMES = ['R1', 'R2', 'DZA']
    
    def __init__(self, config: Optional[StructuralConfig] = None):
        super().__init__(config)
        self._config = config or StructuralConfig()
        
        # 内部子模型
        self._radius_model = DualRadiusModel(config)
        self._height_model = HeightModel(config)
    
    def get_state_spec(self) -> ComponentStateSpec:
        return ComponentStateSpec.from_names(self.STATE_NAMES)
    
    def predict(
        self, 
        x_component: np.ndarray, 
        dt: float,
        full_state: Optional[np.ndarray] = None
    ) -> np.ndarray:
        """随机游走: 状态保持不变"""
        return x_component.copy()
    
    def build_Q(self, dt: float) -> np.ndarray:
        """构建组合过程噪声矩阵"""
        Q_r = self._radius_model.build_Q(dt)
        Q_h = self._height_model.build_Q(dt)
        
        Q = np.zeros((3, 3))
        Q[:2, :2] = Q_r
        Q[2, 2] = Q_h[0, 0]
        
        return Q
    
    def get_initial_state(self) -> np.ndarray:
        """初始状态"""
        return np.array([
            self._config.initial_r1,
            self._config.initial_r2,
            self._config.initial_dza
        ])
    
    def get_initial_covariance(self) -> np.ndarray:
        """初始协方差"""
        return np.diag([0.01, 0.01, 0.05])
