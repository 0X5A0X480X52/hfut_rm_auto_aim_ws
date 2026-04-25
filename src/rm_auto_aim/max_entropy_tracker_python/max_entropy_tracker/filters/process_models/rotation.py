"""
旋转运动模型组件

支持 CV (常角速度)、CA (常角加速度) 两种模型
专门用于 delta 角度的预测（Yaw分解后的连续角度部分）
"""

import numpy as np
from enum import Enum
from typing import Optional, Any
from dataclasses import dataclass

from .base import ProcessModelComponent, ComponentStateSpec


class RotationModelType(Enum):
    """旋转运动模型类型"""
    CV = "cv"  # 常角速度模型
    CA = "ca"  # 常角加速度模型


@dataclass
class RotationConfig:
    """旋转模型配置"""
    # CV模型参数
    cv_process_noise_rate: float = 0.5  # 角速度过程噪声
    
    # CA模型参数
    ca_process_noise_acc: float = 1.0  # 角加速度过程噪声


class CVRotation(ProcessModelComponent):
    """
    常角速度(CV)旋转模型
    
    状态: [delta, delta_rate] (2维)
    
    运动方程:
        delta_k+1 = delta_k + delta_rate_k * dt
        delta_rate_k+1 = delta_rate_k
    
    过程噪声: 连续白噪声角加速度模型
        Q = q^2 * [[dt^3/3, dt^2/2],
                   [dt^2/2, dt    ]]
    
    注意: delta是Yaw分解后的连续角度，范围在[-π/2, π/2]
    """
    
    STATE_NAMES = ['DELTA', 'DELTA_RATE']
    
    def __init__(self, config: Optional[RotationConfig] = None):
        super().__init__(config)
        self._config = config or RotationConfig()
    
    def get_state_spec(self) -> ComponentStateSpec:
        return ComponentStateSpec.from_names(self.STATE_NAMES)
    
    def predict(
        self, 
        x_component: np.ndarray, 
        dt: float,
        full_state: Optional[np.ndarray] = None
    ) -> np.ndarray:
        """CV旋转模型预测"""
        x_next = x_component.copy()
        # delta_k+1 = delta_k + delta_rate_k * dt
        x_next[0] = x_component[0] + x_component[1] * dt
        # delta_rate保持不变
        return x_next
    
    def build_Q(self, dt: float) -> np.ndarray:
        """构建CV旋转模型过程噪声矩阵"""
        q = self._config.cv_process_noise_rate
        
        Q = q**2 * np.array([
            [dt**3 / 3, dt**2 / 2],
            [dt**2 / 2, dt]
        ])
        
        return Q
    
    def get_initial_covariance(self) -> np.ndarray:
        """初始协方差"""
        return np.diag([0.3, 0.5])  # delta, delta_rate


class CARotation(ProcessModelComponent):
    """
    常角加速度(CA)旋转模型
    
    状态: [delta, delta_rate, delta_acc] (3维)
    
    运动方程:
        delta_k+1 = delta_k + delta_rate_k * dt + 0.5 * delta_acc_k * dt^2
        delta_rate_k+1 = delta_rate_k + delta_acc_k * dt
        delta_acc_k+1 = delta_acc_k
    
    过程噪声: 连续白噪声jerk模型
    """
    
    STATE_NAMES = ['DELTA', 'DELTA_RATE', 'DELTA_ACC']
    
    def __init__(self, config: Optional[RotationConfig] = None):
        super().__init__(config)
        self._config = config or RotationConfig()
    
    def get_state_spec(self) -> ComponentStateSpec:
        return ComponentStateSpec.from_names(self.STATE_NAMES)
    
    def predict(
        self, 
        x_component: np.ndarray, 
        dt: float,
        full_state: Optional[np.ndarray] = None
    ) -> np.ndarray:
        """CA旋转模型预测"""
        x_next = x_component.copy()
        
        delta = x_component[0]
        delta_rate = x_component[1]
        delta_acc = x_component[2]
        
        # delta_k+1 = delta_k + delta_rate_k * dt + 0.5 * delta_acc_k * dt^2
        x_next[0] = delta + delta_rate * dt + 0.5 * delta_acc * dt**2
        # delta_rate_k+1 = delta_rate_k + delta_acc_k * dt
        x_next[1] = delta_rate + delta_acc * dt
        # delta_acc保持不变
        
        return x_next
    
    def build_Q(self, dt: float) -> np.ndarray:
        """构建CA旋转模型过程噪声矩阵"""
        q = self._config.ca_process_noise_acc
        
        Q = q**2 * np.array([
            [dt**5 / 20, dt**4 / 8, dt**3 / 6],
            [dt**4 / 8,  dt**3 / 3, dt**2 / 2],
            [dt**3 / 6,  dt**2 / 2, dt]
        ])
        
        return Q
    
    def get_initial_covariance(self) -> np.ndarray:
        """初始协方差"""
        return np.diag([0.3, 0.5, 1.0])  # delta, delta_rate, delta_acc


def create_rotation_model(
    model_type: RotationModelType,
    config: Optional[RotationConfig] = None
) -> ProcessModelComponent:
    """
    工厂函数: 创建旋转模型
    
    Args:
        model_type: 模型类型
        config: 模型配置
        
    Returns:
        对应的旋转模型组件
    """
    config = config or RotationConfig()
    
    if model_type == RotationModelType.CV:
        return CVRotation(config)
    elif model_type == RotationModelType.CA:
        return CARotation(config)
    else:
        raise ValueError(f"Unknown rotation model type: {model_type}")
