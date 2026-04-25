"""
平移运动模型组件

支持 CV (常速度)、CA (常加速度)、Singer (时间相关加速度) 三种模型
"""

import numpy as np
from enum import Enum
from typing import Optional, Any, List
from dataclasses import dataclass

from .base import ProcessModelComponent, ComponentStateSpec


class TranslationModelType(Enum):
    """平移运动模型类型"""
    CV = "cv"      # 常速度模型 (Constant Velocity)
    CA = "ca"      # 常加速度模型 (Constant Acceleration)  
    SINGER = "singer"  # Singer机动模型


@dataclass
class TranslationConfig:
    """平移模型配置"""
    # CV模型参数
    cv_process_noise_vel: float = 0.5  # 速度过程噪声
    
    # CA模型参数
    ca_process_noise_acc: float = 1.0  # 加速度过程噪声
    
    # Singer模型参数
    singer_alpha: float = 0.5   # 机动频率
    singer_sigma: float = 2.0   # 机动加速度标准差


class CVTranslation(ProcessModelComponent):
    """
    常速度(CV)平移模型
    
    状态: [x, vx, y, vy, z, vz] (6维)
    
    运动方程:
        x_k+1 = x_k + vx_k * dt
        vx_k+1 = vx_k
        (y, z类似)
    
    过程噪声: 连续白噪声加速度模型 (CWNA)
        Q_block = q^2 * [[dt^3/3, dt^2/2],
                         [dt^2/2, dt    ]]
    """
    
    # 状态名称: [位置X, 速度X, 位置Y, 速度Y, 位置Z, 速度Z]
    STATE_NAMES = ['X', 'VX', 'Y', 'VY', 'Z', 'VZ']
    
    def __init__(
        self, 
        config: Optional[TranslationConfig] = None,
        n_dims: int = 3  # 空间维度 (2D或3D)
    ):
        super().__init__(config)
        self._config = config or TranslationConfig()
        self._n_dims = n_dims
        
        # 动态生成状态名称
        if n_dims == 3:
            self._state_names = ['X', 'VX', 'Y', 'VY', 'Z', 'VZ']
        elif n_dims == 2:
            self._state_names = ['X', 'VX', 'Y', 'VY']
        else:
            raise ValueError(f"Unsupported n_dims: {n_dims}")
    
    def get_state_spec(self) -> ComponentStateSpec:
        return ComponentStateSpec.from_names(self._state_names)
    
    def predict(
        self, 
        x_component: np.ndarray, 
        dt: float,
        full_state: Optional[np.ndarray] = None
    ) -> np.ndarray:
        """CV模型预测: pos += vel * dt"""
        x_next = x_component.copy()
        for i in range(self._n_dims):
            pos_idx = i * 2
            vel_idx = i * 2 + 1
            x_next[pos_idx] = x_component[pos_idx] + x_component[vel_idx] * dt
            # 速度保持不变
        return x_next
    
    def build_Q(self, dt: float) -> np.ndarray:
        """
        构建CV模型过程噪声矩阵
        
        使用连续白噪声加速度(CWNA)模型
        """
        q = self._config.cv_process_noise_vel
        
        # 单轴过程噪声块
        Q_block = q**2 * np.array([
            [dt**3 / 3, dt**2 / 2],
            [dt**2 / 2, dt]
        ])
        
        # 组装完整矩阵
        Q = np.zeros((self.state_dim, self.state_dim))
        for i in range(self._n_dims):
            idx = i * 2
            Q[idx:idx+2, idx:idx+2] = Q_block
        
        return Q
    
    def get_initial_covariance(self) -> np.ndarray:
        """初始协方差"""
        P = np.eye(self.state_dim) * 0.1
        return P


class CATranslation(ProcessModelComponent):
    """
    常加速度(CA)平移模型
    
    状态: [x, vx, ax, y, vy, ay, z, vz, az] (9维)
    
    运动方程:
        x_k+1 = x_k + vx_k * dt + 0.5 * ax_k * dt^2
        vx_k+1 = vx_k + ax_k * dt
        ax_k+1 = ax_k
        (y, z类似)
    
    过程噪声: 连续白噪声jerk模型 (CWNJ)
        Q_block = q^2 * [[dt^5/20, dt^4/8, dt^3/6],
                         [dt^4/8,  dt^3/3, dt^2/2],
                         [dt^3/6,  dt^2/2, dt    ]]
    """
    
    def __init__(
        self, 
        config: Optional[TranslationConfig] = None,
        n_dims: int = 3
    ):
        super().__init__(config)
        self._config = config or TranslationConfig()
        self._n_dims = n_dims
        
        # 动态生成状态名称
        if n_dims == 3:
            self._state_names = ['X', 'VX', 'AX', 'Y', 'VY', 'AY', 'Z', 'VZ', 'AZ']
        elif n_dims == 2:
            self._state_names = ['X', 'VX', 'AX', 'Y', 'VY', 'AY']
        else:
            raise ValueError(f"Unsupported n_dims: {n_dims}")
    
    def get_state_spec(self) -> ComponentStateSpec:
        return ComponentStateSpec.from_names(self._state_names)
    
    def predict(
        self, 
        x_component: np.ndarray, 
        dt: float,
        full_state: Optional[np.ndarray] = None
    ) -> np.ndarray:
        """CA模型预测"""
        x_next = x_component.copy()
        for i in range(self._n_dims):
            pos_idx = i * 3
            vel_idx = i * 3 + 1
            acc_idx = i * 3 + 2
            
            # x_k+1 = x_k + vx_k * dt + 0.5 * ax_k * dt^2
            x_next[pos_idx] = (x_component[pos_idx] + 
                              x_component[vel_idx] * dt + 
                              0.5 * x_component[acc_idx] * dt**2)
            # vx_k+1 = vx_k + ax_k * dt
            x_next[vel_idx] = x_component[vel_idx] + x_component[acc_idx] * dt
            # ax_k+1 = ax_k
        return x_next
    
    def build_Q(self, dt: float) -> np.ndarray:
        """
        构建CA模型过程噪声矩阵
        
        使用连续白噪声jerk(CWNJ)模型
        """
        q = self._config.ca_process_noise_acc
        
        # 单轴过程噪声块
        Q_block = q**2 * np.array([
            [dt**5 / 20, dt**4 / 8, dt**3 / 6],
            [dt**4 / 8,  dt**3 / 3, dt**2 / 2],
            [dt**3 / 6,  dt**2 / 2, dt]
        ])
        
        # 组装完整矩阵
        Q = np.zeros((self.state_dim, self.state_dim))
        for i in range(self._n_dims):
            idx = i * 3
            Q[idx:idx+3, idx:idx+3] = Q_block
        
        return Q
    
    def get_initial_covariance(self) -> np.ndarray:
        """初始协方差"""
        P = np.eye(self.state_dim) * 0.1
        # 加速度初始不确定性更大
        for i in range(self._n_dims):
            acc_idx = i * 3 + 2
            P[acc_idx, acc_idx] = 1.0
        return P


class SingerTranslation(ProcessModelComponent):
    """
    Singer机动模型
    
    状态: [x, vx, ax, y, vy, ay, z, vz, az] (9维)
    
    特点: 加速度具有时间相关性，呈一阶Markov过程
        a_k+1 = e^(-alpha*dt) * a_k + w_k
        
    其中 alpha 是机动频率（越大表示加速度变化越快）
    """
    
    def __init__(
        self, 
        config: Optional[TranslationConfig] = None,
        n_dims: int = 3
    ):
        super().__init__(config)
        self._config = config or TranslationConfig()
        self._n_dims = n_dims
        
        # 状态名称与CA模型相同
        if n_dims == 3:
            self._state_names = ['X', 'VX', 'AX', 'Y', 'VY', 'AY', 'Z', 'VZ', 'AZ']
        elif n_dims == 2:
            self._state_names = ['X', 'VX', 'AX', 'Y', 'VY', 'AY']
        else:
            raise ValueError(f"Unsupported n_dims: {n_dims}")
    
    def get_state_spec(self) -> ComponentStateSpec:
        return ComponentStateSpec.from_names(self._state_names)
    
    def predict(
        self, 
        x_component: np.ndarray, 
        dt: float,
        full_state: Optional[np.ndarray] = None
    ) -> np.ndarray:
        """Singer模型预测"""
        alpha = self._config.singer_alpha
        x_next = x_component.copy()
        
        # 加速度衰减因子
        rho = np.exp(-alpha * dt)
        
        for i in range(self._n_dims):
            pos_idx = i * 3
            vel_idx = i * 3 + 1
            acc_idx = i * 3 + 2
            
            a_k = x_component[acc_idx]
            v_k = x_component[vel_idx]
            p_k = x_component[pos_idx]
            
            # 位置更新 (考虑加速度衰减的积分)
            if abs(alpha) > 1e-6:
                x_next[pos_idx] = (p_k + v_k * dt + 
                                   a_k * (dt - (1 - rho) / alpha) / alpha)
                x_next[vel_idx] = v_k + a_k * (1 - rho) / alpha
            else:
                # alpha接近0时退化为CA模型
                x_next[pos_idx] = p_k + v_k * dt + 0.5 * a_k * dt**2
                x_next[vel_idx] = v_k + a_k * dt
            
            # 加速度衰减
            x_next[acc_idx] = rho * a_k
        
        return x_next
    
    def build_Q(self, dt: float) -> np.ndarray:
        """
        构建Singer模型过程噪声矩阵
        
        参考: Bar-Shalom "Estimation with Applications to Tracking and Navigation"
        """
        alpha = self._config.singer_alpha
        sigma = self._config.singer_sigma
        
        if abs(alpha) < 1e-6:
            # 退化为CA模型
            ca_config = TranslationConfig(ca_process_noise_acc=sigma)
            ca_model = CATranslation(ca_config, self._n_dims)
            return ca_model.build_Q(dt)
        
        # Singer过程噪声系数
        T = dt
        a = alpha
        rho = np.exp(-a * T)
        
        # Q矩阵元素 (Bar-Shalom公式)
        q11 = (1 - np.exp(-2*a*T) + 2*a*T + 
               (2*a**3 * T**3)/3 - 2*a**2 * T**2 - 4*a*T*np.exp(-a*T)) / (2*a**5)
        q12 = (np.exp(-2*a*T) + 1 - 2*np.exp(-a*T) + 
               2*a*T*np.exp(-a*T) - 2*a*T + a**2*T**2) / (2*a**4)
        q13 = (1 - np.exp(-2*a*T) - 2*a*T*np.exp(-a*T)) / (2*a**3)
        q22 = (4*np.exp(-a*T) - 3 - np.exp(-2*a*T) + 2*a*T) / (2*a**3)
        q23 = (np.exp(-2*a*T) + 1 - 2*np.exp(-a*T)) / (2*a**2)
        q33 = (1 - np.exp(-2*a*T)) / (2*a)
        
        # 单轴过程噪声块
        Q_block = 2 * a * sigma**2 * np.array([
            [q11, q12, q13],
            [q12, q22, q23],
            [q13, q23, q33]
        ])
        
        # 组装完整矩阵
        Q = np.zeros((self.state_dim, self.state_dim))
        for i in range(self._n_dims):
            idx = i * 3
            Q[idx:idx+3, idx:idx+3] = Q_block
        
        return Q
    
    def get_initial_covariance(self) -> np.ndarray:
        """初始协方差"""
        P = np.eye(self.state_dim) * 0.1
        # 加速度初始协方差使用稳态值
        sigma = self._config.singer_sigma
        for i in range(self._n_dims):
            acc_idx = i * 3 + 2
            P[acc_idx, acc_idx] = sigma**2
        return P


def create_translation_model(
    model_type: TranslationModelType,
    config: Optional[TranslationConfig] = None,
    n_dims: int = 3
) -> ProcessModelComponent:
    """
    工厂函数: 创建平移模型
    
    Args:
        model_type: 模型类型
        config: 模型配置
        n_dims: 空间维度
        
    Returns:
        对应的平移模型组件
    """
    config = config or TranslationConfig()
    
    if model_type == TranslationModelType.CV:
        return CVTranslation(config, n_dims)
    elif model_type == TranslationModelType.CA:
        return CATranslation(config, n_dims)
    elif model_type == TranslationModelType.SINGER:
        return SingerTranslation(config, n_dims)
    else:
        raise ValueError(f"Unknown translation model type: {model_type}")
