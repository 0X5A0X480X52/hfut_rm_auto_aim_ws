"""
UKF抽象基类

定义UKF的统一接口，不包含具体状态布局
子类需要定义自己的状态索引和实现具体方法
"""

import numpy as np
from abc import ABC, abstractmethod
from typing import List, Optional, Dict, Any
import logging

from ..core.config import UnifiedConfig
from ..core.observation import ObservationData
from ..utils.sigma_points import SigmaPointGenerator
from ..utils.constraints import ensure_positive_definite

logger = logging.getLogger(__name__)


class BaseUKF(ABC):
    """
    UKF抽象基类
    
    设计原则:
    1. 不定义状态维度和索引 - 由子类决定
    2. 提供通用工具方法 - Sigma点生成、协方差管理等
    3. 定义必须实现的抽象方法 - initialize, predict, update等
    4. 接口使用列表输入 - 支持多观测场景
    
    子类必须实现:
    - state_dim: 状态维度属性
    - _build_process_noise(): 构建过程噪声矩阵
    - _process_model(): 状态转移方程
    - _observation_model(): 观测方程
    - initialize(): 初始化状态
    - predict(): 预测步骤
    - update(): 更新步骤（接受观测列表）
    """
    
    def __init__(self, config: UnifiedConfig, dt: float = 0.1):
        """
        初始化UKF基类
        
        Args:
            config: 统一配置对象
            dt: 默认时间步长
        """
        self.config = config
        self.dt = dt
        
        # 状态和协方差（由子类初始化具体维度）
        self._x: Optional[np.ndarray] = None
        self._P: Optional[np.ndarray] = None
        self._Q: Optional[np.ndarray] = None
        
        # Sigma点生成器（在子类初始化后设置）
        self._sigma_generator: Optional[SigmaPointGenerator] = None
        
        # 初始化标志
        self._initialized: bool = False
        
        logger.debug(f"BaseUKF created with dt={dt}")
    
    # ==================== 抽象属性 ====================
    
    @property
    @abstractmethod
    def state_dim(self) -> int:
        """状态维度（由子类定义）"""
        pass
    
    @property
    @abstractmethod
    def obs_dim(self) -> int:
        """观测维度（由子类定义）"""
        pass
    
    # ==================== 抽象方法 ====================
    
    @abstractmethod
    def initialize(self, observations: List[ObservationData], **kwargs) -> None:
        """
        初始化滤波器状态
        
        Args:
            observations: 初始观测列表
            **kwargs: 其他初始化参数（如初始半径等）
        """
        pass
    
    @abstractmethod
    def predict(self, dt: Optional[float] = None) -> None:
        """
        预测步骤
        
        Args:
            dt: 时间步长，如果为None则使用默认dt
        """
        pass
    
    @abstractmethod
    def update(self, observations: List[ObservationData], **kwargs) -> bool:
        """
        更新步骤
        
        Args:
            observations: 观测列表（支持单观测和多观测）
            **kwargs: 其他更新参数
            
        Returns:
            是否更新成功
        """
        pass
    
    @abstractmethod
    def _build_process_noise(self) -> np.ndarray:
        """
        构建过程噪声矩阵Q
        
        Returns:
            过程噪声矩阵 (state_dim, state_dim)
        """
        pass
    
    @abstractmethod
    def _process_model(self, x: np.ndarray, dt: float) -> np.ndarray:
        """
        状态转移方程: x_{k+1} = f(x_k, dt)
        
        Args:
            x: 当前状态向量
            dt: 时间步长
            
        Returns:
            预测状态向量
        """
        pass
    
    @abstractmethod
    def _observation_model(self, x: np.ndarray, **kwargs) -> np.ndarray:
        """
        观测方程: z = h(x)
        
        Args:
            x: 状态向量
            **kwargs: 观测相关参数（如半径选择、层级等）
            
        Returns:
            预测观测向量
        """
        pass
    
    # ==================== 通用属性 ====================
    
    @property
    def x(self) -> np.ndarray:
        """状态向量"""
        if self._x is None:
            raise RuntimeError("State not initialized")
        return self._x
    
    @x.setter
    def x(self, value: np.ndarray):
        self._x = value
    
    @property
    def P(self) -> np.ndarray:
        """协方差矩阵"""
        if self._P is None:
            raise RuntimeError("Covariance not initialized")
        return self._P
    
    @P.setter
    def P(self, value: np.ndarray):
        self._P = value
    
    @property
    def Q(self) -> np.ndarray:
        """过程噪声矩阵"""
        if self._Q is None:
            raise RuntimeError("Process noise not initialized")
        return self._Q
    
    @Q.setter
    def Q(self, value: np.ndarray):
        self._Q = value
    
    @property
    def initialized(self) -> bool:
        """是否已初始化"""
        return self._initialized
    
    @initialized.setter
    def initialized(self, value: bool):
        self._initialized = value
    
    # ==================== 通用工具方法 ====================
    
    def _init_sigma_generator(self):
        """初始化Sigma点生成器（在子类设置state_dim后调用）"""
        self._sigma_generator = SigmaPointGenerator(
            n=self.state_dim,
            alpha=self.config.ukf.alpha,
            beta=self.config.ukf.beta,
            kappa=self.config.ukf.kappa
        )
    
    def generate_sigma_points(self, x: np.ndarray, P: np.ndarray) -> np.ndarray:
        """
        生成Sigma点
        
        Args:
            x: 状态向量
            P: 协方差矩阵
            
        Returns:
            Sigma点矩阵 (2n+1, n)
        """
        if self._sigma_generator is None:
            self._init_sigma_generator()
        return self._sigma_generator.generate(x, P)
    
    def get_sigma_weights(self):
        """获取Sigma点权重"""
        if self._sigma_generator is None:
            self._init_sigma_generator()
        return self._sigma_generator.get_weights()
    
    def ensure_covariance_valid(self):
        """确保协方差矩阵正定"""
        self._P = ensure_positive_definite(self._P)
    
    def compute_kalman_gain(
        self,
        Pxz: np.ndarray,
        Pzz: np.ndarray
    ) -> Optional[np.ndarray]:
        """
        计算Kalman增益
        
        Args:
            Pxz: 交叉协方差 (n, m)
            Pzz: 观测协方差 (m, m)
            
        Returns:
            Kalman增益 (n, m)，如果矩阵奇异则返回None
        """
        try:
            K = Pxz @ np.linalg.inv(Pzz)
            return K
        except np.linalg.LinAlgError:
            logger.warning("Singular Pzz matrix, cannot compute Kalman gain")
            return None
    
    def apply_kalman_update(
        self,
        K: np.ndarray,
        innovation: np.ndarray,
        Pzz: np.ndarray
    ):
        """
        应用Kalman更新
        
        Args:
            K: Kalman增益
            innovation: 观测残差
            Pzz: 观测协方差
        """
        self._x = self._x + K @ innovation
        self._P = self._P - K @ Pzz @ K.T
        self.ensure_covariance_valid()
    
    # ==================== 状态获取接口 ====================
    
    @abstractmethod
    def get_state_dict(self) -> Dict[str, Any]:
        """
        获取状态字典（便于调试和可视化）
        
        Returns:
            包含状态各分量的字典
        """
        pass
