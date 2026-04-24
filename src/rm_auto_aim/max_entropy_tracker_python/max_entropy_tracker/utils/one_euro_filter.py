"""
One Euro Filter — 自适应低通滤波器

论文: "1€ Filter: A Simple Speed-based Low-pass Filter for Noisy Input in Interactive Systems"
(Géry Casiez, Nicolas Roussel, Daniel Vogel, CHI 2012)

核心思想:
- 静止/慢速运动时 → 使用低截止频率 → 强平滑 → 消除抖动
- 快速运动时 → 使用高截止频率 → 弱平滑 → 保持响应性

参数:
- min_cutoff: 最小截止频率 (Hz)，控制静止时的平滑强度。越小越平滑。
- beta: 速度系数，控制响应速度。越大对速度变化越敏感。
- d_cutoff: 导数滤波截止频率 (Hz)，滤波速度估计。

典型参数:
- 位置: min_cutoff=1.0, beta=0.007, d_cutoff=1.0
- 角度: min_cutoff=1.0, beta=0.001, d_cutoff=1.0
- 结构参数: min_cutoff=0.3, beta=0.0, d_cutoff=1.0 (纯低通)
"""

import numpy as np
from typing import Optional
import math


class LowPassFilter:
    """一阶低通滤波器 (指数移动平均)"""
    
    __slots__ = ['_y', '_initialized']
    
    def __init__(self):
        self._y: float = 0.0
        self._initialized: bool = False
    
    @property
    def value(self) -> float:
        return self._y
    
    @property
    def initialized(self) -> bool:
        return self._initialized
    
    def filter(self, x: float, alpha: float) -> float:
        """
        Args:
            x: 输入值
            alpha: 平滑系数 ∈ (0, 1]，越大越不平滑
        """
        if not self._initialized:
            self._y = x
            self._initialized = True
        else:
            self._y = alpha * x + (1.0 - alpha) * self._y
        return self._y
    
    def reset(self):
        self._initialized = False
        self._y = 0.0


class OneEuroFilter:
    """
    1-Euro 滤波器 (标量版本)
    
    自适应截止频率公式:
        fc = min_cutoff + beta * |dx/dt|
    
    当速度为0时: fc = min_cutoff → 强平滑
    当速度增大时: fc 增大 → 减弱平滑，提高跟随性
    """
    
    def __init__(
        self,
        freq: float = 30.0,
        min_cutoff: float = 1.0,
        beta: float = 0.007,
        d_cutoff: float = 1.0
    ):
        """
        Args:
            freq: 初始采样频率 (Hz)
            min_cutoff: 最小截止频率 (Hz)，控制静态平滑强度
            beta: 速度响应系数，越大越跟随速度变化
            d_cutoff: 导数信号的截止频率 (Hz)
        """
        if freq <= 0:
            raise ValueError(f"freq must be > 0, got {freq}")
        if min_cutoff <= 0:
            raise ValueError(f"min_cutoff must be > 0, got {min_cutoff}")
        
        self.freq = freq
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        
        self._x_filter = LowPassFilter()
        self._dx_filter = LowPassFilter()
        self._last_time: Optional[float] = None
    
    @staticmethod
    def _alpha(cutoff: float, freq: float) -> float:
        """计算低通滤波器的平滑系数 alpha"""
        tau = 1.0 / (2.0 * math.pi * cutoff)
        te = 1.0 / freq
        return 1.0 / (1.0 + tau / te)
    
    def filter(self, x: float, timestamp: Optional[float] = None) -> float:
        """
        滤波一个新值
        
        Args:
            x: 原始输入值
            timestamp: 时间戳 (秒)。如果提供，将自动计算采样频率
            
        Returns:
            滤波后的值
        """
        # 动态更新采样频率
        if timestamp is not None and self._last_time is not None:
            dt = timestamp - self._last_time
            if dt > 1e-6:
                self.freq = 1.0 / dt
        if timestamp is not None:
            self._last_time = timestamp
        
        # 计算导数 (速度)
        if self._x_filter.initialized:
            dx = (x - self._x_filter.value) * self.freq
        else:
            dx = 0.0
        
        # 滤波导数
        alpha_d = self._alpha(self.d_cutoff, self.freq)
        dx_hat = self._dx_filter.filter(dx, alpha_d)
        
        # 自适应截止频率: fc = min_cutoff + beta * |speed|
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        
        # 滤波信号
        alpha = self._alpha(cutoff, self.freq)
        return self._x_filter.filter(x, alpha)
    
    @property
    def value(self) -> float:
        """当前滤波值"""
        return self._x_filter.value
    
    def reset(self):
        """重置滤波器状态"""
        self._x_filter.reset()
        self._dx_filter.reset()
        self._last_time = None


class OneEuroFilter3D:
    """
    3D 版本的 1-Euro 滤波器
    
    对 x, y, z 三个分量独立滤波
    """
    
    def __init__(
        self,
        freq: float = 30.0,
        min_cutoff: float = 1.0,
        beta: float = 0.007,
        d_cutoff: float = 1.0
    ):
        self.filters = [
            OneEuroFilter(freq, min_cutoff, beta, d_cutoff)
            for _ in range(3)
        ]
    
    def filter(
        self, 
        x: np.ndarray, 
        timestamp: Optional[float] = None
    ) -> np.ndarray:
        """
        Args:
            x: 3D向量 [x, y, z]
            timestamp: 时间戳 (秒)
            
        Returns:
            滤波后的 3D 向量
        """
        return np.array([
            self.filters[i].filter(float(x[i]), timestamp)
            for i in range(3)
        ])
    
    @property
    def value(self) -> np.ndarray:
        return np.array([f.value for f in self.filters])
    
    def reset(self):
        for f in self.filters:
            f.reset()


class OneEuroFilterAngle:
    """
    角度版本的 1-Euro 滤波器
    
    处理角度环绕 (wrap-around) 问题:
    - 在角度差分时使用 normalize_angle
    - 确保输出在 [-π, π] 范围内
    """
    
    def __init__(
        self,
        freq: float = 30.0,
        min_cutoff: float = 1.0,
        beta: float = 0.001,
        d_cutoff: float = 1.0
    ):
        self.freq = freq
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        
        self._value: Optional[float] = None
        self._dx_filter = LowPassFilter()
        self._last_time: Optional[float] = None
    
    @staticmethod
    def _alpha(cutoff: float, freq: float) -> float:
        tau = 1.0 / (2.0 * math.pi * cutoff)
        te = 1.0 / freq
        return 1.0 / (1.0 + tau / te)
    
    @staticmethod
    def _normalize(angle: float) -> float:
        """将角度归一化到 [-π, π]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    @staticmethod
    def _shortest_angle_diff(target: float, source: float) -> float:
        """
        计算从 source 到 target 的最短角度差
        
        返回值 ∈ [-π, π]，表示从 source 转到 target 所需的最短旋转
        """
        diff = target - source
        # 使用 atan2(sin, cos) 可靠地处理环绕
        return math.atan2(math.sin(diff), math.cos(diff))
    
    def filter(self, x: float, timestamp: Optional[float] = None) -> float:
        """
        滤波角度值
        
        Args:
            x: 角度 (弧度)
            timestamp: 时间戳 (秒)
            
        Returns:
            滤波后的角度
        """
        # 更新采样频率
        if timestamp is not None and self._last_time is not None:
            dt = timestamp - self._last_time
            if dt > 1e-6:
                self.freq = 1.0 / dt
        if timestamp is not None:
            self._last_time = timestamp
        
        # 归一化输入
        x = self._normalize(x)
        
        if self._value is None:
            self._value = x
            self._dx_filter.filter(0.0, 1.0)
            return x
        
        # 角度差 (取最短路径，考虑环绕)
        diff = self._shortest_angle_diff(x, self._value)
        dx = diff * self.freq
        
        # 滤波导数
        alpha_d = self._alpha(self.d_cutoff, self.freq)
        dx_hat = self._dx_filter.filter(dx, alpha_d)
        
        # 自适应截止频率
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        
        # 滤波 (在差异空间操作以处理环绕)
        alpha = self._alpha(cutoff, self.freq)
        self._value = self._normalize(self._value + alpha * diff)
        
        return self._value
    
    @property
    def value(self) -> float:
        return self._value if self._value is not None else 0.0
    
    def reset(self):
        self._value = None
        self._dx_filter.reset()
        self._last_time = None
