"""
观测数据结构

定义观测数据的统一格式
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional


@dataclass
class ObservationData:
    """
    装甲板观测数据
    
    封装单个装甲板的观测信息，用于UKF和Tracker的统一接口
    """
    
    # 位置 (必需)
    x: float
    y: float
    z: float
    
    # 朝向 (必需)
    yaw: float
    
    # 可选元信息
    panel_id: Optional[int] = None  # 装甲板ID (0,1,2,3)
    layer: Optional[str] = None     # 层级 ('upper', 'lower', None)
    confidence: float = 1.0         # 观测置信度
    timestamp: Optional[float] = None  # 时间戳
    
    @property
    def position(self) -> np.ndarray:
        """获取位置向量 [x, y, z]"""
        return np.array([self.x, self.y, self.z])
    
    @property
    def as_4d(self) -> np.ndarray:
        """获取4D观测向量 [x, y, z, yaw]"""
        return np.array([self.x, self.y, self.z, self.yaw])
    
    @classmethod
    def from_array(cls, arr: np.ndarray, yaw: float, **kwargs) -> 'ObservationData':
        """
        从数组创建观测数据
        
        Args:
            arr: 位置数组 [x, y] 或 [x, y, z]
            yaw: 朝向角
            **kwargs: 其他可选参数
            
        Returns:
            ObservationData实例
        """
        x = arr[0]
        y = arr[1]
        z = arr[2] if len(arr) > 2 else 0.0
        return cls(x=x, y=y, z=z, yaw=yaw, **kwargs)
    
    @classmethod
    def from_4d(cls, arr: np.ndarray, **kwargs) -> 'ObservationData':
        """
        从4D数组创建观测数据
        
        Args:
            arr: [x, y, z, yaw]
            **kwargs: 其他可选参数
            
        Returns:
            ObservationData实例
        """
        return cls(x=arr[0], y=arr[1], z=arr[2], yaw=arr[3], **kwargs)
