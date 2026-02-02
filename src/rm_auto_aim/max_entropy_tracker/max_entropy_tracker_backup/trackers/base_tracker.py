"""
Tracker抽象基类

定义跟踪器的统一接口，不包含具体实现
子类需要实现具体的数据关联、状态管理等方法
"""

import numpy as np
from abc import ABC, abstractmethod
from enum import Enum
from typing import List, Optional, Dict, Any, Tuple

from ..core.observation import ObservationData


class TrackerState(Enum):
    """跟踪器状态"""
    INITIALIZING = 0  # 初始化中
    TRACKING = 1      # 正常跟踪
    TEMP_LOST = 2     # 临时丢失
    LOST = 3          # 完全丢失


class BaseTracker(ABC):
    """
    Tracker抽象基类
    
    设计原则:
    1. 接口使用列表输入 - 支持多观测场景
    2. 不包含具体实现 - 由子类实现数据关联、状态管理
    3. 定义状态转换框架 - TrackerState枚举
    4. 提供统一的状态获取接口
    
    子类必须实现:
    - initialize(): 初始化跟踪
    - predict(): 预测步骤
    - update(): 更新步骤（接受观测列表和yaw列表）
    - get_center_position(): 获取中心位置
    - get_yaw(): 获取yaw角
    - get_radii(): 获取半径估计
    - get_state(): 获取完整状态字典
    """
    
    def __init__(self, dt: float):
        """
        初始化Tracker基类
        
        Args:
            dt: 时间步长
        """
        self.dt = dt
        self._state = TrackerState.INITIALIZING
        self._frame_count = 0
        self._lost_count = 0
    
    # ==================== 抽象方法 ====================
    
    @abstractmethod
    def initialize(
        self,
        observations: List[ObservationData],
        **kwargs
    ) -> None:
        """
        初始化跟踪器
        
        Args:
            observations: 初始观测列表（至少1个）
            **kwargs: 其他初始化参数
        """
        pass
    
    @abstractmethod
    def predict(self) -> None:
        """
        预测步骤
        
        在无观测时调用，推进状态
        """
        pass
    
    @abstractmethod
    def update(
        self,
        observations: List[ObservationData],
        **kwargs
    ) -> bool:
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
    def get_center_position(self) -> np.ndarray:
        """
        获取中心位置
        
        Returns:
            位置向量 [x, y, z]
        """
        pass
    
    @abstractmethod
    def get_yaw(self) -> float:
        """
        获取yaw角
        
        Returns:
            yaw角（弧度）
        """
        pass
    
    @abstractmethod
    def get_radii(self) -> Tuple[float, float]:
        """
        获取半径估计
        
        Returns:
            (r1, r2)
        """
        pass
    
    @abstractmethod
    def get_state(self) -> Dict[str, Any]:
        """
        获取完整状态
        
        Returns:
            包含所有状态信息的字典
        """
        pass
    
    # ==================== 状态管理 ====================
    
    @property
    def state(self) -> TrackerState:
        """跟踪状态"""
        return self._state
    
    @property
    def is_initialized(self) -> bool:
        """是否已初始化"""
        return self._state != TrackerState.INITIALIZING
    
    @property
    def is_tracking(self) -> bool:
        """是否正在跟踪"""
        return self._state == TrackerState.TRACKING
    
    @property
    def is_lost(self) -> bool:
        """是否已丢失"""
        return self._state == TrackerState.LOST
    
    @property
    def frame_count(self) -> int:
        """帧计数"""
        return self._frame_count
    
    def _transition_to(self, new_state: TrackerState) -> None:
        """
        状态转换
        
        Args:
            new_state: 新状态
        """
        if self._state != new_state:
            self._state = new_state
    
    def _increment_frame(self) -> None:
        """增加帧计数"""
        self._frame_count += 1
    
    def _handle_observation_loss(self, tracking_threshold: int, lost_threshold: int) -> None:
        """
        处理观测丢失
        
        Args:
            tracking_threshold: 恢复到跟踪状态所需的连续观测数
            lost_threshold: 判定完全丢失的丢失帧数
        """
        self._lost_count += 1
        
        if self._lost_count >= lost_threshold:
            self._transition_to(TrackerState.LOST)
        elif self._state == TrackerState.TRACKING:
            self._transition_to(TrackerState.TEMP_LOST)
    
    def _handle_observation_received(self, tracking_threshold: int) -> None:
        """
        处理接收到观测
        
        Args:
            tracking_threshold: 恢复到跟踪状态所需的连续观测数
        """
        self._lost_count = 0
        
        if self._state == TrackerState.TEMP_LOST:
            self._transition_to(TrackerState.TRACKING)
        elif self._state == TrackerState.INITIALIZING:
            if self._frame_count >= tracking_threshold:
                self._transition_to(TrackerState.TRACKING)
