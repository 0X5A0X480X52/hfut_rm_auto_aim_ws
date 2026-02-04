"""
Tracker抽象基类

定义跟踪器的统一接口，不包含具体实现
子类需要实现具体的数据关联、状态管理等方法
"""

import numpy as np
import logging
from abc import ABC, abstractmethod
from enum import Enum
from typing import List, Optional, Dict, Any, Tuple

from ..core.observation import ObservationData

logger = logging.getLogger(__name__)


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
            dt: 默认时间步长（当无法获取真实时间时使用）
        """
        self.dt = dt  # 默认dt，作为fallback
        self._state = TrackerState.INITIALIZING
        self._frame_count = 0
        self._lost_count = 0
        
        # 时间同步相关
        self._last_update_time: Optional[float] = None  # 上次更新时间戳
        self._current_time: Optional[float] = None      # 当前滤波器时间
        self._dt_history: List[float] = []              # dt历史记录（用于诊断）
        self._max_dt: float = 0.5                       # 最大允许dt（防止异常跳变）
        self._min_dt: float = 0.001                     # 最小dt（避免数值问题）
    
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
    def predict(self, target_time: Optional[float] = None) -> None:
        """
        预测步骤
        
        Args:
            target_time: 目标时间戳。如果为None，使用默认dt向前预测
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
    
    # ==================== 时间同步方法 ====================
    
    def _compute_dt(self, target_time: Optional[float]) -> float:
        """
        计算实际dt
        
        Args:
            target_time: 目标时间戳
            
        Returns:
            实际dt（秒）
        """
        if target_time is None or self._current_time is None:
            return self.dt  # fallback到默认dt
        
        dt = target_time - self._current_time
        
        # 边界保护
        if dt < 0:
            logger.warning(f"Time went backwards: dt={dt:.3f}s, using 0")
            return 0.0
        
        if dt < self._min_dt:
            return self._min_dt
        
        if dt > self._max_dt:
            logger.warning(f"dt={dt:.3f}s exceeds max_dt={self._max_dt}s, clamping")
            return self._max_dt
        
        return dt
    
    def _update_time(self, new_time: float) -> None:
        """
        更新内部时间
        
        Args:
            new_time: 新的时间戳
        """
        self._current_time = new_time
        self._last_update_time = new_time
    
    def get_dt_statistics(self) -> Dict[str, float]:
        """
        获取dt统计信息（诊断用）
        
        Returns:
            dt统计字典
        """
        if not self._dt_history:
            return {'count': 0, 'default_dt': self.dt}
        
        dt_arr = np.array(self._dt_history)
        return {
            'mean': float(np.mean(dt_arr)),
            'std': float(np.std(dt_arr)),
            'min': float(np.min(dt_arr)),
            'max': float(np.max(dt_arr)),
            'count': len(dt_arr),
            'default_dt': self.dt,
            'deviation_from_default': float(np.mean(dt_arr) - self.dt)
        }
    
    def process(self, observations: List[ObservationData], 
                current_time: Optional[float] = None) -> bool:
        """
        统一处理接口（推荐使用）
        
        自动处理predict和update的时间同步
        
        Args:
            observations: 观测列表（可为空表示无观测）
            current_time: 当前时间戳（如果观测为空且需要预测则必须提供）
            
        Returns:
            是否成功处理
        """
        # 确定目标时间
        if observations and observations[0].timestamp is not None:
            target_time = observations[0].timestamp
        elif current_time is not None:
            target_time = current_time
        else:
            target_time = None  # fallback to default dt
        
        # 有观测：update（update内部会自动predict）
        if observations:
            return self.update(observations)
        
        # 无观测：仅predict
        else:
            self.predict(target_time)
            # 使用默认阈值处理观测丢失
            tracking_thres = 2
            lost_thres = 8
            if hasattr(self, 'config'):
                tracking_thres = self.config.tracker.tracking_thres
                lost_thres = self.config.tracker.lost_thres
            self._handle_observation_loss(tracking_thres, lost_thres)
            return False
