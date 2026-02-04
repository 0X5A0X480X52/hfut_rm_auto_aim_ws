"""
多机器人跟踪管理器

为每个机器人ID维护独立的 AdaptiveArmorTracker 实例
"""

import time
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
import logging

from .trackers.adaptive_armor_tracker import AdaptiveArmorTracker
from .core.config import UnifiedConfig
from .core.observation import ObservationData

logger = logging.getLogger(__name__)


@dataclass
class TrackerState:
    """单个跟踪器的状态"""
    tracker: AdaptiveArmorTracker
    last_update_time: float  # 最后更新时间（秒）
    last_predict_time: float  # 最后预测时间（秒）
    observation_count: int  # 累计观测次数


class TrackerManager:
    """
    多机器人跟踪管理器
    
    功能:
    1. 为每个机器人ID创建和管理独立的跟踪器
    2. 处理跟踪器初始化和超时移除
    3. 批量预测和更新
    """
    
    def __init__(
        self,
        config: UnifiedConfig,
        dt: float = 0.01,
        default_r1: float = 0.15,
        default_r2: float = 0.20,
        default_dza: float = 0.0,
        timeout_seconds: float = 3.0,
        enable_oscillation_detection: bool = False
    ):
        """
        初始化跟踪管理器
        
        Args:
            config: 统一配置
            dt: 时间步长
            default_r1: 默认半径1
            default_r2: 默认半径2
            default_dza: 默认装甲板高度差
            timeout_seconds: 跟踪器超时时间（秒）
        """
        self.config = config
        self.dt = dt
        self.default_r1 = default_r1
        self.default_r2 = default_r2
        self.default_dza = default_dza
        self.timeout_seconds = timeout_seconds
        self.enable_oscillation_detection = enable_oscillation_detection
        
        # 跟踪器字典: robot_id -> TrackerState
        self._trackers: Dict[str, TrackerState] = {}
        
        logger.info(f"TrackerManager initialized: dt={dt}, timeout={timeout_seconds}s, enable_oscillation_detection={enable_oscillation_detection}")
    
    def get_or_create_tracker(
        self,
        robot_id: str,
        initial_observations: Optional[List[ObservationData]] = None
    ) -> Optional[AdaptiveArmorTracker]:
        """
        获取或创建跟踪器
        
        Args:
            robot_id: 机器人ID
            initial_observations: 初始观测（仅在创建新跟踪器时使用）
            
        Returns:
            AdaptiveArmorTracker 实例，如果创建失败返回 None
        """
        current_time = time.time()
        
        if robot_id in self._trackers:
            return self._trackers[robot_id].tracker
        
        # 创建新跟踪器
        if initial_observations is None or len(initial_observations) == 0:
            logger.warning(f"Cannot create tracker for {robot_id}: no initial observations")
            return None
        
        try:
            tracker = AdaptiveArmorTracker(
                config=self.config,
                dt=self.dt,
                enable_oscillation_detection=self.enable_oscillation_detection
            )
            
            tracker.initialize(
                observations=initial_observations,
                r1=self.default_r1,
                r2=self.default_r2,
                dza=self.default_dza
            )
            
            self._trackers[robot_id] = TrackerState(
                tracker=tracker,
                last_update_time=current_time,
                last_predict_time=current_time,
                observation_count=len(initial_observations)
            )
            
            logger.info(f"Created new tracker for robot: {robot_id}")
            return tracker
            
        except Exception as e:
            logger.error(f"Failed to create tracker for {robot_id}: {e}")
            return None
    
    def predict_all(self, target_time: Optional[float] = None) -> None:
        """
        对所有跟踪器执行预测到目标时间
        
        Args:
            target_time: 目标时间戳（秒），None则使用系统时间
        """
        if target_time is None:
            target_time = time.time()
        
        for robot_id, state in list(self._trackers.items()):
            try:
                if state.tracker.is_initialized:
                    # 使用基于时间戳的预测方法
                    state.tracker.predict(target_time)
                    state.last_predict_time = target_time
            except Exception as e:
                logger.error(f"Predict failed for {robot_id}: {e}")
    
    def update(
        self,
        robot_id: str,
        observations: List[ObservationData],
        current_time: Optional[float] = None
    ) -> bool:
        """
        更新指定机器人的跟踪器
        
        Args:
            robot_id: 机器人ID
            observations: 观测列表（应包含时间戳）
            current_time: 当前时间（秒），用于更新last_update_time
            
        Returns:
            是否更新成功
        
        Note:
            tracker.update() 会自动使用观测中的时间戳进行预测和更新
            current_time 仅用于记录上次更新时间以判断超时
        """
        if current_time is None:
            current_time = time.time()
        
        if len(observations) == 0:
            return False
        
        # 获取或创建跟踪器
        if robot_id not in self._trackers:
            tracker = self.get_or_create_tracker(robot_id, observations)
            return tracker is not None
        
        state = self._trackers[robot_id]
        
        try:
            # tracker.update() 内部会自动使用观测的timestamp进行预测
            success = state.tracker.update(observations)
            if success:
                state.last_update_time = current_time
                state.observation_count += len(observations)
            return success
        except Exception as e:
            logger.error(f"Update failed for {robot_id}: {e}")
            return False
    
    def update_batch(
        self,
        observations_by_robot: Dict[str, List[ObservationData]],
        current_time: Optional[float] = None
    ) -> Dict[str, bool]:
        """
        批量更新所有机器人
        
        Args:
            observations_by_robot: 按机器人ID分组的观测
            current_time: 当前时间
            
        Returns:
            各机器人更新结果
        """
        results = {}
        for robot_id, observations in observations_by_robot.items():
            results[robot_id] = self.update(robot_id, observations, current_time)
        return results
    
    def remove_stale_trackers(self, current_time: Optional[float] = None) -> List[str]:
        """
        移除超时的跟踪器
        
        Args:
            current_time: 当前时间
            
        Returns:
            被移除的机器人ID列表
        """
        if current_time is None:
            current_time = time.time()
        
        removed = []
        for robot_id, state in list(self._trackers.items()):
            if current_time - state.last_update_time > self.timeout_seconds:
                del self._trackers[robot_id]
                removed.append(robot_id)
                logger.info(f"Removed stale tracker: {robot_id}")
        
        return removed
    
    def get_tracker(self, robot_id: str) -> Optional[AdaptiveArmorTracker]:
        """
        获取指定机器人的跟踪器
        
        Args:
            robot_id: 机器人ID
            
        Returns:
            跟踪器实例，不存在返回 None
        """
        if robot_id in self._trackers:
            return self._trackers[robot_id].tracker
        return None
    
    def get_all_trackers(self) -> Dict[str, AdaptiveArmorTracker]:
        """
        获取所有跟踪器
        
        Returns:
            robot_id -> tracker 字典
        """
        return {rid: state.tracker for rid, state in self._trackers.items()}
    
    def get_tracking_robots(self) -> List[str]:
        """
        获取所有正在跟踪的机器人ID
        
        Returns:
            正在跟踪状态的机器人ID列表
        """
        return [
            rid for rid, state in self._trackers.items()
            if state.tracker.is_tracking
        ]
    
    def get_all_states(self) -> Dict[str, Dict[str, Any]]:
        """
        获取所有跟踪器的状态
        
        Returns:
            robot_id -> state_dict
        """
        result = {}
        for robot_id, state in self._trackers.items():
            try:
                tracker_state = state.tracker.get_state()
                tracker_state['last_update_time'] = state.last_update_time
                tracker_state['observation_count'] = state.observation_count
                result[robot_id] = tracker_state
            except Exception as e:
                logger.error(f"Get state failed for {robot_id}: {e}")
        return result
    
    def clear_all(self) -> None:
        """清除所有跟踪器"""
        self._trackers.clear()
        logger.info("All trackers cleared")
    
    @property
    def num_trackers(self) -> int:
        """跟踪器数量"""
        return len(self._trackers)
    
    @property
    def robot_ids(self) -> List[str]:
        """所有机器人ID"""
        return list(self._trackers.keys())
