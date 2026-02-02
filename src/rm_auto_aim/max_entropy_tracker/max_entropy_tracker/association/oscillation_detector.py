"""
参数震荡检测器

检测r1/r2参数的异常震荡并触发重启
"""

import numpy as np
from typing import List, Optional, Tuple
import logging

logger = logging.getLogger(__name__)


class OscillationDetector:
    """
    参数震荡检测器
    
    核心功能:
    1. 监控r1/r2历史变化
    2. 检测持续震荡模式
    3. 提供参数重启建议
    """
    
    def __init__(
        self,
        window_size: int = 50,
        oscillation_threshold: float = 0.05,
        count_threshold: int = 5,
        min_reset_interval: int = 100,
        enabled: bool = False
    ):
        """
        初始化检测器
        
        Args:
            window_size: 监控窗口大小（帧数）
            oscillation_threshold: 震荡阈值（米）
            count_threshold: 连续震荡次数阈值
            min_reset_interval: 最小重启间隔（帧数）
            enabled: 是否启用检测
        """
        self.window_size = window_size
        self.oscillation_threshold = oscillation_threshold
        self.count_threshold = count_threshold
        self.min_reset_interval = min_reset_interval
        self.enabled = enabled
        
        self._r1_history: List[float] = []
        self._r2_history: List[float] = []
        self._oscillation_count: int = 0
        self._last_reset_frame: int = -100
        self._current_frame: int = 0
    
    def update(self, r1: float, r2: float) -> bool:
        """
        更新参数历史并检测震荡
        
        Args:
            r1: 当前r1估计值
            r2: 当前r2估计值
            
        Returns:
            是否建议重启参数
        """
        self._current_frame += 1
        
        if not self.enabled:
            return False
        
        # 更新历史
        self._r1_history.append(r1)
        self._r2_history.append(r2)
        
        # 保持窗口大小
        if len(self._r1_history) > self.window_size:
            self._r1_history.pop(0)
            self._r2_history.pop(0)
        
        # 需要足够的历史数据
        if len(self._r1_history) < self.window_size:
            return False
        
        # 检测震荡
        is_oscillating = self._detect_oscillation()
        
        if is_oscillating:
            self._oscillation_count += 1
        else:
            self._oscillation_count = 0
        
        # 判断是否需要重启
        should_reset = (
            self._oscillation_count >= self.count_threshold and
            self._current_frame - self._last_reset_frame > self.min_reset_interval
        )
        
        if should_reset:
            self._last_reset_frame = self._current_frame
            self._oscillation_count = 0
            logger.warning(
                f"Oscillation detected at frame {self._current_frame}, "
                f"recommending parameter reset"
            )
        
        return should_reset
    
    def _detect_oscillation(self) -> bool:
        """
        检测是否存在震荡
        
        震荡判定标准:
        - 标准差超过阈值
        - 且近期变化率较大
        """
        r1_std = np.std(self._r1_history)
        r2_std = np.std(self._r2_history)
        
        # 计算近期变化率（最近10帧）
        r1_recent_change = 0
        r2_recent_change = 0
        if len(self._r1_history) >= 10:
            r1_recent_change = abs(self._r1_history[-1] - self._r1_history[-10])
            r2_recent_change = abs(self._r2_history[-1] - self._r2_history[-10])
        
        # 需要同时满足：标准差大 且 变化率大
        is_oscillating = (
            (r1_std > self.oscillation_threshold and r1_recent_change > 0.02) or
            (r2_std > self.oscillation_threshold and r2_recent_change > 0.02)
        )
        
        return is_oscillating
    
    def get_reset_values(self) -> Tuple[float, float]:
        """
        获取重启时应使用的参数值
        
        使用窗口内的中位数（比平均值更鲁棒）
        
        Returns:
            (r1_median, r2_median)
        """
        r1_median = np.median(self._r1_history) if self._r1_history else 0.15
        r2_median = np.median(self._r2_history) if self._r2_history else 0.20
        return r1_median, r2_median
    
    def get_statistics(self) -> dict:
        """获取统计信息"""
        if not self._r1_history:
            return {}
        
        return {
            'r1_mean': np.mean(self._r1_history),
            'r1_std': np.std(self._r1_history),
            'r2_mean': np.mean(self._r2_history),
            'r2_std': np.std(self._r2_history),
            'oscillation_count': self._oscillation_count,
            'resets': self._last_reset_frame // self.min_reset_interval if self._last_reset_frame > 0 else 0,
        }
    
    def reset(self):
        """重置检测器状态"""
        self._r1_history.clear()
        self._r2_history.clear()
        self._oscillation_count = 0
        self._current_frame = 0
