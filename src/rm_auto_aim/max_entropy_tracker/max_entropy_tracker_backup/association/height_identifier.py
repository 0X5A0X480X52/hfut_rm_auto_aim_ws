"""
高度层级识别器

基于最大熵原则识别装甲板的上/下层级
"""

import numpy as np
from enum import Enum
from typing import Optional, Tuple, List
import logging

logger = logging.getLogger(__name__)


class HeightLabel(Enum):
    """装甲板高度标签"""
    UNKNOWN = 0  # 信息不足时的安全状态（最大熵）
    UPPER = 1    # 上层装甲板
    LOWER = 2    # 下层装甲板


class HeightIdentifier:
    """
    高度层级识别器
    
    核心功能:
    1. 单观测识别：基于历史信息和panel跳变检测
    2. 双观测识别：基于z坐标差直接判断
    3. 基于最大熵原则：信息不足时返回UNKNOWN
    """
    
    def __init__(self):
        """初始化识别器"""
        self._z_last_labeled: Optional[float] = None
        self._label_last: Optional[HeightLabel] = None
        self._panel_last: Optional[int] = None
        
        # 调试记录
        self._history: List[Tuple] = []
    
    def identify_single(
        self,
        z_obs: float,
        panel_id: int,
        z_mean: Optional[float] = None,
        dza: Optional[float] = None,
        dza_converged: bool = False
    ) -> Tuple[HeightLabel, float]:
        """
        单观测条件下的高度识别
        
        信息来源（按强度排序）:
        1. 状态估计（如果dza已收敛）
        2. Panel跳变检测
        3. 历史一致性检测
        
        Args:
            z_obs: 观测到的z坐标
            panel_id: 当前panel索引
            z_mean: 预测的中心z（可选）
            dza: 预测的高度差（可选）
            dza_converged: dza是否已收敛
            
        Returns:
            (HeightLabel, confidence)
        """
        # 机制1：基于状态估计（最可靠）
        if dza_converged and z_mean is not None and dza is not None:
            z_upper = z_mean + dza
            z_lower = z_mean - dza
            
            dist_to_upper = abs(z_obs - z_upper)
            dist_to_lower = abs(z_obs - z_lower)
            
            if dist_to_upper < dist_to_lower:
                label = HeightLabel.UPPER
                confidence = 1.0 - dist_to_upper / (dist_to_upper + dist_to_lower + 0.001)
            else:
                label = HeightLabel.LOWER
                confidence = 1.0 - dist_to_lower / (dist_to_upper + dist_to_lower + 0.001)
            
            self._update_history(z_obs, panel_id, label)
            return label, confidence
        
        # 机制2：Panel跳变检测
        if self._panel_last is not None and self._label_last is not None:
            panel_diff = abs(panel_id - self._panel_last)
            # 相邻panel时，upper/lower交替
            if panel_diff == 1 or panel_diff == 3:
                if self._label_last == HeightLabel.UPPER:
                    label = HeightLabel.LOWER
                else:
                    label = HeightLabel.UPPER
                
                self._update_history(z_obs, panel_id, label)
                return label, 0.7
        
        # 机制3：历史一致性检测
        if self._z_last_labeled is not None and self._label_last is not None:
            z_diff = abs(z_obs - self._z_last_labeled)
            if z_diff < 0.05:  # 5cm阈值
                confidence = max(0.3, 0.6 - z_diff * 2.0)
                self._update_history(z_obs, panel_id, self._label_last)
                return self._label_last, confidence
        
        # 无法识别：返回UNKNOWN（最大熵）
        self._panel_last = panel_id
        return HeightLabel.UNKNOWN, 0.0
    
    def identify_dual(
        self,
        z1: float,
        z2: float,
        z_diff_threshold: float = 0.015
    ) -> Tuple[Optional[str], Optional[str], float]:
        """
        双观测条件下的高度识别
        
        基于z坐标差直接判断upper/lower
        
        Args:
            z1: 第一个观测的z坐标
            z2: 第二个观测的z坐标
            z_diff_threshold: z差阈值（默认1.5cm）
            
        Returns:
            (layer_1, layer_2, confidence)
        """
        z_diff = abs(z1 - z2)
        
        if z_diff > z_diff_threshold:
            # z较大的为upper
            if z1 > z2:
                layer_1, layer_2 = 'upper', 'lower'
            else:
                layer_1, layer_2 = 'lower', 'upper'
            
            # 置信度根据z差异调整
            if z_diff > 0.05:
                confidence = 1.0
            elif z_diff > 0.03:
                confidence = 0.8
            else:
                confidence = 0.6
            
            return layer_1, layer_2, confidence
        
        # z差异太小，尝试使用历史信息
        if self._label_last is not None and self._z_last_labeled is not None:
            z1_diff_hist = abs(z1 - self._z_last_labeled)
            z2_diff_hist = abs(z2 - self._z_last_labeled)
            
            if z1_diff_hist < z2_diff_hist and z1_diff_hist < 0.03:
                layer_1 = 'upper' if self._label_last == HeightLabel.UPPER else 'lower'
                layer_2 = 'lower' if self._label_last == HeightLabel.UPPER else 'upper'
                return layer_1, layer_2, 0.4
            elif z2_diff_hist < 0.03:
                layer_1 = 'lower' if self._label_last == HeightLabel.UPPER else 'upper'
                layer_2 = 'upper' if self._label_last == HeightLabel.UPPER else 'lower'
                return layer_1, layer_2, 0.4
        
        # 无法识别
        return None, None, 0.0
    
    def _update_history(self, z: float, panel_id: int, label: HeightLabel):
        """更新历史信息"""
        self._z_last_labeled = z
        self._label_last = label
        self._panel_last = panel_id
    
    def reset(self):
        """重置识别器状态"""
        self._z_last_labeled = None
        self._label_last = None
        self._panel_last = None
        self._history.clear()
    
    @property
    def last_label(self) -> Optional[HeightLabel]:
        """上次识别的标签"""
        return self._label_last
