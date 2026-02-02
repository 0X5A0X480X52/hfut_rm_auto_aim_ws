"""
Max Entropy Tracker ROS2 Package

基于 Max Entropy UKF 的机器人位姿估计器，提供：
- 多机器人跟踪管理 (TrackerManager)
- TF 坐标变换 (TFHandler)
- ROS2 消息转换 (msg_converter)
- 可视化 Marker 生成 (visualization)
- CSV 日志记录 (CSVLogger)

核心算法模块：
- 纯抽象基类 (BaseUKF, BaseTracker)
- 双半径自旋UKF实现 (DualRadiusSpinUKF)
- 自适应装甲板跟踪器 (AdaptiveArmorTracker)
"""

from .filters import BaseUKF, DualRadiusSpinUKF
from .trackers import BaseTracker, AdaptiveArmorTracker
from .core import UnifiedConfig, ObservationData

__all__ = [
    # 核心算法
    'BaseUKF',
    'DualRadiusSpinUKF',
    'BaseTracker',
    'AdaptiveArmorTracker',
    'UnifiedConfig',
    'ObservationData',
]
