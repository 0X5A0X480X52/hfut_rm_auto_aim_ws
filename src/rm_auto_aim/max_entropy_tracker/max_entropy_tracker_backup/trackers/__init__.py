"""
跟踪器模块

提供Tracker抽象基类和具体实现
"""

from .base_tracker import BaseTracker, TrackerState
from .adaptive_armor_tracker import AdaptiveArmorTracker

__all__ = [
    'BaseTracker',
    'TrackerState',
    'AdaptiveArmorTracker',
]
