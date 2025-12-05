# Copyright (C) FYT Vision Group. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Target Manager Module

Manages target robot selection from target_selector service.
Filters armor predictions based on the selected target robot's bound armor IDs.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
import threading


@dataclass
class RobotInfo:
    """机器人信息"""
    
    robot_id: str                          # 机器人ID ("1"~"5", "outpost", "base")
    robot_type: int                        # 机器人类型
    center_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # 中心位置
    center_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # 中心速度
    yaw: float = 0.0                       # yaw角
    yaw_velocity: float = 0.0             # yaw角速度
    radius: float = 0.0                   # 旋转半径
    bound_armor_ids: List[str] = field(default_factory=list)  # 绑定的装甲板ID列表
    confidence: float = 0.0               # 置信度
    num_armors: int = 0                   # 装甲板数量
    
    # 机器人类型常量
    BALANCE_2 = 0
    STANDARD_4 = 1
    HERO_4 = 2
    OUTPOST_3 = 3
    SENTRY = 4
    BASE = 5
    UNKNOWN = 255


@dataclass
class TargetManagerConfig:
    """目标管理器配置"""
    
    # 目标超时时间 (秒)
    target_timeout: float = 2.0
    
    # 机器人信息超时时间 (秒)
    robot_info_timeout: float = 1.0
    
    # 最小置信度
    min_confidence: float = 0.3


class TargetManager:
    """
    目标管理器
    
    功能:
    1. 接收 target_selector 通过 service 设置的目标机器人ID
    2. 维护 robot_pose_estimator 发布的机器人信息
    3. 根据目标机器人ID获取其绑定的装甲板列表
    4. 提供目标过滤接口给 TargetPredictor
    """
    
    def __init__(self, config: Optional[TargetManagerConfig] = None):
        """
        初始化目标管理器
        
        Args:
            config: 管理器配置
        """
        self.config = config if config else TargetManagerConfig()
        
        # 当前目标机器人ID
        self._target_robot_id: Optional[str] = None
        
        # 上一个目标机器人ID
        self._previous_robot_id: Optional[str] = None
        
        # 机器人信息缓存
        self._robots: Dict[str, RobotInfo] = {}
        
        # 最后更新时间戳
        self._last_robots_update_time: Optional[float] = None
        self._last_target_set_time: Optional[float] = None
        
        # 线程锁
        self._lock = threading.RLock()
        
        # 跟踪状态
        self._is_tracking: bool = False
    
    def set_target_robot(self, robot_id: str) -> Tuple[bool, str, str]:
        """
        设置目标机器人ID
        
        由 target_selector 通过 SetTargetRobot service 调用
        
        Args:
            robot_id: 目标机器人ID，空字符串表示清除目标
        
        Returns:
            (success, message, previous_robot_id)
        """
        with self._lock:
            self._previous_robot_id = self._target_robot_id
            previous_id = self._previous_robot_id if self._previous_robot_id else ""
            
            if robot_id == "":
                # 清除目标
                self._target_robot_id = None
                self._is_tracking = False
                return True, "Target cleared", previous_id
            
            self._target_robot_id = robot_id
            self._is_tracking = True
            
            # 检查机器人是否存在
            if robot_id in self._robots:
                robot_info = self._robots[robot_id]
                if robot_info.confidence >= self.config.min_confidence:
                    return True, f"Target set to robot {robot_id}", previous_id
                else:
                    return True, f"Target set to robot {robot_id} (low confidence)", previous_id
            else:
                return True, f"Target set to robot {robot_id} (not tracked yet)", previous_id
    
    def update_robots(self, tracked_robots_msg, current_time: float) -> None:
        """
        更新机器人信息
        
        Args:
            tracked_robots_msg: TrackedRobots 消息
            current_time: 当前时间戳
        """
        with self._lock:
            self._robots.clear()
            
            for robot_msg in tracked_robots_msg.robots:
                robot_info = RobotInfo(
                    robot_id=robot_msg.robot_id,
                    robot_type=robot_msg.robot_type,
                    center_position=(
                        robot_msg.center_position.x,
                        robot_msg.center_position.y,
                        robot_msg.center_position.z
                    ),
                    center_velocity=(
                        robot_msg.center_velocity.x,
                        robot_msg.center_velocity.y,
                        robot_msg.center_velocity.z
                    ),
                    yaw=robot_msg.yaw,
                    yaw_velocity=robot_msg.yaw_velocity,
                    radius=robot_msg.radius,
                    bound_armor_ids=list(robot_msg.bound_armor_ids),
                    confidence=robot_msg.confidence,
                    num_armors=robot_msg.num_armors
                )
                self._robots[robot_msg.robot_id] = robot_info
            
            self._last_robots_update_time = current_time
    
    def get_target_armor_ids(self) -> List[str]:
        """
        获取目标机器人绑定的装甲板ID列表
        
        Returns:
            装甲板ID列表
        """
        with self._lock:
            if self._target_robot_id is None:
                return []
            
            if self._target_robot_id in self._robots:
                return self._robots[self._target_robot_id].bound_armor_ids.copy()
            
            # 如果机器人不在列表中，返回与robot_id相同的armor_id
            # 这是一个fallback，因为有时armor_id就是robot_id
            return [self._target_robot_id]
    
    def get_target_robot_info(self) -> Optional[RobotInfo]:
        """
        获取目标机器人的完整信息
        
        Returns:
            机器人信息，无目标返回None
        """
        with self._lock:
            if self._target_robot_id is None:
                return None
            
            return self._robots.get(self._target_robot_id)
    
    def get_robot_info(self, robot_id: str) -> Optional[RobotInfo]:
        """
        获取指定机器人的信息
        
        Args:
            robot_id: 机器人ID
        
        Returns:
            机器人信息
        """
        with self._lock:
            return self._robots.get(robot_id)
    
    def is_target_valid(self, current_time: float) -> bool:
        """
        检查当前目标是否有效
        
        Args:
            current_time: 当前时间戳
        
        Returns:
            目标是否有效
        """
        with self._lock:
            if self._target_robot_id is None:
                return False
            
            # 检查机器人信息是否过期
            if self._last_robots_update_time is not None:
                if current_time - self._last_robots_update_time > self.config.robot_info_timeout:
                    return False
            
            # 检查目标机器人是否在列表中
            if self._target_robot_id not in self._robots:
                return False
            
            # 检查置信度
            robot_info = self._robots[self._target_robot_id]
            if robot_info.confidence < self.config.min_confidence:
                return False
            
            return True
    
    def clear_target(self) -> None:
        """清除当前目标"""
        with self._lock:
            self._previous_robot_id = self._target_robot_id
            self._target_robot_id = None
            self._is_tracking = False
    
    @property
    def target_robot_id(self) -> Optional[str]:
        """获取当前目标机器人ID"""
        with self._lock:
            return self._target_robot_id
    
    @property
    def previous_robot_id(self) -> Optional[str]:
        """获取上一个目标机器人ID"""
        with self._lock:
            return self._previous_robot_id
    
    @property
    def is_tracking(self) -> bool:
        """是否正在跟踪目标"""
        with self._lock:
            return self._is_tracking
    
    @property
    def available_robot_ids(self) -> List[str]:
        """获取所有可用的机器人ID"""
        with self._lock:
            return list(self._robots.keys())
