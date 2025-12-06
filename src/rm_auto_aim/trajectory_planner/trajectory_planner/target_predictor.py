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
Target Predictor Module

Extracts and processes target prediction trajectories from armor_tracker's
prediction window topic. Provides target yaw angle trajectories for MPC.
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
import math


@dataclass
class ArmorPrediction:
    """单个装甲板的预测信息"""
    
    track_id: int                    # 跟踪ID
    armor_id: str                    # 装甲板ID ("1"~"5", "outpost", "base")
    armor_type: str                  # 装甲板类型 ("small", "large")
    
    # 预测状态序列
    timestamps: List[float] = field(default_factory=list)      # 时间戳列表
    positions: List[np.ndarray] = field(default_factory=list)  # 3D位置列表
    velocities: List[np.ndarray] = field(default_factory=list) # 3D速度列表
    yaws: List[float] = field(default_factory=list)            # yaw角列表
    yaw_velocities: List[float] = field(default_factory=list)  # yaw角速度列表
    confidences: List[float] = field(default_factory=list)     # 置信度列表
    
    # 计算属性缓存
    yaw_angles_from_origin: Optional[np.ndarray] = None  # 从原点看装甲板的方位角


@dataclass
class TargetPredictorConfig:
    """目标预测器配置"""
    
    # 最小置信度阈值
    min_confidence: float = 0.3
    
    # 预测步数 (与MPC horizon匹配)
    prediction_steps: int = 18
    
    # 时间步长 (秒)
    dt: float = 0.01
    
    # 装甲板选择策略: "nearest_yaw", "highest_confidence", "nearest_distance"
    selection_strategy: str = "nearest_yaw"
    
    # 最大有效距离 (米)
    max_distance: float = 10.0


class TargetPredictor:
    """
    目标预测器
    
    功能:
    1. 接收 armor_tracker 的预测窗口数据
    2. 根据目标机器人ID过滤相关装甲板
    3. 选择最佳装甲板作为打击目标
    4. 提取目标的预测轨迹供MPC使用
    """
    
    def __init__(self, config: Optional[TargetPredictorConfig] = None):
        """
        初始化目标预测器
        
        Args:
            config: 预测器配置
        """
        self.config = config if config else TargetPredictorConfig()
        
        # 存储最新的预测数据
        self._predictions: Dict[int, ArmorPrediction] = {}
        
        # 当前目标的装甲板track_id列表
        self._target_armor_track_ids: List[int] = []
        
        # 上次更新时间戳
        self._last_update_time: Optional[float] = None
        
        # 当前选中的目标装甲板
        self._selected_armor_track_id: Optional[int] = None
    
    def update_predictions(self, prediction_windows_msg) -> None:
        """
        更新预测数据
        
        Args:
            prediction_windows_msg: TrackPredictionWindows 消息
        """
        self._predictions.clear()
        
        for window in prediction_windows_msg.windows:
            prediction = ArmorPrediction(
                track_id=window.track_id,
                armor_id=window.armor_id,
                armor_type=window.armor_type
            )
            
            for state in window.predictions:
                prediction.timestamps.append(
                    state.timestamp.sec + state.timestamp.nanosec * 1e-9
                )
                prediction.positions.append(np.array([
                    state.position.x,
                    state.position.y,
                    state.position.z
                ]))
                prediction.velocities.append(np.array([
                    state.velocity.x,
                    state.velocity.y,
                    state.velocity.z
                ]))
                prediction.yaws.append(state.yaw)
                prediction.yaw_velocities.append(state.yaw_velocity)
                prediction.confidences.append(state.confidence)
            
            # 计算从原点看装甲板的方位角 (yaw方向)
            if prediction.positions:
                yaw_angles = []
                for pos in prediction.positions:
                    # atan2(y, x) 给出水平面上的方位角
                    yaw_angle = math.atan2(pos[1], pos[0])
                    yaw_angles.append(yaw_angle)
                prediction.yaw_angles_from_origin = np.array(yaw_angles)
            
            self._predictions[window.track_id] = prediction
        
        self._last_update_time = (
            prediction_windows_msg.header.stamp.sec +
            prediction_windows_msg.header.stamp.nanosec * 1e-9
        )
    
    def set_target_armor_ids(self, armor_ids: List[str]) -> None:
        """
        设置目标装甲板ID列表
        
        通常从 robot_pose_estimator 的机器人信息中获取 bound_armor_ids
        
        Args:
            armor_ids: 目标机器人绑定的装甲板ID列表
        """
        self._target_armor_track_ids = []
        
        # 根据 armor_id 找到对应的 track_id
        for track_id, prediction in self._predictions.items():
            if prediction.armor_id in armor_ids:
                self._target_armor_track_ids.append(track_id)
    
    def filter_by_robot_id(self, robot_id: str) -> List[int]:
        """
        根据机器人ID过滤装甲板
        
        Args:
            robot_id: 目标机器人ID
        
        Returns:
            匹配的装甲板track_id列表
        """
        matching_track_ids = []
        for track_id, prediction in self._predictions.items():
            if prediction.armor_id == robot_id:
                matching_track_ids.append(track_id)
        print(f"DEBUG: filter_by_robot_id({robot_id}) -> {matching_track_ids}")
        return matching_track_ids
    
    def select_best_armor(self, 
                          reference_yaw: float = 0.0,
                          target_track_ids: Optional[List[int]] = None) -> Optional[int]:
        """
        选择最佳打击装甲板
        
        Args:
            reference_yaw: 参考yaw方向 (当前云台yaw)
            target_track_ids: 候选装甲板track_id列表，None则使用所有
        
        Returns:
            选中的装甲板track_id，无有效目标返回None
        """
        candidates = target_track_ids if target_track_ids else list(self._predictions.keys())
        
        print(f"DEBUG: select_best_armor candidates: {candidates}")
        
        if not candidates:
            self._selected_armor_track_id = None
            return None
        
        best_track_id = None
        best_score = float('inf')
        
        for track_id in candidates:
            if track_id not in self._predictions:
                print(f"DEBUG: track_id {track_id} not in predictions")
                continue
            
            prediction = self._predictions[track_id]
            
            # 检查置信度
            if not prediction.confidences or prediction.confidences[0] < self.config.min_confidence:
                print(f"DEBUG: track_id {track_id} low confidence: {prediction.confidences[0] if prediction.confidences else 'no confidences'} < {self.config.min_confidence}")
                continue
            
            # 检查有效数据
            if not prediction.positions or prediction.yaw_angles_from_origin is None:
                print(f"DEBUG: track_id {track_id} missing data: positions={len(prediction.positions) if prediction.positions else 0}, yaw_angles={prediction.yaw_angles_from_origin is not None}")
                continue
            
            # 检查距离
            pos = prediction.positions[0]
            distance = np.linalg.norm(pos)
            if distance > self.config.max_distance:
                print(f"DEBUG: track_id {track_id} too far: {distance} > {self.config.max_distance}")
                continue
            
            print(f"DEBUG: track_id {track_id} passed all checks, distance={distance:.2f}")
            
            # 根据策略计算评分
            if self.config.selection_strategy == "nearest_yaw":
                # 选择与当前云台yaw最接近的装甲板
                yaw_from_origin = prediction.yaw_angles_from_origin[0]
                score = abs(self._wrap_angle(yaw_from_origin - reference_yaw))
            
            elif self.config.selection_strategy == "nearest_distance":
                score = distance
            
            elif self.config.selection_strategy == "highest_confidence":
                score = -prediction.confidences[0]  # 负值因为要最小化
            
            else:
                score = abs(self._wrap_angle(
                    prediction.yaw_angles_from_origin[0] - reference_yaw))
            
            print(f"DEBUG: track_id {track_id} score: {score}")
            
            if score < best_score:
                best_score = score
                best_track_id = track_id
        
        print(f"DEBUG: select_best_armor result: {best_track_id}")
        self._selected_armor_track_id = best_track_id
        return best_track_id
    
    def get_target_yaw_trajectory(self, 
                                  track_id: Optional[int] = None,
                                  horizon: Optional[int] = None) -> Optional[np.ndarray]:
        """
        获取目标装甲板的yaw角轨迹
        
        Args:
            track_id: 装甲板track_id，None则使用当前选中的
            horizon: 预测步数，None则使用配置值
        
        Returns:
            yaw角轨迹 (N,)，无有效数据返回None
        """
        tid = track_id if track_id is not None else self._selected_armor_track_id
        
        if tid is None or tid not in self._predictions:
            return None
        
        prediction = self._predictions[tid]
        
        if prediction.yaw_angles_from_origin is None:
            return None
        
        N = horizon if horizon else self.config.prediction_steps
        
        # 获取预测的yaw角序列
        yaw_trajectory = prediction.yaw_angles_from_origin
        
        # 调整到所需长度
        if len(yaw_trajectory) >= N:
            return yaw_trajectory[:N]
        else:
            # 线性外推
            trajectory = np.zeros(N)
            trajectory[:len(yaw_trajectory)] = yaw_trajectory
            
            if len(yaw_trajectory) >= 2:
                # 使用最后两个点的趋势外推
                yaw_rate = (yaw_trajectory[-1] - yaw_trajectory[-2]) / self.config.dt
                for i in range(len(yaw_trajectory), N):
                    trajectory[i] = trajectory[i-1] + yaw_rate * self.config.dt
            else:
                # 保持最后一个值
                trajectory[len(yaw_trajectory):] = yaw_trajectory[-1]
            
            return trajectory
    
    def get_target_position(self, 
                            track_id: Optional[int] = None,
                            step: int = 0) -> Optional[np.ndarray]:
        """
        获取目标装甲板的3D位置
        
        Args:
            track_id: 装甲板track_id
            step: 预测步索引
        
        Returns:
            3D位置 [x, y, z]，无有效数据返回None
        """
        tid = track_id if track_id is not None else self._selected_armor_track_id
        
        if tid is None or tid not in self._predictions:
            return None
        
        prediction = self._predictions[tid]
        
        if not prediction.positions or step >= len(prediction.positions):
            return None
        
        return prediction.positions[step].copy()
    
    def get_target_velocity(self,
                            track_id: Optional[int] = None,
                            step: int = 0) -> Optional[np.ndarray]:
        """
        获取目标装甲板的3D速度
        
        Args:
            track_id: 装甲板track_id
            step: 预测步索引
        
        Returns:
            3D速度 [vx, vy, vz]，无有效数据返回None
        """
        tid = track_id if track_id is not None else self._selected_armor_track_id
        
        if tid is None or tid not in self._predictions:
            return None
        
        prediction = self._predictions[tid]
        
        if not prediction.velocities or step >= len(prediction.velocities):
            return None
        
        return prediction.velocities[step].copy()
    
    def get_target_info(self, track_id: Optional[int] = None) -> Optional[dict]:
        """
        获取目标装甲板的完整信息
        
        Args:
            track_id: 装甲板track_id
        
        Returns:
            包含目标信息的字典
        """
        tid = track_id if track_id is not None else self._selected_armor_track_id
        
        if tid is None or tid not in self._predictions:
            return None
        
        prediction = self._predictions[tid]
        
        if not prediction.positions:
            return None
        
        pos = prediction.positions[0]
        distance = np.linalg.norm(pos)
        
        return {
            'track_id': tid,
            'armor_id': prediction.armor_id,
            'armor_type': prediction.armor_type,
            'position': pos,
            'velocity': prediction.velocities[0] if prediction.velocities else np.zeros(3),
            'yaw_from_origin': prediction.yaw_angles_from_origin[0] if prediction.yaw_angles_from_origin is not None else 0.0,
            'distance': distance,
            'confidence': prediction.confidences[0] if prediction.confidences else 0.0
        }
    
    def has_valid_target(self, track_id: Optional[int] = None) -> bool:
        """
        检查是否有有效目标
        
        Args:
            track_id: 装甲板track_id，None检查当前选中的
        
        Returns:
            是否有有效目标
        """
        tid = track_id if track_id is not None else self._selected_armor_track_id
        return tid is not None and tid in self._predictions
    
    @property
    def selected_track_id(self) -> Optional[int]:
        """获取当前选中的装甲板track_id"""
        return self._selected_armor_track_id
    
    @property
    def available_track_ids(self) -> List[int]:
        """获取所有可用的装甲板track_id"""
        return list(self._predictions.keys())
    
    @staticmethod
    def _wrap_angle(angle: float) -> float:
        """将角度归一化到 [-π, π]"""
        return (angle + math.pi) % (2 * math.pi) - math.pi
