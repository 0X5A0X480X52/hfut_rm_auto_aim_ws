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
Gimbal Dynamics Model Module

Provides a configurable gimbal dynamics model with state-space representation.
Supports third-order model (angle-velocity-acceleration) with jerk input.
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Tuple, Optional


@dataclass
class GimbalConfig:
    """云台配置参数"""
    
    # 时间步长 (秒)
    dt: float = 0.01
    
    # 角速度限制 (rad/s)
    omega_min: float = -2.0
    omega_max: float = 2.0
    
    # 角加速度限制 (rad/s^2)
    alpha_min: float = -20.0
    alpha_max: float = 20.0
    
    # 角加加速度限制 (rad/s^3) - MPC控制输入
    jerk_min: float = -200.0
    jerk_max: float = 200.0
    
    # 角度边界 (rad)
    theta_min: float = -1.57  # -90 degrees
    theta_max: float = 1.57   # +90 degrees
    
    # 安全系数
    safety_factor: float = 0.8
    
    @classmethod
    def from_dict(cls, config_dict: dict) -> 'GimbalConfig':
        """从字典创建配置"""
        return cls(**{k: v for k, v in config_dict.items() if k in cls.__dataclass_fields__})
    
    def compute_limits_from_omega(self, omega_limit_deg: float):
        """
        根据期望角速度上限自动计算约束
        
        Args:
            omega_limit_deg: 期望角速度上限 (deg/s)
        """
        omega_max = np.deg2rad(omega_limit_deg) * self.safety_factor
        self.omega_min = -omega_max
        self.omega_max = omega_max
        
        alpha_max = omega_max / self.dt * self.safety_factor
        self.alpha_min = -alpha_max
        self.alpha_max = alpha_max
        
        jerk_max = alpha_max / self.dt * self.safety_factor
        self.jerk_min = -jerk_max
        self.jerk_max = jerk_max


class GimbalModel:
    """
    云台三阶动力学模型
    
    状态向量: x = [theta, omega, alpha]^T
    - theta: 角度 (rad)
    - omega: 角速度 (rad/s)
    - alpha: 角加速度 (rad/s^2)
    
    控制输入: u = jerk (角加加速度, rad/s^3)
    
    离散化状态转移方程:
    x_{k+1} = A * x_k + B * u_k
    """
    
    # 状态维度和控制维度
    STATE_DIM = 3
    CONTROL_DIM = 1
    
    # 状态索引
    IDX_THETA = 0
    IDX_OMEGA = 1
    IDX_ALPHA = 2
    
    def __init__(self, config: Optional[GimbalConfig] = None):
        """
        初始化云台模型
        
        Args:
            config: 云台配置参数
        """
        self.config = config if config is not None else GimbalConfig()
        self._build_matrices()
    
    def _build_matrices(self):
        """构建状态转移矩阵"""
        dt = self.config.dt
        
        # 状态转移矩阵 A (3x3)
        # theta_{k+1} = theta_k + omega_k * dt + 0.5 * alpha_k * dt^2
        # omega_{k+1} = omega_k + alpha_k * dt
        # alpha_{k+1} = alpha_k
        self.A = np.array([
            [1.0, dt, 0.5 * dt**2],
            [0.0, 1.0, dt],
            [0.0, 0.0, 1.0]
        ])
        
        # 控制输入矩阵 B (3x1)
        # 考虑 jerk 对各状态的影响
        self.B = np.array([
            [dt**3 / 6],
            [dt**2 / 2],
            [dt]
        ])
    
    def update_dt(self, dt: float):
        """
        更新时间步长并重新构建矩阵
        
        Args:
            dt: 新的时间步长 (秒)
        """
        self.config.dt = dt
        self._build_matrices()
    
    def predict(self, state: np.ndarray, control: float) -> np.ndarray:
        """
        预测下一时刻状态
        
        Args:
            state: 当前状态 [theta, omega, alpha]
            control: 控制输入 (jerk)
        
        Returns:
            下一时刻状态
        """
        next_state = self.A @ state + self.B.flatten() * control
        return self.apply_constraints(next_state)
    
    def apply_constraints(self, state: np.ndarray) -> np.ndarray:
        """
        应用状态约束
        
        Args:
            state: 状态向量
        
        Returns:
            约束后的状态向量
        """
        constrained = state.copy()
        
        # 约束角速度
        constrained[self.IDX_OMEGA] = np.clip(
            constrained[self.IDX_OMEGA],
            self.config.omega_min,
            self.config.omega_max
        )
        
        # 约束角加速度
        constrained[self.IDX_ALPHA] = np.clip(
            constrained[self.IDX_ALPHA],
            self.config.alpha_min,
            self.config.alpha_max
        )
        
        # 约束角度
        constrained[self.IDX_THETA] = self._wrap_angle(constrained[self.IDX_THETA])
        
        # 硬边界约束
        if constrained[self.IDX_THETA] > self.config.theta_max:
            constrained[self.IDX_THETA] = self.config.theta_max
            constrained[self.IDX_OMEGA] = min(0.0, constrained[self.IDX_OMEGA])
        elif constrained[self.IDX_THETA] < self.config.theta_min:
            constrained[self.IDX_THETA] = self.config.theta_min
            constrained[self.IDX_OMEGA] = max(0.0, constrained[self.IDX_OMEGA])
        
        return constrained
    
    def clip_control(self, control: float) -> float:
        """
        约束控制输入
        
        Args:
            control: 原始控制输入
        
        Returns:
            约束后的控制输入
        """
        return np.clip(control, self.config.jerk_min, self.config.jerk_max)
    
    def get_prediction_matrices(self, horizon: int) -> Tuple[np.ndarray, np.ndarray]:
        """
        构建MPC预测矩阵
        
        X_future = A_pred * x0 + B_pred * U
        
        Args:
            horizon: 预测时域步数
        
        Returns:
            A_pred: (horizon * n_state, n_state) 预测矩阵
            B_pred: (horizon * n_state, horizon * n_control) 控制矩阵
        """
        n = self.STATE_DIM
        m = self.CONTROL_DIM
        
        A_pred = np.zeros((horizon * n, n))
        B_pred = np.zeros((horizon * n, horizon * m))
        
        A_power = self.A.copy()
        for i in range(horizon):
            # A_pred 的第 i 行块
            A_pred[i*n:(i+1)*n, :] = A_power
            
            # B_pred 的第 i 行块
            A_power_j = np.eye(n)
            for j in range(i + 1):
                B_pred[i*n:(i+1)*n, j*m:(j+1)*m] = A_power_j @ self.B
                A_power_j = self.A @ A_power_j
            
            A_power = self.A @ A_power
        
        return A_pred, B_pred
    
    @staticmethod
    def _wrap_angle(angle: float) -> float:
        """
        将角度归一化到 [-π, π]
        
        Args:
            angle: 原始角度
        
        Returns:
            归一化后的角度
        """
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    @staticmethod
    def angle_difference(angle1: float, angle2: float) -> float:
        """
        计算两个角度之差，考虑角度环绕
        
        Args:
            angle1: 第一个角度
            angle2: 第二个角度
        
        Returns:
            angle1 - angle2，归一化到 [-π, π]
        """
        return GimbalModel._wrap_angle(angle1 - angle2)
