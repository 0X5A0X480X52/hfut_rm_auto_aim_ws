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
MPC Controller Module

A QP-based Model Predictive Controller for gimbal yaw control.
Based on quadratic programming optimization with tracking error,
control effort, and control smoothness objectives.

Performance optimized with OSQP solver (if available) for real-time control.
"""

import numpy as np
from scipy.optimize import minimize
from scipy import sparse
from dataclasses import dataclass
from typing import Optional, Tuple, List
import time

# 尝试导入 OSQP（高性能 QP 求解器）
try:
    import osqp
    OSQP_AVAILABLE = True
except ImportError:
    OSQP_AVAILABLE = False

from trajectory_planner.gimbal_model import GimbalModel, GimbalConfig


@dataclass
class MPCConfig:
    """MPC控制器配置参数"""
    
    # 预测时域步数
    prediction_horizon: int = 18
    
    # 时间步长 (秒)
    dt: float = 0.01
    
    # 跟踪误差权重
    q_theta: float = 100.0    # 角度误差权重
    q_omega: float = 10.0     # 角速度误差权重
    q_alpha: float = 1.0      # 角加速度误差权重
    
    # 控制输入权重
    r_control: float = 0.01   # 控制输入权重
    
    # 控制变化率权重 (平滑性)
    s_smooth: float = 5.0     # 控制变化率权重
    
    # 优化器设置
    max_iterations: int = 30
    ftol: float = 1e-5
    
    # 是否使用 OSQP（如果可用）
    use_osqp: bool = True
    
    # OSQP 设置
    osqp_warm_start: bool = True
    osqp_verbose: bool = False
    osqp_polish: bool = False  # 关闭 polish 以提高速度
    osqp_eps_abs: float = 1e-4
    osqp_eps_rel: float = 1e-4
    
    @classmethod
    def from_dict(cls, config_dict: dict) -> 'MPCConfig':
        """从字典创建配置"""
        return cls(**{k: v for k, v in config_dict.items() if k in cls.__dataclass_fields__})


class MPCController:
    """
    基于二次规划的MPC控制器
    
    优化目标:
    min J = sum_{k=1}^{N} [(x_k - x_ref_k)^T Q (x_k - x_ref_k)]  # 跟踪误差
          + sum_{k=0}^{N-1} [u_k^T R u_k]                         # 控制能量
          + sum_{k=0}^{N-1} [(u_k - u_{k-1})^T S (u_k - u_{k-1})] # 控制平滑性
    
    约束:
    - 状态方程: x_{k+1} = A x_k + B u_k
    - 控制约束: jerk_min <= u_k <= jerk_max
    - 状态约束: theta_min <= theta_k <= theta_max
    """
    
    def __init__(self, 
                 gimbal_model: Optional[GimbalModel] = None,
                 mpc_config: Optional[MPCConfig] = None):
        """
        初始化MPC控制器
        
        Args:
            gimbal_model: 云台动力学模型
            mpc_config: MPC配置参数
        """
        self.gimbal_model = gimbal_model if gimbal_model else GimbalModel()
        self.config = mpc_config if mpc_config else MPCConfig()
        
        # 同步时间步长
        if self.gimbal_model.config.dt != self.config.dt:
            self.gimbal_model.update_dt(self.config.dt)
        
        # 预计算矩阵
        self._build_optimization_matrices()
        
        # OSQP 求解器实例（懒初始化）
        self._osqp_solver: Optional[osqp.OSQP] = None
        self._use_osqp = OSQP_AVAILABLE and self.config.use_osqp
        
        # 上一次的控制输入 (用于热启动)
        self.u_prev = 0.0
        self._U_prev: Optional[np.ndarray] = None
        
        # 性能统计
        self.last_solve_time = 0.0
        self.last_success = True
    
    def _build_optimization_matrices(self):
        """构建优化所需的矩阵"""
        N = self.config.prediction_horizon
        n_state = GimbalModel.STATE_DIM
        n_control = GimbalModel.CONTROL_DIM
        
        # 获取预测矩阵
        self.A_pred, self.B_pred = self.gimbal_model.get_prediction_matrices(N)
        
        # 构建跟踪误差权重矩阵 Q (N*n_state x N*n_state)
        self.Q = np.eye(N * n_state)
        for i in range(N):
            self.Q[i*n_state + 0, i*n_state + 0] = self.config.q_theta
            self.Q[i*n_state + 1, i*n_state + 1] = self.config.q_omega
            self.Q[i*n_state + 2, i*n_state + 2] = self.config.q_alpha
        
        # 控制输入权重矩阵 R (N*n_control x N*n_control)
        self.R = self.config.r_control * np.eye(N * n_control)
        
        # 控制变化率权重矩阵 S
        self.S = self.config.s_smooth * np.eye(N * n_control)
        
        # 差分矩阵 D (用于控制变化率惩罚)
        self.D = self._build_difference_matrix(N)
    
    def _build_difference_matrix(self, N: int) -> np.ndarray:
        """
        构建差分矩阵
        
        D*U = [u_0, u_1-u_0, u_2-u_1, ..., u_{N-1}-u_{N-2}]
        
        Args:
            N: 预测时域步数
        
        Returns:
            差分矩阵
        """
        D = np.zeros((N, N))
        D[0, 0] = 1.0  # 第一个控制输入与0的差
        for i in range(1, N):
            D[i, i] = 1.0
            D[i, i-1] = -1.0
        return D
    
    def update_config(self, mpc_config: MPCConfig):
        """
        更新MPC配置
        
        Args:
            mpc_config: 新的MPC配置
        """
        self.config = mpc_config
        
        # 同步时间步长
        if self.gimbal_model.config.dt != self.config.dt:
            self.gimbal_model.update_dt(self.config.dt)
        
        # 重新构建矩阵
        self._build_optimization_matrices()
    
    def solve(self, 
              current_state: np.ndarray,
              target_trajectory: np.ndarray) -> Tuple[float, np.ndarray, float, str, np.ndarray]:
        """
        求解MPC优化问题
        
        Args:
            current_state: 当前状态 [theta, omega, alpha] (3,)
            target_trajectory: 目标状态序列 (N, 3) 或 (N,) 只包含角度
        
        Returns:
            u_optimal: 最优控制输入 (第一个时间步)
            U_sequence: 完整控制序列 (N,)
            solve_time: 求解时间 (毫秒)
            solver_status: 求解状态 ('solved', 'solved_inaccurate', 'failed')
            predicted_trajectory: 预测状态轨迹 (N+1, 3)
        """
        start_time = time.perf_counter()
        
        N = self.config.prediction_horizon
        n_state = GimbalModel.STATE_DIM
        
        # 处理目标轨迹格式
        X_target = self._prepare_target_trajectory(target_trajectory)
        X_target_flat = X_target.flatten()
        
        # 预计算常量项
        A_pred_x0 = self.A_pred @ current_state
        
        # 构建QP问题的Hessian矩阵和梯度向量
        # J = 0.5 * U^T * H * U + f^T * U + const
        H = 2 * (self.B_pred.T @ self.Q @ self.B_pred + self.R + self.D.T @ self.S @ self.D)
        f = 2 * self.B_pred.T @ self.Q @ (A_pred_x0 - X_target_flat)
        
        # 尝试使用 OSQP（更快）
        solver_status = 'failed'
        if self._use_osqp:
            result = self._solve_osqp(H, f, current_state, A_pred_x0)
            if result is not None:
                u, U_optimal, osqp_status = result
                solve_time = (time.perf_counter() - start_time) * 1000  # 转换为毫秒
                self.last_solve_time = solve_time
                self.last_success = True
                solver_status = osqp_status
                # 计算预测轨迹
                predicted_traj = self.predict_trajectory(current_state, U_optimal)
                return u, U_optimal, solve_time, solver_status, predicted_traj
        
        # OSQP 失败或不可用，使用 scipy 后备方案
        result = self._solve_scipy(H, f, current_state, A_pred_x0)
        solve_time = (time.perf_counter() - start_time) * 1000  # 转换为毫秒
        self.last_solve_time = solve_time
        
        if result is not None:
            u, U_optimal = result
            self.last_success = True
            solver_status = 'solved_scipy'
            predicted_traj = self.predict_trajectory(current_state, U_optimal)
            return u, U_optimal, solve_time, solver_status, predicted_traj
        else:
            self.last_success = False
            solver_status = 'failed'
            predicted_traj = np.tile(current_state, (N+1, 1))  # 失败时返回静止轨迹
            return 0.0, np.zeros(N), solve_time, solver_status, predicted_traj

    def predict_trajectory(self, current_state: np.ndarray, U_sequence: np.ndarray) -> np.ndarray:
        """基于当前状态和控制序列预测状态轨迹（N+1, state_dim）

        Returns:
            predicted_traj: numpy array of shape (N+1, n_state)
        """
        N = self.config.prediction_horizon
        n_state = GimbalModel.STATE_DIM
        # A_pred @ current_state gives stacked states for steps 1..N
        A_pred_x0 = self.A_pred @ current_state
        X_pred_flat = A_pred_x0 + self.B_pred @ U_sequence
        try:
            X_pred = X_pred_flat.reshape(N, n_state)
        except Exception:
            # 出错时返回静止轨迹
            return np.tile(current_state, (N+1, 1))
        # 包含初始状态
        predicted_traj = np.vstack([current_state.reshape(1, n_state), X_pred])
        return predicted_traj
    
    def _solve_osqp(self, H: np.ndarray, f: np.ndarray, 
                    current_state: np.ndarray, A_pred_x0: np.ndarray) -> Optional[Tuple[float, np.ndarray, str]]:
        """
        使用 OSQP 求解 QP 问题
        
        Args:
            H: Hessian 矩阵
            f: 梯度向量
            current_state: 当前状态
            A_pred_x0: 预测矩阵与初始状态的乘积
        
        Returns:
            (u_optimal, U_sequence, solver_status) 或 None（如果求解失败）
        """
        N = self.config.prediction_horizon
        n_state = GimbalModel.STATE_DIM
        
        # 边界约束
        jerk_min = self.gimbal_model.config.jerk_min
        jerk_max = self.gimbal_model.config.jerk_max
        
        # 角度边界
        theta_min = self.gimbal_model.config.theta_min
        theta_max = self.gimbal_model.config.theta_max
        
        # 构建约束矩阵
        # 1. 控制输入边界: jerk_min <= U <= jerk_max
        # 2. 角度约束: theta_min <= theta_k <= theta_max
        
        # 角度约束: theta_k = (A_pred_x0 + B_pred @ U)[k*n_state]
        # 提取 B_pred 中与 theta 相关的行
        B_theta = self.B_pred[::n_state, :]  # 每隔 n_state 行取一行（theta 分量）
        
        # 构建约束矩阵 A_con @ U <= b_upper 且 A_con @ U >= b_lower
        # 角度约束: theta_min <= A_pred_x0[::n_state] + B_theta @ U <= theta_max
        # 即: theta_min - A_pred_x0[::n_state] <= B_theta @ U <= theta_max - A_pred_x0[::n_state]
        
        A_pred_x0_theta = A_pred_x0[::n_state]  # 提取 theta 分量
        
        # 合并约束：[I; B_theta]
        A_con = sparse.vstack([
            sparse.eye(N),  # 控制输入约束
            sparse.csc_matrix(B_theta)  # 角度约束
        ], format='csc')
        
        # 下界和上界
        l = np.concatenate([
            np.full(N, jerk_min),  # 控制输入下界
            np.full(N, theta_min) - A_pred_x0_theta  # 角度下界
        ])
        u = np.concatenate([
            np.full(N, jerk_max),  # 控制输入上界
            np.full(N, theta_max) - A_pred_x0_theta  # 角度上界
        ])
        
        # 转换 H 为稀疏矩阵
        P = sparse.csc_matrix(H)
        
        try:
            # 创建或更新 OSQP 问题
            if self._osqp_solver is None:
                self._osqp_solver = osqp.OSQP()
                self._osqp_solver.setup(
                    P=P, q=f, A=A_con, l=l, u=u,
                    warm_start=self.config.osqp_warm_start,
                    verbose=self.config.osqp_verbose,
                    polish=self.config.osqp_polish,
                    eps_abs=self.config.osqp_eps_abs,
                    eps_rel=self.config.osqp_eps_rel,
                    max_iter=self.config.max_iterations * 10  # OSQP 迭代次数
                )
            else:
                # 更新问题参数（热启动）
                self._osqp_solver.update(q=f, l=l, u=u)
            
            # 设置热启动初值
            if self.config.osqp_warm_start and self._U_prev is not None:
                self._osqp_solver.warm_start(x=self._U_prev)
            
            # 求解
            result = self._osqp_solver.solve()
            
            if result.info.status == 'solved' or result.info.status == 'solved_inaccurate':
                U_optimal = result.x
                u = self.gimbal_model.clip_control(U_optimal[0])
                self.u_prev = u
                self._U_prev = U_optimal
                return u, U_optimal, result.info.status
            else:
                # 求解失败，重置求解器
                self._osqp_solver = None
                return None
                
        except Exception:
            # 出错时重置求解器
            self._osqp_solver = None
            return None
    
    def _solve_scipy(self, H: np.ndarray, f: np.ndarray, 
                     current_state: np.ndarray, A_pred_x0: np.ndarray) -> Optional[Tuple[float, np.ndarray]]:
        """
        使用 scipy 求解 QP 问题（后备方案）
        
        Args:
            H: Hessian 矩阵
            f: 梯度向量
            current_state: 当前状态
            A_pred_x0: 预测矩阵与初始状态的乘积
        
        Returns:
            (u_optimal, U_sequence) 或 None（如果求解失败）
        """
        N = self.config.prediction_horizon
        n_state = GimbalModel.STATE_DIM
        
        # 边界约束
        jerk_min = self.gimbal_model.config.jerk_min
        jerk_max = self.gimbal_model.config.jerk_max
        bounds = [(jerk_min, jerk_max) for _ in range(N)]
        
        # 角度边界约束函数
        theta_min = self.gimbal_model.config.theta_min
        theta_max = self.gimbal_model.config.theta_max
        
        def angle_constraints(U):
            X_pred = A_pred_x0 + self.B_pred @ U
            constraints = []
            for i in range(N):
                theta_pred = X_pred[i * n_state]
                # theta_pred >= theta_min
                constraints.append(theta_pred - theta_min)
                # theta_pred <= theta_max
                constraints.append(theta_max - theta_pred)
            return np.array(constraints)
        
        # 初始猜测 (热启动)
        if self._U_prev is not None:
            U0 = self._U_prev
        else:
            U0 = np.zeros(N)
            U0[0] = self.u_prev
        
        # 求解QP问题
        result = minimize(
            fun=lambda U: 0.5 * U @ H @ U + f @ U,
            x0=U0,
            method='SLSQP',
            jac=lambda U: H @ U + f,
            bounds=bounds,
            constraints=[{'type': 'ineq', 'fun': angle_constraints}],
            options={
                'maxiter': self.config.max_iterations,
                'ftol': self.config.ftol,
                'disp': False
            }
        )
        
        if result.success:
            U_optimal = result.x
            u = self.gimbal_model.clip_control(U_optimal[0])
            self.u_prev = u
            self._U_prev = U_optimal
            return u, U_optimal
        else:
            return None
    
    def _prepare_target_trajectory(self, target_trajectory: np.ndarray) -> np.ndarray:
        """
        准备目标轨迹，确保格式正确
        
        Args:
            target_trajectory: 输入目标轨迹
        
        Returns:
            标准化的目标轨迹 (N, 3)
        """
        N = self.config.prediction_horizon
        n_state = GimbalModel.STATE_DIM
        
        if target_trajectory.ndim == 1:
            # 只有角度，需要补充速度和加速度目标
            if len(target_trajectory) != N:
                # 重采样到正确长度
                indices = np.linspace(0, len(target_trajectory) - 1, N).astype(int)
                target_theta = target_trajectory[indices]
            else:
                target_theta = target_trajectory
            
            # 估计目标角速度 (数值微分)
            target_omega = np.zeros(N)
            for i in range(N - 1):
                target_omega[i] = GimbalModel.angle_difference(
                    target_theta[i + 1], target_theta[i]) / self.config.dt
            target_omega[-1] = target_omega[-2] if N > 1 else 0.0
            
            # 目标角加速度设为0
            target_alpha = np.zeros(N)
            
            X_target = np.column_stack([target_theta, target_omega, target_alpha])
        
        elif target_trajectory.shape == (N, n_state):
            X_target = target_trajectory
        else:
            # 尝试调整形状
            X_target = np.zeros((N, n_state))
            for i in range(min(N, len(target_trajectory))):
                if isinstance(target_trajectory[i], (list, np.ndarray)):
                    X_target[i, :len(target_trajectory[i])] = target_trajectory[i][:n_state]
                else:
                    X_target[i, 0] = target_trajectory[i]
        
        return X_target
    
    def predict_state(self, 
                      current_state: np.ndarray, 
                      control_sequence: np.ndarray) -> np.ndarray:
        """
        根据控制序列预测状态轨迹
        
        Args:
            current_state: 当前状态
            control_sequence: 控制序列
        
        Returns:
            预测状态轨迹 (N+1, 3)，包含初始状态
        """
        N = len(control_sequence)
        trajectory = np.zeros((N + 1, GimbalModel.STATE_DIM))
        trajectory[0] = current_state
        
        state = current_state.copy()
        for i in range(N):
            state = self.gimbal_model.predict(state, control_sequence[i])
            trajectory[i + 1] = state
        
        return trajectory
    
    def reset(self):
        """重置控制器状态"""
        self.u_prev = 0.0
        self._U_prev = None
        self._osqp_solver = None
        self.last_solve_time = 0.0
        self.last_success = True
