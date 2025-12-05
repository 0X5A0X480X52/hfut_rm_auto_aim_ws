"""
基于二次规划(QP)的MPC装甲板击打仿真 - 含整体匀加速运动
整合 CS_KF 卡尔曼滤波预测和二次规划MPC控制

系统特性：
1. 使用三阶云台动力学模型 (角度-角速度-角加速度)
2. 4个装甲板使用独立的CS_KF进行状态估计和预测
3. 装甲板整体进行匀加速运动（中心角度以恒定角加速度运动）
4. 装甲板相对于中心进行旋转运动
5. 基于二次规划求解MPC优化问题
6. 包含跟踪误差、控制输入和控制变化率的多目标优化
"""

import numpy as np
import matplotlib.pyplot as plt
import time
import sys
import os
from scipy.optimize import minimize
from scipy import linalg

# 添加路径以导入CS_KF
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'code', 'IMM', 'models'))
from models.CS_KF import CS_KF

# 配置 matplotlib 中文字体支持
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'SimSun', 'KaiTi', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False

np.random.seed(42)

# ==================== 系统参数 ====================
dt = 0.02  # 时间步长 (s)
T = 20.0   # 仿真总时长 (s)
steps = int(T / dt)
time_array = np.arange(steps) * dt

# ==================== 云台三阶动力学模型 ====================
# 状态: x = [theta, omega, alpha]^T (角度、角速度、角加速度)
# 控制输入: u = jerk (角加加速度)

A_gimbal = np.array([[1.0,  dt,  0.5*dt**2],
                     [0.0, 1.0,        dt],
                     [0.0, 0.0,       1.0]])

B_gimbal = np.array([[dt**3/6],
                     [dt**2/2],
                     [dt]])

# 云台状态维度
n_state = 3
n_control = 1

# ==================== MPC 参数 ====================
N = 18  # 预测时域 (10步 -> 0.2秒) - 减少以提升性能

# 权重矩阵
Q = np.eye(N * n_state)  # 跟踪误差权重矩阵
for i in range(N):
    Q[i*n_state, i*n_state] = 100.0      # 角度误差权重
    Q[i*n_state+1, i*n_state+1] = 10.0   # 角速度误差权重
    Q[i*n_state+2, i*n_state+2] = 1.0    # 角加速度误差权重

R = 0.01 * np.eye(N * n_control)  # 控制输入权重矩阵
S = 5.0 * np.eye(N * n_control)   # 控制变化率权重矩阵

# 控制约束

def compute_motion_limits(dt, omega_limit_deg, safety_factor=0.8):
    """
    根据采样时间 dt 和期望角速度上限自动计算合理的角速度、角加速度和角加加速度限制

    参数:
        dt: float, 采样时间(s)
        omega_limit_deg: float, 期望角速度上限 (°/s)
        safety_factor: float, 安全系数(0~1)，避免极限值过高，默认0.8

    返回:
        dict 包含:
            'omega_min', 'omega_max'  # rad/s
            'alpha_min', 'alpha_max'  # rad/s²
            'jerk_min',  'jerk_max'   # rad/s³
    """
    # 角速度限制
    omega_max = np.deg2rad(omega_limit_deg) * safety_factor
    omega_min = -omega_max

    # 假设从0加速到最大角速度，需要的最大角加速度
    alpha_max = omega_max / dt * safety_factor
    alpha_min = -alpha_max

    # 假设从0加速度到最大角加速度，需要的最大角加加速度
    jerk_max = alpha_max / dt * safety_factor
    jerk_min = -jerk_max

    return {
        'omega_min': omega_min,
        'omega_max': omega_max,
        'alpha_min': alpha_min,
        'alpha_max': alpha_max,
        'jerk_min': jerk_min,
        'jerk_max': jerk_max
    }

# 示例
limits = compute_motion_limits(dt=dt, omega_limit_deg=10)
print(limits)

omega_min = limits['omega_min']  # 最小角速度
omega_max = limits['omega_max']   # 最大角速度
alpha_min = limits['alpha_min']  # 最小角加速度
alpha_max = limits['alpha_max']   # 最大角加速度
jerk_min = limits['jerk_min']  # 最小角加加速度
jerk_max = limits['jerk_max']   # 最大角加加速度

# 云台角度边界约束
theta_min = np.deg2rad(-30.0)  # 最小角度（向左60度）
theta_max = np.deg2rad(30.0)   # 最大角度（向右60度）

# ==================== 装甲板参数 ====================
num_plates = 4
plate_base_angles = np.array([0.0, np.pi/2, np.pi, 3*np.pi/2])
base_omega = 2.0
# plate_omegas = base_omega + 0.3 * np.random.randn(num_plates)
plate_omegas = np.array([2.0, 2.0, 2.0, 2.0])  # 稳定的不同角速度

# 装甲板整体运动参数（匀加速运动）
center_angle_init = 0.0  # 初始中心角度位置 (rad)
center_omega_init = 0.5  # 初始中心角速度 (rad/s)
center_alpha = 0.1       # 中心角加速度 (rad/s²) - 匀加速运动

# 装甲板真实状态
plate_angles = plate_base_angles.copy()  # 相对于中心的角度
plate_omegas_true = plate_omegas.copy()   # 相对旋转角速度

# 装甲板中心的全局状态
center_angle = center_angle_init
center_omega = center_omega_init

# ==================== CS_KF 滤波器初始化 ====================
# CS模型参数
T_kf = dt
a_kf = 1.0
A_max_kf = np.deg2rad(10.0)  # 最大角加速度
Dim_kf = 2  # 2维（装甲板角度 + 中心角度）

# 观测误差协方差矩阵（角度测量噪声）
R_kf = np.array([[np.deg2rad(2.0)**2, 0.0],
                 [0.0, np.deg2rad(2.0)**2]])

# 为每个装甲板创建KF实例（2维：相对角度 + 中心角度）
kf_filters = []
for i in range(num_plates):
    kf = CS_KF(T_kf, a_kf, A_max_kf, Dim_kf, R_kf)
    # 初始状态 [相对角度, 相对角速度, 相对角加速度, 中心角度, 中心角速度, 中心角加速度]
    X_0 = np.array([plate_base_angles[i], plate_omegas[i], 0.0,
                    center_angle_init, center_omega_init, center_alpha])
    kf.KalmanFliter__init__(X_0)
    kf_filters.append(kf)

# ==================== 命中概率模型 ====================
sigma_theta = np.deg2rad(15.0)
inv_sigma_sq = 1.0 / (sigma_theta ** 2)

def wrap_angle(a):
    """角度归一化到 [-π, π]"""
    return (a + np.pi) % (2 * np.pi) - np.pi

def hit_probability(theta_gimbal, theta_plate):
    """计算命中概率"""
    diff = wrap_angle(theta_gimbal - theta_plate)
    return np.exp(-0.5 * diff * diff * inv_sigma_sq)

def total_hit_probability(theta_gimbal, plate_angles_array):
    """计算对所有装甲板的最大命中概率"""
    probs = np.array([hit_probability(theta_gimbal, pa) for pa in plate_angles_array])
    return np.max(probs)

# ==================== 构建MPC预测矩阵 ====================
def build_prediction_matrices(A, B, N):
    """
    构建预测矩阵 A_pred 和 B_pred
    X_future = A_pred * x0 + B_pred * U
    """
    n = A.shape[0]
    m = B.shape[1]
    
    A_pred = np.zeros((N * n, n))
    B_pred = np.zeros((N * n, N * m))
    
    A_power = A.copy()
    for i in range(N):
        # A_pred 的第 i 行块
        A_pred[i*n:(i+1)*n, :] = A_power
        
        # B_pred 的第 i 行块
        A_power_j = np.eye(n)
        for j in range(i + 1):
            B_pred[i*n:(i+1)*n, j*m:(j+1)*m] = A_power_j @ B
            A_power_j = A @ A_power_j
        
        A_power = A @ A_power
    
    return A_pred, B_pred

# 构建差分矩阵 D (用于控制变化率惩罚)
def build_difference_matrix(N):
    """构建差分矩阵 D，使得 D*U = [u_1-u_0, u_2-u_1, ..., u_N-u_{N-1}]"""
    D = np.zeros((N, N))
    D[0, 0] = 1.0  # 第一个控制输入与0的差
    for i in range(1, N):
        D[i, i] = 1.0
        D[i, i-1] = -1.0
    return D

# 预计算预测矩阵
A_pred, B_pred = build_prediction_matrices(A_gimbal, B_gimbal, N)
D = build_difference_matrix(N)

print("=" * 70)
print("基于二次规划的MPC装甲板击打仿真 - 含整体匀加速运动")
print("=" * 70)
print(f"系统配置：")
print(f"  云台模型: 三阶线性系统 (角度-角速度-角加速度)")
print(f"  状态维度: {n_state}, 控制维度: {n_control}")
print(f"  MPC预测时域: N={N} 步 ({N*dt:.2f} 秒)")
print(f"  装甲板数量: {num_plates}")
print(f"  卡尔曼滤波: CS模型 2维 (相对角度 + 中心角度)")
print(f"  过程噪声: a={a_kf}, A_max={np.rad2deg(A_max_kf):.1f} deg/s²")
print(f"\n装甲板运动模式：")
print(f"  相对旋转角速度: {base_omega:.2f} rad/s ({np.rad2deg(base_omega):.1f} deg/s)")
print(f"  中心角加速度: {center_alpha:.3f} rad/s² ({np.rad2deg(center_alpha):.2f} deg/s²)")
print(f"  初始中心角速度: {center_omega_init:.2f} rad/s ({np.rad2deg(center_omega_init):.1f} deg/s)")
print(f"\n权重参数：")
print(f"  跟踪误差权重: Q_theta={Q[0,0]}, Q_omega={Q[1,1]}, Q_alpha={Q[2,2]}")
print(f"  控制输入权重: R={R[0,0]}")
print(f"  控制变化率权重: S={S[0,0]}")
print(f"\n控制约束：")
print(f"  云台角度范围: [{np.rad2deg(theta_min):.0f}, {np.rad2deg(theta_max):.0f}] deg")
print(f"  角加加速度: [{np.rad2deg(jerk_min):.0f}, {np.rad2deg(jerk_max):.0f}] deg/s³")
print(f"  角速度: [{np.rad2deg(omega_min):.0f}, {np.rad2deg(omega_max):.0f}] deg/s")
print(f"  角加速度: [{np.rad2deg(alpha_min):.0f}, {np.rad2deg(alpha_max):.0f}] deg/s²")
print("=" * 70)

# ==================== MPC 求解器（二次规划）====================
def solve_qp_mpc(x0, X_target, u_prev):
    """
    使用scipy优化求解二次规划MPC问题
    
    参数:
        x0: 当前状态 (n_state,)
        X_target: 目标状态序列 (N, n_state)
        u_prev: 上一时刻的控制输入
    
    返回:
        U_opt: 最优控制序列 (N,)
        solve_time: 求解时间
    """
    start_time = time.perf_counter()
    
    # 目标状态序列（展平）
    X_target_flat = X_target.flatten()
    
    # 预计算常量项
    A_pred_x0 = A_pred @ x0
    
    # 定义目标函数
    # J = (X_pred - X_target)^T * Q * (X_pred - X_target) + U^T * R * U + (D*U)^T * S * (D*U)
    # 其中 X_pred = A_pred @ x0 + B_pred @ U
    
    # 展开后得到标准二次型: J = 0.5 * U^T * H * U + f^T * U + const
    # H = 2 * (B_pred^T * Q * B_pred + R + D^T * S * D)
    # f = 2 * B_pred^T * Q * (A_pred @ x0 - X_target)
    
    H = 2 * (B_pred.T @ Q @ B_pred + R + D.T @ S @ D)
    f = 2 * B_pred.T @ Q @ (A_pred_x0 - X_target_flat)
    
    # 定义代价函数
    def cost_function(U):
        return 0.5 * U @ H @ U + f @ U
    
    # 定义代价函数的梯度
    def cost_gradient(U):
        return H @ U + f
    
    # 定义约束函数：角度边界约束
    def angle_constraints(U):
        """
        确保预测的云台角度在 [theta_min, theta_max] 范围内
        返回约束值数组，约束形式为 g(U) >= 0
        """
        X_pred = A_pred_x0 + B_pred @ U
        constraints = []
        
        for i in range(N):
            theta_pred = X_pred[i * n_state]  # 预测的角度
            # theta_pred >= theta_min
            constraints.append(theta_pred - theta_min)
            # theta_pred <= theta_max
            constraints.append(theta_max - theta_pred)
        
        return np.array(constraints)
    
    # 边界约束
    bounds = [(jerk_min, jerk_max) for _ in range(N)]
    
    # 构建约束字典（用于SLSQP）
    constraints = [
        {'type': 'ineq', 'fun': angle_constraints}
    ]
    
    # 初始猜测（使用热启动）
    U0 = np.zeros(N)
    if u_prev is not None:
        U0[0] = u_prev
    
    # 使用SLSQP求解（包含角度边界约束）
    result = minimize(
        fun=cost_function,
        x0=U0,
        method='SLSQP',
        jac=cost_gradient,
        bounds=bounds,
        constraints=constraints,
        options={'maxiter': 30, 'ftol': 1e-5, 'disp': False}
    )
    
    solve_time = time.perf_counter() - start_time
    
    if result.success:
        U_opt = result.x
        return U_opt, solve_time
    else:
        # 优化失败，返回零控制
        return np.zeros(N), solve_time

# ==================== 装甲板预测（基于卡尔曼滤波）====================
def predict_plates_with_kf(kf_filters, measurements, N):
    """
    使用卡尔曼滤波器预测装甲板未来N步的状态
    
    参数:
        kf_filters: KF滤波器列表
        measurements: 当前观测值 (num_plates, 2) - [相对角度, 中心角度]
        N: 预测步数
    
    返回:
        predictions: (N+1, num_plates) 预测绝对角度序列（包含当前时刻）
        center_predictions: (N+1,) 预测中心角度序列
    """
    predictions = np.zeros((N + 1, num_plates))
    center_predictions = np.zeros(N + 1)
    
    for i, kf in enumerate(kf_filters):
        # 更新滤波器
        kf.KalmanFliter_iterator(measurements[i, :])
        
        # 当前状态：相对角度[0] + 中心角度[3]
        plate_relative_angle = kf.X_after[0]
        center_angle_current = kf.X_after[3]
        predictions[0, i] = wrap_angle(plate_relative_angle + center_angle_current)
        
        if i == 0:  # 只需计算一次中心角度预测
            center_predictions[0] = center_angle_current
        
        # 预测未来N步
        X_pred = kf.X_after.copy()
        for k in range(N):
            X_pred = kf.F @ X_pred
            plate_relative_angle_pred = X_pred[0]
            center_angle_pred = X_pred[3]
            predictions[k+1, i] = wrap_angle(plate_relative_angle_pred + center_angle_pred)
            
            if i == 0:
                center_predictions[k+1] = center_angle_pred
    
    return predictions, center_predictions

# ==================== 选择目标装甲板 ====================
def select_target_plate(theta_gimbal, plate_predictions, omega_gimbal):
    """
    选择最优目标装甲板
    策略：选择未来时刻命中概率最高的装甲板
    """
    N_pred = plate_predictions.shape[0]
    scores = np.zeros(num_plates)
    
    # 预测云台未来位置（简单线性外推）
    for i in range(num_plates):
        future_hit_prob = 0.0
        for k in range(1, min(N_pred, 10)):  # 考虑未来10步
            theta_future = theta_gimbal + omega_gimbal * k * dt
            hit_prob = hit_probability(theta_future, plate_predictions[k, i])
            future_hit_prob += hit_prob * (0.9 ** k)  # 时间折扣
        scores[i] = future_hit_prob
    
    target_idx = np.argmax(scores)
    return target_idx

# ==================== 仿真主循环 ====================
# 状态存储
x_gimbal = np.array([0.0, 0.0, 0.0])  # [角度, 角速度, 角加速度]
theta_log = np.zeros(steps)
omega_log = np.zeros(steps)
alpha_log = np.zeros(steps)
u_log = np.zeros(steps)
plate_angles_log = np.zeros((steps, num_plates))
plate_measurements_log = np.zeros((steps, num_plates))
plate_predictions_log = np.zeros((steps, num_plates))
target_plate_log = np.zeros(steps, dtype=int)
hit_prob_log = np.zeros(steps)
solve_time_log = np.zeros(steps)
tracking_error_log = np.zeros(steps)

u_prev = 0.0

# 观测噪声
measurement_noise_std = np.deg2rad(2.0)

# 中心运动记录
center_angle_log = np.zeros(steps)
center_omega_log = np.zeros(steps)
center_alpha_log = np.zeros(steps)

print("\n" + "=" * 70)
print("仿真参数：")
print(f"  总时长: {T:.2f} 秒, 步长: {dt:.3f} 秒, 步数: {steps}")
print(f"  装甲板数量: {num_plates}, 卡尔曼滤波: CS模型 2维")
print(f"  装甲板相对旋转角速度: {base_omega:.2f} rad/s")
print(f"  装甲板角速度扰动： {plate_omegas}")
print(f"  装甲板中心加速度: {center_alpha:.3f} rad/s² ({np.rad2deg(center_alpha):.2f} deg/s²)")
print(f"  卡尔曼滤波过程噪声: a={a_kf}, A_max={np.rad2deg(A_max_kf):.1f} deg/s²")
print(f"  卡尔曼滤波观测噪声: {np.rad2deg(R_kf[0,0]**0.5):.2f} deg")
print(f"  观测噪声标准差: {np.rad2deg(measurement_noise_std):.2f} deg")
print(f"  MPC预测时域: N={N} 步 ({N*dt:.2f} 秒)")
print("=" * 70)

print("\n开始仿真...")
last_print_time = 0

for k in range(steps):
    current_time = time.perf_counter()
    if current_time - last_print_time > 2.0 or k % 100 == 0:  # 每2秒或每100步打印一次
        print(f"进度: {k}/{steps} ({k/steps*100:.1f}%) - 时间: {time_array[k]:.2f}s")
        last_print_time = current_time
    
    # ==================== 更新装甲板真实状态 ====================
    # 更新中心运动（匀加速）
    center_omega += center_alpha * dt
    center_angle = wrap_angle(center_angle + center_omega * dt)
    
    # 更新相对旋转
    plate_omegas_true += 0.01 * np.random.randn(num_plates) * 0.3
    plate_angles = wrap_angle(plate_angles + plate_omegas_true * dt)
    
    # 计算绝对角度（相对角度 + 中心角度）
    absolute_plate_angles = wrap_angle(plate_angles + center_angle)
    plate_angles_log[k, :] = absolute_plate_angles
    
    # 记录中心运动
    center_angle_log[k] = center_angle
    center_omega_log[k] = center_omega
    center_alpha_log[k] = center_alpha
    
    # ==================== 生成观测（带噪声）====================
    # 观测：[相对角度, 中心角度]
    measurements = np.zeros((num_plates, 2))
    for i in range(num_plates):
        measurements[i, 0] = wrap_angle(plate_angles[i] + np.random.randn() * measurement_noise_std)
        measurements[i, 1] = wrap_angle(center_angle + np.random.randn() * measurement_noise_std)
    
    # 记录绝对角度观测（用于可视化）
    plate_measurements_log[k, :] = wrap_angle(measurements[:, 0] + measurements[:, 1])
    
    # ==================== 卡尔曼滤波预测 ====================
    plate_predictions, center_predictions = predict_plates_with_kf(kf_filters, measurements, N)
    
    # ==================== 选择目标装甲板 ====================
    target_idx = select_target_plate(x_gimbal[0], plate_predictions, x_gimbal[1])
    target_plate_log[k] = target_idx
    
    # 记录预测值
    plate_predictions_log[k, :] = plate_predictions[0, :]
    
    # ==================== 构建目标状态序列 ====================
    # 目标：跟踪选定装甲板的预测轨迹
    X_target = np.zeros((N, n_state))
    for i in range(N):
        theta_target = plate_predictions[i+1, target_idx]
        # 估计目标角速度（数值微分）
        if i < N - 1:
            omega_target = wrap_angle(plate_predictions[i+2, target_idx] - 
                                     plate_predictions[i+1, target_idx]) / dt
        else:
            omega_target = wrap_angle(plate_predictions[i+1, target_idx] - 
                                     plate_predictions[i, target_idx]) / dt
        
        X_target[i, :] = [theta_target, omega_target, 0.0]
    
    # ==================== MPC 优化求解 ====================
    U_opt, solve_time = solve_qp_mpc(x_gimbal, X_target, u_prev)
    
    # 应用第一个控制输入
    u = U_opt[0] if U_opt is not None else 0.0
    u = np.clip(u, jerk_min, jerk_max)
    
    # ==================== 状态更新 ====================
    x_gimbal = A_gimbal @ x_gimbal + B_gimbal.flatten() * u
    
    # 约束状态
    x_gimbal[1] = np.clip(x_gimbal[1], omega_min, omega_max)
    x_gimbal[2] = np.clip(x_gimbal[2], alpha_min, alpha_max)
    
    # 角度边界约束和归一化
    x_gimbal[0] = wrap_angle(x_gimbal[0])
    
    # 硬性限制角度在边界内
    if x_gimbal[0] > theta_max:
        x_gimbal[0] = theta_max
        x_gimbal[1] = 0.0  # 到达边界时停止
    elif x_gimbal[0] < theta_min:
        x_gimbal[0] = theta_min
        x_gimbal[1] = 0.0  # 到达边界时停止
    
    # ==================== 记录数据 ====================
    theta_log[k] = x_gimbal[0]
    omega_log[k] = x_gimbal[1]
    alpha_log[k] = x_gimbal[2]
    u_log[k] = u
    hit_prob_log[k] = total_hit_probability(x_gimbal[0], absolute_plate_angles)
    solve_time_log[k] = solve_time
    tracking_error_log[k] = abs(wrap_angle(x_gimbal[0] - absolute_plate_angles[target_idx]))
    
    u_prev = u

print("仿真完成！")

# ==================== 结果分析 ====================
print("\n" + "=" * 70)
print("结果统计：")
print("=" * 70)

mean_hit_prob = np.mean(hit_prob_log)
high_hit_ratio_05 = np.sum(hit_prob_log > 0.5) / steps * 100
high_hit_ratio_07 = np.sum(hit_prob_log > 0.7) / steps * 100

mean_tracking_error = np.mean(tracking_error_log) * 180 / np.pi
std_tracking_error = np.std(tracking_error_log) * 180 / np.pi

mean_solve_time = np.mean(solve_time_log) * 1000
std_solve_time = np.std(solve_time_log) * 1000
max_solve_time = np.max(solve_time_log) * 1000
mean_frequency = 1.0 / np.mean(solve_time_log)

print(f"命中性能：")
print(f"  平均命中概率: {mean_hit_prob:.4f}")
print(f"  高命中时间占比 (>0.5): {high_hit_ratio_05:.2f}%")
print(f"  高命中时间占比 (>0.7): {high_hit_ratio_07:.2f}%")

print(f"\n跟踪性能：")
print(f"  平均跟踪误差: {mean_tracking_error:.2f} deg")
print(f"  跟踪误差标准差: {std_tracking_error:.2f} deg")

print(f"\n控制性能：")
print(f"  平均角速度幅值: {np.mean(np.abs(omega_log)) * 180/np.pi:.2f} deg/s")
print(f"  平均角加速度幅值: {np.mean(np.abs(alpha_log)) * 180/np.pi:.2f} deg/s²")
print(f"  平均角加加速度幅值: {np.mean(np.abs(u_log)) * 180/np.pi:.2f} deg/s³")

# 边界触碰统计
boundary_margin = np.deg2rad(1.0)  # 1度的边界容差
near_upper = np.sum(theta_log > (theta_max - boundary_margin))
near_lower = np.sum(theta_log < (theta_min + boundary_margin))
at_boundary = near_upper + near_lower

print(f"\n角度边界统计：")
print(f"  云台角度范围: [{np.rad2deg(theta_min):.0f}°, {np.rad2deg(theta_max):.0f}°]")
print(f"  最大角度: {np.rad2deg(np.max(theta_log)):.2f}°")
print(f"  最小角度: {np.rad2deg(np.min(theta_log)):.2f}°")
print(f"  接近上边界次数 (>{theta_max-boundary_margin:.3f} rad): {near_upper} ({near_upper/steps*100:.2f}%)")
print(f"  接近下边界次数 (<{theta_min+boundary_margin:.3f} rad): {near_lower} ({near_lower/steps*100:.2f}%)")
print(f"  总边界触碰占比: {at_boundary/steps*100:.2f}%")

print(f"\n计算性能：")
print(f"  平均求解时间: {mean_solve_time:.2f} ms")
print(f"  求解时间标准差: {std_solve_time:.2f} ms")
print(f"  最大求解时间: {max_solve_time:.2f} ms")
print(f"  平均帧率: {mean_frequency:.2f} Hz")

if mean_frequency > 50:
    print(f"  ✓ 满足实时要求 (>50Hz)")
elif mean_frequency > 30:
    print(f"  ⚠ 基本满足实时要求 (30-50Hz)")
else:
    print(f"  ✗ 不满足实时要求 (<30Hz)")

print("\n目标切换统计：")
for i in range(num_plates):
    ratio = np.sum(target_plate_log == i) / steps * 100
    print(f"  装甲板 {i+1}: {ratio:.2f}%")

# ==================== 可视化 ====================
fig, axes = plt.subplots(7, 1, figsize=(14, 18))

# 子图1：角度跟踪
axes[0].plot(time_array, theta_log * 180/np.pi, 'b-', linewidth=2.5, label='云台角度')
# 绘制角度边界
axes[0].axhline(y=np.rad2deg(theta_max), color='r', linestyle='-.', linewidth=2, 
               alpha=0.7, label=f'角度上限 ({np.rad2deg(theta_max):.0f}°)')
axes[0].axhline(y=np.rad2deg(theta_min), color='r', linestyle='-.', linewidth=2, 
               alpha=0.7, label=f'角度下限 ({np.rad2deg(theta_min):.0f}°)')
# 绘制中心运动轨迹
axes[0].plot(time_array, center_angle_log * 180/np.pi, 'k--', linewidth=2, alpha=0.7, label='装甲板中心')
for i in range(num_plates):
    axes[0].plot(time_array, plate_angles_log[:, i] * 180/np.pi, '--', 
                alpha=0.5, linewidth=1.5, label=f'装甲板 {i+1}')
axes[0].set_ylabel('角度 (deg)')
axes[0].set_title('QP-MPC云台控制 - 角度跟踪（含角度边界约束）', fontsize=14, fontweight='bold')
axes[0].legend(loc='upper right', ncol=4, fontsize=9)
axes[0].grid(True, alpha=0.3)

# 子图2：命中概率
axes[1].plot(time_array, hit_prob_log, 'r-', linewidth=2)
axes[1].axhline(y=0.5, color='k', linestyle='--', alpha=0.3, linewidth=1.5)
axes[1].axhline(y=0.7, color='k', linestyle='--', alpha=0.3, linewidth=1.5)
axes[1].fill_between(time_array, 0, hit_prob_log, where=(hit_prob_log > 0.5), 
                     alpha=0.3, color='green')
axes[1].set_ylabel('命中概率')
axes[1].set_title(f'命中概率 (平均: {mean_hit_prob:.3f})', fontsize=14, fontweight='bold')
axes[1].grid(True, alpha=0.3)
axes[1].set_ylim([0, 1.05])

# 子图3：控制输入（角加加速度）
axes[2].plot(time_array, u_log * 180/np.pi, 'g-', linewidth=1.5)
axes[2].axhline(y=0, color='k', linestyle='-', alpha=0.2)
axes[2].set_ylabel('角加加速度 (deg/s³)')
axes[2].set_title('MPC控制输入 (Jerk)', fontsize=14, fontweight='bold')
axes[2].grid(True, alpha=0.3)

# 子图4：云台状态（角速度和角加速度）
ax4_1 = axes[3]
ax4_2 = ax4_1.twinx()
line1 = ax4_1.plot(time_array, omega_log * 180/np.pi, 'm-', linewidth=1.5, label='角速度')
line2 = ax4_2.plot(time_array, alpha_log * 180/np.pi, 'c-', linewidth=1.5, label='角加速度')
ax4_1.set_ylabel('角速度 (deg/s)', color='m')
ax4_2.set_ylabel('角加速度 (deg/s²)', color='c')
ax4_1.tick_params(axis='y', labelcolor='m')
ax4_2.tick_params(axis='y', labelcolor='c')
ax4_1.set_title('云台运动状态', fontsize=14, fontweight='bold')
ax4_1.grid(True, alpha=0.3)
lines = line1 + line2
labels = [l.get_label() for l in lines]
ax4_1.legend(lines, labels, loc='upper right')

# 子图5：卡尔曼滤波效果
axes[4].plot(time_array, plate_angles_log[:, 0] * 180/np.pi, 'b-', 
            linewidth=2, label='真实角度 (板1)', alpha=0.7)
axes[4].scatter(time_array[::5], plate_measurements_log[::5, 0] * 180/np.pi, 
               s=10, c='r', alpha=0.5, label='观测值')
axes[4].plot(time_array, plate_predictions_log[:, 0] * 180/np.pi, 'g--', 
            linewidth=2, label='KF预测', alpha=0.8)
axes[4].set_ylabel('角度 (deg)')
axes[4].set_title('卡尔曼滤波效果示例 (装甲板1)', fontsize=14, fontweight='bold')
axes[4].legend(loc='upper right')
axes[4].grid(True, alpha=0.3)

# 子图6：装甲板中心运动状态
ax6_1 = axes[5]
ax6_2 = ax6_1.twinx()
line1 = ax6_1.plot(time_array, center_angle_log * 180/np.pi, 'k-', linewidth=2, label='中心角度')
line2 = ax6_2.plot(time_array, center_omega_log * 180/np.pi, 'purple', linewidth=1.5, label='中心角速度')
ax6_1.set_ylabel('中心角度 (deg)', color='k')
ax6_2.set_ylabel('中心角速度 (deg/s)', color='purple')
ax6_1.tick_params(axis='y', labelcolor='k')
ax6_2.tick_params(axis='y', labelcolor='purple')
ax6_1.set_title(f'装甲板中心匀加速运动 (加速度: {np.rad2deg(center_alpha):.2f} deg/s²)', 
               fontsize=14, fontweight='bold')
ax6_1.grid(True, alpha=0.3)
lines = line1 + line2
labels = [l.get_label() for l in lines]
ax6_1.legend(lines, labels, loc='upper left')

# 子图7：求解时间
axes[6].plot(time_array, solve_time_log * 1000, 'orange', linewidth=1, alpha=0.7)
axes[6].axhline(y=mean_solve_time, color='r', linestyle='--', linewidth=2, 
               label=f'平均: {mean_solve_time:.2f} ms')
axes[6].axhline(y=20, color='g', linestyle='--', linewidth=1.5, alpha=0.7, 
               label='目标: 20 ms (50Hz)')
axes[6].set_xlabel('时间 (s)')
axes[6].set_ylabel('求解时间 (ms)')
axes[6].set_title(f'QP求解性能 - 平均帧率: {mean_frequency:.2f} Hz', 
                 fontsize=14, fontweight='bold')
axes[6].legend(loc='upper right')
axes[6].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('test05_qp_mpc_with_acceleration_results.png', dpi=150, bbox_inches='tight')
print(f"\n结果图已保存: test05_qp_mpc_with_acceleration_results.png")
plt.show()

# ==================== 附加分析图 ====================
fig2, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(14, 10))

# 跟踪误差分布
ax1.hist(tracking_error_log * 180/np.pi, bins=50, color='skyblue', 
        edgecolor='black', alpha=0.7)
ax1.axvline(mean_tracking_error, color='r', linestyle='--', linewidth=2, 
           label=f'平均: {mean_tracking_error:.2f}°')
ax1.set_xlabel('跟踪误差 (deg)')
ax1.set_ylabel('频数')
ax1.set_title('跟踪误差分布')
ax1.legend()
ax1.grid(True, alpha=0.3)

# 求解时间CDF
sorted_times = np.sort(solve_time_log * 1000)
cdf = np.arange(1, len(sorted_times) + 1) / len(sorted_times)
ax2.plot(sorted_times, cdf * 100, 'b-', linewidth=2)
ax2.axvline(20, color='g', linestyle='--', linewidth=1.5, label='目标: 20 ms')
ax2.axhline(95, color='orange', linestyle='--', linewidth=1, alpha=0.5)
ax2.set_xlabel('求解时间 (ms)')
ax2.set_ylabel('累积概率 (%)')
ax2.set_title('求解时间累积分布 (CDF)')
ax2.legend()
ax2.grid(True, alpha=0.3)

# 目标切换时间线
target_colors = ['red', 'blue', 'green', 'orange']
for i in range(num_plates):
    mask = target_plate_log == i
    ax3.scatter(time_array[mask], np.ones(np.sum(mask)) * i, 
               c=target_colors[i], s=1, alpha=0.5, label=f'板{i+1}')
ax3.set_xlabel('时间 (s)')
ax3.set_ylabel('目标装甲板')
ax3.set_title('目标选择时间线')
ax3.set_yticks(range(num_plates))
ax3.set_yticklabels([f'板{i+1}' for i in range(num_plates)])
ax3.legend()
ax3.grid(True, alpha=0.3)

# 控制输入功率谱密度
from scipy import signal
freqs, psd = signal.welch(u_log, fs=1/dt, nperseg=256)
ax4.semilogy(freqs, psd, 'b-', linewidth=1.5)
ax4.set_xlabel('频率 (Hz)')
ax4.set_ylabel('功率谱密度')
ax4.set_title('控制输入频谱分析')
ax4.grid(True, alpha=0.3, which='both')

plt.tight_layout()
plt.savefig('test05_qp_mpc_with_acceleration_analysis.png', dpi=150, bbox_inches='tight')
print(f"分析图已保存: test05_qp_mpc_with_acceleration_analysis.png")
plt.show()

print("\n" + "=" * 70)
print("仿真结束")
print("=" * 70)
