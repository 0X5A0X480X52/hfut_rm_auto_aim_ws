import numpy as np
from models.models import models

class CTRA_EKF(models):
    def __init__(self, T, R, Q=None):
        self.T = T  # 采样周期
        self.R = R  # 测量噪声协方差矩阵
        self.Dim = 5  # 状态维度 [x, y, v, phi, a]

        # 初始化过程噪声协方差矩阵
        if Q is None:
            self.Q = np.eye(self.Dim) * 0.1  # 过程噪声协方差矩阵
        else:
            assert Q.shape == (self.Dim, self.Dim), 'Q should be a square matrix of shape (5, 5)'
            self.Q = Q

        self.P_after = np.eye(self.Dim)  # 后验误差协方差矩阵
        self.X_after = np.zeros(self.Dim)  # 后验状态估计

        # 定义观测模型（仅含观测位置）
        self.H = np.zeros((2, self.Dim))
        self.H[0, 0] = 1
        self.H[1, 1] = 1

    def KalmanFliter__init__(self, X_0):
        self.X_after = X_0  # 初始状态
        self.P_after = np.eye(self.Dim)  # 初始误差协方差矩阵

    def KalmanFliter_wholeProcess(self, measurements):
        predict = []
        for Z in measurements:
            X_pred = self.KalmanFliter_iterator(Z)
            predict.append(X_pred)
        return np.array(predict)

    def KalmanFliter_iterator(self, Z):
        # 提取上一步的状态和协方差
        X_prior = self.X_after
        P_prior = self.P_after

        # 状态预测
        x, y, v, phi, a = X_prior
        # \( f_x = x_k + v_k T \cos(\phi_k) + \frac{1}{2} a_k T^2 \cos(\phi_k) \)
        fx = x + v * self.T * np.cos(phi) + 0.5 * a * self.T**2 * np.cos(phi)
        # \( f_y = y_k + v_k T \sin(\phi_k) + \frac{1}{2} a_k T^2 \sin(\phi_k) \)
        fy = y + v * self.T * np.sin(phi) + 0.5 * a * self.T**2 * np.sin(phi)
        # \( f_v = v_k + a_k T \)
        fv = v + a * self.T
        # \( f_\phi = \phi_k \)
        fphi = phi  # 假设航向角不变
        # \( f_a = a_k \)
        fa = a  # 假设加速度不变
        X_predict = np.array([fx, fy, fv, fphi, fa])

        # 计算状态转移矩阵的雅可比矩阵 F
        F = np.eye(self.Dim)
        # \( F_{0,2} = \frac{\partial f_x}{\partial v} = T \cos(\phi) \)
        F[0,2] = self.T * np.cos(phi)
        # \( F_{0,3} = \frac{\partial f_x}{\partial \phi} = -v T \sin(\phi) - 0.5 a T^2 \sin(\phi) \)
        F[0,3] = -v * self.T * np.sin(phi) - 0.5 * a * self.T**2 * np.sin(phi)
        # \( F_{0,4} = \frac{\partial f_x}{\partial a} = 0.5 T^2 \cos(\phi) \)
        F[0,4] = 0.5 * self.T**2 * np.cos(phi)
        # \( F_{1,2} = \frac{\partial f_y}{\partial v} = T \sin(\phi) \)
        F[1,2] = self.T * np.sin(phi)
        # \( F_{1,3} = \frac{\partial f_y}{\partial \phi} = v T \cos(\phi) + 0.5 a T^2 \cos(\phi) \)
        F[1,3] = v * self.T * np.cos(phi) + 0.5 * a * self.T**2 * np.cos(phi)
        # \( F_{1,4} = \frac{\partial f_y}{\partial a} = 0.5 T^2 \sin(\phi) \)
        F[1,4] = 0.5 * self.T**2 * np.sin(phi)
        # \( F_{2,4} = \frac{\partial f_v}{\partial a} = T \)
        F[2,4] = self.T

        # 预测协方差
        P_predict = F @ P_prior @ F.T + self.Q

        # 计算卡尔曼增益
        S = self.H @ P_predict @ self.H.T + self.R
        K = P_predict @ self.H.T @ np.linalg.inv(S)

        # 更新状态估计
        Y = Z - self.H @ X_predict
        self.X_after = X_predict + K @ Y

        # 更新协方差矩阵
        self.P_after = (np.eye(self.Dim) - K @ self.H) @ P_predict

        # 返回位置预测值
        return self.X_after[:2]

    def getLambda(self, Z):
        pi = 3.14
        r = Z - self.H @ self.X_after
        S = self.H @ self.P_after @ self.H.T + self.R
        Lambda = (1 / (abs(2 * pi * np.linalg.det(S)))**0.5) * np.exp((-1 / 2) * (r.T @ np.linalg.inv(S) @ r))
        return Lambda
