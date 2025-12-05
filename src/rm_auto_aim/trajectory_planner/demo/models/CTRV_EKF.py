import numpy as np
from models.models import models

class CTRV_EKF(models):
    def __init__(self, T, R):
        super().__init__()
        self.T = T  # 采样周期
        self.R = R  # 测量噪声协方差矩阵
        self.Dim = 5  # 状态维度 [x, y, v, theta, omega]
        
        # 初始化状态转移矩阵的雅可比矩阵和过程噪声协方差矩阵

        self.Q = np.eye(self.Dim) * 0.1  # 过程噪声协方差矩阵
        self.P_after = np.eye(self.Dim)  # 后验误差协方差矩阵
        self.X_after = np.zeros(self.Dim)  # 后验状态估计
        
        ## 定义观测模型（仅含观测位置）
        # 观测矩阵的雅可比矩阵 H
        self.H = np.zeros((2, self.Dim))
        self.H[0,0] = 1
        self.H[1,1] = 1
        
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
        theta = X_prior[3]

        # 动态计算过程噪声协方差矩阵 Q
        sigma_a = 0.1  # 加速度标准差，根据需要设置
        sigma_omega_dot = 0.01  # 角速度变化率标准差，根据需要设置
        delta_t = self.T
        # \( Q = \begin{bmatrix}
        # \left(\frac{1}{2} \Delta t^2 \sigma_a \cos(\theta)\right)^2 & \frac{1}{4} \Delta t^4 \sigma_a^2 \sin(\theta)\cos(\theta) & \frac{1}{2} \Delta t^3 \sigma_a^2 \cos(\theta) & 0 & 0 \\
        # \frac{1}{4} \Delta t^4 \sigma_a^2 \sin(\theta)\cos(\theta) & \left(\frac{1}{2} \Delta t^2 \sigma_a \sin(\theta)\right)^2 & \frac{1}{2} \Delta t^3 \sigma_a^2 \sin(\theta) & 0 & 0 \\
        # \frac{1}{2} \Delta t^3 \sigma_a^2 \cos(\theta) & \frac{1}{2} \Delta t^3 \sigma_a^2 \sin(\theta) & \Delta t^2 \sigma_a^2 & 0 & 0 \\
        # 0 & 0 & 0 & \left(\frac{1}{2} \Delta t^2 \sigma_{\dot{\omega}}\right)^2 & \frac{1}{2} \Delta t^3 \sigma_{\dot{\omega}}^2 \\
        # 0 & 0 & 0 & \frac{1}{2} \Delta t^3 \sigma_{\dot{\omega}}^2 & \Delta t^2 \sigma_{\dot{\omega}}^2 \\
        # \end{bmatrix} \)
        q11 = (0.5 * delta_t**2 * sigma_a * np.cos(theta))**2
        q12 = 0.25 * delta_t**4 * sigma_a**2 * np.sin(theta) * np.cos(theta)
        q13 = 0.5 * delta_t**3 * sigma_a**2 * np.cos(theta)
        q22 = (0.5 * delta_t**2 * sigma_a * np.sin(theta))**2
        q23 = 0.5 * delta_t**3 * sigma_a**2 * np.sin(theta)
        q33 = delta_t**2 * sigma_a**2
        q44 = (0.5 * delta_t**2 * sigma_omega_dot)**2
        q45 = 0.5 * delta_t**3 * sigma_omega_dot**2
        q55 = delta_t**2 * sigma_omega_dot**2
        self.Q = np.array([
            [q11, q12, q13, 0,   0],
            [q12, q22, q23, 0,   0],
            [q13, q23, q33, 0,   0],
            [0,   0,   0,   q44, q45],
            [0,   0,   0,   q45, q55]
        ])

        # 状态转移函数
        theta = X_prior[3]
        omega = X_prior[4]
        if omega != 0:
            # \( f_x = x_k + \frac{v_k}{\omega_k} \left( \sin(\theta_k + \omega_k T) - \sin(\theta_k) \right) \)
            fx = X_prior[0] + (X_prior[2]/omega) * (np.sin(theta + omega*self.T) - np.sin(theta))
            # \( f_y = y_k + \frac{v_k}{\omega_k} \left( -\cos(\theta_k + \omega_k T) + \cos(\theta_k) \right) \)
            fy = X_prior[1] + (X_prior[2]/omega) * (-np.cos(theta + omega*self.T) + np.cos(theta))
        else:
            # \( f_x = x_k + v_k T \cos(\theta_k) \)
            fx = X_prior[0] + X_prior[2]*self.T*np.cos(theta)
            # \( f_y = y_k + v_k T \sin(\theta_k) \)
            fy = X_prior[1] + X_prior[2]*self.T*np.sin(theta)
        # \( f_v = v_k \)
        fv = X_prior[2]
        # \( f_\theta = \theta_k + \omega_k T \)
        ftheta = theta + omega*self.T
        # \( f_\omega = \omega_k \)
        fomega = omega
        X_predict = np.array([fx, fy, fv, ftheta, fomega])

        # 计算状态转移矩阵的雅可比矩阵 F（注释使用 latex 格式注释了对应公式）
        F = np.eye(self.Dim)
        if omega != 0:
            # \( F_{0,2} = \frac{\partial f_x}{\partial v} = \frac{1}{\omega} \left( \sin(\theta + \omega T) - \sin(\theta) \right) \)
            F[0,2] = (1/omega)*(np.sin(theta + omega*self.T) - np.sin(theta))
            # \( F_{0,3} = \frac{\partial f_x}{\partial \theta} = \frac{v}{\omega} \left( \cos(\theta + \omega T) - \cos(\theta) \right) \)
            F[0,3] = (X_prior[2]/omega)*(np.cos(theta + omega*self.T) - np.cos(theta))
            # \( F_{0,4} = \frac{\partial f_x}{\partial \omega} = \frac{v}{\omega^2} \left( \sin(\theta) - \sin(\theta + \omega T) \right) + \frac{v T}{\omega} \cos(\theta + \omega T) \)
            F[0,4] = (X_prior[2]/omega**2)*(np.sin(theta) - np.sin(theta + omega*self.T)) + \
                      (X_prior[2]*self.T/omega)*np.cos(theta + omega*self.T)
            # \( F_{1,2} = \frac{\partial f_y}{\partial v} = \frac{1}{\omega} \left( -\cos(\theta + \omega T) + \cos(\theta) \right) \)
            F[1,2] = (1/omega)*(-np.cos(theta + omega*self.T) + np.cos(theta))
            # \( F_{1,3} = \frac{\partial f_y}{\partial \theta} = \frac{v}{\omega} \left( \sin(\theta + \omega T) - \sin(\theta) \right) \)
            F[1,3] = (X_prior[2]/omega)*(np.sin(theta + omega*self.T) - np.sin(theta))
            # \( F_{1,4} = \frac{\partial f_y}{\partial \omega} = \frac{v}{\omega^2} \left( \cos(\theta + \omega T) - \cos(\theta) \right) + \frac{v T}{\omega} \sin(\theta + \omega T) \)
            F[1,4] = (X_prior[2]/omega**2)*(np.cos(theta + omega*self.T) - np.cos(theta)) + \
                      (X_prior[2]*self.T/omega)*np.sin(theta + omega*self.T)
        else:
            # \( F_{0,2} = \frac{\partial f_x}{\partial v} = T \cos(\theta) \)
            F[0,2] = self.T*np.cos(theta)
            # \( F_{0,3} = \frac{\partial f_x}{\partial \theta} = -v T \sin(\theta) \)
            F[0,3] = -X_prior[2]*self.T*np.sin(theta)
            # \( F_{1,2} = \frac{\partial f_y}{\partial v} = T \sin(\theta) \)
            F[1,2] = self.T*np.sin(theta)
            # \( F_{1,3} = \frac{\partial f_y}{\partial \theta} = v T \cos(\theta) \)
            F[1,3] = X_prior[2]*self.T*np.cos(theta)
        # \( F_{3,4} = \frac{\partial f_\theta}{\partial \omega} = T \)
        F[3,4] = self.T

        # 预测协方差
        P_predict = F @ P_prior @ F.T + self.Q

        # 计算卡尔曼增益
        S = self.H @ P_predict @ self.H.T + self.R
        Kal_Gain = P_predict @ self.H.T @ np.linalg.inv(S)

        # 更新状态估计
        Y = Z - self.H @ X_predict    
        self.X_after = X_predict +  Kal_Gain @ Y

        # 更新协方差矩阵
        self.P_after = (np.eye(self.Dim) -  Kal_Gain @ self.H) @ P_predict

        # 返回位置预测值
        return self.X_after[:2]

    def getLambda(self, Z):
        Z = self.set_Z(Z)
        pi = 3.14
        r = Z - self.H @ self.X_after
        S = self.H @ self.P_after @ self.H.T + self.R 
        Lambda = (1/(abs(2*pi*np.linalg.det(S)))**0.5)*np.exp((-1/2)*(r.T @ np.linalg.inv(S) @ r))
        return Lambda