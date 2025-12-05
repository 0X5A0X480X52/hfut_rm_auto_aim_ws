import numpy as np
from models.models import models

class CV_KF(models):
    def __init__(self, T, Dim, R) -> None:
        super().__init__()
        self.T = T
        self.Dim = Dim
        
        I_dim = np.eye(self.Dim)

        ## 定义 CV 模型的建模噪声：
        q11 = T**3 / 3
        q12 = T**2 / 2
        q21 = q12
        q22 = T

        self.Q_D1 = np.array([[q11, q12],
                              [q21, q22]])

        self.Q = np.kron(I_dim, self.Q_D1) # 最终得到的建模噪声协方差矩阵
        self.w_means = np.zeros(2 * Dim) # 建模噪声的均值（零均值模型）

        ## 定义 CV 模型的状态转移矩阵：
        F_D1 = np.array([[1, T],
                         [0, 1]])

        self.F = np.kron(I_dim, F_D1) # 最终得到的状态转移矩阵

        ## 定义观测模型（仅含观测位置）
        H_D1 = np.array([1, 0])
        self.H = np.kron(I_dim, H_D1) # 观测转移矩阵

        # 观测误差协方差矩阵
        self.R = R
        self.v_means = np.zeros(Dim) # 观测噪声均值（零均值）

        # 设置默认的初始状态
        x = y = v_x = v_y = 0
        X_0 = np.array([x, v_x, y, v_y]).T
        self.KalmanFliter__init__(X_0)
    
    # 设置滤波器的初始状态
    def KalmanFliter__init__(self, X_0):
        self.X_0 = X_0
        self.X_prior = self.X_0

        self.X_after = self.X_prior.copy()
        self.P_after = np.zeros(self.F.shape)
        self.I = np.eye(2 * self.Dim)
    
    # 传入观测序列，返回滤波预测序列（全过程预测）
    def KalmanFliter_wholeProcess(self, measurements):
        ## 卡尔曼滤波：
        predict = measurements.copy()

        for k in range(len(measurements)):
            # 预测：
            self.X_prior = self.F @ self.X_after
            self.P_prior = self.F @ self.P_after @ self.F.T + self.Q
            
            # 校正：
            Kal_Gain = self.P_prior @ self.H.T @ np.linalg.inv(self.H @ self.P_prior @ self.H.T + self.R)
            
            self.Z = measurements[k]
            self.X_after = self.X_prior + Kal_Gain @ (self.Z - self.H @ self.X_prior)
            
            self.P_after = (self.I - Kal_Gain @ self.H) @ self.P_prior
            
            predict[k] = self.H @ self.X_after
            
        return predict
    
    # 传入观测状态，返回滤波预测状态（迭代进行一次预测）
    def KalmanFliter_iterator(self, Z):
        # 预测：
        self.X_prior = self.F @ self.X_after
        self.P_prior = self.F @ self.P_after @ self.F.T + self.Q
            
        # 校正：
        Kal_Gain = self.P_prior @ self.H.T @ np.linalg.inv(self.H @ self.P_prior @ self.H.T + self.R)
            
        self.Z = Z
        self.X_after = self.X_prior + Kal_Gain @ (self.Z - self.H @ self.X_prior)
            
        self.P_after = (self.I - Kal_Gain @ self.H) @ self.P_prior
            
        return self.H @ self.X_after
    
    def getLambda(self, Z):
        Z = self.set_Z(Z)
        pi = 3.14
        r = Z - self.H @ self.X_after
        S = self.H @ self.P_after @ self.H.T + self.R 
        Lambda = (1/(abs(2*pi*np.linalg.det(S)))**0.5)*np.exp((-1/2)*(r.T @ np.linalg.inv(S) @ r))
        return Lambda
