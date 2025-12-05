import numpy as np
import matplotlib.pyplot as plt

from models.models import models

class Singer_KF(models):
    def __init__(self, T, a, sigma, Dim, R ) -> None:
        super().__init__()
        self.T = T
        self.a = a
        self.sigma = sigma
        self.Dim = Dim
        
        I_dim = np.eye(self.Dim)

        ## 定义 Singer 模型的建模噪声：
        q11=(1-np.exp(-2*a*T) + 2*a*T + 2*a**3*T**3/3 - 2*a**2*T**2 - 4*a*T*np.exp(-a*T) )/(2*a**5)
        q12=(np.exp(-2*a*T)+1-2*np.exp(-a*T)+2*a*T*np.exp(-a*T)-2*a*T+a**2*T**2)/(2*a**4)
        q13=(1-np.exp(-2*a*T)-2*a*T*np.exp(-a*T))/(2*a**3) 
        q21=q12
        q22=(4*np.exp(-a*T)-3-np.exp(-2*a*T)+2*a*T)/(2*a**3) 
        q23=(np.exp(-2*a*T)+1-2*np.exp(-a*T))/(2*a**2) 
        q31=q13 
        q32=q23 
        q33=(1-np.exp(-2*a*T))/(2*a)

        Q_D1 = np.array([[q11,q12,q13],
                         [q21,q22,q23],
                         [q31,q32,q33]])

        self.Q = 2 * a * sigma ** 2 * np.kron(I_dim,Q_D1) # 最终得到的建模噪声协方差矩阵
        self.w_means = np.zeros(3*Dim) # 建模噪声的均值（零均值模型）

        ## 定义 Singer 模型的状态转移矩阵：

        F_D1 = np.array([[1, T, (a*T-1+np.exp(-a*T))/a**2],
                        [0, 1,        (1-np.exp(-a*T))/a],
                        [0, 0,              np.exp(-a*T)]]) 

        self.F = np.kron( I_dim, F_D1 ) # 最终得到的状态转移矩阵

        ## 定义观测模型（仅含观测位置）

        H_D1 = np.array([1,0,0])
        self.H = np.kron( I_dim, H_D1 ) # 观测转移矩阵

        # 观测误差协方差矩阵
        self.R = R
        self.v_means = np.zeros(2) # 观测噪声均值（零均值）

        # 设置默认的初始状态
        x = y = v_x = v_y = a_x = a_y = 0

        X_0 = np.array([x, v_x, a_x, y, v_y, a_y]).T
        self.KalmanFliter__init__(X_0)
        
    # 设置滤波器的初始状态
    def KalmanFliter__init__(self, X_0):
        self.X_0 = X_0
        self.X_prior = self.X_0

        self.X_after = self.X_prior.copy()
        self.P_after = np.zeros(self.F.shape)
        self.I = np.eye(3*self.Dim)
    
    # 传入观测序列，返回滤波预测序列（全过程预测）
    def KalmanFliter_wholeProcess(self, measurements):
        ## 卡尔曼滤波：
        predict = measurements.copy()

        for k in range(len(measurements)):
            
            # 预测：
            self.X_prior = self.F @ self.X_after
            self.P_prior = self.F @ self.P_after @ self.F.T + self.Q
            
            # 校正：
            Kal_Gain = self.P_prior @ self.H.T @ np.linalg.inv( self.H @ self.P_prior @ self.H.T + self.R )
            
            self.Z = measurements[k]
            self.X_after = self.X_prior + Kal_Gain @ ( self.Z - self.H @ self.X_prior )
            
            self.P_after = ( self.I - Kal_Gain @ self.H ) @ self.P_prior
            
            predict[k] =  self.H @ self.X_after
            
        return predict
    
    # 传入观测状态，返回滤波预测状态（迭代进行一次预测）
    def KalmanFliter_iterator(self, Z):
        
        # 预测：
        self.X_prior = self.F @ self.X_after
        self.P_prior = self.F @ self.P_after @ self.F.T + self.Q
            
        # 校正：
        Kal_Gain = self.P_prior @ self.H.T @ np.linalg.inv( self.H @ self.P_prior @ self.H.T + self.R )
            
        self.Z = Z
        self.X_after = self.X_prior + Kal_Gain @ ( self.Z - self.H @ self.X_prior )
            
        self.P_after = ( self.I - Kal_Gain @ self.H ) @ self.P_prior
            
        return self.H @ self.X_after
    
    def getLambda(self, Z):
        pi = 3.14
        r = Z - self.H @ self.X_after
        S = self.H @ self.P_after @ self.H.T + self.R 
        Lambda = (1/(abs(2*pi*np.linalg.det(S)))**0.5)*np.exp((-1/2)*(r.T @ np.linalg.inv(S) @ r))
        return Lambda

# # 定义目标轨迹函数
# start = 1
# stop = 30
# num = 250

# def target_trajectory(x, a, b):
#     return np.exp(a * x) * np.cos(b * x) + np.log(x)

# # 生成目标轨迹数据
# x_values = np.linspace(start, stop, num)  # 自变量范围，避免 ln(0)
# a, b = 0.1, 1  # 参数
# y_values = target_trajectory(x_values, a, b)
# true_trajectory = np.array([x_values,y_values]).T

# # Singer 模型相关参数

# T = ( stop - start ) / num * 1.05
# a = 1
# sigma = 1
# Dim = 2

# # 观测误差协方差矩阵
# R = np.array([[0.05,0.001],
#               [0.001,0.05]])
# v_means = np.zeros(2) # 观测噪声均值（零均值）

# # 生成测量数据（带噪声）

# np.random.seed(42)  # 设置随机种子以获得可重复的结果
# measurements = true_trajectory.copy()
# for k in range(len(measurements)):
#     V = np.random.multivariate_normal(v_means, R, 1)[0]
#     measurements[k] = true_trajectory[k] + V

# # 设置初始状态

# x = 1
# y = target_trajectory(x, a, b)
# v_x = v_y = a_x = a_y = 0
# X_0 = np.array([x, v_x, a_x, y, v_y, a_y]).T

# singer = Singer_KF(T,a,sigma,Dim,R)
# singer.KalmanFliter__init__(X_0)
# # predict_Singer = singer.KalmanFliter_wholeProcess(measurements)

# predict_Singer = measurements.copy()
# for k in range(len(measurements)):
#     predict_Singer[k] = singer.KalmanFliter_iterator( measurements[k] )

# # 可视化结果
# plt.figure(figsize=(12, 6))
# plt.plot(x_values, y_values, label='True Trajectory', color='blue', linestyle='--')
# plt.scatter(measurements[:,0], measurements[:,1], label='Measurements', color='red', s=10)
# plt.plot(predict_Singer[:, 0], predict_Singer[:, 1], label='Predicted Trajectory', color='green')
# plt.title('Target Trajectory Prediction using Singer Model')
# plt.xlabel('X-axis')
# plt.ylabel('Y-axis')
# plt.legend()
# plt.grid()
# plt.show()