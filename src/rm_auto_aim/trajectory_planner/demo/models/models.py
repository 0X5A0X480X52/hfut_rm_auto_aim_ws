import numpy as np
from abc import ABC, abstractmethod

class models(ABC):

    def __init__(self) -> None:
        self.Dim = None  # 维度
        ## 定义 Singer 模型的建模噪声：

        self.Q = np.array([])  # 最终得到的建模噪声协方差矩阵
        self.w_means = np.array([])  # 建模噪声的均值（零均值模型）

        ## 定义 Singer 模型的状态转移矩阵：

        self.F = np.array([])  # 最终得到的状态转移矩阵

        ## 定义观测模型（仅含观测位置）

        self.H = np.array([])  # 观测转移矩阵

        # 观测误差协方差矩阵
        self.R = np.array([])
        self.v_means = np.array([])  # 观测噪声均值（零均值）
        
        self.X_after = np.array([])  # 滤波后的状态
        self.P_after = np.array([])  # 滤波后的协方差矩阵
        
        # 设置和获取滤波后的状态和协方差矩阵的默认函数
        self.set_X_after = self.defualt_set_X_after
        self.set_P_after = self.defualt_set_P_after
        
        self.get_X_after = self.defualt_get_X_after
        self.get_P_after = self.defualt_get_P_after
        
        self.set_Z = self.defualt_set_Z  # 设置观测值的默认函数

        
    # 设置滤波器的初始状态
    @abstractmethod
    def KalmanFliter__init__(self, X_0):
        pass
    
    # 传入观测序列，返回滤波预测序列（全过程预测）
    @abstractmethod
    def KalmanFliter_wholeProcess(self, measurements):
        pass
    
    # 传入观测状态，返回滤波预测状态（迭代进行一次预测）
    @abstractmethod
    def KalmanFliter_iterator(self, Z):
        pass
    
    # 计算似然函数
    def getLambda(self, Z):
        Z = self.set_Z(Z)
        pi = 3.14
        r = Z - self.H @ self.X_after  # 残差
        S = self.H @ self.P_after @ self.H.T + self.R  # 残差协方差
        Lambda = (1/(abs(2*pi*np.linalg.det(S)))**0.5)*np.exp((-1/2)*(r.T @ np.linalg.inv(S) @ r))  # 似然函数
        return Lambda
    
    # 默认设置滤波后的状态
    def defualt_set_X_after(self, X_after):
        self.X_after = X_after
    
    # 默认设置滤波后的协方差矩阵
    def defualt_set_P_after(self, P_after):
        self.P_after = P_after
    
    # 设置自定义的设置滤波后状态的函数
    def set_set_X_after_function(self, set_X_after):
        self.set_X_after = set_X_after
    
    # 设置自定义的设置滤波后协方差矩阵的���数
    def set_set_P_after_function(self, set_P_after):
        self.set_P_after = set_P_after
        
    # 默认获取滤波后的状态
    def defualt_get_X_after(self):
        return self.X_after
    
    # 默认获取滤波后的协方差矩阵
    def defualt_get_P_after(self,):
        return self.P_after
    
    # 设置自定义的获取滤波后状态的函数
    def set_get_X_after_function(self, get_X_after):
        self.get_X_after = get_X_after
    
    # 设置自定义的获取滤波后协方差矩阵的函数
    def set_get_P_after_function(self, get_P_after):
        self.get_P_after = get_P_after
        
    # 默认设置观测值
    def defualt_set_Z(self, Z):
        return Z
    
    # 设置自定义的设置观测值的函数
    def set_set_Z_function(self, set_Z):
        self.set_Z = set_Z