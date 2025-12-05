import numpy as np
from models.CTRV_EKF import CTRV_EKF
from models.CA_KF import CA_KF

class CA_CTRV_EKF:
    def __init__(self, T, R_ctrv, R_ca):
        self.T = T
        self.ctrv = CTRV_EKF(T, R_ctrv)
        self.ca = CA_KF(T, 1, R_ca)
        self.Dim = 8  # 总状态维度

        self.X_after = np.zeros(self.Dim)
        self.P_after = np.eye(self.Dim)
        self.split_function = self.default_split_function

    def KalmanFliter__init__(self, X_0):
        X_0_ctrv = X_0[:5]
        X_0_ca = X_0[5:]
        self.ctrv.KalmanFliter__init__(X_0_ctrv)
        self.ca.KalmanFliter__init__(X_0_ca)
        self.X_after[:5] = self.ctrv.X_after
        self.X_after[5:] = self.ca.X_after
        self.P_after[:5, :5] = self.ctrv.P_after
        self.P_after[5:, 5:] = self.ca.P_after[:3, :3]  # 修复形状问题

    def KalmanFliter_wholeProcess(self, measurements):
        predict = []
        for Z in measurements:
            X_pred = self.KalmanFliter_iterator(Z)
            predict.append(X_pred)
        return np.array(predict)

    def KalmanFliter_iterator(self, Z):
        Z_ctrv, Z_ca = self.split_function(Z)

        # 提取上一步的状态和协方差
        X_prior = self.X_after
        P_prior = self.P_after

        # 分别进行CTRV和CA模型的预测和更新
        pred_ctrv = self.ctrv.KalmanFliter_iterator(Z_ctrv)
        pred_ca = self.ca.KalmanFliter_iterator(Z_ca)

        # 合并预测结果
        self.X_after[:5] = self.ctrv.X_after
        self.X_after[5:] = self.ca.X_after

        # 合并协方差矩阵
        self.P_after[:5, :5] = self.ctrv.P_after
        self.P_after[5:, 5:] = self.ca.P_after[:3, :3]  # 修复形状问题

        predict = np.concatenate([pred_ctrv, pred_ca])
        
        # 返回位置预测值
        return predict

    def getLambda(self, Z):
        Z_ctrv, Z_ca = self.split_function(Z)
        Lambda_ctrv = self.ctrv.getLambda(Z_ctrv)
        Lambda_ca = self.ca.getLambda(Z_ca)
        return Lambda_ctrv * Lambda_ca

    def default_split_function(self, Z):
        Z_ctrv = Z[:2]  # 取前两个元素作为CTRV模型的观测值
        Z_ca = Z[2:]   # 取后一个元素作为CA模型的观测值
        return Z_ctrv, Z_ca

    def set_split_function(self, split_function):
        self.split_function = split_function
