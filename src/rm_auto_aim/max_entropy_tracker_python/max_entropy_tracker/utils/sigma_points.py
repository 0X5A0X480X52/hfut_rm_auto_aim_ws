"""
Sigma点生成器

独立的Sigma点采样工具，支持不同的UKF变体
"""

import numpy as np
from typing import Tuple
import logging

logger = logging.getLogger(__name__)


class SigmaPointGenerator:
    """
    Sigma点生成器
    
    使用Scaled Unscented Transform生成Sigma点
    支持任意维度的状态向量
    """
    
    def __init__(
        self,
        n: int,
        alpha: float = 0.001,
        beta: float = 2.0,
        kappa: float = 0.0
    ):
        """
        初始化Sigma点生成器
        
        Args:
            n: 状态维度
            alpha: 控制sigma点分布的参数 (通常很小，如0.001)
            beta: 分布先验参数 (高斯分布时为2)
            kappa: 缩放参数 (通常为0或3-n)
        """
        self.n = n
        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa
        
        # 计算lambda参数
        self.lambda_ = alpha**2 * (n + kappa) - n
        
        # 预计算权重
        self._compute_weights()
    
    def _compute_weights(self):
        """计算Sigma点权重"""
        n = self.n
        lambda_ = self.lambda_
        
        # 均值权重
        self.Wm = np.zeros(2 * n + 1)
        self.Wm[0] = lambda_ / (n + lambda_)
        self.Wm[1:] = 0.5 / (n + lambda_)
        
        # 协方差权重
        self.Wc = self.Wm.copy()
        self.Wc[0] = self.Wm[0] + (1 - self.alpha**2 + self.beta)
    
    def generate(self, x: np.ndarray, P: np.ndarray) -> np.ndarray:
        """
        生成Sigma点
        
        Args:
            x: 状态向量 (n,)
            P: 协方差矩阵 (n, n)
            
        Returns:
            Sigma点矩阵 (2n+1, n)
        """
        n = len(x)
        sigma_points = np.zeros((2 * n + 1, n))
        
        # 中心点
        sigma_points[0] = x
        
        # Cholesky分解
        # 在尝试 Cholesky 前打印 P 的基本信息以便调试数值问题
        # try:
        #     logger.warning(f"SigmaPoints: P diag min={np.min(np.diag(P)):.6e}, max={np.max(np.diag(P)):.6e}, any_nan={np.isnan(P).any()}")
        #     eigs = np.linalg.eigvals(P)
        #     logger.warning(f"SigmaPoints: P eig min={np.min(eigs):.6e}, max={np.max(eigs):.6e}")
        # except Exception:
        #     logger.warning("SigmaPoints: failed to compute P diagnostics")

        try:
            L = np.linalg.cholesky((n + self.lambda_) * P)
        except np.linalg.LinAlgError:
            # 如果分解失败，尝试逐步增加正则化项以稳定计算
            logger.warning("Cholesky decomposition failed, trying progressive regularization")
            P_reg = P.copy()
            reg_vals = [1e-6, 1e-5, 1e-4, 1e-3, 1e-2]
            L = None
            for reg in reg_vals:
                try:
                    P_reg = P + np.eye(n) * reg
                    # 打印更详细信息
                    try:
                        eigs_reg = np.linalg.eigvals(P_reg)
                        logger.debug(f"Trying reg={reg}, P_reg eig min={np.min(eigs_reg):.6e}")
                    except Exception:
                        pass
                    L = np.linalg.cholesky((n + self.lambda_) * P_reg)
                    logger.info(f"Cholesky succeeded with reg={reg}")
                    break
                except np.linalg.LinAlgError:
                    continue
            if L is None:
                # 最后仍失败，抛出异常以便上层捕获
                logger.error("Cholesky failed even after progressive regularization")
                raise

        
        # 生成对称点
        for i in range(n):
            sigma_points[i + 1] = x + L[:, i]
            sigma_points[n + i + 1] = x - L[:, i]
        
        return sigma_points
    
    def get_weights(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        获取权重
        
        Returns:
            (Wm, Wc): 均值权重和协方差权重
        """
        return self.Wm, self.Wc
    
    def unscented_transform(
        self,
        sigma_points: np.ndarray,
        noise_cov: np.ndarray = None
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Unscented变换：从Sigma点恢复均值和协方差
        
        Args:
            sigma_points: Sigma点矩阵 (2n+1, m)
            noise_cov: 可选的加性噪声协方差 (m, m)
            
        Returns:
            (mean, cov): 均值向量和协方差矩阵
        """
        # 计算均值
        mean = np.sum(self.Wm[:, np.newaxis] * sigma_points, axis=0)
        
        # 计算协方差
        diff = sigma_points - mean
        cov = np.sum(
            self.Wc[:, np.newaxis, np.newaxis] * diff[:, :, np.newaxis] * diff[:, np.newaxis, :],
            axis=0
        )
        
        # 添加噪声
        if noise_cov is not None:
            cov += noise_cov
        
        return mean, cov
    
    def cross_covariance(
        self,
        sigma_points_x: np.ndarray,
        sigma_points_z: np.ndarray,
        mean_x: np.ndarray,
        mean_z: np.ndarray
    ) -> np.ndarray:
        """
        计算交叉协方差
        
        Args:
            sigma_points_x: 状态Sigma点 (2n+1, n)
            sigma_points_z: 观测Sigma点 (2n+1, m)
            mean_x: 状态均值 (n,)
            mean_z: 观测均值 (m,)
            
        Returns:
            交叉协方差矩阵 (n, m)
        """
        diff_x = sigma_points_x - mean_x
        diff_z = sigma_points_z - mean_z
        
        Pxz = np.sum(
            self.Wc[:, np.newaxis, np.newaxis] * diff_x[:, :, np.newaxis] * diff_z[:, np.newaxis, :],
            axis=0
        )
        
        return Pxz
