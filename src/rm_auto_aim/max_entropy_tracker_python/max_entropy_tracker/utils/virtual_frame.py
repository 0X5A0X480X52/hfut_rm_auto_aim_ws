"""
虚拟坐标系工具模块

提供虚拟坐标系变换功能：
- 虚拟坐标系与相机坐标系原点重合
- 虚拟坐标系相对于 odom 只保留 yaw 旋转，消除 roll 和 pitch
- 虚拟坐标系的 Z 轴始终与 odom 的 Z 轴平行

这样可以确保 UKF 和 tracker 中假设机器人 Z 轴与坐标系 Z 轴平行的几何模型成立
"""

import numpy as np
from typing import Tuple, Optional
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3, Transform, Pose


def quaternion_to_euler(q: Quaternion) -> Tuple[float, float, float]:
    """
    从四元数提取欧拉角 (roll, pitch, yaw)
    
    使用 ZYX 欧拉角顺序（先 yaw，再 pitch，最后 roll）
    
    Args:
        q: 四元数消息
        
    Returns:
        (roll, pitch, yaw) 弧度
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    sinp = np.clip(sinp, -1.0, 1.0)  # 限制范围避免 asin 域错误
    pitch = np.arcsin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """
    从欧拉角构建四元数
    
    使用 ZYX 欧拉角顺序
    
    Args:
        roll: 绕 X 轴旋转角
        pitch: 绕 Y 轴旋转角
        yaw: 绕 Z 轴旋转角
        
    Returns:
        Quaternion 消息
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    
    return q


def yaw_only_quaternion(yaw: float) -> Quaternion:
    """
    构建只包含 yaw 旋转的四元数（roll=0, pitch=0）
    
    Args:
        yaw: yaw 角（弧度）
        
    Returns:
        Quaternion 消息
    """
    return euler_to_quaternion(0.0, 0.0, yaw)


def quaternion_multiply(q1: Quaternion, q2: Quaternion) -> Quaternion:
    """
    四元数乘法 q1 * q2
    
    Args:
        q1: 第一个四元数
        q2: 第二个四元数
        
    Returns:
        结果四元数
    """
    result = Quaternion()
    result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
    result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
    result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x
    result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
    return result


def quaternion_inverse(q: Quaternion) -> Quaternion:
    """
    四元数求逆（假设单位四元数，逆等于共轭）
    
    Args:
        q: 输入四元数
        
    Returns:
        逆四元数
    """
    result = Quaternion()
    result.w = q.w
    result.x = -q.x
    result.y = -q.y
    result.z = -q.z
    return result


def quaternion_to_rotation_matrix(q: Quaternion) -> np.ndarray:
    """
    将四元数转换为 3x3 旋转矩阵
    
    Args:
        q: 四元数消息
        
    Returns:
        3x3 旋转矩阵
    """
    w, x, y, z = q.w, q.x, q.y, q.z
    
    R = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
    ])
    
    return R


def rotation_matrix_to_quaternion(R: np.ndarray) -> Quaternion:
    """
    将 3x3 旋转矩阵转换为四元数
    
    Args:
        R: 3x3 旋转矩阵
        
    Returns:
        Quaternion 消息
    """
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    
    q = Quaternion()
    
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        q.w = 0.25 / s
        q.x = (R[2, 1] - R[1, 2]) * s
        q.y = (R[0, 2] - R[2, 0]) * s
        q.z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        q.w = (R[2, 1] - R[1, 2]) / s
        q.x = 0.25 * s
        q.y = (R[0, 1] + R[1, 0]) / s
        q.z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        q.w = (R[0, 2] - R[2, 0]) / s
        q.x = (R[0, 1] + R[1, 0]) / s
        q.y = 0.25 * s
        q.z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        q.w = (R[1, 0] - R[0, 1]) / s
        q.x = (R[0, 2] + R[2, 0]) / s
        q.y = (R[1, 2] + R[2, 1]) / s
        q.z = 0.25 * s
    
    return q


class VirtualFrameTransform:
    """
    虚拟坐标系变换器
    
    虚拟坐标系定义：
    - 原点位于 odom 坐标系原点
    - 相对于 odom 只有相机的 yaw 旋转，消除 roll 和 pitch
    - 坐标轴方向与标准相机坐标系方向一致
    - Z 轴与 odom 的 Z 轴平行（向上）
    
    变换关系：
    - odom -> camera: T_odom_camera（完整 6-DOF 变换）
    - odom -> virtual: T_odom_virtual（只含 yaw 旋转，原点在 odom）
    - virtual -> camera: T_virtual_camera（包含平移和 roll/pitch 旋转）
    
    使用场景：
    1. 将 odom 坐标系下的装甲板位姿变换到虚拟坐标系
    2. 在虚拟坐标系中运行 UKF 和几何模型（Z 轴平行假设成立）
    3. 可视化时可选发布到任意坐标系
    """
    
    def __init__(self, virtual_frame_id: str = "virtual_camera_frame"):
        """
        初始化虚拟坐标系变换器
        
        Args:
            virtual_frame_id: 虚拟坐标系的 frame_id
        """
        self.virtual_frame_id = virtual_frame_id
        
        # 缓存最近一次的变换
        self._odom_to_camera_yaw: float = 0.0
        self._odom_to_camera_translation: np.ndarray = np.zeros(3)
        self._camera_roll: float = 0.0
        self._camera_pitch: float = 0.0
        
        # 变换矩阵缓存
        self._T_odom_virtual: Optional[np.ndarray] = None  # odom -> virtual
        self._T_virtual_odom: Optional[np.ndarray] = None  # virtual -> odom
        self._T_camera_virtual: Optional[np.ndarray] = None  # camera -> virtual
        self._T_virtual_camera: Optional[np.ndarray] = None  # virtual -> camera
    
    def update_from_transform(self, odom_to_camera: TransformStamped) -> None:
        """
        从 odom->camera 的 TF 变换更新虚拟坐标系
        
        Args:
            odom_to_camera: odom 到 camera 的变换
        """
        q = odom_to_camera.transform.rotation
        t = odom_to_camera.transform.translation
        
        # 提取欧拉角
        roll, pitch, yaw = quaternion_to_euler(q)
        
        # 保存分解结果
        self._camera_roll = roll
        self._camera_pitch = pitch
        self._odom_to_camera_yaw = yaw
        self._odom_to_camera_translation = np.array([t.x, t.y, t.z])
        
        # 更新变换矩阵
        self._update_transforms()
    
    def _update_transforms(self) -> None:
        """
        更新所有变换矩阵
        """
        # 1. T_odom_virtual: odom -> virtual
        # 只含 yaw 旋转，原点在 odom，无平移
        R_yaw = self._rotation_matrix_z(self._odom_to_camera_yaw)
        self._T_odom_virtual = self._make_transform_matrix(
            R_yaw, np.zeros(3)  # 原点在 odom，平移为 0
        )
        self._T_virtual_odom = np.linalg.inv(self._T_odom_virtual)
        
        # 2. T_virtual_camera: virtual -> camera
        # 包含完整平移（从 odom 到相机）和 roll/pitch 旋转
        R_rp = self._rotation_matrix_rp(self._camera_roll, self._camera_pitch)
        # 平移需要在虚拟坐标系中表示，先逆旋转 yaw
        t_camera_in_virtual = R_yaw.T @ self._odom_to_camera_translation
        self._T_virtual_camera = self._make_transform_matrix(R_rp, t_camera_in_virtual)
        self._T_camera_virtual = np.linalg.inv(self._T_virtual_camera)
    
    @staticmethod
    def _rotation_matrix_z(yaw: float) -> np.ndarray:
        """绕 Z 轴旋转矩阵"""
        c, s = np.cos(yaw), np.sin(yaw)
        return np.array([
            [c, -s, 0],
            [s, c, 0],
            [0, 0, 1]
        ])
    
    @staticmethod
    def _rotation_matrix_rp(roll: float, pitch: float) -> np.ndarray:
        """Roll-Pitch 旋转矩阵（ZYX 顺序中的 YX 部分）"""
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        
        # R = R_y(pitch) * R_x(roll)
        R = np.array([
            [cp, sr*sp, cr*sp],
            [0, cr, -sr],
            [-sp, sr*cp, cr*cp]
        ])
        return R
    
    @staticmethod
    def _make_transform_matrix(R: np.ndarray, t: np.ndarray) -> np.ndarray:
        """构建 4x4 齐次变换矩阵"""
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t
        return T
    
    def transform_point_camera_to_virtual(self, point: np.ndarray) -> np.ndarray:
        """
        将点从相机坐标系变换到虚拟坐标系
        
        Args:
            point: 相机坐标系下的点 [x, y, z]
            
        Returns:
            虚拟坐标系下的点 [x, y, z]
        """
        if self._T_camera_virtual is None:
            return point  # 未初始化时返回原始点
        
        p_homo = np.append(point, 1.0)
        p_virtual = self._T_camera_virtual @ p_homo
        return p_virtual[:3]
    
    def transform_point_virtual_to_camera(self, point: np.ndarray) -> np.ndarray:
        """
        将点从虚拟坐标系变换到相机坐标系
        
        Args:
            point: 虚拟坐标系下的点 [x, y, z]
            
        Returns:
            相机坐标系下的点 [x, y, z]
        """
        if self._T_virtual_camera is None:
            return point
        
        p_homo = np.append(point, 1.0)
        p_camera = self._T_virtual_camera @ p_homo
        return p_camera[:3]
    
    def transform_point_odom_to_virtual(self, point: np.ndarray) -> np.ndarray:
        """
        将点从 odom 坐标系变换到虚拟坐标系
        
        Args:
            point: odom 坐标系下的点 [x, y, z]
            
        Returns:
            虚拟坐标系下的点 [x, y, z]
        """
        if self._T_virtual_odom is None:
            return point
        
        p_homo = np.append(point, 1.0)
        p_virtual = self._T_virtual_odom @ p_homo
        return p_virtual[:3]
    
    def transform_point_virtual_to_odom(self, point: np.ndarray) -> np.ndarray:
        """
        将点从虚拟坐标系变换到 odom 坐标系
        
        Args:
            point: 虚拟坐标系下的点 [x, y, z]
            
        Returns:
            odom 坐标系下的点 [x, y, z]
        """
        if self._T_odom_virtual is None:
            return point
        
        p_homo = np.append(point, 1.0)
        p_odom = self._T_odom_virtual @ p_homo
        return p_odom[:3]
    
    def transform_yaw_camera_to_virtual(self, yaw_camera: float) -> float:
        """
        将 yaw 角从相机坐标系变换到虚拟坐标系
        
        由于虚拟坐标系消除了 roll 和 pitch，yaw 需要补偿
        
        简化假设：当 roll/pitch 较小时，yaw 变化可近似忽略
        
        Args:
            yaw_camera: 相机坐标系下的 yaw 角
            
        Returns:
            虚拟坐标系下的 yaw 角
        """
        # 对于小角度，yaw 变换近似不变
        # 完整变换需要将方向向量变换后重新计算 yaw
        return yaw_camera
    
    def transform_pose_odom_to_virtual(
        self, 
        position: np.ndarray, 
        yaw: float
    ) -> Tuple[np.ndarray, float]:
        """
        将位姿从 odom 坐标系变换到虚拟坐标系
        
        Args:
            position: odom 坐标系下的位置 [x, y, z]
            yaw: odom 坐标系下的 yaw 角
            
        Returns:
            (虚拟坐标系下的位置, 虚拟坐标系下的 yaw)
        """
        pos_virtual = self.transform_point_odom_to_virtual(position)
        # yaw 需要减去 odom->virtual 的 yaw（即相机的 yaw）
        yaw_virtual = yaw - self._odom_to_camera_yaw
        return pos_virtual, yaw_virtual
    
    def transform_pose_virtual_to_odom(
        self, 
        position: np.ndarray, 
        yaw: float
    ) -> Tuple[np.ndarray, float]:
        """
        将位姿从虚拟坐标系变换到 odom 坐标系
        
        Args:
            position: 虚拟坐标系下的位置 [x, y, z]
            yaw: 虚拟坐标系下的 yaw 角
            
        Returns:
            (odom 坐标系下的位置, odom 坐标系下的 yaw)
        """
        pos_odom = self.transform_point_virtual_to_odom(position)
        # yaw 需要加上 odom->virtual 的 yaw
        yaw_odom = yaw + self._odom_to_camera_yaw
        return pos_odom, yaw_odom
    
    def get_virtual_frame_transform(self, parent_frame: str, stamp) -> TransformStamped:
        """
        获取虚拟坐标系相对于父坐标系的 TF 变换
        
        Args:
            parent_frame: 父坐标系（通常是 odom）
            stamp: 时间戳
            
        Returns:
            TransformStamped 消息
        """
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = parent_frame
        tf_msg.child_frame_id = self.virtual_frame_id
        
        # 原点在 odom，平移为 0
        tf_msg.transform.translation.x = 0.0
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 0.0
        
        # 只有 yaw 旋转
        tf_msg.transform.rotation = yaw_only_quaternion(self._odom_to_camera_yaw)
        
        return tf_msg
    
    @property
    def camera_roll(self) -> float:
        """相机的 roll 角"""
        return self._camera_roll
    
    @property
    def camera_pitch(self) -> float:
        """相机的 pitch 角"""
        return self._camera_pitch
    
    @property
    def camera_yaw(self) -> float:
        """相机的 yaw 角（相对于 odom）"""
        return self._odom_to_camera_yaw
    
    @property
    def is_initialized(self) -> bool:
        """是否已初始化"""
        return self._T_odom_virtual is not None
