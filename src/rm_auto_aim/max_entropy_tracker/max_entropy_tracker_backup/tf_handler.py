"""
TF 坐标变换处理模块

提供 TF2 坐标变换功能，将装甲板从相机坐标系变换到世界坐标系
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, TransformStamped
from rm_interfaces.msg import Armor
import tf2_ros
import tf2_geometry_msgs
from typing import Optional, Tuple
import logging

logger = logging.getLogger(__name__)


class TFHandler:
    """
    TF 变换处理器
    
    管理 TF2 Buffer 和 Listener，提供坐标变换功能
    """
    
    def __init__(self, node: Node, target_frame: str = 'odom'):
        """
        初始化 TF 处理器
        
        Args:
            node: ROS2 节点实例
            target_frame: 目标坐标系（世界坐标系）
        """
        self.node = node
        self.target_frame = target_frame
        
        # 创建 TF2 Buffer 和 Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)
        
        # 变换查找超时时间
        self.lookup_timeout = Duration(seconds=0.1)
        
        logger.info(f"TFHandler initialized with target_frame: {target_frame}")
    
    def transform_armor(
        self,
        armor: Armor,
        source_frame: str,
        timestamp: Time
    ) -> Optional[PoseStamped]:
        """
        将装甲板位姿变换到目标坐标系
        
        Args:
            armor: Armor 消息
            source_frame: 源坐标系（通常是相机坐标系）
            timestamp: 消息时间戳
            
        Returns:
            变换后的 PoseStamped，失败返回 None
        """
        # 构造 PoseStamped
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = source_frame
        pose_stamped.header.stamp = timestamp.to_msg()
        pose_stamped.pose = armor.pose
        
        return self.transform_pose(pose_stamped)
    
    def transform_pose(self, pose_stamped: PoseStamped) -> Optional[PoseStamped]:
        """
        将 PoseStamped 变换到目标坐标系
        
        Args:
            pose_stamped: 输入位姿
            
        Returns:
            变换后的 PoseStamped，失败返回 None
        """
        source_frame = pose_stamped.header.frame_id
        timestamp = Time.from_msg(pose_stamped.header.stamp)
        
        try:
            # 尝试使用消息时间戳查找变换
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                timestamp,
                timeout=self.lookup_timeout
            )
        except tf2_ros.LookupException:
            logger.warning(f"TF lookup failed: {source_frame} -> {self.target_frame}")
            return self._try_latest_transform(pose_stamped, source_frame)
        except tf2_ros.ConnectivityException:
            logger.warning(f"TF connectivity error: {source_frame} -> {self.target_frame}")
            return self._try_latest_transform(pose_stamped, source_frame)
        except tf2_ros.ExtrapolationException:
            logger.debug(f"TF extrapolation, using latest: {source_frame} -> {self.target_frame}")
            return self._try_latest_transform(pose_stamped, source_frame)
        
        try:
            transformed = tf2_geometry_msgs.do_transform_pose_stamped(pose_stamped, transform)
            return transformed
        except Exception as e:
            logger.error(f"Transform execution failed: {e}")
            return None
    
    def _try_latest_transform(
        self,
        pose_stamped: PoseStamped,
        source_frame: str
    ) -> Optional[PoseStamped]:
        """
        使用最新可用的变换
        
        Args:
            pose_stamped: 输入位姿
            source_frame: 源坐标系
            
        Returns:
            变换后的 PoseStamped，失败返回 None
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                Time(),  # 使用最新时间
                timeout=self.lookup_timeout
            )
            transformed = tf2_geometry_msgs.do_transform_pose_stamped(pose_stamped, transform)
            return transformed
        except Exception as e:
            logger.warning(f"Latest transform also failed: {e}")
            return None
    
    def can_transform(self, source_frame: str) -> bool:
        """
        检查是否可以进行变换
        
        Args:
            source_frame: 源坐标系
            
        Returns:
            是否可以变换
        """
        return self.tf_buffer.can_transform(
            self.target_frame,
            source_frame,
            Time(),
            timeout=Duration(seconds=0.01)
        )
    
    def get_transform(
        self,
        source_frame: str,
        timestamp: Optional[Time] = None
    ) -> Optional[TransformStamped]:
        """
        获取变换
        
        Args:
            source_frame: 源坐标系
            timestamp: 时间戳，None 表示最新
            
        Returns:
            TransformStamped，失败返回 None
        """
        if timestamp is None:
            timestamp = Time()
        
        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                timestamp,
                timeout=self.lookup_timeout
            )
        except Exception as e:
            logger.debug(f"Get transform failed: {e}")
            return None
