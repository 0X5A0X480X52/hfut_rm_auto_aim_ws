"""
CSV 日志记录模块

记录观测和估计结果到 CSV 文件，用于离线分析和调试
"""

import csv
import os
from typing import Optional
from datetime import datetime
import logging

from .core.observation import ObservationData
from .trackers.adaptive_armor_tracker import AdaptiveArmorTracker

logger = logging.getLogger(__name__)


class CSVLogger:
    """
    CSV 日志记录器
    
    记录每次观测和对应的估计结果
    """
    
    # CSV 列定义
    COLUMNS = [
        'timestamp',           # 时间戳（秒）
        'robot_id',            # 机器人ID
        # 原始观测值（相机坐标系）
        'raw_obs_x',           # 原始观测位置 X（相机坐标系）
        'raw_obs_y',           # 原始观测位置 Y（相机坐标系）
        'raw_obs_z',           # 原始观测位置 Z（相机坐标系）
        'raw_obs_yaw',         # 原始观测 Yaw（相机坐标系）
        # 观测值（世界坐标系）
        'obs_x',               # 观测位置 X
        'obs_y',               # 观测位置 Y
        'obs_z',               # 观测位置 Z
        'obs_yaw',             # 观测 Yaw
        # 估计值
        'est_x',               # 估计中心位置 X
        'est_y',               # 估计中心位置 Y
        'est_z',               # 估计中心位置 Z
        'est_yaw',             # 估计 Yaw
        'est_vx',              # 估计速度 X
        'est_vy',              # 估计速度 Y
        'est_vz',              # 估计速度 Z
        'est_v_yaw',           # 估计角速度
        'est_r1',              # 估计半径1
        'est_r2',              # 估计半径2
        'est_dza',             # 估计高度差
        # 跟踪状态
        'tracker_state',       # 跟踪器状态
        'panel_id',            # 当前面板ID
        'height_label',        # 高度标签
    ]
    
    def __init__(self, log_path: str):
        """
        初始化日志记录器
        
        Args:
            log_path: CSV 文件路径
        """
        self.log_path = log_path
        self._file = None
        self._writer = None
        self._row_count = 0
        
        self._open_file()
    
    def _open_file(self):
        """打开文件并写入表头"""
        try:
            # 确保目录存在
            log_dir = os.path.dirname(self.log_path)
            if log_dir and not os.path.exists(log_dir):
                os.makedirs(log_dir)
            
            # 生成带时间戳的文件名
            if os.path.exists(self.log_path):
                # 如果文件已存在，添加时间戳
                base, ext = os.path.splitext(self.log_path)
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                self.log_path = f"{base}_{timestamp}{ext}"
            
            self._file = open(self.log_path, 'w', newline='')
            self._writer = csv.DictWriter(self._file, fieldnames=self.COLUMNS)
            self._writer.writeheader()
            
            logger.info(f"CSV logger opened: {self.log_path}")
            
        except Exception as e:
            logger.error(f"Failed to open CSV log file: {e}")
            self._file = None
            self._writer = None
    
    def log(
        self,
        timestamp: float,
        robot_id: str,
        raw_observation: ObservationData,
        observation: ObservationData,
        tracker: AdaptiveArmorTracker
    ):
        """
        记录一条观测和估计结果
        
        Args:
            timestamp: 时间戳（秒）
            robot_id: 机器人ID
            raw_observation: 原始观测数据（相机坐标系）
            observation: 变换后的观测数据（世界坐标系）
            tracker: 跟踪器实例
        """
        if self._writer is None:
            return
        
        try:
            pos = tracker.get_center_position()
            r1, r2 = tracker.get_radii()
            state = tracker.get_state()
            
            row = {
                'timestamp': f"{timestamp:.6f}",
                'robot_id': robot_id,
                # 原始观测值（相机坐标系）
                'raw_obs_x': f"{raw_observation.x:.6f}",
                'raw_obs_y': f"{raw_observation.y:.6f}",
                'raw_obs_z': f"{raw_observation.z:.6f}",
                'raw_obs_yaw': f"{raw_observation.yaw:.6f}",
                # 观测值（世界坐标系）
                'obs_x': f"{observation.x:.6f}",
                'obs_y': f"{observation.y:.6f}",
                'obs_z': f"{observation.z:.6f}",
                'obs_yaw': f"{observation.yaw:.6f}",
                # 估计值
                'est_x': f"{pos[0]:.6f}",
                'est_y': f"{pos[1]:.6f}",
                'est_z': f"{pos[2]:.6f}",
                'est_yaw': f"{tracker.get_yaw():.6f}",
                'est_vx': f"{state.get('vx', 0.0):.6f}",
                'est_vy': f"{state.get('vy', 0.0):.6f}",
                'est_vz': f"{state.get('vz', 0.0):.6f}",
                'est_v_yaw': f"{state.get('delta_rate', 0.0):.6f}",
                'est_r1': f"{r1:.6f}",
                'est_r2': f"{r2:.6f}",
                'est_dza': f"{tracker.get_dza():.6f}",
                # 跟踪状态
                'tracker_state': state.get('tracker_state', 'UNKNOWN'),
                'panel_id': str(tracker.get_panel_id()),
                'height_label': tracker.get_height_label().name,
            }
            
            self._writer.writerow(row)
            self._row_count += 1
            
            # 每100行刷新一次
            if self._row_count % 100 == 0:
                self._file.flush()
                
        except Exception as e:
            logger.error(f"Failed to write CSV log: {e}")
    
    def flush(self):
        """刷新文件缓冲"""
        if self._file is not None:
            try:
                self._file.flush()
            except Exception as e:
                logger.error(f"Failed to flush CSV log: {e}")
    
    def close(self):
        """关闭文件"""
        if self._file is not None:
            try:
                self._file.close()
                logger.info(f"CSV logger closed: {self.log_path} ({self._row_count} rows)")
            except Exception as e:
                logger.error(f"Failed to close CSV log: {e}")
            finally:
                self._file = None
                self._writer = None
    
    @property
    def row_count(self) -> int:
        """已写入的行数"""
        return self._row_count
    
    def __del__(self):
        """析构时关闭文件"""
        self.close()
