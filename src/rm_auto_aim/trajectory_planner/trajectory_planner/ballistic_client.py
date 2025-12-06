# Copyright (C) FYT Vision Group. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Ballistic Client Module

Provides a service client for the ballistic_solver node to calculate
pitch angle given a target position and velocity.
"""

import math
from dataclasses import dataclass
from typing import Optional, Tuple
import threading

import rclpy
from rclpy.node import Node
from rm_interfaces.srv import SolveBallistic
from geometry_msgs.msg import Point, Vector3


@dataclass
class BallisticResult:
    """弹道解算结果"""
    
    pitch: float = 0.0          # 云台pitch角 (弧度)
    yaw: float = 0.0            # 云台yaw角 (弧度)
    flight_time: float = 0.0    # 飞行时间 (秒)
    success: bool = False       # 是否成功
    message: str = ""           # 消息


@dataclass
class BallisticClientConfig:
    """弹道解算客户端配置"""
    
    # 服务名称
    service_name: str = "/ballistic_solver/solve"
    
    # 服务调用超时时间 (秒)
    timeout: float = 0.5
    
    # 默认子弹速度 (m/s)
    default_bullet_speed: float = 28.0


class BallisticClient:
    """
    弹道解算服务客户端
    
    功能:
    1. 异步调用 ballistic_solver 服务
    2. 给定目标3D位置和速度，获取pitch和yaw角
    3. 支持同步和异步调用方式
    """
    
    def __init__(self, 
                 node: Node,
                 config: Optional[BallisticClientConfig] = None):
        """
        初始化弹道解算客户端
        
        Args:
            node: ROS2节点引用
            config: 客户端配置
        """
        self.node = node
        self.config = config if config else BallisticClientConfig()
        
        # 创建服务客户端
        self.client = self.node.create_client(
            SolveBallistic,
            self.config.service_name
        )
        
        # 缓存的结果
        self._last_result: Optional[BallisticResult] = None
        
        # 线程锁
        self._lock = threading.Lock()
        
        # 服务可用状态
        self._service_available = False
    
    def wait_for_service(self, timeout_sec: float = 5.0) -> bool:
        """
        等待服务可用
        
        Args:
            timeout_sec: 超时时间 (秒)
        
        Returns:
            服务是否可用
        """
        self._service_available = self.client.wait_for_service(timeout_sec=timeout_sec)
        return self._service_available
    
    def is_service_available(self) -> bool:
        """检查服务是否可用"""
        return self.client.service_is_ready()
    
    def solve_sync(self,
                   target_position: Tuple[float, float, float],
                   target_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                   bullet_speed: Optional[float] = None) -> BallisticResult:
        """
        同步调用弹道解算服务
        
        Args:
            target_position: 目标3D位置 (x, y, z) 米
            target_velocity: 目标3D速度 (vx, vy, vz) 米/秒
            bullet_speed: 子弹速度，None则使用默认值
        
        Returns:
            弹道解算结果
        """
        if not self.is_service_available():
            return BallisticResult(
                success=False,
                message="Ballistic solver service not available"
            )
        
        # 构建请求
        request = SolveBallistic.Request()
        request.target_position = Point(
            x=target_position[0],
            y=target_position[1],
            z=target_position[2]
        )
        request.target_velocity = Vector3(
            x=target_velocity[0],
            y=target_velocity[1],
            z=target_velocity[2]
        )
        request.bullet_speed = bullet_speed if bullet_speed else self.config.default_bullet_speed
        
        # 同步调用
        try:
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(
                self.node,
                future,
                timeout_sec=self.config.timeout
            )
            
            if future.done():
                response = future.result()
                result = BallisticResult(
                    pitch=response.pitch,
                    yaw=response.yaw,
                    flight_time=response.flight_time,
                    success=response.success,
                    message=response.message
                )
                
                with self._lock:
                    self._last_result = result
                
                return result
            else:
                return BallisticResult(
                    success=False,
                    message="Service call timed out"
                )
                
        except Exception as e:
            return BallisticResult(
                success=False,
                message=f"Service call failed: {str(e)}"
            )
    
    def solve_async(self,
                    target_position: Tuple[float, float, float],
                    target_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                    bullet_speed: Optional[float] = None,
                    callback=None):
        """
        异步调用弹道解算服务
        
        Args:
            target_position: 目标3D位置
            target_velocity: 目标3D速度
            bullet_speed: 子弹速度
            callback: 完成回调函数，签名: callback(result: BallisticResult)
        
        Returns:
            Future对象
        """
        if not self.is_service_available():
            if callback:
                callback(BallisticResult(
                    success=False,
                    message="Ballistic solver service not available"
                ))
            return None
        
        # 构建请求
        request = SolveBallistic.Request()
        request.target_position = Point(
            x=target_position[0],
            y=target_position[1],
            z=target_position[2]
        )
        request.target_velocity = Vector3(
            x=target_velocity[0],
            y=target_velocity[1],
            z=target_velocity[2]
        )
        request.bullet_speed = bullet_speed if bullet_speed else self.config.default_bullet_speed
        
        future = self.client.call_async(request)
        
        if callback:
            def done_callback(fut):
                try:
                    response = fut.result()
                    result = BallisticResult(
                        pitch=response.pitch,
                        yaw=response.yaw,
                        flight_time=response.flight_time,
                        success=response.success,
                        message=response.message
                    )
                except Exception as e:
                    result = BallisticResult(
                        success=False,
                        message=f"Service call failed: {str(e)}"
                    )
                
                with self._lock:
                    self._last_result = result
                
                callback(result)
            
            future.add_done_callback(done_callback)
        
        return future
    
    def compute_pitch_from_yaw(self,
                               yaw: float,
                               target_position: Tuple[float, float, float],
                               target_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                               bullet_speed: Optional[float] = None) -> BallisticResult:
        """
        根据给定的yaw角计算对应的pitch
        
        这个方法将目标投影到yaw方向上，然后计算垂直方向的弹道
        
        Args:
            yaw: 给定的yaw角 (弧度)
            target_position: 目标3D位置
            target_velocity: 目标3D速度
            bullet_speed: 子弹速度
        
        Returns:
            弹道解算结果 (pitch角)
        """
        # 计算目标在yaw方向上的投影距离
        x, y, z = target_position
        horizontal_distance = math.sqrt(x**2 + y**2)
        
        # 将目标投影到yaw方向
        projected_x = horizontal_distance * math.cos(yaw)
        projected_y = horizontal_distance * math.sin(yaw)
        
        # 投影速度
        vx, vy, vz = target_velocity
        # 速度在yaw方向上的分量
        v_horizontal = math.sqrt(vx**2 + vy**2)
        target_yaw = math.atan2(y, x)
        yaw_diff = yaw - target_yaw
        
        projected_vx = v_horizontal * math.cos(yaw)
        projected_vy = v_horizontal * math.sin(yaw)
        
        return self.solve_sync(
            target_position=(projected_x, projected_y, z),
            target_velocity=(projected_vx, projected_vy, vz),
            bullet_speed=bullet_speed
        )
    
    def simple_pitch_estimate(self,
                              distance: float,
                              height: float,
                              bullet_speed: Optional[float] = None) -> float:
        """
        简单的pitch角估算 (不考虑空气阻力)
        
        使用抛物线弹道公式的近似解
        
        Args:
            distance: 水平距离 (米)
            height: 垂直高度差 (米，正值表示目标在上方)
            bullet_speed: 子弹速度
        
        Returns:
            估算的pitch角 (弧度)
        """
        v = bullet_speed if bullet_speed else self.config.default_bullet_speed
        g = 9.8  # 重力加速度
        
        # 检查无效输入
        if distance <= 0.01 or v <= 0:  # 最小距离1cm
            # 距离太近，直接返回基于高度的角度
            if abs(height) < 0.01:
                return 0.0
            # 简单地指向目标高度方向
            return math.atan2(height, max(distance, 0.01))
        
        # 飞行时间近似
        t = distance / v
        
        # 考虑重力补偿的pitch角
        # 目标高度 = v * sin(pitch) * t - 0.5 * g * t^2
        # sin_pitch = (height + 0.5 * g * t^2) / (v * t)
        denominator = v * t
        if abs(denominator) < 1e-6:
            return math.atan2(height, distance)
        
        sin_pitch = (height + 0.5 * g * t**2) / denominator
        
        # 限制范围
        sin_pitch = max(-1.0, min(1.0, sin_pitch))
        
        return math.asin(sin_pitch)
    
    @property
    def last_result(self) -> Optional[BallisticResult]:
        """获取最后一次解算结果"""
        with self._lock:
            return self._last_result
