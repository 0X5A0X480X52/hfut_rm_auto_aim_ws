#!/usr/bin/env python3
"""
3D 复杂轨迹数据生成器

模拟场景：
1. 机器人在 3D 空间中进行变加速度机动
2. 机器人上有 4 个特征点（前后左右），相对机器人中心固定
3. 4 个特征点绕机器人轴心进行变加速度旋转
4. 相机也在进行变加速度机动
5. 只有在相机视场角内且在有效深度范围内的特征才能被识别

输出：CSV 格式的 3D 检测数据，可供 point_tracker_test_3d 使用
"""

import numpy as np
import csv
import argparse
import os
from dataclasses import dataclass
from typing import List, Tuple, Optional
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


@dataclass
class CameraState3D:
    """3D 相机状态"""
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    ax: float
    ay: float
    az: float
    # 相机朝向（球坐标）
    theta: float  # 水平角度（弧度）
    phi: float    # 俯仰角度（弧度）
    fov_h: float  # 水平视场角
    fov_v: float  # 垂直视场角
    view_distance_min: float  # 最小视距
    view_distance_max: float  # 最大视距


@dataclass
class RobotState3D:
    """3D 机器人状态"""
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    ax: float
    ay: float
    az: float
    rotation_angle: float    # 绕 z 轴旋转角度
    rotation_speed: float    # 旋转角速度
    rotation_accel: float    # 旋转角加速度


@dataclass
class Detection3D:
    """3D 检测结果"""
    frame_id: int
    feature_id: int
    x: float
    y: float
    z: float
    confidence: float


class ComplexTrajectoryGenerator3D:
    """3D 复杂轨迹生成器"""
    
    def __init__(self,
                 num_frames: int = 500,
                 dt: float = 0.01,
                 feature_radius: float = 0.5,  # 特征点到机器人中心的距离（米）
                 noise_std: float = 0.02,      # 测量噪声标准差（米）
                 detection_prob: float = 0.95,
                 seed: int = 42):
        """
        初始化 3D 生成器
        
        Args:
            num_frames: 总帧数
            dt: 时间步长（秒）
            feature_radius: 特征点到机器人中心的距离（米）
            noise_std: 测量噪声标准差（米）
            detection_prob: 检测概率
            seed: 随机种子
        """
        self.num_frames = num_frames
        self.dt = dt
        self.feature_radius = feature_radius
        self.noise_std = noise_std
        self.detection_prob = detection_prob
        
        np.random.seed(seed)
        
        # 空间范围（米）
        self.space_min = np.array([-5.0, -5.0, 0.0])
        self.space_max = np.array([5.0, 5.0, 3.0])
        
        # 初始化状态
        self.camera_states: List[CameraState3D] = []
        self.robot_states: List[RobotState3D] = []
        self.detections: List[List[Detection3D]] = []
        self.ground_truth: List[List[Tuple[int, float, float, float]]] = []
        
    def _generate_variable_acceleration(self,
                                         base_accel: float,
                                         freq: float,
                                         phase: float,
                                         t: float) -> float:
        """生成变加速度"""
        return base_accel * np.sin(2 * np.pi * freq * t + phase)
    
    def generate_camera_trajectory(self):
        """生成相机 3D 轨迹 - 变加速度机动"""
        # 初始状态
        camera = CameraState3D(
            x=0.0, y=-3.0, z=1.5,
            vx=0.0, vy=0.0, vz=0.0,
            ax=0.0, ay=0.0, az=0.0,
            theta=np.pi/2,   # 朝向 +y
            phi=0.0,         # 水平
            fov_h=np.radians(90),
            fov_v=np.radians(60),
            view_distance_min=0.5,
            view_distance_max=10.0
        )
        
        # 加速度参数
        accel_base_x = 1.0
        accel_base_y = 0.8
        accel_base_z = 0.3
        freq_x = 0.05
        freq_y = 0.07
        freq_z = 0.03
        
        for i in range(self.num_frames):
            t = i * self.dt
            
            # 变加速度
            camera.ax = self._generate_variable_acceleration(accel_base_x, freq_x, 0, t)
            camera.ay = self._generate_variable_acceleration(accel_base_y, freq_y, np.pi/4, t)
            camera.az = self._generate_variable_acceleration(accel_base_z, freq_z, np.pi/3, t)
            
            # 更新速度和位置
            camera.vx += camera.ax * self.dt
            camera.vy += camera.ay * self.dt
            camera.vz += camera.az * self.dt
            
            # 限制速度
            max_speed = 3.0
            speed = np.sqrt(camera.vx**2 + camera.vy**2 + camera.vz**2)
            if speed > max_speed:
                factor = max_speed / speed
                camera.vx *= factor
                camera.vy *= factor
                camera.vz *= factor
            
            camera.x += camera.vx * self.dt
            camera.y += camera.vy * self.dt
            camera.z += camera.vz * self.dt
            
            # 边界限制
            camera.x = np.clip(camera.x, self.space_min[0], self.space_max[0])
            camera.y = np.clip(camera.y, self.space_min[1], self.space_max[1])
            camera.z = np.clip(camera.z, 0.5, self.space_max[2])  # 相机不能太低
            
            # 相机朝向缓慢变化
            camera.theta += 0.01 * np.sin(0.1 * t)
            camera.phi += 0.005 * np.sin(0.08 * t)
            camera.phi = np.clip(camera.phi, -np.pi/4, np.pi/4)  # 限制俯仰
            
            self.camera_states.append(CameraState3D(
                x=camera.x, y=camera.y, z=camera.z,
                vx=camera.vx, vy=camera.vy, vz=camera.vz,
                ax=camera.ax, ay=camera.ay, az=camera.az,
                theta=camera.theta, phi=camera.phi,
                fov_h=camera.fov_h, fov_v=camera.fov_v,
                view_distance_min=camera.view_distance_min,
                view_distance_max=camera.view_distance_max
            ))
    
    def generate_robot_trajectory(self):
        """生成机器人 3D 轨迹 - 变加速度机动 + 特征旋转"""
        # 初始状态
        robot = RobotState3D(
            x=0.0, y=2.0, z=1.0,
            vx=0.5, vy=0.3, vz=0.1,
            ax=0.0, ay=0.0, az=0.0,
            rotation_angle=0.0,
            rotation_speed=1.0,  # rad/s
            rotation_accel=0.0
        )
        
        # 机器人加速度参数
        accel_base_x = 1.5
        accel_base_y = 1.2
        accel_base_z = 0.5
        freq_x = 0.03
        freq_y = 0.04
        freq_z = 0.02
        
        # 旋转加速度参数
        rot_accel_base = 2.0
        rot_freq = 0.08
        
        for i in range(self.num_frames):
            t = i * self.dt
            
            # 变加速度
            robot.ax = self._generate_variable_acceleration(accel_base_x, freq_x, np.pi/3, t)
            robot.ay = self._generate_variable_acceleration(accel_base_y, freq_y, np.pi/6, t)
            robot.az = self._generate_variable_acceleration(accel_base_z, freq_z, np.pi/2, t)
            
            # 更新速度和位置
            robot.vx += robot.ax * self.dt
            robot.vy += robot.ay * self.dt
            robot.vz += robot.az * self.dt
            
            # 限制速度
            max_speed = 4.0
            speed = np.sqrt(robot.vx**2 + robot.vy**2 + robot.vz**2)
            if speed > max_speed:
                factor = max_speed / speed
                robot.vx *= factor
                robot.vy *= factor
                robot.vz *= factor
            
            robot.x += robot.vx * self.dt
            robot.y += robot.vy * self.dt
            robot.z += robot.vz * self.dt
            
            # 边界反弹
            if robot.x < self.space_min[0] or robot.x > self.space_max[0]:
                robot.vx *= -0.8
                robot.x = np.clip(robot.x, self.space_min[0], self.space_max[0])
            if robot.y < self.space_min[1] or robot.y > self.space_max[1]:
                robot.vy *= -0.8
                robot.y = np.clip(robot.y, self.space_min[1], self.space_max[1])
            if robot.z < self.space_min[2] or robot.z > self.space_max[2]:
                robot.vz *= -0.8
                robot.z = np.clip(robot.z, self.space_min[2], self.space_max[2])
            
            # 旋转变加速度
            robot.rotation_accel = self._generate_variable_acceleration(rot_accel_base, rot_freq, 0, t)
            robot.rotation_speed += robot.rotation_accel * self.dt
            robot.rotation_speed = np.clip(robot.rotation_speed, -5.0, 5.0)
            robot.rotation_angle += robot.rotation_speed * self.dt
            
            self.robot_states.append(RobotState3D(
                x=robot.x, y=robot.y, z=robot.z,
                vx=robot.vx, vy=robot.vy, vz=robot.vz,
                ax=robot.ax, ay=robot.ay, az=robot.az,
                rotation_angle=robot.rotation_angle,
                rotation_speed=robot.rotation_speed,
                rotation_accel=robot.rotation_accel
            ))
    
    def get_feature_positions(self, robot: RobotState3D) -> List[Tuple[int, float, float, float]]:
        """
        获取 4 个特征点的 3D 位置
        
        特征点在 xy 平面上绕机器人中心旋转：
        - 特征0: 前方
        - 特征1: 右方
        - 特征2: 后方
        - 特征3: 左方
        """
        features = []
        base_angles = [np.pi/2, 0, -np.pi/2, np.pi]  # 前右后左
        
        for i, base_angle in enumerate(base_angles):
            angle = base_angle + robot.rotation_angle
            fx = robot.x + self.feature_radius * np.cos(angle)
            fy = robot.y + self.feature_radius * np.sin(angle)
            fz = robot.z  # z 保持与机器人相同
            features.append((i, fx, fy, fz))
        
        return features
    
    def is_in_camera_view(self, camera: CameraState3D, x: float, y: float, z: float) -> bool:
        """检查 3D 点是否在相机视场内"""
        # 计算点相对于相机的向量
        dx = x - camera.x
        dy = y - camera.y
        dz = z - camera.z
        
        # 计算距离
        distance = np.sqrt(dx**2 + dy**2 + dz**2)
        if distance < camera.view_distance_min or distance > camera.view_distance_max:
            return False
        
        # 计算相机朝向向量
        cam_dir_x = np.cos(camera.phi) * np.cos(camera.theta)
        cam_dir_y = np.cos(camera.phi) * np.sin(camera.theta)
        cam_dir_z = np.sin(camera.phi)
        
        # 计算点到相机的单位向量
        inv_dist = 1.0 / distance
        dir_x = dx * inv_dist
        dir_y = dy * inv_dist
        dir_z = dz * inv_dist
        
        # 计算夹角（使用点积）
        cos_angle = cam_dir_x * dir_x + cam_dir_y * dir_y + cam_dir_z * dir_z
        
        # 检查是否在视场内（简化：用最大 FOV）
        max_half_fov = max(camera.fov_h, camera.fov_v) / 2
        return cos_angle >= np.cos(max_half_fov)
    
    def generate_detections(self):
        """生成 3D 检测数据"""
        for frame_id in range(self.num_frames):
            camera = self.camera_states[frame_id]
            robot = self.robot_states[frame_id]
            
            # 获取特征点位置
            features = self.get_feature_positions(robot)
            self.ground_truth.append(features)
            
            frame_detections = []
            for feature_id, fx, fy, fz in features:
                # 检查是否在视场内
                if not self.is_in_camera_view(camera, fx, fy, fz):
                    continue
                
                # 模拟漏检
                if np.random.random() > self.detection_prob:
                    continue
                
                # 添加测量噪声
                noisy_x = fx + np.random.normal(0, self.noise_std)
                noisy_y = fy + np.random.normal(0, self.noise_std)
                noisy_z = fz + np.random.normal(0, self.noise_std)
                
                det = Detection3D(
                    frame_id=frame_id,
                    feature_id=feature_id,
                    x=noisy_x,
                    y=noisy_y,
                    z=noisy_z,
                    confidence=0.8 + 0.2 * np.random.random()
                )
                frame_detections.append(det)
            
            self.detections.append(frame_detections)
    
    def generate(self):
        """生成完整的 3D 轨迹数据"""
        print("生成相机 3D 轨迹...")
        self.generate_camera_trajectory()
        
        print("生成机器人 3D 轨迹...")
        self.generate_robot_trajectory()
        
        print("生成 3D 检测数据...")
        self.generate_detections()
        
        # 统计
        total_detections = sum(len(d) for d in self.detections)
        total_possible = self.num_frames * 4
        print(f"总帧数: {self.num_frames}")
        print(f"总检测数: {total_detections}")
        print(f"可能的最大检测数: {total_possible}")
        print(f"检测率: {100*total_detections/total_possible:.1f}%")
    
    def save_detections_csv(self, filename: str):
        """保存 3D 检测数据为 CSV 格式"""
        os.makedirs(os.path.dirname(filename) if os.path.dirname(filename) else '.', exist_ok=True)
        
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['frame_id', 'feature_id', 'x', 'y', 'z', 'confidence'])
            
            for frame_id, frame_dets in enumerate(self.detections):
                for det in frame_dets:
                    writer.writerow([
                        frame_id, det.feature_id,
                        f"{det.x:.6f}", f"{det.y:.6f}", f"{det.z:.6f}",
                        f"{det.confidence:.3f}"
                    ])
        
        print(f"3D 检测数据已保存到: {filename}")
    
    def save_ground_truth_csv(self, filename: str):
        """保存 3D 真值数据"""
        os.makedirs(os.path.dirname(filename) if os.path.dirname(filename) else '.', exist_ok=True)
        
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['frame_id', 'feature_id', 'x', 'y', 'z',
                           'robot_x', 'robot_y', 'robot_z',
                           'robot_vx', 'robot_vy', 'robot_vz',
                           'camera_x', 'camera_y', 'camera_z'])
            
            for frame_id in range(self.num_frames):
                robot = self.robot_states[frame_id]
                camera = self.camera_states[frame_id]
                
                for feature_id, fx, fy, fz in self.ground_truth[frame_id]:
                    writer.writerow([
                        frame_id, feature_id,
                        f"{fx:.6f}", f"{fy:.6f}", f"{fz:.6f}",
                        f"{robot.x:.6f}", f"{robot.y:.6f}", f"{robot.z:.6f}",
                        f"{robot.vx:.6f}", f"{robot.vy:.6f}", f"{robot.vz:.6f}",
                        f"{camera.x:.6f}", f"{camera.y:.6f}", f"{camera.z:.6f}"
                    ])
        
        print(f"3D 真值数据已保存到: {filename}")
    
    def save_trajectory_info(self, filename: str):
        """保存轨迹信息"""
        os.makedirs(os.path.dirname(filename) if os.path.dirname(filename) else '.', exist_ok=True)
        
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['frame_id', 'time',
                           'robot_x', 'robot_y', 'robot_z',
                           'robot_vx', 'robot_vy', 'robot_vz',
                           'robot_ax', 'robot_ay', 'robot_az',
                           'robot_rot_angle', 'robot_rot_speed',
                           'camera_x', 'camera_y', 'camera_z',
                           'camera_vx', 'camera_vy', 'camera_vz',
                           'num_detections'])
            
            for frame_id in range(self.num_frames):
                robot = self.robot_states[frame_id]
                camera = self.camera_states[frame_id]
                
                writer.writerow([
                    frame_id, f"{frame_id * self.dt:.4f}",
                    f"{robot.x:.6f}", f"{robot.y:.6f}", f"{robot.z:.6f}",
                    f"{robot.vx:.6f}", f"{robot.vy:.6f}", f"{robot.vz:.6f}",
                    f"{robot.ax:.6f}", f"{robot.ay:.6f}", f"{robot.az:.6f}",
                    f"{robot.rotation_angle:.4f}", f"{robot.rotation_speed:.4f}",
                    f"{camera.x:.6f}", f"{camera.y:.6f}", f"{camera.z:.6f}",
                    f"{camera.vx:.6f}", f"{camera.vy:.6f}", f"{camera.vz:.6f}",
                    len(self.detections[frame_id])
                ])
        
        print(f"轨迹信息已保存到: {filename}")
    
    def visualize(self, output_file: str = None, show: bool = True):
        """可视化 3D 轨迹"""
        fig = plt.figure(figsize=(16, 12))
        
        # 1. 3D 轨迹图
        ax1 = fig.add_subplot(2, 2, 1, projection='3d')
        
        robot_x = [s.x for s in self.robot_states]
        robot_y = [s.y for s in self.robot_states]
        robot_z = [s.z for s in self.robot_states]
        camera_x = [s.x for s in self.camera_states]
        camera_y = [s.y for s in self.camera_states]
        camera_z = [s.z for s in self.camera_states]
        
        ax1.plot3D(robot_x, robot_y, robot_z, 'b-', label='Robot', alpha=0.7)
        ax1.plot3D(camera_x, camera_y, camera_z, 'r-', label='Camera', alpha=0.7)
        ax1.scatter([robot_x[0]], [robot_y[0]], [robot_z[0]], c='b', s=100, marker='o')
        ax1.scatter([camera_x[0]], [camera_y[0]], [camera_z[0]], c='r', s=100, marker='s')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.set_title('3D Trajectories')
        ax1.legend()
        
        # 2. XY 投影
        ax2 = fig.add_subplot(2, 2, 2)
        ax2.plot(robot_x, robot_y, 'b-', label='Robot', alpha=0.7)
        ax2.plot(camera_x, camera_y, 'r-', label='Camera', alpha=0.7)
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        ax2.set_title('XY Projection')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        ax2.set_aspect('equal')
        
        # 3. 速度变化图
        ax3 = fig.add_subplot(2, 2, 3)
        t = np.arange(self.num_frames) * self.dt
        robot_speed = [np.sqrt(s.vx**2 + s.vy**2 + s.vz**2) for s in self.robot_states]
        camera_speed = [np.sqrt(s.vx**2 + s.vy**2 + s.vz**2) for s in self.camera_states]
        
        ax3.plot(t, robot_speed, 'b-', label='Robot speed')
        ax3.plot(t, camera_speed, 'r-', label='Camera speed')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Speed (m/s)')
        ax3.set_title('Speed over Time')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # 4. 检测统计图
        ax4 = fig.add_subplot(2, 2, 4)
        det_counts = [len(d) for d in self.detections]
        
        ax4.bar(t, det_counts, width=self.dt, color='blue', alpha=0.7)
        ax4.axhline(y=np.mean(det_counts), color='r', linestyle='--', 
                   label=f'Mean: {np.mean(det_counts):.1f}')
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Detection Count')
        ax4.set_title('Detections per Frame')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if output_file:
            plt.savefig(output_file, dpi=150, bbox_inches='tight')
            print(f"可视化图像已保存到: {output_file}")
        
        if show:
            plt.show()
        
        plt.close()


def main():
    parser = argparse.ArgumentParser(description='生成 3D 复杂轨迹测试数据')
    parser.add_argument('--frames', type=int, default=500, help='总帧数')
    parser.add_argument('--dt', type=float, default=0.01, help='时间步长（秒）')
    parser.add_argument('--noise', type=float, default=0.02, help='测量噪声标准差（米）')
    parser.add_argument('--detection-prob', type=float, default=0.95, help='检测概率')
    parser.add_argument('--output-dir', type=str, default='data', help='输出目录')
    parser.add_argument('--visualize', action='store_true', help='显示可视化')
    parser.add_argument('--seed', type=int, default=42, help='随机种子')
    
    args = parser.parse_args()
    
    # 创建生成器
    generator = ComplexTrajectoryGenerator3D(
        num_frames=args.frames,
        dt=args.dt,
        noise_std=args.noise,
        detection_prob=args.detection_prob,
        seed=args.seed
    )
    
    # 生成数据
    generator.generate()
    
    # 保存数据
    output_dir = args.output_dir
    generator.save_detections_csv(f'{output_dir}/complex_detections_3d.csv')
    generator.save_ground_truth_csv(f'{output_dir}/complex_ground_truth_3d.csv')
    generator.save_trajectory_info(f'{output_dir}/complex_trajectory_info_3d.csv')
    
    # 可视化
    generator.visualize(f'{output_dir}/trajectory_overview_3d.png', show=args.visualize)


if __name__ == '__main__':
    main()
