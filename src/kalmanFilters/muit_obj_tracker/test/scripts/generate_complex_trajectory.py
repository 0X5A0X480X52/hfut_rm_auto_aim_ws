#!/usr/bin/env python3
"""
复杂轨迹数据生成器

模拟场景：
1. 机器人在平面上进行变加速度机动
2. 机器人上有4个特征点（前后左右），相对机器人中心固定
3. 4个特征点绕机器人轴心进行变加速度旋转
4. 相机也在进行变加速度机动
5. 只有在相机视场角内的特征才能被识别

输出：CSV 格式的检测数据，可供 point_tracker_test 使用
"""

import numpy as np
import csv
import argparse
import os
from dataclasses import dataclass
from typing import List, Tuple, Optional
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyArrow
from matplotlib.animation import FuncAnimation
from matplotlib.font_manager import FontProperties

# 添加中文支持
FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
myfont = FontProperties(fname=FONT_PATH)

plt.rcParams['font.family'] = myfont.get_name()     # 强制使用该中文字体
plt.rcParams['font.sans-serif'] = [myfont.get_name()]
plt.rcParams['axes.unicode_minus'] = False


@dataclass
class CameraState:
    """相机状态"""
    x: float  # 相机位置 x
    y: float  # 相机位置 y
    vx: float  # 速度 x
    vy: float  # 速度 y
    ax: float  # 加速度 x
    ay: float  # 加速度 y
    theta: float  # 相机朝向角度（弧度）
    fov: float  # 视场角（弧度）
    view_distance: float  # 视距


@dataclass
class RobotState:
    """机器人状态"""
    x: float  # 机器人中心位置 x
    y: float  # 机器人中心位置 y
    vx: float  # 速度 x
    vy: float  # 速度 y
    ax: float  # 加速度 x
    ay: float  # 加速度 y
    rotation_angle: float  # 特征点旋转角度（弧度）
    rotation_speed: float  # 旋转角速度
    rotation_accel: float  # 旋转角加速度


@dataclass
class Detection:
    """检测结果"""
    frame_id: int
    feature_id: int
    x: float
    y: float
    confidence: float


class ComplexTrajectoryGenerator:
    """复杂轨迹生成器"""
    
    def __init__(self, 
                 num_frames: int = 500,
                 dt: float = 0.033,
                 feature_radius: float = 50.0,  # 特征点到机器人中心的距离
                 bbox_size: int = 40,
                 noise_std: float = 2.0,
                 detection_prob: float = 0.95,  # 检测概率
                 seed: int = 42):
        """
        初始化生成器
        
        Args:
            num_frames: 总帧数
            dt: 时间步长（秒）
            feature_radius: 特征点到机器人中心的距离
            bbox_size: 检测框大小
            noise_std: 测量噪声标准差
            detection_prob: 检测概率（模拟漏检）
            seed: 随机种子
        """
        self.num_frames = num_frames
        self.dt = dt
        self.feature_radius = feature_radius
        self.bbox_size = bbox_size
        self.noise_std = noise_std
        self.detection_prob = detection_prob
        
        np.random.seed(seed)
        
        # 图像尺寸（用于视场角计算）
        self.image_width = 1280
        self.image_height = 720
        
        # 初始化状态
        self.camera_states: List[CameraState] = []
        self.robot_states: List[RobotState] = []
        self.detections: List[List[Detection]] = []
        self.ground_truth: List[List[Tuple[int, float, float]]] = []  # (feature_id, x, y)
        
    def _generate_variable_acceleration(self, 
                                        base_accel: float, 
                                        freq: float, 
                                        phase: float,
                                        t: float) -> float:
        """生成变加速度"""
        return base_accel * np.sin(2 * np.pi * freq * t + phase)
    
    def generate_camera_trajectory(self):
        """生成相机轨迹 - 变加速度机动"""
        # 初始状态
        camera = CameraState(
            x=640.0, y=360.0,  # 图像中心
            vx=0.0, vy=0.0,
            ax=0.0, ay=0.0,
            theta=0.0,  # 朝向
            fov=np.radians(90),  # 90度视场角
            view_distance=800.0
        )
        
        # 加速度参数
        accel_base_x = 50.0  # 基础加速度
        accel_base_y = 30.0
        freq_x = 0.05  # 加速度变化频率
        freq_y = 0.07
        
        for i in range(self.num_frames):
            t = i * self.dt
            
            # 变加速度
            camera.ax = self._generate_variable_acceleration(accel_base_x, freq_x, 0, t)
            camera.ay = self._generate_variable_acceleration(accel_base_y, freq_y, np.pi/4, t)
            
            # 更新速度和位置
            camera.vx += camera.ax * self.dt
            camera.vy += camera.ay * self.dt
            
            # 限制速度
            max_speed = 200.0
            speed = np.sqrt(camera.vx**2 + camera.vy**2)
            if speed > max_speed:
                camera.vx *= max_speed / speed
                camera.vy *= max_speed / speed
            
            camera.x += camera.vx * self.dt
            camera.y += camera.vy * self.dt
            
            # 边界限制
            camera.x = np.clip(camera.x, 100, self.image_width - 100)
            camera.y = np.clip(camera.y, 100, self.image_height - 100)
            
            # 相机朝向缓慢变化
            camera.theta += 0.01 * np.sin(0.1 * t)
            
            self.camera_states.append(CameraState(
                x=camera.x, y=camera.y,
                vx=camera.vx, vy=camera.vy,
                ax=camera.ax, ay=camera.ay,
                theta=camera.theta,
                fov=camera.fov,
                view_distance=camera.view_distance
            ))
    
    def generate_robot_trajectory(self):
        """生成机器人轨迹 - 变加速度机动 + 特征旋转"""
        # 初始状态
        robot = RobotState(
            x=640.0, y=360.0,
            vx=30.0, vy=20.0,
            ax=0.0, ay=0.0,
            rotation_angle=0.0,
            rotation_speed=1.0,  # 初始旋转速度 rad/s
            rotation_accel=0.0
        )
        
        # 机器人加速度参数
        accel_base_x = 80.0
        accel_base_y = 60.0
        freq_x = 0.03
        freq_y = 0.04
        
        # 旋转加速度参数
        rot_accel_base = 2.0  # 旋转角加速度基础值
        rot_freq = 0.08
        
        for i in range(self.num_frames):
            t = i * self.dt
            
            # 变加速度
            robot.ax = self._generate_variable_acceleration(accel_base_x, freq_x, np.pi/3, t)
            robot.ay = self._generate_variable_acceleration(accel_base_y, freq_y, np.pi/6, t)
            
            # 更新速度和位置
            robot.vx += robot.ax * self.dt
            robot.vy += robot.ay * self.dt
            
            # 限制速度
            max_speed = 300.0
            speed = np.sqrt(robot.vx**2 + robot.vy**2)
            if speed > max_speed:
                robot.vx *= max_speed / speed
                robot.vy *= max_speed / speed
            
            robot.x += robot.vx * self.dt
            robot.y += robot.vy * self.dt
            
            # 边界反弹
            if robot.x < 50 or robot.x > self.image_width - 50:
                robot.vx *= -0.8
                robot.x = np.clip(robot.x, 50, self.image_width - 50)
            if robot.y < 50 or robot.y > self.image_height - 50:
                robot.vy *= -0.8
                robot.y = np.clip(robot.y, 50, self.image_height - 50)
            
            # 旋转变加速度
            robot.rotation_accel = self._generate_variable_acceleration(rot_accel_base, rot_freq, 0, t)
            robot.rotation_speed += robot.rotation_accel * self.dt
            
            # 限制旋转速度
            robot.rotation_speed = np.clip(robot.rotation_speed, -5.0, 5.0)
            robot.rotation_angle += robot.rotation_speed * self.dt
            
            self.robot_states.append(RobotState(
                x=robot.x, y=robot.y,
                vx=robot.vx, vy=robot.vy,
                ax=robot.ax, ay=robot.ay,
                rotation_angle=robot.rotation_angle,
                rotation_speed=robot.rotation_speed,
                rotation_accel=robot.rotation_accel
            ))
    
    def get_feature_positions(self, robot: RobotState) -> List[Tuple[int, float, float]]:
        """
        获取4个特征点的世界坐标位置
        
        特征点相对机器人中心的初始位置：
        - 特征0: 前方 (0, -r)
        - 特征1: 右方 (r, 0)
        - 特征2: 后方 (0, r)
        - 特征3: 左方 (-r, 0)
        """
        features = []
        base_angles = [np.pi/2, 0, -np.pi/2, np.pi]  # 前右后左
        
        for i, base_angle in enumerate(base_angles):
            angle = base_angle + robot.rotation_angle
            fx = robot.x + self.feature_radius * np.cos(angle)
            fy = robot.y + self.feature_radius * np.sin(angle)
            features.append((i, fx, fy))
        
        return features
    
    def is_in_camera_view(self, camera: CameraState, x: float, y: float) -> bool:
        """检查点是否在相机视场角内"""
        # 计算点相对于相机的位置
        dx = x - camera.x
        dy = y - camera.y
        
        # 计算距离
        distance = np.sqrt(dx**2 + dy**2)
        if distance > camera.view_distance:
            return False
        
        # 计算角度
        angle_to_point = np.arctan2(dy, dx)
        angle_diff = angle_to_point - camera.theta
        
        # 归一化角度差到 [-pi, pi]
        while angle_diff > np.pi:
            angle_diff -= 2 * np.pi
        while angle_diff < -np.pi:
            angle_diff += 2 * np.pi
        
        # 检查是否在视场角内
        return abs(angle_diff) <= camera.fov / 2
    
    def generate_detections(self):
        """生成检测数据"""
        for frame_id in range(self.num_frames):
            camera = self.camera_states[frame_id]
            robot = self.robot_states[frame_id]
            
            # 获取特征点位置
            features = self.get_feature_positions(robot)
            self.ground_truth.append(features)
            
            frame_detections = []
            for feature_id, fx, fy in features:
                # 检查是否在视场角内
                if not self.is_in_camera_view(camera, fx, fy):
                    continue
                
                # 模拟漏检
                if np.random.random() > self.detection_prob:
                    continue
                
                # 添加测量噪声
                noisy_x = fx + np.random.normal(0, self.noise_std)
                noisy_y = fy + np.random.normal(0, self.noise_std)
                
                # 检查是否在图像范围内
                if 0 <= noisy_x <= self.image_width and 0 <= noisy_y <= self.image_height:
                    det = Detection(
                        frame_id=frame_id,
                        feature_id=feature_id,
                        x=noisy_x,
                        y=noisy_y,
                        confidence=0.8 + 0.2 * np.random.random()
                    )
                    frame_detections.append(det)
            
            self.detections.append(frame_detections)
    
    def generate(self):
        """生成完整的轨迹数据"""
        print("生成相机轨迹...")
        self.generate_camera_trajectory()
        
        print("生成机器人轨迹...")
        self.generate_robot_trajectory()
        
        print("生成检测数据...")
        self.generate_detections()
        
        # 统计
        total_detections = sum(len(d) for d in self.detections)
        total_possible = self.num_frames * 4
        print(f"总帧数: {self.num_frames}")
        print(f"总检测数: {total_detections}")
        print(f"可能的最大检测数: {total_possible}")
        print(f"检测率: {100*total_detections/total_possible:.1f}%")
    
    def save_detections_csv(self, filename: str):
        """保存检测数据为 CSV 格式"""
        os.makedirs(os.path.dirname(filename) if os.path.dirname(filename) else '.', exist_ok=True)
        
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['frame_id', 'feature_id', 'x', 'y', 'width', 'height', 'confidence'])
            
            for frame_id, frame_dets in enumerate(self.detections):
                for det in frame_dets:
                    # 转换为边界框格式
                    x = det.x - self.bbox_size / 2
                    y = det.y - self.bbox_size / 2
                    writer.writerow([
                        frame_id, det.feature_id, 
                        f"{x:.2f}", f"{y:.2f}",
                        self.bbox_size, self.bbox_size,
                        f"{det.confidence:.3f}"
                    ])
        
        print(f"检测数据已保存到: {filename}")
    
    def save_ground_truth_csv(self, filename: str):
        """保存真值数据为 CSV 格式"""
        os.makedirs(os.path.dirname(filename) if os.path.dirname(filename) else '.', exist_ok=True)
        
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['frame_id', 'feature_id', 'x', 'y', 
                           'robot_x', 'robot_y', 'robot_vx', 'robot_vy',
                           'camera_x', 'camera_y'])
            
            for frame_id in range(self.num_frames):
                robot = self.robot_states[frame_id]
                camera = self.camera_states[frame_id]
                
                for feature_id, fx, fy in self.ground_truth[frame_id]:
                    writer.writerow([
                        frame_id, feature_id,
                        f"{fx:.2f}", f"{fy:.2f}",
                        f"{robot.x:.2f}", f"{robot.y:.2f}",
                        f"{robot.vx:.2f}", f"{robot.vy:.2f}",
                        f"{camera.x:.2f}", f"{camera.y:.2f}"
                    ])
        
        print(f"真值数据已保存到: {filename}")
    
    def save_trajectory_info(self, filename: str):
        """保存轨迹信息（用于分析）"""
        os.makedirs(os.path.dirname(filename) if os.path.dirname(filename) else '.', exist_ok=True)
        
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['frame_id', 'time',
                           'robot_x', 'robot_y', 'robot_vx', 'robot_vy', 'robot_ax', 'robot_ay',
                           'robot_rot_angle', 'robot_rot_speed', 'robot_rot_accel',
                           'camera_x', 'camera_y', 'camera_vx', 'camera_vy', 'camera_ax', 'camera_ay',
                           'camera_theta', 'num_detections'])
            
            for frame_id in range(self.num_frames):
                robot = self.robot_states[frame_id]
                camera = self.camera_states[frame_id]
                
                writer.writerow([
                    frame_id, f"{frame_id * self.dt:.4f}",
                    f"{robot.x:.2f}", f"{robot.y:.2f}",
                    f"{robot.vx:.2f}", f"{robot.vy:.2f}",
                    f"{robot.ax:.2f}", f"{robot.ay:.2f}",
                    f"{robot.rotation_angle:.4f}", f"{robot.rotation_speed:.4f}", f"{robot.rotation_accel:.4f}",
                    f"{camera.x:.2f}", f"{camera.y:.2f}",
                    f"{camera.vx:.2f}", f"{camera.vy:.2f}",
                    f"{camera.ax:.2f}", f"{camera.ay:.2f}",
                    f"{camera.theta:.4f}",
                    len(self.detections[frame_id])
                ])
        
        print(f"轨迹信息已保存到: {filename}")
    
    def visualize(self, output_file: str = None, show: bool = True):
        """可视化轨迹"""
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        
        # 1. 整体轨迹图
        ax1 = axes[0, 0]
        robot_x = [s.x for s in self.robot_states]
        robot_y = [s.y for s in self.robot_states]
        camera_x = [s.x for s in self.camera_states]
        camera_y = [s.y for s in self.camera_states]
        
        ax1.plot(robot_x, robot_y, 'b-', label='机器人轨迹', alpha=0.7)
        ax1.plot(camera_x, camera_y, 'r-', label='相机轨迹', alpha=0.7)
        ax1.scatter([robot_x[0]], [robot_y[0]], c='b', s=100, marker='o', label='机器人起点')
        ax1.scatter([camera_x[0]], [camera_y[0]], c='r', s=100, marker='s', label='相机起点')
        ax1.set_xlim(0, self.image_width)
        ax1.set_ylim(0, self.image_height)
        ax1.set_xlabel('X (像素)')
        ax1.set_ylabel('Y (像素)')
        ax1.set_title('机器人和相机轨迹')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.set_aspect('equal')
        
        # 2. 速度变化图
        ax2 = axes[0, 1]
        t = np.arange(self.num_frames) * self.dt
        robot_speed = [np.sqrt(s.vx**2 + s.vy**2) for s in self.robot_states]
        camera_speed = [np.sqrt(s.vx**2 + s.vy**2) for s in self.camera_states]
        
        ax2.plot(t, robot_speed, 'b-', label='机器人速度')
        ax2.plot(t, camera_speed, 'r-', label='相机速度')
        ax2.set_xlabel('时间 (秒)')
        ax2.set_ylabel('速度 (像素/秒)')
        ax2.set_title('速度变化')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # 3. 旋转状态图
        ax3 = axes[1, 0]
        rot_angle = [np.degrees(s.rotation_angle) % 360 for s in self.robot_states]
        rot_speed = [np.degrees(s.rotation_speed) for s in self.robot_states]
        
        ax3_twin = ax3.twinx()
        line1, = ax3.plot(t, rot_angle, 'g-', label='旋转角度')
        line2, = ax3_twin.plot(t, rot_speed, 'm-', label='旋转角速度')
        ax3.set_xlabel('时间 (秒)')
        ax3.set_ylabel('角度 (度)', color='g')
        ax3_twin.set_ylabel('角速度 (度/秒)', color='m')
        ax3.set_title('特征旋转状态')
        ax3.legend(handles=[line1, line2], loc='upper right')
        ax3.grid(True, alpha=0.3)
        
        # 4. 检测统计图
        ax4 = axes[1, 1]
        det_counts = [len(d) for d in self.detections]
        
        ax4.bar(t, det_counts, width=self.dt, color='blue', alpha=0.7)
        ax4.axhline(y=np.mean(det_counts), color='r', linestyle='--', label=f'平均: {np.mean(det_counts):.1f}')
        ax4.set_xlabel('时间 (秒)')
        ax4.set_ylabel('检测数量')
        ax4.set_title('每帧检测数量')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if output_file:
            plt.savefig(output_file, dpi=150, bbox_inches='tight')
            print(f"可视化图像已保存到: {output_file}")
        
        if show:
            plt.show()
        
        plt.close()
    
    def create_animation(self, output_file: str = None, show: bool = True, fps: int = 30):
        """创建动画"""
        fig, ax = plt.subplots(figsize=(12, 8))
        
        def init():
            ax.set_xlim(0, self.image_width)
            ax.set_ylim(0, self.image_height)
            ax.set_aspect('equal')
            ax.set_xlabel('X (像素)')
            ax.set_ylabel('Y (像素)')
            return []
        
        def update(frame):
            ax.clear()
            ax.set_xlim(0, self.image_width)
            ax.set_ylim(0, self.image_height)
            ax.set_aspect('equal')
            
            robot = self.robot_states[frame]
            camera = self.camera_states[frame]
            
            # 绘制相机视场角
            fov_left = camera.theta - camera.fov / 2
            fov_right = camera.theta + camera.fov / 2
            fov_dist = camera.view_distance
            
            # 视场角边界线
            ax.plot([camera.x, camera.x + fov_dist * np.cos(fov_left)],
                   [camera.y, camera.y + fov_dist * np.sin(fov_left)], 'r--', alpha=0.5)
            ax.plot([camera.x, camera.x + fov_dist * np.cos(fov_right)],
                   [camera.y, camera.y + fov_dist * np.sin(fov_right)], 'r--', alpha=0.5)
            
            # 绘制相机
            ax.scatter([camera.x], [camera.y], c='red', s=150, marker='s', label='相机', zorder=5)
            
            # 绘制机器人
            ax.scatter([robot.x], [robot.y], c='blue', s=200, marker='o', label='机器人', zorder=5)
            
            # 绘制特征点
            features = self.get_feature_positions(robot)
            colors = ['green', 'orange', 'purple', 'cyan']
            labels = ['前', '右', '后', '左']
            
            for (fid, fx, fy), color, label in zip(features, colors, labels):
                in_view = self.is_in_camera_view(camera, fx, fy)
                marker = 'o' if in_view else 'x'
                alpha = 1.0 if in_view else 0.3
                ax.scatter([fx], [fy], c=color, s=100, marker=marker, alpha=alpha, label=f'特征{fid}({label})', zorder=4)
                # 连接机器人中心和特征点
                ax.plot([robot.x, fx], [robot.y, fy], color=color, alpha=0.3, linewidth=1)
            
            # 绘制检测框
            for det in self.detections[frame]:
                rect = Rectangle(
                    (det.x - self.bbox_size/2, det.y - self.bbox_size/2),
                    self.bbox_size, self.bbox_size,
                    fill=False, edgecolor='lime', linewidth=2
                )
                ax.add_patch(rect)
            
            ax.set_title(f'帧: {frame}/{self.num_frames}, 检测数: {len(self.detections[frame])}')
            ax.legend(loc='upper right', fontsize=8)
            ax.grid(True, alpha=0.3)
            
            return []
        
        anim = FuncAnimation(fig, update, frames=range(0, self.num_frames, 2),
                           init_func=init, blit=False, interval=1000/fps)
        
        if output_file:
            print(f"正在保存动画到: {output_file}")
            anim.save(output_file, writer='pillow', fps=fps)
            print("动画保存完成")
        
        if show:
            plt.show()
        
        plt.close()


def main():
    parser = argparse.ArgumentParser(description='生成复杂轨迹测试数据')
    parser.add_argument('--frames', type=int, default=500, help='总帧数')
    parser.add_argument('--dt', type=float, default=0.033, help='时间步长（秒）')
    parser.add_argument('--noise', type=float, default=2.0, help='测量噪声标准差')
    parser.add_argument('--detection-prob', type=float, default=0.95, help='检测概率')
    parser.add_argument('--output-dir', type=str, default='data', help='输出目录')
    parser.add_argument('--visualize', action='store_true', help='显示可视化')
    parser.add_argument('--save-animation', action='store_true', help='保存动画')
    parser.add_argument('--seed', type=int, default=42, help='随机种子')
    
    args = parser.parse_args()
    
    # 创建生成器
    generator = ComplexTrajectoryGenerator(
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
    generator.save_detections_csv(f'{output_dir}/complex_detections.csv')
    generator.save_ground_truth_csv(f'{output_dir}/complex_ground_truth.csv')
    generator.save_trajectory_info(f'{output_dir}/complex_trajectory_info.csv')
    
    # 可视化
    if args.visualize:
        generator.visualize(f'{output_dir}/trajectory_overview.png', show=True)
    else:
        generator.visualize(f'{output_dir}/trajectory_overview.png', show=False)
    
    # 保存动画
    if args.save_animation:
        generator.create_animation(f'{output_dir}/trajectory_animation.gif', show=False, fps=15)


if __name__ == '__main__':
    main()
