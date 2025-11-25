#!/usr/bin/env python3
"""
Generate test data for Kalman Filter testing
生成卡尔曼滤波器测试数据
"""

import numpy as np
import csv
import argparse
from pathlib import Path


class TrajectoryGenerator:
    """Base class for trajectory generation"""
    
    def __init__(self, dt=0.01, duration=10.0, noise_std=0.1):
        """
        Args:
            dt: Time step (seconds)
            duration: Total duration (seconds)
            noise_std: Measurement noise standard deviation
        """
        self.dt = dt
        self.duration = duration
        self.noise_std = noise_std
        self.time_steps = int(duration / dt)
        
    def add_noise(self, measurements):
        """Add Gaussian noise to measurements"""
        noise = np.random.normal(0, self.noise_std, measurements.shape)
        return measurements + noise


class ConstantVelocity2D(TrajectoryGenerator):
    """2D Constant Velocity trajectory"""
    
    def generate(self, v_x=1.0, v_y=0.5, x0=0.0, y0=0.0):
        """Generate 2D constant velocity trajectory
        
        Args:
            v_x, v_y: Velocity in x and y directions
            x0, y0: Initial position
        """
        data = []
        for i in range(self.time_steps):
            t = i * self.dt
            x = x0 + v_x * t
            y = y0 + v_y * t
            
            # True state: [x, vx, y, vy]
            true_state = [x, v_x, y, v_y]
            
            # Measurement: [x, y] with noise
            measurement = self.add_noise(np.array([x, y]))
            
            data.append([t, x, y, v_x, v_y, measurement[0], measurement[1]])
            
        return data, ['time', 'true_x', 'true_y', 'true_vx', 'true_vy', 'meas_x', 'meas_y']


class ConstantAcceleration2D(TrajectoryGenerator):
    """2D Constant Acceleration trajectory"""
    
    def generate(self, a_x=0.5, a_y=0.3, v_x0=0.0, v_y0=0.0, x0=0.0, y0=0.0):
        """Generate 2D constant acceleration trajectory"""
        data = []
        for i in range(self.time_steps):
            t = i * self.dt
            x = x0 + v_x0 * t + 0.5 * a_x * t**2
            y = y0 + v_y0 * t + 0.5 * a_y * t**2
            v_x = v_x0 + a_x * t
            v_y = v_y0 + a_y * t
            
            # True state: [x, vx, ax, y, vy, ay]
            true_state = [x, v_x, a_x, y, v_y, a_y]
            
            # Measurement: [x, y] with noise
            measurement = self.add_noise(np.array([x, y]))
            
            data.append([t, x, y, v_x, v_y, a_x, a_y, 
                        measurement[0], measurement[1]])
            
        return data, ['time', 'true_x', 'true_y', 'true_vx', 'true_vy', 
                     'true_ax', 'true_ay', 'meas_x', 'meas_y']


class CircularMotion2D(TrajectoryGenerator):
    """2D Circular motion (constant turn rate)"""
    
    def generate(self, radius=5.0, angular_velocity=0.5, x0=0.0, y0=0.0):
        """Generate 2D circular trajectory
        
        Args:
            radius: Circle radius
            angular_velocity: Angular velocity (rad/s)
            x0, y0: Center of circle
        """
        data = []
        for i in range(self.time_steps):
            t = i * self.dt
            theta = angular_velocity * t
            
            x = x0 + radius * np.cos(theta)
            y = y0 + radius * np.sin(theta)
            v_x = -radius * angular_velocity * np.sin(theta)
            v_y = radius * angular_velocity * np.cos(theta)
            
            # Measurement: [x, y] with noise
            measurement = self.add_noise(np.array([x, y]))
            
            data.append([t, x, y, v_x, v_y, theta, angular_velocity,
                        measurement[0], measurement[1]])
            
        return data, ['time', 'true_x', 'true_y', 'true_vx', 'true_vy',
                     'true_theta', 'true_omega', 'meas_x', 'meas_y']


class ConstantVelocity3D(TrajectoryGenerator):
    """3D Constant Velocity trajectory"""
    
    def generate(self, v_x=1.0, v_y=0.5, v_z=0.3, x0=0.0, y0=0.0, z0=0.0):
        """Generate 3D constant velocity trajectory"""
        data = []
        for i in range(self.time_steps):
            t = i * self.dt
            x = x0 + v_x * t
            y = y0 + v_y * t
            z = z0 + v_z * t
            
            # True state: [x, vx, y, vy, z, vz]
            true_state = [x, v_x, y, v_y, z, v_z]
            
            # Measurement: [x, y, z] with noise
            measurement = self.add_noise(np.array([x, y, z]))
            
            data.append([t, x, y, z, v_x, v_y, v_z,
                        measurement[0], measurement[1], measurement[2]])
            
        return data, ['time', 'true_x', 'true_y', 'true_z', 
                     'true_vx', 'true_vy', 'true_vz',
                     'meas_x', 'meas_y', 'meas_z']


class ConstantAcceleration3D(TrajectoryGenerator):
    """3D Constant Acceleration trajectory"""
    
    def generate(self, a_x=0.5, a_y=0.3, a_z=0.2,
                 v_x0=0.0, v_y0=0.0, v_z0=0.0,
                 x0=0.0, y0=0.0, z0=0.0):
        """Generate 3D constant acceleration trajectory"""
        data = []
        for i in range(self.time_steps):
            t = i * self.dt
            x = x0 + v_x0 * t + 0.5 * a_x * t**2
            y = y0 + v_y0 * t + 0.5 * a_y * t**2
            z = z0 + v_z0 * t + 0.5 * a_z * t**2
            v_x = v_x0 + a_x * t
            v_y = v_y0 + a_y * t
            v_z = v_z0 + a_z * t
            
            # Measurement: [x, y, z] with noise
            measurement = self.add_noise(np.array([x, y, z]))
            
            data.append([t, x, y, z, v_x, v_y, v_z, a_x, a_y, a_z,
                        measurement[0], measurement[1], measurement[2]])
            
        return data, ['time', 'true_x', 'true_y', 'true_z',
                     'true_vx', 'true_vy', 'true_vz',
                     'true_ax', 'true_ay', 'true_az',
                     'meas_x', 'meas_y', 'meas_z']


class HelixMotion3D(TrajectoryGenerator):
    """3D Helix (螺旋) motion"""
    
    def generate(self, radius=5.0, angular_velocity=0.5, v_z=1.0,
                 x0=0.0, y0=0.0, z0=0.0):
        """Generate 3D helix trajectory"""
        data = []
        for i in range(self.time_steps):
            t = i * self.dt
            theta = angular_velocity * t
            
            x = x0 + radius * np.cos(theta)
            y = y0 + radius * np.sin(theta)
            z = z0 + v_z * t
            
            v_x = -radius * angular_velocity * np.sin(theta)
            v_y = radius * angular_velocity * np.cos(theta)
            
            # Measurement: [x, y, z] with noise
            measurement = self.add_noise(np.array([x, y, z]))
            
            data.append([t, x, y, z, v_x, v_y, v_z,
                        measurement[0], measurement[1], measurement[2]])
            
        return data, ['time', 'true_x', 'true_y', 'true_z',
                     'true_vx', 'true_vy', 'true_vz',
                     'meas_x', 'meas_y', 'meas_z']


class MixedMotion3D(TrajectoryGenerator):
    """3D Mixed motion - changes between different motion patterns"""
    
    def generate(self):
        """Generate 3D trajectory with mixed motion patterns"""
        data = []
        
        # Divide into 3 segments
        segment_steps = self.time_steps // 3
        
        # Segment 1: Constant velocity
        for i in range(segment_steps):
            t = i * self.dt
            x = 1.0 * t
            y = 0.5 * t
            z = 0.3 * t
            v_x, v_y, v_z = 1.0, 0.5, 0.3
            a_x, a_y, a_z = 0.0, 0.0, 0.0
            
            measurement = self.add_noise(np.array([x, y, z]))
            data.append([t, x, y, z, v_x, v_y, v_z, a_x, a_y, a_z,
                        measurement[0], measurement[1], measurement[2]])
        
        # Segment 2: Constant acceleration
        x0, y0, z0 = data[-1][1], data[-1][2], data[-1][3]
        v_x0, v_y0, v_z0 = 1.0, 0.5, 0.3
        a_x, a_y, a_z = 0.5, 0.3, -0.2
        
        for i in range(segment_steps):
            t = (segment_steps + i) * self.dt
            t_seg = i * self.dt
            x = x0 + v_x0 * t_seg + 0.5 * a_x * t_seg**2
            y = y0 + v_y0 * t_seg + 0.5 * a_y * t_seg**2
            z = z0 + v_z0 * t_seg + 0.5 * a_z * t_seg**2
            v_x = v_x0 + a_x * t_seg
            v_y = v_y0 + a_y * t_seg
            v_z = v_z0 + a_z * t_seg
            
            measurement = self.add_noise(np.array([x, y, z]))
            data.append([t, x, y, z, v_x, v_y, v_z, a_x, a_y, a_z,
                        measurement[0], measurement[1], measurement[2]])
        
        # Segment 3: Circular motion with constant z velocity
        x0, y0, z0 = data[-1][1], data[-1][2], data[-1][3]
        radius = 3.0
        omega = 0.5
        v_z = 0.5
        
        for i in range(self.time_steps - 2 * segment_steps):
            t = (2 * segment_steps + i) * self.dt
            t_seg = i * self.dt
            theta = omega * t_seg
            
            x = x0 + radius * (np.cos(theta) - 1)
            y = y0 + radius * np.sin(theta)
            z = z0 + v_z * t_seg
            v_x = -radius * omega * np.sin(theta)
            v_y = radius * omega * np.cos(theta)
            a_x = -radius * omega**2 * np.cos(theta)
            a_y = -radius * omega**2 * np.sin(theta)
            a_z = 0.0
            
            measurement = self.add_noise(np.array([x, y, z]))
            data.append([t, x, y, z, v_x, v_y, v_z, a_x, a_y, a_z,
                        measurement[0], measurement[1], measurement[2]])
        
        return data, ['time', 'true_x', 'true_y', 'true_z',
                     'true_vx', 'true_vy', 'true_vz',
                     'true_ax', 'true_ay', 'true_az',
                     'meas_x', 'meas_y', 'meas_z']


def save_to_csv(data, headers, filename):
    """Save data to CSV file"""
    Path(filename).parent.mkdir(parents=True, exist_ok=True)
    
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        writer.writerows(data)
    
    print(f"Generated: {filename} ({len(data)} samples)")


def main():
    parser = argparse.ArgumentParser(description='Generate test data for Kalman filters')
    parser.add_argument('--output-dir', '-o', type=str, 
                       default='./test_data',
                       help='Output directory for CSV files')
    parser.add_argument('--duration', '-d', type=float, default=10.0,
                       help='Duration in seconds')
    parser.add_argument('--dt', type=float, default=0.01,
                       help='Time step in seconds')
    parser.add_argument('--noise', '-n', type=float, default=0.1,
                       help='Measurement noise standard deviation')
    parser.add_argument('--seed', '-s', type=int, default=42,
                       help='Random seed for reproducibility')
    
    args = parser.parse_args()
    
    # Set random seed
    np.random.seed(args.seed)
    
    output_dir = Path(args.output_dir)
    
    print(f"Generating test data...")
    print(f"  Duration: {args.duration}s, dt: {args.dt}s, Noise std: {args.noise}")
    print(f"  Output directory: {output_dir}")
    print()
    
    # 2D trajectories
    print("2D Trajectories:")
    
    gen = ConstantVelocity2D(args.dt, args.duration, args.noise)
    data, headers = gen.generate(v_x=2.0, v_y=1.5)
    save_to_csv(data, headers, output_dir / 'cv_2d.csv')
    
    gen = ConstantAcceleration2D(args.dt, args.duration, args.noise)
    data, headers = gen.generate(a_x=0.5, a_y=0.3, v_x0=1.0, v_y0=0.5)
    save_to_csv(data, headers, output_dir / 'ca_2d.csv')
    
    gen = CircularMotion2D(args.dt, args.duration, args.noise)
    data, headers = gen.generate(radius=5.0, angular_velocity=0.5)
    save_to_csv(data, headers, output_dir / 'circular_2d.csv')
    
    # 3D trajectories
    print("\n3D Trajectories:")
    
    gen = ConstantVelocity3D(args.dt, args.duration, args.noise)
    data, headers = gen.generate(v_x=2.0, v_y=1.5, v_z=1.0)
    save_to_csv(data, headers, output_dir / 'cv_3d.csv')
    
    gen = ConstantAcceleration3D(args.dt, args.duration, args.noise)
    data, headers = gen.generate(a_x=0.5, a_y=0.3, a_z=0.2,
                                 v_x0=1.0, v_y0=0.5, v_z0=0.3)
    save_to_csv(data, headers, output_dir / 'ca_3d.csv')
    
    gen = HelixMotion3D(args.dt, args.duration, args.noise)
    data, headers = gen.generate(radius=5.0, angular_velocity=0.5, v_z=1.0)
    save_to_csv(data, headers, output_dir / 'helix_3d.csv')
    
    gen = MixedMotion3D(args.dt, args.duration, args.noise)
    data, headers = gen.generate()
    save_to_csv(data, headers, output_dir / 'mixed_3d.csv')
    
    print("\nDone!")


if __name__ == '__main__':
    main()
