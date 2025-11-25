#!/usr/bin/env python3
"""
Analyze Kalman Filter test results
分析卡尔曼滤波器测试结果
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import argparse
from pathlib import Path
import json


class FilterResultAnalyzer:
    """Analyze and visualize Kalman filter results"""
    
    def __init__(self, test_data_file, result_file, output_dir='./analysis'):
        """
        Args:
            test_data_file: Original test data CSV file (with ground truth)
            result_file: Filter result CSV file
            output_dir: Output directory for plots and statistics
        """
        self.test_data_file = test_data_file
        self.result_file = result_file
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Load data
        self.test_data = pd.read_csv(test_data_file)
        self.result_data = pd.read_csv(result_file)
        
        # Detect dimension
        self.dim = self._detect_dimension()
        
        print(f"Loaded test data: {test_data_file}")
        print(f"  Shape: {self.test_data.shape}")
        print(f"Loaded result data: {result_file}")
        print(f"  Shape: {self.result_data.shape}")
        print(f"  Detected dimension: {self.dim}D")
        
    def _detect_dimension(self):
        """Detect spatial dimension from data"""
        if 'meas_z' in self.result_data.columns or 'true_z' in self.test_data.columns:
            return 3
        return 2
    
    def _get_est_col(self, axis):
        """Get the column name for estimated position
        
        Args:
            axis: 'x', 'y', or 'z'
        
        Returns:
            Column name in result_data (e.g., 'est_x' or 'est_state_0')
        """
        axis_map = {'x': 0, 'y': 1, 'z': 2}
        est_col = f'est_{axis}'
        
        # Try standard format first (est_x, est_y, est_z)
        # If not available, try IMM format (est_state_0, est_state_1, est_state_2)
        if est_col not in self.result_data.columns:
            est_col = f'est_state_{axis_map[axis]}'
        
        return est_col
    
    def calculate_errors(self):
        """Calculate position and velocity errors"""
        errors = {}
        
        axes = ['x', 'y'] if self.dim == 2 else ['x', 'y', 'z']
        
        # Position errors
        for axis in axes:
            true_col = f'true_{axis}'
            est_col = self._get_est_col(axis)
            
            if true_col in self.test_data.columns and est_col in self.result_data.columns:
                # convert to numpy arrays to avoid pandas multidimensional-indexing changes
                true_vals = self.test_data[true_col].to_numpy()
                est_vals = self.result_data[est_col].to_numpy()
                
                error = est_vals - true_vals
                errors[f'pos_{axis}_error'] = error
                errors[f'pos_{axis}_rmse'] = np.sqrt(np.mean(error**2))
                errors[f'pos_{axis}_mae'] = np.mean(np.abs(error))
                errors[f'pos_{axis}_std'] = np.std(error)
        
        # Velocity errors (if available)
        for axis in axes:
            true_col = f'true_v{axis}'
            est_col = f'est_v{axis}'
            
            if true_col in self.test_data.columns and est_col in self.result_data.columns:
                # convert to numpy arrays to avoid pandas multidimensional-indexing changes
                true_vals = self.test_data[true_col].to_numpy()
                est_vals = self.result_data[est_col].to_numpy()
                
                error = est_vals - true_vals
                errors[f'vel_{axis}_error'] = error
                errors[f'vel_{axis}_rmse'] = np.sqrt(np.mean(error**2))
                errors[f'vel_{axis}_mae'] = np.mean(np.abs(error))
                errors[f'vel_{axis}_std'] = np.std(error)
        
        # 3D position error magnitude (only if all position errors are available)
        if self.dim == 3:
            if all(f'pos_{axis}_error' in errors for axis in ['x', 'y', 'z']):
                pos_error_3d = np.sqrt(
                    errors['pos_x_error']**2 + 
                    errors['pos_y_error']**2 + 
                    errors['pos_z_error']**2
                )
                errors['pos_3d_error'] = pos_error_3d
                errors['pos_3d_rmse'] = np.sqrt(np.mean(pos_error_3d**2))
                errors['pos_3d_mae'] = np.mean(pos_error_3d)
        else:
            if all(f'pos_{axis}_error' in errors for axis in ['x', 'y']):
                pos_error_2d = np.sqrt(
                    errors['pos_x_error']**2 + 
                    errors['pos_y_error']**2
                )
                errors['pos_2d_error'] = pos_error_2d
                errors['pos_2d_rmse'] = np.sqrt(np.mean(pos_error_2d**2))
                errors['pos_2d_mae'] = np.mean(pos_error_2d)
        
        return errors
    
    def plot_trajectory(self, errors):
        """Plot trajectory comparison"""
        fig = plt.figure(figsize=(12, 10))

        if self.dim == 3:
            # 3D plot
            ax = fig.add_subplot(221, projection='3d')

            # convert relevant series to numpy arrays once
            tx = self.test_data['true_x'].to_numpy()
            ty = self.test_data['true_y'].to_numpy()
            tz = self.test_data['true_z'].to_numpy()

            ex = self.result_data[self._get_est_col('x')].to_numpy()
            ey = self.result_data[self._get_est_col('y')].to_numpy()
            ez = self.result_data[self._get_est_col('z')].to_numpy()

            mx = self.result_data['meas_x'].to_numpy()
            my = self.result_data['meas_y'].to_numpy()
            mz = self.result_data['meas_z'].to_numpy()

            ax.plot(tx, ty, tz, 'g-', label='Ground Truth', linewidth=2)
            ax.plot(ex, ey, ez, 'b--', label='Filtered', linewidth=1.5)
            ax.plot(mx, my, mz, 'r.', label='Measurements', alpha=0.3, markersize=1)
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.set_title('3D Trajectory')
            ax.legend()
            
            # XY projection
            ax2 = fig.add_subplot(222)
            ax2.plot(tx, ty, 'g-', label='Ground Truth', linewidth=2)
            ax2.plot(ex, ey, 'b--', label='Filtered', linewidth=1.5)
            ax2.plot(mx, my, 'r.', label='Measurements', alpha=0.3, markersize=1)
            ax2.set_xlabel('X')
            ax2.set_ylabel('Y')
            ax2.set_title('XY Projection')
            ax2.legend()
            ax2.grid(True)
            
            # Position error over time
            ax3 = fig.add_subplot(223)
            t = self.result_data['time'].to_numpy()
            if 'pos_3d_error' in errors:
                ax3.plot(t, errors['pos_3d_error'], 'b-', linewidth=1)
                ax3.set_xlabel('Time (s)')
                ax3.set_ylabel('Position Error (m)')
                ax3.set_title(f'3D Position Error (RMSE: {errors["pos_3d_rmse"]:.4f})')
                ax3.grid(True)

        else:
            # 2D trajectory
            ax = fig.add_subplot(221)

            tx = self.test_data['true_x'].to_numpy()
            ty = self.test_data['true_y'].to_numpy()

            ex = self.result_data[self._get_est_col('x')].to_numpy()
            ey = self.result_data[self._get_est_col('y')].to_numpy()

            mx = self.result_data['meas_x'].to_numpy()
            my = self.result_data['meas_y'].to_numpy()

            ax.plot(tx, ty, 'g-', label='Ground Truth', linewidth=2)
            ax.plot(ex, ey, 'b--', label='Filtered', linewidth=1.5)
            ax.plot(mx, my, 'r.', label='Measurements', alpha=0.3, markersize=1)
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_title('2D Trajectory')
            ax.legend()
            ax.grid(True)
            ax.axis('equal')
            
            # X position over time
            ax2 = fig.add_subplot(222)
            t = self.result_data['time'].to_numpy()
            ax2.plot(self.test_data['time'].to_numpy(), self.test_data['true_x'].to_numpy(), 'g-', label='True X', linewidth=2)
            ax2.plot(t, self.result_data[self._get_est_col('x')].to_numpy(), 'b--', label='Est X', linewidth=1.5)
            ax2.plot(t, self.result_data['meas_x'].to_numpy(), 'r.', label='Meas X', alpha=0.3, markersize=1)
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('X Position')
            ax2.set_title('X Position vs Time')
            ax2.legend()
            ax2.grid(True)
            
            # Y position over time
            ax3 = fig.add_subplot(223)
            ax3.plot(self.test_data['time'].to_numpy(), self.test_data['true_y'].to_numpy(), 'g-', label='True Y', linewidth=2)
            ax3.plot(t, self.result_data[self._get_est_col('y')].to_numpy(), 'b--', label='Est Y', linewidth=1.5)
            ax3.plot(t, self.result_data['meas_y'].to_numpy(), 'r.', label='Meas Y', alpha=0.3, markersize=1)
            ax3.set_xlabel('Time (s)')
            ax3.set_ylabel('Y Position')
            ax3.set_title('Y Position vs Time')
            ax3.legend()
            ax3.grid(True)
        
        # Error statistics text
        ax4 = fig.add_subplot(224)
        ax4.axis('off')
        
        error_text = "Error Statistics:\n\n"
        axes = ['x', 'y'] if self.dim == 2 else ['x', 'y', 'z']
        
        for axis in axes:
            if f'pos_{axis}_rmse' in errors:
                error_text += f"Position {axis.upper()}:\n"
                error_text += f"  RMSE: {errors[f'pos_{axis}_rmse']:.6f}\n"
                error_text += f"  MAE:  {errors[f'pos_{axis}_mae']:.6f}\n"
                error_text += f"  STD:  {errors[f'pos_{axis}_std']:.6f}\n"
        
        if self.dim == 3:
            error_text += f"\n3D Position:\n"
            error_text += f"  RMSE: {errors['pos_3d_rmse']:.6f}\n"
            error_text += f"  MAE:  {errors['pos_3d_mae']:.6f}\n"
        else:
            error_text += f"\n2D Position:\n"
            error_text += f"  RMSE: {errors['pos_2d_rmse']:.6f}\n"
            error_text += f"  MAE:  {errors['pos_2d_mae']:.6f}\n"
        
        ax4.text(0.1, 0.9, error_text, transform=ax4.transAxes,
                fontsize=10, verticalalignment='top', fontfamily='monospace')
        
        plt.tight_layout()
        
        output_file = self.output_dir / 'trajectory.png'
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Saved trajectory plot: {output_file}")
        plt.close()
    
    def plot_errors(self, errors):
        """Plot error analysis"""
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        
        axes_list = ['x', 'y'] if self.dim == 2 else ['x', 'y', 'z']
        
        # Position errors over time
        ax = axes[0, 0]
        for axis in axes_list:
            if f'pos_{axis}_error' in errors:
                ax.plot(self.result_data['time'].to_numpy(), errors[f'pos_{axis}_error'], 
                       label=f'{axis.upper()} error')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position Error')
        ax.set_title('Position Errors Over Time')
        ax.legend()
        ax.grid(True)
        
        # Velocity errors over time (if available)
        ax = axes[0, 1]
        has_vel = False
        for axis in axes_list:
            if f'vel_{axis}_error' in errors:
                ax.plot(self.result_data['time'].to_numpy(), errors[f'vel_{axis}_error'], 
                        label=f'v{axis.upper()} error')
                has_vel = True
        if has_vel:
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Velocity Error')
            ax.set_title('Velocity Errors Over Time')
            ax.legend()
            ax.grid(True)
        else:
            ax.text(0.5, 0.5, 'No velocity data', ha='center', va='center',
                   transform=ax.transAxes)
        
        # Position error histogram
        ax = axes[1, 0]
        error_key = 'pos_3d_error' if self.dim == 3 else 'pos_2d_error'
        if error_key in errors:
            arr = np.asarray(errors[error_key])
            # skip histogram when all values are NaN or non-finite
            if not np.isfinite(arr).any():
                print(f"Warning: all values for {error_key} are non-finite, skipping histogram and percentiles")
            else:
                ax.hist(arr, bins=50, alpha=0.7, edgecolor='black')
                ax.axvline(errors[error_key.replace('error', 'rmse')], 
                          color='r', linestyle='--', linewidth=2, label='RMSE')
                ax.set_xlabel('Position Error')
                ax.set_ylabel('Frequency')
                ax.set_title('Position Error Distribution')
                ax.legend()
                ax.grid(True, alpha=0.3)
        
        # Error percentiles
        ax = axes[1, 1]
        if error_key in errors:
            arr = np.asarray(errors[error_key])
            if np.isfinite(arr).any():
                percentiles = np.percentile(arr, 
                                           [0, 25, 50, 75, 90, 95, 99, 100])
                labels = ['Min', '25%', '50%', '75%', '90%', '95%', '99%', 'Max']
                bars = ax.bar(labels, percentiles, alpha=0.7, edgecolor='black')
                ax.set_ylabel('Position Error')
                ax.set_title('Error Percentiles')
                ax.grid(True, alpha=0.3, axis='y')
                # Add value labels on bars
                for bar, val in zip(bars, percentiles):
                    height = bar.get_height()
                    ax.text(bar.get_x() + bar.get_width()/2., height,
                           f'{val:.4f}', ha='center', va='bottom', fontsize=8)
            else:
                ax.text(0.5, 0.5, 'No finite errors', ha='center', va='center', transform=ax.transAxes)
            
            
        
        plt.tight_layout()
        
        output_file = self.output_dir / 'errors.png'
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Saved error plot: {output_file}")
        plt.close()
    
    def save_statistics(self, errors):
        """Save error statistics to JSON file"""
        stats = {}
        
        # Position statistics
        axes = ['x', 'y'] if self.dim == 2 else ['x', 'y', 'z']
        for axis in axes:
            if f'pos_{axis}_rmse' in errors:
                stats[f'position_{axis}'] = {
                    'rmse': float(errors[f'pos_{axis}_rmse']),
                    'mae': float(errors[f'pos_{axis}_mae']),
                    'std': float(errors[f'pos_{axis}_std'])
                }
        
        # Overall position error
        error_key = 'pos_3d' if self.dim == 3 else 'pos_2d'
        if f'{error_key}_rmse' in errors:
            stats['position_overall'] = {
                'rmse': float(errors[f'{error_key}_rmse']),
                'mae': float(errors[f'{error_key}_mae'])
            }
        
        # Velocity statistics (if available)
        for axis in axes:
            if f'vel_{axis}_rmse' in errors:
                stats[f'velocity_{axis}'] = {
                    'rmse': float(errors[f'vel_{axis}_rmse']),
                    'mae': float(errors[f'vel_{axis}_mae']),
                    'std': float(errors[f'vel_{axis}_std'])
                }
        
        # Save to JSON
        output_file = self.output_dir / 'statistics.json'
        with open(output_file, 'w') as f:
            json.dump(stats, f, indent=2)
        
        print(f"Saved statistics: {output_file}")
        
        # Also print to console
        print("\n=== Error Statistics ===")
        print(json.dumps(stats, indent=2))
    
    def analyze(self):
        """Run complete analysis"""
        print("\n=== Running Analysis ===\n")
        
        # Calculate errors
        print("Calculating errors...")
        errors = self.calculate_errors()
        
        # Generate plots
        print("Generating plots...")
        self.plot_trajectory(errors)
        self.plot_errors(errors)
        
        # Save statistics
        print("Saving statistics...")
        self.save_statistics(errors)
        
        print(f"\n=== Analysis Complete ===")
        print(f"Results saved to: {self.output_dir}")
    
    def calculate_n_step_prediction_errors(self, n_steps_list=[1, 5, 10, 20, 50]):
        """Calculate N-step ahead prediction errors
        
        Args:
            n_steps_list: List of prediction horizons to evaluate
        
        Returns:
            Dictionary containing prediction errors for each horizon
        """
        print(f"\n=== Calculating N-Step Prediction Errors ===")
        print(f"Prediction horizons: {n_steps_list}")
        
        prediction_results = {}
        axes = ['x', 'y'] if self.dim == 2 else ['x', 'y', 'z']
        
        # Get velocity columns if available
        has_velocity = {}
        for axis in axes:
            vel_col = f'est_v{axis}'
            has_velocity[axis] = vel_col in self.result_data.columns
        
        # For each prediction horizon
        for n in n_steps_list:
            print(f"  Evaluating {n}-step prediction...")
            
            n_step_errors = {
                'horizon': n,
                'num_samples': 0
            }
            
            # Storage for all prediction errors
            all_pred_errors = {axis: [] for axis in axes}
            
            # For each time step where we can make n-step prediction
            for i in range(len(self.result_data) - n):
                # Current estimated state
                current_pos = {}
                current_vel = {}
                
                for axis in axes:
                    current_pos[axis] = self.result_data[self._get_est_col(axis)].iloc[i]
                    if has_velocity[axis]:
                        current_vel[axis] = self.result_data[f'est_v{axis}'].iloc[i]
                
                # Predict n steps ahead (simple linear extrapolation)
                # If velocity is available, use constant velocity model
                # Otherwise, use constant position model
                dt = self.result_data['time'].iloc[i+1] - self.result_data['time'].iloc[i]
                total_dt = n * dt
                
                predicted_pos = {}
                for axis in axes:
                    if has_velocity[axis]:
                        # Constant velocity prediction
                        predicted_pos[axis] = current_pos[axis] + current_vel[axis] * total_dt
                    else:
                        # Constant position prediction
                        predicted_pos[axis] = current_pos[axis]
                
                # Get true position at n steps ahead
                true_pos = {}
                for axis in axes:
                    true_pos[axis] = self.test_data[f'true_{axis}'].iloc[i + n]
                
                # Calculate prediction error
                for axis in axes:
                    pred_error = predicted_pos[axis] - true_pos[axis]
                    all_pred_errors[axis].append(pred_error)
            
            # Calculate statistics
            n_step_errors['num_samples'] = len(all_pred_errors['x'])
            
            for axis in axes:
                errors_array = np.array(all_pred_errors[axis])
                n_step_errors[f'pos_{axis}_rmse'] = float(np.sqrt(np.mean(errors_array**2)))
                n_step_errors[f'pos_{axis}_mae'] = float(np.mean(np.abs(errors_array)))
                n_step_errors[f'pos_{axis}_std'] = float(np.std(errors_array))
                n_step_errors[f'pos_{axis}_errors'] = errors_array  # Store for visualization
            
            # Calculate overall 3D/2D position error
            error_magnitudes = []
            for i in range(len(all_pred_errors['x'])):
                if self.dim == 3:
                    mag = np.sqrt(
                        all_pred_errors['x'][i]**2 +
                        all_pred_errors['y'][i]**2 +
                        all_pred_errors['z'][i]**2
                    )
                else:
                    mag = np.sqrt(
                        all_pred_errors['x'][i]**2 +
                        all_pred_errors['y'][i]**2
                    )
                error_magnitudes.append(mag)
            
            error_magnitudes = np.array(error_magnitudes)
            n_step_errors['pos_overall_rmse'] = float(np.sqrt(np.mean(error_magnitudes**2)))
            n_step_errors['pos_overall_mae'] = float(np.mean(error_magnitudes))
            n_step_errors['pos_overall_std'] = float(np.std(error_magnitudes))
            n_step_errors['pos_overall_errors'] = error_magnitudes
            
            prediction_results[n] = n_step_errors
            
            print(f"    {n}-step RMSE: {n_step_errors['pos_overall_rmse']:.4f} m")
        
        return prediction_results
    
    def plot_n_step_predictions(self, prediction_results):
        """Plot N-step prediction error analysis
        
        Args:
            prediction_results: Dictionary from calculate_n_step_prediction_errors
        """
        if not prediction_results:
            return
        
        fig = plt.figure(figsize=(15, 10))
        
        horizons = sorted(prediction_results.keys())
        axes_list = ['x', 'y'] if self.dim == 2 else ['x', 'y', 'z']
        
        # Plot 1: RMSE vs Prediction Horizon
        ax1 = fig.add_subplot(2, 3, 1)
        overall_rmse = [prediction_results[h]['pos_overall_rmse'] for h in horizons]
        ax1.plot(horizons, overall_rmse, 'o-', linewidth=2, markersize=8)
        ax1.set_xlabel('Prediction Horizon (steps)')
        ax1.set_ylabel('RMSE (m)')
        ax1.set_title('Overall Position RMSE vs Prediction Horizon')
        ax1.grid(True, alpha=0.3)
        
        # Plot 2: MAE vs Prediction Horizon
        ax2 = fig.add_subplot(2, 3, 2)
        overall_mae = [prediction_results[h]['pos_overall_mae'] for h in horizons]
        ax2.plot(horizons, overall_mae, 's-', linewidth=2, markersize=8, color='orange')
        ax2.set_xlabel('Prediction Horizon (steps)')
        ax2.set_ylabel('MAE (m)')
        ax2.set_title('Overall Position MAE vs Prediction Horizon')
        ax2.grid(True, alpha=0.3)
        
        # Plot 3: Per-axis RMSE comparison
        ax3 = fig.add_subplot(2, 3, 3)
        for axis in axes_list:
            axis_rmse = [prediction_results[h][f'pos_{axis}_rmse'] for h in horizons]
            ax3.plot(horizons, axis_rmse, 'o-', linewidth=2, markersize=6, label=f'{axis.upper()}-axis')
        ax3.set_xlabel('Prediction Horizon (steps)')
        ax3.set_ylabel('RMSE (m)')
        ax3.set_title('Per-Axis RMSE vs Prediction Horizon')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # Plot 4-6: Error distributions for selected horizons
        selected_horizons = [horizons[0], horizons[len(horizons)//2], horizons[-1]] if len(horizons) >= 3 else horizons
        
        for idx, h in enumerate(selected_horizons[:3]):
            ax = fig.add_subplot(2, 3, 4 + idx)
            errors = prediction_results[h]['pos_overall_errors']
            
            ax.hist(errors, bins=30, alpha=0.7, edgecolor='black')
            ax.axvline(0, color='r', linestyle='--', linewidth=2, label='Zero Error')
            ax.axvline(np.mean(errors), color='g', linestyle='--', linewidth=2, 
                      label=f'Mean: {np.mean(errors):.3f}m')
            ax.set_xlabel('Prediction Error (m)')
            ax.set_ylabel('Frequency')
            ax.set_title(f'{h}-Step Prediction Error Distribution\nRMSE: {prediction_results[h]["pos_overall_rmse"]:.4f}m')
            ax.legend(fontsize=8)
            ax.grid(True, alpha=0.3, axis='y')
        
        plt.tight_layout()
        output_file = self.output_dir / 'n_step_predictions.png'
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        plt.close()
        
        print(f"Saved N-step prediction plot: {output_file}")
    
    def save_n_step_statistics(self, prediction_results):
        """Save N-step prediction statistics to JSON
        
        Args:
            prediction_results: Dictionary from calculate_n_step_prediction_errors
        """
        # Convert numpy arrays to lists for JSON serialization
        stats = {}
        for horizon, results in prediction_results.items():
            stats[f'{horizon}_steps'] = {
                'horizon': results['horizon'],
                'num_samples': results['num_samples'],
                'overall': {
                    'rmse': results['pos_overall_rmse'],
                    'mae': results['pos_overall_mae'],
                    'std': results['pos_overall_std']
                }
            }
            
            # Add per-axis statistics
            axes = ['x', 'y'] if self.dim == 2 else ['x', 'y', 'z']
            for axis in axes:
                stats[f'{horizon}_steps'][axis] = {
                    'rmse': results[f'pos_{axis}_rmse'],
                    'mae': results[f'pos_{axis}_mae'],
                    'std': results[f'pos_{axis}_std']
                }
        
        output_file = self.output_dir / 'n_step_predictions.json'
        with open(output_file, 'w') as f:
            json.dump(stats, f, indent=2)
        
        print(f"Saved N-step statistics: {output_file}")
        
        # Print summary
        print("\n=== N-Step Prediction Statistics ===")
        for horizon in sorted(prediction_results.keys()):
            print(f"{horizon}-step prediction:")
            print(f"  RMSE: {prediction_results[horizon]['pos_overall_rmse']:.4f} m")
            print(f"  MAE:  {prediction_results[horizon]['pos_overall_mae']:.4f} m")
    
    def analyze_with_predictions(self, n_steps_list=[1, 5, 10, 20, 50]):
        """Run complete analysis including N-step predictions
        
        Args:
            n_steps_list: List of prediction horizons to evaluate
        """
        # Run standard analysis
        self.analyze()
        
        # Add N-step prediction analysis
        prediction_results = self.calculate_n_step_prediction_errors(n_steps_list)
        self.plot_n_step_predictions(prediction_results)
        self.save_n_step_statistics(prediction_results)
        
        print(f"\n=== Complete Analysis Finished ===")
        print(f"All results saved to: {self.output_dir}")



def main():
    parser = argparse.ArgumentParser(
        description='Analyze Kalman filter test results'
    )
    parser.add_argument('test_data', type=str,
                       help='Original test data CSV file (with ground truth)')
    parser.add_argument('result', type=str,
                       help='Filter result CSV file')
    parser.add_argument('--output', '-o', type=str, default='./analysis',
                       help='Output directory for analysis results')
    parser.add_argument('--n-step', '-n', action='store_true',
                       help='Enable N-step prediction analysis')
    parser.add_argument('--horizons', type=int, nargs='+', 
                       default=[1, 5, 10, 20, 50],
                       help='Prediction horizons to evaluate (default: 1 5 10 20 50)')
    
    args = parser.parse_args()
    
    # Run analysis
    analyzer = FilterResultAnalyzer(args.test_data, args.result, args.output)
    
    if args.n_step:
        analyzer.analyze_with_predictions(args.horizons)
    else:
        analyzer.analyze()


if __name__ == '__main__':
    main()
