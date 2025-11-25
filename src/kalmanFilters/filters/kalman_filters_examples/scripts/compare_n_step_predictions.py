#!/usr/bin/env python3
"""
Compare N-step prediction performance across different filters
对比不同滤波器的N步预测性能
"""

import json
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import argparse


def load_n_step_results(analysis_dir):
    """Load N-step prediction results from JSON file"""
    json_path = Path(analysis_dir) / 'n_step_predictions.json'
    if not json_path.exists():
        return None
    
    with open(json_path, 'r') as f:
        return json.load(f)


def extract_metrics(results):
    """Extract RMSE and MAE for each horizon"""
    horizons = []
    rmse_values = []
    mae_values = []
    
    for key in sorted(results.keys(), key=lambda x: int(x.split('_')[0])):
        horizon = results[key]['horizon']
        rmse = results[key]['overall']['rmse']
        mae = results[key]['overall']['mae']
        
        horizons.append(horizon)
        rmse_values.append(rmse)
        mae_values.append(mae)
    
    return horizons, rmse_values, mae_values


def plot_comparison(filter_results, output_path):
    """Create comparison plots for multiple filters"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # Prepare data
    all_data = {}
    for name, results in filter_results.items():
        if results is None:
            continue
        horizons, rmse, mae = extract_metrics(results)
        all_data[name] = {
            'horizons': horizons,
            'rmse': rmse,
            'mae': mae
        }
    
    if not all_data:
        print("No valid results to plot")
        return
    
    # Define colors and markers for different filters
    colors = {
        'CV_KF': '#1f77b4',
        'CA_KF': '#ff7f0e',
        'CS_KF': '#2ca02c',
        'Singer_KF': '#d62728',
        'CTRV_EKF': '#9467bd',
        'IMM': '#8c564b'
    }
    
    markers = {
        'CV_KF': 'o',
        'CA_KF': 's',
        'CS_KF': '^',
        'Singer_KF': 'd',
        'CTRV_EKF': 'v',
        'IMM': 'p'
    }
    
    # Plot 1: RMSE comparison
    ax1 = axes[0, 0]
    for name, data in all_data.items():
        color = colors.get(name, 'gray')
        marker = markers.get(name, 'o')
        ax1.plot(data['horizons'], data['rmse'], 
                marker=marker, linewidth=2, markersize=8,
                label=name, color=color)
    
    ax1.set_xlabel('Prediction Horizon (steps)', fontsize=11)
    ax1.set_ylabel('RMSE (m)', fontsize=11)
    ax1.set_title('RMSE vs Prediction Horizon\n(Lower is Better)', fontsize=12, fontweight='bold')
    ax1.legend(loc='upper left', fontsize=9)
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: MAE comparison
    ax2 = axes[0, 1]
    for name, data in all_data.items():
        color = colors.get(name, 'gray')
        marker = markers.get(name, 'o')
        ax2.plot(data['horizons'], data['mae'], 
                marker=marker, linewidth=2, markersize=8,
                label=name, color=color)
    
    ax2.set_xlabel('Prediction Horizon (steps)', fontsize=11)
    ax2.set_ylabel('MAE (m)', fontsize=11)
    ax2.set_title('MAE vs Prediction Horizon\n(Lower is Better)', fontsize=12, fontweight='bold')
    ax2.legend(loc='upper left', fontsize=9)
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Relative performance (normalized RMSE)
    ax3 = axes[1, 0]
    for name, data in all_data.items():
        color = colors.get(name, 'gray')
        marker = markers.get(name, 'o')
        # Normalize RMSE by 1-step RMSE
        normalized_rmse = np.array(data['rmse']) / data['rmse'][0]
        ax3.plot(data['horizons'], normalized_rmse,
                marker=marker, linewidth=2, markersize=8,
                label=name, color=color)
    
    ax3.set_xlabel('Prediction Horizon (steps)', fontsize=11)
    ax3.set_ylabel('Normalized RMSE (relative to 1-step)', fontsize=11)
    ax3.set_title('Error Growth Rate\n(Flatter is Better)', fontsize=12, fontweight='bold')
    ax3.legend(loc='upper left', fontsize=9)
    ax3.grid(True, alpha=0.3)
    ax3.axhline(y=1.0, color='k', linestyle='--', alpha=0.3)
    
    # Plot 4: Summary table
    ax4 = axes[1, 1]
    ax4.axis('off')
    
    # Create summary table
    table_data = []
    headers = ['Filter', '1-step', '10-step', '20-step', 'Growth']
    
    for name, data in all_data.items():
        horizons = data['horizons']
        rmse = data['rmse']
        
        # Find indices for specific horizons
        idx_1 = horizons.index(1) if 1 in horizons else 0
        idx_10 = horizons.index(10) if 10 in horizons else -1
        idx_20 = horizons.index(20) if 20 in horizons else -1
        
        rmse_1 = f"{rmse[idx_1]:.4f}m"
        rmse_10 = f"{rmse[idx_10]:.4f}m" if idx_10 >= 0 else "N/A"
        rmse_20 = f"{rmse[idx_20]:.4f}m" if idx_20 >= 0 else "N/A"
        
        # Calculate growth rate (RMSE at max horizon / RMSE at 1-step)
        growth = rmse[-1] / rmse[0]
        growth_str = f"{growth:.2f}x"
        
        table_data.append([name, rmse_1, rmse_10, rmse_20, growth_str])
    
    # Sort by 1-step RMSE
    table_data.sort(key=lambda x: float(x[1].replace('m', '')))
    
    table = ax4.table(cellText=table_data, colLabels=headers,
                     cellLoc='center', loc='center',
                     colWidths=[0.15, 0.15, 0.15, 0.15, 0.12])
    
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1.2, 2.0)
    
    # Style header
    for i in range(len(headers)):
        table[(0, i)].set_facecolor('#4CAF50')
        table[(0, i)].set_text_props(weight='bold', color='white')
    
    # Style rows
    for i in range(1, len(table_data) + 1):
        for j in range(len(headers)):
            if i % 2 == 0:
                table[(i, j)].set_facecolor('#f0f0f0')
    
    ax4.set_title('Performance Summary\n(RMSE in meters)', 
                 fontsize=12, fontweight='bold', pad=20)
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Comparison plot saved to: {output_path}")
    
    # Also save to PDF for better quality
    pdf_path = output_path.replace('.png', '.pdf')
    plt.savefig(pdf_path, bbox_inches='tight')
    print(f"PDF version saved to: {pdf_path}")
    
    plt.close()


def main():
    parser = argparse.ArgumentParser(
        description='Compare N-step prediction performance across filters'
    )
    parser.add_argument('--analysis-dir', type=str, 
                       default='./analysis',
                       help='Base directory containing analysis results')
    parser.add_argument('--output', '-o', type=str,
                       default='./analysis/n_step_comparison.png',
                       help='Output path for comparison plot')
    parser.add_argument('--filters', type=str, nargs='+',
                       default=['cv_kf_3d', 'ca_kf_3d', 'cs_kf_3d', 
                               'singer_kf_3d', 'imm_cv_ca_cs_3d'],
                       help='List of filter names to compare')
    
    args = parser.parse_args()
    
    print("=== N-Step Prediction Performance Comparison ===\n")
    
    # Load results for each filter
    filter_results = {}
    for filter_name in args.filters:
        filter_dir = Path(args.analysis_dir) / filter_name
        print(f"Loading {filter_name}...", end=' ')
        
        results = load_n_step_results(filter_dir)
        if results:
            # Map internal names to display names
            display_name = filter_name.upper().replace('_3D', '').replace('_2D', '')
            filter_results[display_name] = results
            print("✓")
        else:
            print(f"✗ (not found or no N-step data)")
    
    print()
    
    if not filter_results:
        print("Error: No valid filter results found")
        print("Please run N-step prediction analysis first:")
        print("  python3 scripts/analyze_results.py <data> <result> --n-step")
        return 1
    
    # Create comparison plot
    print(f"Creating comparison plot with {len(filter_results)} filters...")
    plot_comparison(filter_results, args.output)
    
    print("\n=== Comparison Complete ===")
    return 0


if __name__ == '__main__':
    exit(main())
