#!/usr/bin/env python3
"""
Collaborative Localization Data Processing and Visualization

Generates comparison plots for collaborative localization evaluation:
1. Trajectory Comparison (GT vs Coloc for both robots)
2. Position Error Comparison
3. Yaw Error Comparison
4. Error Statistics Bar Chart
5. Error Distribution Histogram
6. XY Error Scatter Plot

Usage:
    python3 data_processing_coloc.py
    python3 data_processing_coloc.py --timestamp 20251226_182202
    python3 data_processing_coloc.py --timestamp 20251226_182202 --show
"""

import os
import glob
import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime


class ColocDataProcessor:
    """Process and visualize collaborative localization evaluation data."""
    
    def __init__(self, data_dir: str = None):
        """Initialize with data directory."""
        if data_dir is None:
            data_dir = os.path.expanduser('~/ids_roswk/evaluation_results/multibot/coloc')
        self.data_dir = data_dir
        
        # TB3_0 data
        self.tb3_0_data = None
        
        # TB3_1 data
        self.tb3_1_data = None
        
        self.timestamp_suffix = None
        
    def find_latest_files(self) -> bool:
        """Find the latest set of evaluation files."""
        # Find all coloc_eval files to get timestamps
        eval_files = glob.glob(os.path.join(self.data_dir, 'tb3_0_coloc_eval_*.csv'))
        if not eval_files:
            print(f"No evaluation files found in {self.data_dir}")
            return False
        
        # Get the latest timestamp
        latest_file = max(eval_files, key=os.path.getmtime)
        self.timestamp_suffix = os.path.basename(latest_file).replace('tb3_0_coloc_eval_', '').replace('.csv', '')
        print(f"Using timestamp: {self.timestamp_suffix}")
        return True
    
    def load_data(self, timestamp_suffix: str = None) -> bool:
        """Load collaborative localization evaluation data for both robots."""
        if timestamp_suffix:
            self.timestamp_suffix = timestamp_suffix
        elif self.timestamp_suffix is None:
            if not self.find_latest_files():
                return False
        
        try:
            # Load CSV files
            tb3_0_file = os.path.join(self.data_dir, f'tb3_0_coloc_eval_{self.timestamp_suffix}.csv')
            tb3_1_file = os.path.join(self.data_dir, f'tb3_1_coloc_eval_{self.timestamp_suffix}.csv')
            
            self.tb3_0_data = pd.read_csv(tb3_0_file)
            self.tb3_1_data = pd.read_csv(tb3_1_file)
            
            # Convert timestamp to relative time (starting from 0)
            start_time = min(self.tb3_0_data['timestamp'].min(), self.tb3_1_data['timestamp'].min())
            
            self.tb3_0_data['time'] = self.tb3_0_data['timestamp'] - start_time
            self.tb3_1_data['time'] = self.tb3_1_data['timestamp'] - start_time
            
            print(f"Loaded TB3_0: {len(self.tb3_0_data)} samples")
            print(f"Loaded TB3_1: {len(self.tb3_1_data)} samples")
            
            # Print statistics
            self._print_statistics()
            
            return True
            
        except FileNotFoundError as e:
            print(f"File not found: {e}")
            return False
        except Exception as e:
            print(f"Error loading data: {e}")
            return False
    
    def _print_statistics(self):
        """Print error statistics for both robots."""
        print("\n" + "=" * 80)
        print("Error Statistics")
        print("=" * 80)
        
        for robot_name, data in [('TB3_0', self.tb3_0_data), ('TB3_1', self.tb3_1_data)]:
            print(f"\n{robot_name}:")
            print(f"  Position Error (m):")
            print(f"    RMSE: {data['position_error'].pow(2).mean()**0.5:.4f}")
            print(f"    Mean: {data['position_error'].mean():.4f}")
            print(f"    Max:  {data['position_error'].max():.4f}")
            print(f"  Yaw Error (deg):")
            print(f"    RMSE: {data['yaw_error_deg'].pow(2).mean()**0.5:.2f}")
            print(f"    Mean: {abs(data['yaw_error_deg']).mean():.2f}")
            print(f"    Max:  {abs(data['yaw_error_deg']).max():.2f}")
    
    def plot_trajectory_comparison(self):
        """Plot trajectory comparison for both robots."""
        fig, axes = plt.subplots(1, 2, figsize=(16, 6))
        
        for idx, (robot_name, data, ax) in enumerate([
            ('TB3_0', self.tb3_0_data, axes[0]),
            ('TB3_1', self.tb3_1_data, axes[1])
        ]):
            # Plot ground truth
            ax.plot(data['gt_x'], data['gt_y'], 
                   'b-', linewidth=2, label='Ground Truth', alpha=0.7)
            
            # Plot collaborative localization estimate
            ax.plot(data['est_x'], data['est_y'],
                   'r--', linewidth=1.5, label='Coloc Estimate', alpha=0.7)
            
            # Mark start and end
            ax.plot(data['gt_x'].iloc[0], data['gt_y'].iloc[0],
                   'go', markersize=10, label='Start')
            ax.plot(data['gt_x'].iloc[-1], data['gt_y'].iloc[-1],
                   'ro', markersize=10, label='End')
            
            ax.set_xlabel('X (m)', fontsize=12)
            ax.set_ylabel('Y (m)', fontsize=12)
            ax.set_title(f'{robot_name} - Trajectory Comparison', fontsize=14, fontweight='bold')
            ax.legend(fontsize=10)
            ax.grid(True, alpha=0.3)
            ax.axis('equal')
        
        plt.tight_layout()
        return fig
    
    def plot_position_error_comparison(self):
        """Plot position error over time for both robots."""
        fig, ax = plt.subplots(figsize=(14, 6))
        
        ax.plot(self.tb3_0_data['time'], self.tb3_0_data['position_error'],
               'b-', linewidth=1.5, label='TB3_0', alpha=0.7)
        ax.plot(self.tb3_1_data['time'], self.tb3_1_data['position_error'],
               'r-', linewidth=1.5, label='TB3_1', alpha=0.7)
        
        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('Position Error (m)', fontsize=12)
        ax.set_title('Position Error Comparison', fontsize=14, fontweight='bold')
        ax.legend(fontsize=11)
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        return fig
    
    def plot_yaw_error_comparison(self):
        """Plot yaw error over time for both robots."""
        fig, ax = plt.subplots(figsize=(14, 6))
        
        ax.plot(self.tb3_0_data['time'], self.tb3_0_data['yaw_error_deg'],
               'b-', linewidth=1.5, label='TB3_0', alpha=0.7)
        ax.plot(self.tb3_1_data['time'], self.tb3_1_data['yaw_error_deg'],
               'r-', linewidth=1.5, label='TB3_1', alpha=0.7)
        
        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('Yaw Error (deg)', fontsize=12)
        ax.set_title('Yaw Error Comparison', fontsize=14, fontweight='bold')
        ax.legend(fontsize=11)
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color='k', linestyle='--', linewidth=0.8, alpha=0.5)
        
        plt.tight_layout()
        return fig
    
    def plot_statistics_comparison(self):
        """Plot error statistics comparison as bar chart."""
        fig, axes = plt.subplots(1, 2, figsize=(14, 6))
        
        # Calculate statistics
        stats_data = {
            'TB3_0': {
                'pos_rmse': self.tb3_0_data['position_error'].pow(2).mean()**0.5,
                'pos_mean': self.tb3_0_data['position_error'].mean(),
                'pos_max': self.tb3_0_data['position_error'].max(),
                'yaw_rmse': self.tb3_0_data['yaw_error_deg'].pow(2).mean()**0.5,
                'yaw_mean': abs(self.tb3_0_data['yaw_error_deg']).mean(),
                'yaw_max': abs(self.tb3_0_data['yaw_error_deg']).max()
            },
            'TB3_1': {
                'pos_rmse': self.tb3_1_data['position_error'].pow(2).mean()**0.5,
                'pos_mean': self.tb3_1_data['position_error'].mean(),
                'pos_max': self.tb3_1_data['position_error'].max(),
                'yaw_rmse': self.tb3_1_data['yaw_error_deg'].pow(2).mean()**0.5,
                'yaw_mean': abs(self.tb3_1_data['yaw_error_deg']).mean(),
                'yaw_max': abs(self.tb3_1_data['yaw_error_deg']).max()
            }
        }
        
        # Position error statistics
        x = np.arange(3)
        width = 0.35
        
        pos_tb3_0 = [stats_data['TB3_0']['pos_rmse'], 
                     stats_data['TB3_0']['pos_mean'],
                     stats_data['TB3_0']['pos_max']]
        pos_tb3_1 = [stats_data['TB3_1']['pos_rmse'],
                     stats_data['TB3_1']['pos_mean'],
                     stats_data['TB3_1']['pos_max']]
        
        axes[0].bar(x - width/2, pos_tb3_0, width, label='TB3_0', alpha=0.8)
        axes[0].bar(x + width/2, pos_tb3_1, width, label='TB3_1', alpha=0.8)
        axes[0].set_ylabel('Position Error (m)', fontsize=12)
        axes[0].set_title('Position Error Statistics', fontsize=14, fontweight='bold')
        axes[0].set_xticks(x)
        axes[0].set_xticklabels(['RMSE', 'Mean', 'Max'])
        axes[0].legend()
        axes[0].grid(True, alpha=0.3, axis='y')
        
        # Yaw error statistics
        yaw_tb3_0 = [stats_data['TB3_0']['yaw_rmse'],
                     stats_data['TB3_0']['yaw_mean'],
                     stats_data['TB3_0']['yaw_max']]
        yaw_tb3_1 = [stats_data['TB3_1']['yaw_rmse'],
                     stats_data['TB3_1']['yaw_mean'],
                     stats_data['TB3_1']['yaw_max']]
        
        axes[1].bar(x - width/2, yaw_tb3_0, width, label='TB3_0', alpha=0.8)
        axes[1].bar(x + width/2, yaw_tb3_1, width, label='TB3_1', alpha=0.8)
        axes[1].set_ylabel('Yaw Error (deg)', fontsize=12)
        axes[1].set_title('Yaw Error Statistics', fontsize=14, fontweight='bold')
        axes[1].set_xticks(x)
        axes[1].set_xticklabels(['RMSE', 'Mean', 'Max'])
        axes[1].legend()
        axes[1].grid(True, alpha=0.3, axis='y')
        
        plt.tight_layout()
        return fig
    
    def plot_error_distribution(self):
        """Plot error distribution histograms."""
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        
        # Position error distribution
        axes[0, 0].hist(self.tb3_0_data['position_error'], bins=50, alpha=0.7, label='TB3_0', color='blue')
        axes[0, 0].hist(self.tb3_1_data['position_error'], bins=50, alpha=0.7, label='TB3_1', color='red')
        axes[0, 0].set_xlabel('Position Error (m)', fontsize=11)
        axes[0, 0].set_ylabel('Frequency', fontsize=11)
        axes[0, 0].set_title('Position Error Distribution', fontsize=12, fontweight='bold')
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)
        
        # Yaw error distribution
        axes[0, 1].hist(self.tb3_0_data['yaw_error_deg'], bins=50, alpha=0.7, label='TB3_0', color='blue')
        axes[0, 1].hist(self.tb3_1_data['yaw_error_deg'], bins=50, alpha=0.7, label='TB3_1', color='red')
        axes[0, 1].set_xlabel('Yaw Error (deg)', fontsize=11)
        axes[0, 1].set_ylabel('Frequency', fontsize=11)
        axes[0, 1].set_title('Yaw Error Distribution', fontsize=12, fontweight='bold')
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)
        
        # X error distribution
        axes[1, 0].hist(self.tb3_0_data['x_error'], bins=50, alpha=0.7, label='TB3_0', color='blue')
        axes[1, 0].hist(self.tb3_1_data['x_error'], bins=50, alpha=0.7, label='TB3_1', color='red')
        axes[1, 0].set_xlabel('X Error (m)', fontsize=11)
        axes[1, 0].set_ylabel('Frequency', fontsize=11)
        axes[1, 0].set_title('X Error Distribution', fontsize=12, fontweight='bold')
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)
        
        # Y error distribution
        axes[1, 1].hist(self.tb3_0_data['y_error'], bins=50, alpha=0.7, label='TB3_0', color='blue')
        axes[1, 1].hist(self.tb3_1_data['y_error'], bins=50, alpha=0.7, label='TB3_1', color='red')
        axes[1, 1].set_xlabel('Y Error (m)', fontsize=11)
        axes[1, 1].set_ylabel('Frequency', fontsize=11)
        axes[1, 1].set_title('Y Error Distribution', fontsize=12, fontweight='bold')
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        return fig
    
    def plot_xy_error_scatter(self):
        """Plot XY error scatter plot."""
        fig, axes = plt.subplots(1, 2, figsize=(14, 6))
        
        for idx, (robot_name, data, ax) in enumerate([
            ('TB3_0', self.tb3_0_data, axes[0]),
            ('TB3_1', self.tb3_1_data, axes[1])
        ]):
            scatter = ax.scatter(data['x_error'], data['y_error'],
                               c=data['time'], cmap='viridis',
                               s=20, alpha=0.6)
            
            ax.set_xlabel('X Error (m)', fontsize=12)
            ax.set_ylabel('Y Error (m)', fontsize=12)
            ax.set_title(f'{robot_name} - XY Error Scatter', fontsize=14, fontweight='bold')
            ax.grid(True, alpha=0.3)
            ax.axhline(y=0, color='k', linestyle='--', linewidth=0.8, alpha=0.5)
            ax.axvline(x=0, color='k', linestyle='--', linewidth=0.8, alpha=0.5)
            ax.axis('equal')
            
            # Add colorbar
            cbar = plt.colorbar(scatter, ax=ax)
            cbar.set_label('Time (s)', fontsize=10)
        
        plt.tight_layout()
        return fig
    
    def generate_all_plots(self, output_dir: str = None):
        """Generate and save all plots."""
        if output_dir is None:
            output_dir = os.path.join(self.data_dir, 'plots')
        
        os.makedirs(output_dir, exist_ok=True)
        
        print(f"\nGenerating and saving plots to: {output_dir}")
        
        # Generate plots
        plots = [
            ('trajectory_comparison', self.plot_trajectory_comparison()),
            ('position_error_comparison', self.plot_position_error_comparison()),
            ('yaw_error_comparison', self.plot_yaw_error_comparison()),
            ('statistics_comparison', self.plot_statistics_comparison()),
            ('error_distribution', self.plot_error_distribution()),
            ('xy_error_scatter', self.plot_xy_error_scatter())
        ]
        
        # Save plots
        for name, fig in plots:
            filename = f'{name}_{self.timestamp_suffix}.png'
            filepath = os.path.join(output_dir, filename)
            fig.savefig(filepath, dpi=300, bbox_inches='tight')
            print(f"  Saved: {filename}")
            plt.close(fig)
        
        print("\nAll plots generated successfully!")


def main():
    parser = argparse.ArgumentParser(
        description='Process collaborative localization evaluation data')
    parser.add_argument('--data-dir', type=str, default=None,
                        help='Data directory (default: ~/ids_roswk/evaluation_results/multibot/coloc)')
    parser.add_argument('--timestamp', type=str, default=None,
                        help='Specific timestamp suffix (e.g., 20251226_182202)')
    parser.add_argument('--output-dir', type=str, default=None,
                        help='Output directory for plots (default: <data-dir>/plots)')
    parser.add_argument('--show', action='store_true',
                        help='Show plots interactively instead of saving')
    
    args = parser.parse_args()
    
    print("=" * 80)
    print("Collaborative Localization Evaluation - Data Processor")
    print("=" * 80)
    
    processor = ColocDataProcessor(args.data_dir)
    
    if not processor.load_data(args.timestamp):
        print("\nError: Failed to load data. Please check the data directory and files.")
        return 1
    
    print("\nData loaded successfully!")
    
    if args.show:
        print("\nDisplaying plots interactively...")
        processor.plot_trajectory_comparison()
        processor.plot_position_error_comparison()
        processor.plot_yaw_error_comparison()
        processor.plot_statistics_comparison()
        processor.plot_error_distribution()
        processor.plot_xy_error_scatter()
        plt.show()
    else:
        processor.generate_all_plots(args.output_dir)
    
    print("\n" + "=" * 80)
    print("Processing complete!")
    print("=" * 80)
    
    return 0


if __name__ == '__main__':
    exit(main())

