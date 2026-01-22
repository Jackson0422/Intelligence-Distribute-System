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
    
    def __init__(self, data_dir: str = None, num_robots: int = 2):
        """Initialize with data directory and number of robots."""
        if data_dir is None:
            data_dir = os.path.expanduser('~/ids_roswk/evaluation_results/multibot/coloc')
        self.data_dir = data_dir
        self.num_robots = num_robots
        
        # Robot data dictionary
        self.robot_data = {}
        self.robot_ids = [f'tb3_{i}' if i > 0 else 'tb3_1' 
                          for i in range(num_robots)]
        
        self.timestamp_suffix = None
        
    def find_latest_files(self) -> bool:
        """Find the latest set of evaluation files."""
        # Find all coloc_eval files to get timestamps
        eval_files = glob.glob(os.path.join(self.data_dir, 'tb3_1_coloc_eval_*.csv'))
        if not eval_files:
            print(f"No evaluation files found in {self.data_dir}")
            return False
        
        # Get the latest timestamp
        latest_file = max(eval_files, key=os.path.getmtime)
        self.timestamp_suffix = os.path.basename(latest_file).replace('tb3_1_coloc_eval_', '').replace('.csv', '')
        print(f"Using timestamp: {self.timestamp_suffix}")
        return True
    
    def load_data(self, timestamp_suffix: str = None) -> bool:
        """Load collaborative localization evaluation data for all robots."""
        if timestamp_suffix:
            self.timestamp_suffix = timestamp_suffix
        elif self.timestamp_suffix is None:
            if not self.find_latest_files():
                return False
        
        try:
            # Load CSV files for all robots
            for robot_id in self.robot_ids:
                file_path = os.path.join(self.data_dir, f'{robot_id}_coloc_eval_{self.timestamp_suffix}.csv')
                self.robot_data[robot_id] = pd.read_csv(file_path)
                print(f"Loaded {robot_id.upper()}: {len(self.robot_data[robot_id])} samples")
            
            # Convert timestamp to relative time (starting from 0)
            start_time = min([data['timestamp'].min() for data in self.robot_data.values()])
            
            for robot_id in self.robot_ids:
                self.robot_data[robot_id]['time'] = self.robot_data[robot_id]['timestamp'] - start_time
            
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
        """Print error statistics for all robots."""
        print("\n" + "=" * 80)
        print("Error Statistics")
        print("=" * 80)
        
        for robot_id in self.robot_ids:
            data = self.robot_data[robot_id]
            print(f"\n{robot_id.upper()}:")
            print(f"  Position Error (m):")
            print(f"    RMSE: {data['position_error'].pow(2).mean()**0.5:.4f}")
            print(f"    Mean: {data['position_error'].mean():.4f}")
            print(f"    Max:  {data['position_error'].max():.4f}")
            print(f"  Yaw Error (deg):")
            print(f"    RMSE: {data['yaw_error_deg'].pow(2).mean()**0.5:.2f}")
            print(f"    Mean: {abs(data['yaw_error_deg']).mean():.2f}")
            print(f"    Max:  {abs(data['yaw_error_deg']).max():.2f}")
    
    def plot_trajectory_comparison(self):
        """Plot trajectory comparison for all robots."""
        ncols = min(self.num_robots, 3)  # Max 3 columns
        nrows = (self.num_robots + ncols - 1) // ncols  # Calculate needed rows
        
        fig, axes = plt.subplots(nrows, ncols, figsize=(6*ncols, 6*nrows))
        if nrows == 1 and ncols == 1:
            axes = np.array([axes])
        elif nrows == 1 or ncols == 1:
            axes = axes.flatten()
        else:
            axes = axes.flatten()
        
        for idx, robot_id in enumerate(self.robot_ids):
            data = self.robot_data[robot_id]
            ax = axes[idx]
            robot_name = robot_id.upper()
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
        """Plot position error over time for all robots."""
        fig, ax = plt.subplots(figsize=(14, 6))
        
        colors = ['b', 'r', 'g', 'orange']  # Colors for up to 4 robots
        for idx, robot_id in enumerate(self.robot_ids):
            data = self.robot_data[robot_id]
            ax.plot(data['time'], data['position_error'],
                   f'{colors[idx]}-', linewidth=1.5, label=robot_id.upper(), alpha=0.7)
        
        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('Position Error (m)', fontsize=12)
        ax.set_title('Position Error Comparison', fontsize=14, fontweight='bold')
        ax.legend(fontsize=11)
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        return fig
    
    def plot_yaw_error_comparison(self):
        """Plot yaw error over time for all robots."""
        fig, ax = plt.subplots(figsize=(14, 6))
        
        colors = ['b', 'r', 'g', 'orange']  # Colors for up to 4 robots
        for idx, robot_id in enumerate(self.robot_ids):
            data = self.robot_data[robot_id]
            ax.plot(data['time'], data['yaw_error_deg'],
                   f'{colors[idx]}-', linewidth=1.5, label=robot_id.upper(), alpha=0.7)
        
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
        
        # Calculate statistics for all robots
        stats_data = {}
        for robot_id in self.robot_ids:
            data = self.robot_data[robot_id]
            stats_data[robot_id.upper()] = {
                'pos_rmse': data['position_error'].pow(2).mean()**0.5,
                'pos_mean': data['position_error'].mean(),
                'pos_max': data['position_error'].max(),
                'yaw_rmse': data['yaw_error_deg'].pow(2).mean()**0.5,
                'yaw_mean': abs(data['yaw_error_deg']).mean(),
                'yaw_max': abs(data['yaw_error_deg']).max()
            }
        
        # Position error statistics
        x = np.arange(3)  # RMSE, Mean, Max
        width = 0.8 / self.num_robots  # Dynamic width based on number of robots
        colors = ['blue', 'red', 'green', 'orange']
        
        for idx, robot_id in enumerate(self.robot_ids):
            robot_name = robot_id.upper()
            pos_values = [stats_data[robot_name]['pos_rmse'], 
                         stats_data[robot_name]['pos_mean'],
                         stats_data[robot_name]['pos_max']]
            offset = width * (idx - self.num_robots/2 + 0.5)
            axes[0].bar(x + offset, pos_values, width, 
                       label=robot_name, alpha=0.8, color=colors[idx])
        
        axes[0].set_ylabel('Position Error (m)', fontsize=12)
        axes[0].set_title('Position Error Statistics', fontsize=14, fontweight='bold')
        axes[0].set_xticks(x)
        axes[0].set_xticklabels(['RMSE', 'Mean', 'Max'])
        axes[0].legend()
        axes[0].grid(True, alpha=0.3, axis='y')
        
        # Yaw error statistics
        for idx, robot_id in enumerate(self.robot_ids):
            robot_name = robot_id.upper()
            yaw_values = [stats_data[robot_name]['yaw_rmse'],
                         stats_data[robot_name]['yaw_mean'],
                         stats_data[robot_name]['yaw_max']]
            offset = width * (idx - self.num_robots/2 + 0.5)
            axes[1].bar(x + offset, yaw_values, width,
                       label=robot_name, alpha=0.8, color=colors[idx])
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
        
        colors = ['blue', 'red', 'green', 'orange']
        
        # Position error distribution
        for idx, robot_id in enumerate(self.robot_ids):
            data = self.robot_data[robot_id]
            axes[0, 0].hist(data['position_error'], bins=50, alpha=0.7, 
                           label=robot_id.upper(), color=colors[idx])
        axes[0, 0].set_xlabel('Position Error (m)', fontsize=11)
        axes[0, 0].set_ylabel('Frequency', fontsize=11)
        axes[0, 0].set_title('Position Error Distribution', fontsize=12, fontweight='bold')
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)
        
        # Yaw error distribution
        for idx, robot_id in enumerate(self.robot_ids):
            data = self.robot_data[robot_id]
            axes[0, 1].hist(data['yaw_error_deg'], bins=50, alpha=0.7,
                           label=robot_id.upper(), color=colors[idx])
        axes[0, 1].set_xlabel('Yaw Error (deg)', fontsize=11)
        axes[0, 1].set_ylabel('Frequency', fontsize=11)
        axes[0, 1].set_title('Yaw Error Distribution', fontsize=12, fontweight='bold')
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)
        
        # X error distribution
        for idx, robot_id in enumerate(self.robot_ids):
            data = self.robot_data[robot_id]
            axes[1, 0].hist(data['x_error'], bins=50, alpha=0.7,
                           label=robot_id.upper(), color=colors[idx])
        axes[1, 0].set_xlabel('X Error (m)', fontsize=11)
        axes[1, 0].set_ylabel('Frequency', fontsize=11)
        axes[1, 0].set_title('X Error Distribution', fontsize=12, fontweight='bold')
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)
        
        # Y error distribution
        for idx, robot_id in enumerate(self.robot_ids):
            data = self.robot_data[robot_id]
            axes[1, 1].hist(data['y_error'], bins=50, alpha=0.7,
                           label=robot_id.upper(), color=colors[idx])
        axes[1, 1].set_xlabel('Y Error (m)', fontsize=11)
        axes[1, 1].set_ylabel('Frequency', fontsize=11)
        axes[1, 1].set_title('Y Error Distribution', fontsize=12, fontweight='bold')
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        return fig
    
    def plot_xy_error_scatter(self):
        """Plot XY error scatter plot."""
        ncols = min(self.num_robots, 3)
        nrows = (self.num_robots + ncols - 1) // ncols
        
        fig, axes = plt.subplots(nrows, ncols, figsize=(7*ncols, 6*nrows))
        if nrows == 1 and ncols == 1:
            axes = np.array([axes])
        elif nrows == 1 or ncols == 1:
            axes = axes.flatten()
        else:
            axes = axes.flatten()
        
        for idx, robot_id in enumerate(self.robot_ids):
            data = self.robot_data[robot_id]
            ax = axes[idx]
            robot_name = robot_id.upper()
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
    parser.add_argument('--num-robots', type=int, default=3, choices=[2, 3, 4],
                        help='Number of robots (2-4, default: 3)')
    parser.add_argument('--show', action='store_true',
                        help='Show plots interactively instead of saving')
    
    args = parser.parse_args()
    
    print("=" * 80)
    print(f"Collaborative Localization Evaluation - Data Processor ({args.num_robots} Robots)")
    print("=" * 80)
    
    processor = ColocDataProcessor(args.data_dir, args.num_robots)
    
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

