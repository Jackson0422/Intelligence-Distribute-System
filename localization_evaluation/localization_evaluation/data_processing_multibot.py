#!/usr/bin/env python3
"""
Multi-Robot Data Processing and Visualization for Localization Evaluation

Generates comparison plots for two robots (tb3_1 and tb3_2):
1. Trajectory Comparison (GT vs AMCL for both robots)
2. Position Error Comparison
3. Yaw Error Comparison
4. Error Statistics Bar Chart
5. Error Distribution Histogram
6. XY Error Scatter Plot
"""

import os
import glob
import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime


class MultibotDataProcessor:
    """Process and visualize multi-robot localization evaluation data."""
    
    def __init__(self, data_dir: str = None):
        """Initialize with data directory."""
        if data_dir is None:
            data_dir = os.path.expanduser('~/ids_roswk/evaluation_results/multibot')
        self.data_dir = data_dir
        
        # tb3_1 data
        self.tb3_1_gt = None
        self.tb3_1_est = None
        self.tb3_1_err = None
        
        # tb3_2 data
        self.tb3_2_gt = None
        self.tb3_2_est = None
        self.tb3_2_err = None
        
        self.timestamp_suffix = None
        
    def find_latest_files(self) -> bool:
        """Find the latest set of evaluation files."""
        # Find all statistics files to get timestamps
        stat_files = glob.glob(os.path.join(self.data_dir, 'tb3_1_statistics_*.txt'))
        if not stat_files:
            print(f"No evaluation files found in {self.data_dir}")
            return False
        
        # Get the latest timestamp
        latest_file = max(stat_files, key=os.path.getmtime)
        self.timestamp_suffix = os.path.basename(latest_file).replace('tb3_1_statistics_', '').replace('.txt', '')
        print(f"Using data from timestamp: {self.timestamp_suffix}")
        return True
    
    def load_data(self, timestamp_suffix: str = None) -> bool:
        """Load ground truth, estimated, and error data for both robots."""
        if timestamp_suffix:
            self.timestamp_suffix = timestamp_suffix
        elif self.timestamp_suffix is None:
            if not self.find_latest_files():
                return False
        
        try:
            # tb3_1 files
            tb3_1_gt_file = os.path.join(self.data_dir, f'tb3_1_ground_truth_{self.timestamp_suffix}.csv')
            tb3_1_est_file = os.path.join(self.data_dir, f'tb3_1_estimated_{self.timestamp_suffix}.csv')
            tb3_1_err_file = os.path.join(self.data_dir, f'tb3_1_errors_{self.timestamp_suffix}.csv')
            
            # tb3_2 files
            tb3_2_gt_file = os.path.join(self.data_dir, f'tb3_2_ground_truth_{self.timestamp_suffix}.csv')
            tb3_2_est_file = os.path.join(self.data_dir, f'tb3_2_estimated_{self.timestamp_suffix}.csv')
            tb3_2_err_file = os.path.join(self.data_dir, f'tb3_2_errors_{self.timestamp_suffix}.csv')
            
            # Load tb3_1 data
            self.tb3_1_gt = pd.read_csv(tb3_1_gt_file)
            self.tb3_1_est = pd.read_csv(tb3_1_est_file)
            self.tb3_1_err = pd.read_csv(tb3_1_err_file)
            
            # Load tb3_2 data
            self.tb3_2_gt = pd.read_csv(tb3_2_gt_file)
            self.tb3_2_est = pd.read_csv(tb3_2_est_file)
            self.tb3_2_err = pd.read_csv(tb3_2_err_file)
            
            # Convert timestamp to relative time (starting from 0)
            start_time = min(self.tb3_1_gt['timestamp'].min(), self.tb3_2_gt['timestamp'].min())
            
            self.tb3_1_gt['time'] = self.tb3_1_gt['timestamp'] - start_time
            self.tb3_1_est['time'] = self.tb3_1_est['timestamp'] - start_time
            self.tb3_1_err['time'] = self.tb3_1_err['timestamp'] - start_time
            
            self.tb3_2_gt['time'] = self.tb3_2_gt['timestamp'] - start_time
            self.tb3_2_est['time'] = self.tb3_2_est['timestamp'] - start_time
            self.tb3_2_err['time'] = self.tb3_2_err['timestamp'] - start_time
            
            print(f"Loaded tb3_1: {len(self.tb3_1_gt)} GT, {len(self.tb3_1_est)} estimated, {len(self.tb3_1_err)} error samples")
            print(f"Loaded tb3_2: {len(self.tb3_2_gt)} GT, {len(self.tb3_2_est)} estimated, {len(self.tb3_2_err)} error samples")
            return True
            
        except Exception as e:
            print(f"Error loading data: {e}")
            return False
    
    def plot_trajectory_comparison(self, save_path: str = None):
        """Plot GT and AMCL trajectories for both robots on the same figure."""
        if self.tb3_1_gt is None or self.tb3_2_gt is None:
            print("Data not loaded. Call load_data() first.")
            return
        
        fig, ax = plt.subplots(figsize=(12, 10))
        
        # Plot tb3_1 trajectories
        ax.plot(self.tb3_1_gt['x'], self.tb3_1_gt['y'], 
                'b-', linewidth=2.0, label='tb3_1 Ground Truth', alpha=0.9)
        ax.plot(self.tb3_1_est['x'], self.tb3_1_est['y'], 
                'b--', linewidth=1.5, label='tb3_1 AMCL', alpha=0.7)
        
        # Plot tb3_2 trajectories
        ax.plot(self.tb3_2_gt['x'], self.tb3_2_gt['y'], 
                'r-', linewidth=2.0, label='tb3_2 Ground Truth', alpha=0.9)
        ax.plot(self.tb3_2_est['x'], self.tb3_2_est['y'], 
                'r--', linewidth=1.5, label='tb3_2 AMCL', alpha=0.7)
        
        # Mark start points
        ax.scatter(self.tb3_1_gt['x'].iloc[0], self.tb3_1_gt['y'].iloc[0], 
                   c='green', s=150, marker='o', zorder=5, edgecolors='black', linewidths=1.5)
        ax.scatter(self.tb3_2_gt['x'].iloc[0], self.tb3_2_gt['y'].iloc[0], 
                   c='green', s=150, marker='o', zorder=5, edgecolors='black', linewidths=1.5, 
                   label='Start Position')
        
        # Mark end points
        ax.scatter(self.tb3_1_gt['x'].iloc[-1], self.tb3_1_gt['y'].iloc[-1], 
                   c='purple', s=150, marker='s', zorder=5, edgecolors='black', linewidths=1.5)
        ax.scatter(self.tb3_2_gt['x'].iloc[-1], self.tb3_2_gt['y'].iloc[-1], 
                   c='purple', s=150, marker='s', zorder=5, edgecolors='black', linewidths=1.5,
                   label='End Position')
        
        ax.set_xlabel('X Position (m)', fontsize=14, fontweight='bold')
        ax.set_ylabel('Y Position (m)', fontsize=14, fontweight='bold')
        ax.set_title('Multi-Robot Trajectory Comparison: Ground Truth vs AMCL', 
                     fontsize=16, fontweight='bold', pad=20)
        ax.legend(loc='best', fontsize=11, framealpha=0.9)
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.set_aspect('equal')
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Saved trajectory comparison plot to: {save_path}")
        else:
            plt.show()
        
        plt.close()
    
    def plot_position_error_comparison(self, save_path: str = None):
        """Plot position error comparison for both robots."""
        if self.tb3_1_err is None or self.tb3_2_err is None:
            print("Data not loaded. Call load_data() first.")
            return
        
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
        
        # tb3_1 position error
        ax1.plot(self.tb3_1_err['time'], self.tb3_1_err['position_error'], 
                 'b-', linewidth=1.2, alpha=0.8, label='tb3_1 Position Error')
        ax1.fill_between(self.tb3_1_err['time'], 0, self.tb3_1_err['position_error'], 
                         alpha=0.3, color='blue')
        
        # Calculate tb3_1 statistics
        rmse_0 = np.sqrt(np.mean(self.tb3_1_err['position_error']**2))
        mean_0 = self.tb3_1_err['position_error'].mean()
        
        ax1.axhline(y=rmse_0, color='red', linestyle='--', linewidth=2, 
                    label=f'RMSE: {rmse_0:.4f} m')
        ax1.axhline(y=mean_0, color='orange', linestyle=':', linewidth=2, 
                    label=f'Mean: {mean_0:.4f} m')
        
        ax1.set_ylabel('Position Error (m)', fontsize=12, fontweight='bold')
        ax1.set_title('tb3_1 Position Error vs Time', fontsize=13, fontweight='bold')
        ax1.legend(loc='upper right', fontsize=10)
        ax1.grid(True, alpha=0.3)
        ax1.set_ylim(bottom=0)
        
        # tb3_2 position error
        ax2.plot(self.tb3_2_err['time'], self.tb3_2_err['position_error'], 
                 'r-', linewidth=1.2, alpha=0.8, label='tb3_2 Position Error')
        ax2.fill_between(self.tb3_2_err['time'], 0, self.tb3_2_err['position_error'], 
                         alpha=0.3, color='red')
        
        # Calculate tb3_2 statistics
        rmse_1 = np.sqrt(np.mean(self.tb3_2_err['position_error']**2))
        mean_1 = self.tb3_2_err['position_error'].mean()
        
        ax2.axhline(y=rmse_1, color='darkred', linestyle='--', linewidth=2, 
                    label=f'RMSE: {rmse_1:.4f} m')
        ax2.axhline(y=mean_1, color='orange', linestyle=':', linewidth=2, 
                    label=f'Mean: {mean_1:.4f} m')
        
        ax2.set_xlabel('Time (s)', fontsize=12, fontweight='bold')
        ax2.set_ylabel('Position Error (m)', fontsize=12, fontweight='bold')
        ax2.set_title('tb3_2 Position Error vs Time', fontsize=13, fontweight='bold')
        ax2.legend(loc='upper right', fontsize=10)
        ax2.grid(True, alpha=0.3)
        ax2.set_xlim(left=0)
        ax2.set_ylim(bottom=0)
        
        plt.suptitle('Position Error Comparison', fontsize=15, fontweight='bold', y=0.995)
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Saved position error comparison plot to: {save_path}")
        else:
            plt.show()
        
        plt.close()
    
    def plot_yaw_error_comparison(self, save_path: str = None):
        """Plot yaw error comparison for both robots."""
        if self.tb3_1_err is None or self.tb3_2_err is None:
            print("Data not loaded. Call load_data() first.")
            return
        
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
        
        # Convert to degrees
        yaw_error_0_deg = np.degrees(self.tb3_1_err['yaw_error'])
        yaw_error_1_deg = np.degrees(self.tb3_2_err['yaw_error'])
        
        # tb3_1 yaw error
        ax1.plot(self.tb3_1_err['time'], yaw_error_0_deg, 
                 'b-', linewidth=1.2, alpha=0.8, label='tb3_1 Yaw Error')
        ax1.fill_between(self.tb3_1_err['time'], 0, yaw_error_0_deg, 
                         where=(yaw_error_0_deg >= 0), alpha=0.3, color='blue')
        ax1.fill_between(self.tb3_1_err['time'], 0, yaw_error_0_deg, 
                         where=(yaw_error_0_deg < 0), alpha=0.3, color='cyan')
        
        # Calculate tb3_1 statistics
        rmse_0 = np.sqrt(np.mean(yaw_error_0_deg**2))
        ax1.axhline(y=rmse_0, color='red', linestyle='--', linewidth=2, label=f'RMSE: {rmse_0:.2f}°')
        ax1.axhline(y=-rmse_0, color='red', linestyle='--', linewidth=2)
        
        ax1.set_ylabel('Yaw Error (degrees)', fontsize=12, fontweight='bold')
        ax1.set_title('tb3_1 Yaw Error vs Time', fontsize=13, fontweight='bold')
        ax1.legend(loc='upper right', fontsize=10)
        ax1.grid(True, alpha=0.3)
        ax1.axhline(y=0, color='black', linestyle='-', linewidth=0.8, alpha=0.5)
        
        # tb3_2 yaw error
        ax2.plot(self.tb3_2_err['time'], yaw_error_1_deg, 
                 'r-', linewidth=1.2, alpha=0.8, label='tb3_2 Yaw Error')
        ax2.fill_between(self.tb3_2_err['time'], 0, yaw_error_1_deg, 
                         where=(yaw_error_1_deg >= 0), alpha=0.3, color='red')
        ax2.fill_between(self.tb3_2_err['time'], 0, yaw_error_1_deg, 
                         where=(yaw_error_1_deg < 0), alpha=0.3, color='orange')
        
        # Calculate tb3_2 statistics
        rmse_1 = np.sqrt(np.mean(yaw_error_1_deg**2))
        ax2.axhline(y=rmse_1, color='darkred', linestyle='--', linewidth=2, label=f'RMSE: {rmse_1:.2f}°')
        ax2.axhline(y=-rmse_1, color='darkred', linestyle='--', linewidth=2)
        
        ax2.set_xlabel('Time (s)', fontsize=12, fontweight='bold')
        ax2.set_ylabel('Yaw Error (degrees)', fontsize=12, fontweight='bold')
        ax2.set_title('tb3_2 Yaw Error vs Time', fontsize=13, fontweight='bold')
        ax2.legend(loc='upper right', fontsize=10)
        ax2.grid(True, alpha=0.3)
        ax2.set_xlim(left=0)
        ax2.axhline(y=0, color='black', linestyle='-', linewidth=0.8, alpha=0.5)
        
        plt.suptitle('Yaw Error Comparison', fontsize=15, fontweight='bold', y=0.995)
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Saved yaw error comparison plot to: {save_path}")
        else:
            plt.show()
        
        plt.close()
    
    def plot_statistics_comparison(self, save_path: str = None):
        """Plot bar chart comparing error statistics for both robots."""
        if self.tb3_1_err is None or self.tb3_2_err is None:
            print("Data not loaded. Call load_data() first.")
            return
        
        # Calculate statistics
        pos_rmse_0 = np.sqrt(np.mean(self.tb3_1_err['position_error']**2))
        pos_mean_0 = self.tb3_1_err['position_error'].mean()
        yaw_rmse_0 = np.sqrt(np.mean(np.degrees(self.tb3_1_err['yaw_error'])**2))
        yaw_mean_0 = np.mean(np.abs(np.degrees(self.tb3_1_err['yaw_error'])))
        
        pos_rmse_1 = np.sqrt(np.mean(self.tb3_2_err['position_error']**2))
        pos_mean_1 = self.tb3_2_err['position_error'].mean()
        yaw_rmse_1 = np.sqrt(np.mean(np.degrees(self.tb3_2_err['yaw_error'])**2))
        yaw_mean_1 = np.mean(np.abs(np.degrees(self.tb3_2_err['yaw_error'])))
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
        
        # Position error statistics
        categories = ['RMSE', 'Mean']
        tb3_1_pos = [pos_rmse_0 * 100, pos_mean_0 * 100]  # Convert to cm
        tb3_2_pos = [pos_rmse_1 * 100, pos_mean_1 * 100]
        
        x = np.arange(len(categories))
        width = 0.35
        
        bars1 = ax1.bar(x - width/2, tb3_1_pos, width, label='tb3_1', color='blue', alpha=0.8)
        bars2 = ax1.bar(x + width/2, tb3_2_pos, width, label='tb3_2', color='red', alpha=0.8)
        
        ax1.set_xlabel('Metric', fontsize=12, fontweight='bold')
        ax1.set_ylabel('Position Error (cm)', fontsize=12, fontweight='bold')
        ax1.set_title('Position Error Statistics', fontsize=13, fontweight='bold')
        ax1.set_xticks(x)
        ax1.set_xticklabels(categories)
        ax1.legend(fontsize=11)
        ax1.grid(True, alpha=0.3, axis='y')
        
        # Add value labels on bars
        for bars in [bars1, bars2]:
            for bar in bars:
                height = bar.get_height()
                ax1.text(bar.get_x() + bar.get_width()/2., height,
                        f'{height:.2f}',
                        ha='center', va='bottom', fontsize=9)
        
        # Yaw error statistics
        tb3_1_yaw = [yaw_rmse_0, yaw_mean_0]
        tb3_2_yaw = [yaw_rmse_1, yaw_mean_1]
        
        bars3 = ax2.bar(x - width/2, tb3_1_yaw, width, label='tb3_1', color='blue', alpha=0.8)
        bars4 = ax2.bar(x + width/2, tb3_2_yaw, width, label='tb3_2', color='red', alpha=0.8)
        
        ax2.set_xlabel('Metric', fontsize=12, fontweight='bold')
        ax2.set_ylabel('Yaw Error (degrees)', fontsize=12, fontweight='bold')
        ax2.set_title('Yaw Error Statistics', fontsize=13, fontweight='bold')
        ax2.set_xticks(x)
        ax2.set_xticklabels(categories)
        ax2.legend(fontsize=11)
        ax2.grid(True, alpha=0.3, axis='y')
        
        # Add value labels on bars
        for bars in [bars3, bars4]:
            for bar in bars:
                height = bar.get_height()
                ax2.text(bar.get_x() + bar.get_width()/2., height,
                        f'{height:.2f}',
                        ha='center', va='bottom', fontsize=9)
        
        plt.suptitle('Error Statistics Comparison', fontsize=15, fontweight='bold')
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Saved statistics comparison plot to: {save_path}")
        else:
            plt.show()
        
        plt.close()
    
    def plot_error_distribution(self, save_path: str = None):
        """Plot histogram of error distribution for both robots."""
        if self.tb3_1_err is None or self.tb3_2_err is None:
            print("Data not loaded. Call load_data() first.")
            return
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
        
        # Position error distribution
        pos_err_0 = self.tb3_1_err['position_error'] * 100  # Convert to cm
        pos_err_1 = self.tb3_2_err['position_error'] * 100
        
        ax1.hist(pos_err_0, bins=30, alpha=0.6, color='blue', label='tb3_1', edgecolor='black')
        ax1.hist(pos_err_1, bins=30, alpha=0.6, color='red', label='tb3_2', edgecolor='black')
        
        ax1.set_xlabel('Position Error (cm)', fontsize=12, fontweight='bold')
        ax1.set_ylabel('Frequency', fontsize=12, fontweight='bold')
        ax1.set_title('Position Error Distribution', fontsize=13, fontweight='bold')
        ax1.legend(fontsize=11)
        ax1.grid(True, alpha=0.3, axis='y')
        
        # Yaw error distribution
        yaw_err_0 = np.degrees(self.tb3_1_err['yaw_error'])
        yaw_err_1 = np.degrees(self.tb3_2_err['yaw_error'])
        
        ax2.hist(yaw_err_0, bins=30, alpha=0.6, color='blue', label='tb3_1', edgecolor='black')
        ax2.hist(yaw_err_1, bins=30, alpha=0.6, color='red', label='tb3_2', edgecolor='black')
        
        ax2.set_xlabel('Yaw Error (degrees)', fontsize=12, fontweight='bold')
        ax2.set_ylabel('Frequency', fontsize=12, fontweight='bold')
        ax2.set_title('Yaw Error Distribution', fontsize=13, fontweight='bold')
        ax2.legend(fontsize=11)
        ax2.grid(True, alpha=0.3, axis='y')
        
        plt.suptitle('Error Distribution Comparison', fontsize=15, fontweight='bold')
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Saved error distribution plot to: {save_path}")
        else:
            plt.show()
        
        plt.close()
    
    def plot_xy_error_scatter(self, save_path: str = None):
        """Plot XY error scatter plot for both robots."""
        if self.tb3_1_err is None or self.tb3_2_err is None:
            print("Data not loaded. Call load_data() first.")
            return
        
        fig, ax = plt.subplots(figsize=(10, 10))
        
        # Convert to cm
        x_err_0 = self.tb3_1_err['x_error'] * 100
        y_err_0 = self.tb3_1_err['y_error'] * 100
        x_err_1 = self.tb3_2_err['x_error'] * 100
        y_err_1 = self.tb3_2_err['y_error'] * 100
        
        # Scatter plots
        ax.scatter(x_err_0, y_err_0, c='blue', alpha=0.5, s=20, label='tb3_1', edgecolors='none')
        ax.scatter(x_err_1, y_err_1, c='red', alpha=0.5, s=20, label='tb3_2', edgecolors='none')
        
        # Add origin lines
        ax.axhline(y=0, color='black', linestyle='-', linewidth=0.8, alpha=0.5)
        ax.axvline(x=0, color='black', linestyle='-', linewidth=0.8, alpha=0.5)
        
        # Add circles for reference
        circle_radii = [5, 10, 15]  # cm
        for radius in circle_radii:
            circle = plt.Circle((0, 0), radius, fill=False, edgecolor='gray', 
                              linestyle='--', linewidth=1, alpha=0.3)
            ax.add_patch(circle)
            ax.text(radius/np.sqrt(2), radius/np.sqrt(2), f'{radius}cm', 
                   fontsize=9, color='gray', alpha=0.7)
        
        ax.set_xlabel('X Error (cm)', fontsize=12, fontweight='bold')
        ax.set_ylabel('Y Error (cm)', fontsize=12, fontweight='bold')
        ax.set_title('XY Error Scatter Plot', fontsize=14, fontweight='bold', pad=15)
        ax.legend(fontsize=11, loc='upper right')
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        
        # Set limits based on data
        max_err = max(
            np.abs(x_err_0).max(), np.abs(y_err_0).max(),
            np.abs(x_err_1).max(), np.abs(y_err_1).max()
        )
        limit = max(20, max_err * 1.2)  # At least 20cm, or 120% of max error
        ax.set_xlim(-limit, limit)
        ax.set_ylim(-limit, limit)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Saved XY error scatter plot to: {save_path}")
        else:
            plt.show()
        
        plt.close()
    
    def generate_all_plots(self, output_dir: str = None):
        """Generate all plots and save to output directory."""
        if output_dir is None:
            output_dir = os.path.join(self.data_dir, 'plots')
        
        os.makedirs(output_dir, exist_ok=True)
        
        timestamp = self.timestamp_suffix or datetime.now().strftime('%Y%m%d_%H%M%S')
        
        print("\nGenerating all plots...")
        print("-" * 60)
        
        self.plot_trajectory_comparison(
            os.path.join(output_dir, f'trajectory_comparison_{timestamp}.png'))
        
        self.plot_position_error_comparison(
            os.path.join(output_dir, f'position_error_comparison_{timestamp}.png'))
        
        self.plot_yaw_error_comparison(
            os.path.join(output_dir, f'yaw_error_comparison_{timestamp}.png'))
        
        self.plot_statistics_comparison(
            os.path.join(output_dir, f'error_statistics_{timestamp}.png'))
        
        self.plot_error_distribution(
            os.path.join(output_dir, f'error_distribution_{timestamp}.png'))
        
        self.plot_xy_error_scatter(
            os.path.join(output_dir, f'xy_error_scatter_{timestamp}.png'))
        
        print("-" * 60)
        print(f"\nAll plots saved to: {output_dir}")
        print("Generated 6 plots:")
        print("  1. trajectory_comparison")
        print("  2. position_error_comparison")
        print("  3. yaw_error_comparison")
        print("  4. error_statistics")
        print("  5. error_distribution")
        print("  6. xy_error_scatter")


def main():
    parser = argparse.ArgumentParser(
        description='Process multi-robot localization evaluation data')
    parser.add_argument('--data-dir', type=str, default=None,
                        help='Directory containing evaluation data (default: ~/ids_roswk/evaluation_results/multibot)')
    parser.add_argument('--timestamp', type=str, default=None,
                        help='Specific timestamp suffix (e.g., 20251223_185907)')
    parser.add_argument('--output-dir', type=str, default=None,
                        help='Output directory for plots (default: <data-dir>/plots)')
    parser.add_argument('--show', action='store_true',
                        help='Show plots interactively instead of saving')
    
    args = parser.parse_args()
    
    print("=" * 80)
    print("Multi-Robot Localization Evaluation - Data Processor")
    print("=" * 80)
    
    processor = MultibotDataProcessor(args.data_dir)
    
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
    else:
        processor.generate_all_plots(args.output_dir)
    
    print("\n" + "=" * 80)
    print("Processing complete!")
    print("=" * 80)
    
    return 0


if __name__ == '__main__':
    exit(main())

