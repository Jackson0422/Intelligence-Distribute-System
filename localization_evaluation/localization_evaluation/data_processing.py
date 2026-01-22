#!/usr/bin/env python3
"""
Data Processing and Visualization for Localization Evaluation

Generates:
1. Trajectory Comparison (GT vs AMCL)
2. Position Error vs Time
3. Yaw Error vs Time
"""

import os
import glob
import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime


class LocalizationDataProcessor:
    """Process and visualize localization evaluation data."""
    
    def __init__(self, data_dir: str = None):
        """Initialize with data directory."""
        if data_dir is None:
            data_dir = os.path.expanduser('~/ids_roswk/evaluation_results')
        self.data_dir = data_dir
        self.gt_data = None
        self.est_data = None
        self.error_data = None
        self.timestamp_suffix = None
        
    def find_latest_files(self) -> bool:
        """Find the latest set of evaluation files."""
        # Find all statistics files to get timestamps
        stat_files = glob.glob(os.path.join(self.data_dir, 'statistics_*.txt'))
        if not stat_files:
            print(f"No evaluation files found in {self.data_dir}")
            return False
        
        # Get the latest timestamp
        latest_file = max(stat_files, key=os.path.getmtime)
        self.timestamp_suffix = os.path.basename(latest_file).replace('statistics_', '').replace('.txt', '')
        print(f"Using data from: {self.timestamp_suffix}")
        return True
    
    def load_data(self, timestamp_suffix: str = None) -> bool:
        """Load ground truth, estimated, and error data."""
        if timestamp_suffix:
            self.timestamp_suffix = timestamp_suffix
        elif self.timestamp_suffix is None:
            if not self.find_latest_files():
                return False
        
        try:
            gt_file = os.path.join(self.data_dir, f'ground_truth_{self.timestamp_suffix}.csv')
            est_file = os.path.join(self.data_dir, f'estimated_{self.timestamp_suffix}.csv')
            error_file = os.path.join(self.data_dir, f'errors_{self.timestamp_suffix}.csv')
            
            self.gt_data = pd.read_csv(gt_file)
            self.est_data = pd.read_csv(est_file)
            self.error_data = pd.read_csv(error_file)
            
            # Convert timestamp to relative time (starting from 0)
            start_time = self.gt_data['timestamp'].min()
            self.gt_data['time'] = self.gt_data['timestamp'] - start_time
            self.est_data['time'] = self.est_data['timestamp'] - start_time
            self.error_data['time'] = self.error_data['timestamp'] - start_time
            
            print(f"Loaded {len(self.gt_data)} GT samples, {len(self.est_data)} estimated samples, {len(self.error_data)} error samples")
            return True
            
        except Exception as e:
            print(f"Error loading data: {e}")
            return False
    
    def plot_trajectory_comparison(self, save_path: str = None):
        """Plot GT and AMCL trajectories on the same figure."""
        if self.gt_data is None or self.est_data is None:
            print("Data not loaded. Call load_data() first.")
            return
        
        fig, ax = plt.subplots(figsize=(10, 8))
        
        # Plot Ground Truth trajectory
        ax.plot(self.gt_data['x'], self.gt_data['y'], 
                'b-', linewidth=1.5, label='Ground Truth', alpha=0.8)
        
        # Plot AMCL estimated trajectory
        ax.plot(self.est_data['x'], self.est_data['y'], 
                'r--', linewidth=1.5, label='AMCL Estimated', alpha=0.8)
        
        # Mark start and end points
        ax.scatter(self.gt_data['x'].iloc[0], self.gt_data['y'].iloc[0], 
                   c='green', s=100, marker='o', zorder=5, label='Start')
        ax.scatter(self.gt_data['x'].iloc[-1], self.gt_data['y'].iloc[-1], 
                   c='purple', s=100, marker='s', zorder=5, label='End')
        
        ax.set_xlabel('X Position (m)', fontsize=12)
        ax.set_ylabel('Y Position (m)', fontsize=12)
        ax.set_title('Trajectory Comparison: Ground Truth vs AMCL', fontsize=14, fontweight='bold')
        ax.legend(loc='best', fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Saved trajectory plot to: {save_path}")
        else:
            plt.show()
        
        plt.close()
    
    def plot_position_error(self, save_path: str = None):
        """Plot position error vs time."""
        if self.error_data is None:
            print("Data not loaded. Call load_data() first.")
            return
        
        fig, ax = plt.subplots(figsize=(12, 5))
        
        ax.plot(self.error_data['time'], self.error_data['position_error'], 
                'b-', linewidth=1.0, alpha=0.8)
        ax.fill_between(self.error_data['time'], 0, self.error_data['position_error'], 
                        alpha=0.3, color='blue')
        
        # Calculate and plot statistics
        rmse = np.sqrt(np.mean(self.error_data['position_error']**2))
        mean_error = self.error_data['position_error'].mean()
        max_error = self.error_data['position_error'].max()
        
        ax.axhline(y=rmse, color='r', linestyle='--', linewidth=1.5, 
                   label=f'RMSE: {rmse:.4f} m')
        ax.axhline(y=mean_error, color='orange', linestyle=':', linewidth=1.5, 
                   label=f'Mean: {mean_error:.4f} m')
        
        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('Position Error (m)', fontsize=12)
        ax.set_title('Position Error vs Time', fontsize=14, fontweight='bold')
        ax.legend(loc='upper right', fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.set_xlim(left=0)
        ax.set_ylim(bottom=0)
        
        # Add text box with statistics
        stats_text = f'Max Error: {max_error:.4f} m\nSamples: {len(self.error_data)}'
        ax.text(0.02, 0.98, stats_text, transform=ax.transAxes, fontsize=9,
                verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Saved position error plot to: {save_path}")
        else:
            plt.show()
        
        plt.close()
    
    def plot_yaw_error(self, save_path: str = None):
        """Plot yaw error vs time."""
        if self.error_data is None:
            print("Data not loaded. Call load_data() first.")
            return
        
        fig, ax = plt.subplots(figsize=(12, 5))
        
        # Convert to degrees for better readability
        yaw_error_deg = np.degrees(self.error_data['yaw_error'])
        
        ax.plot(self.error_data['time'], yaw_error_deg, 
                'g-', linewidth=1.0, alpha=0.8)
        ax.fill_between(self.error_data['time'], 0, yaw_error_deg, 
                        where=(yaw_error_deg >= 0), alpha=0.3, color='green')
        ax.fill_between(self.error_data['time'], 0, yaw_error_deg, 
                        where=(yaw_error_deg < 0), alpha=0.3, color='red')
        
        # Calculate and plot statistics
        rmse = np.sqrt(np.mean(yaw_error_deg**2))
        mean_error = np.mean(np.abs(yaw_error_deg))
        max_error = np.max(np.abs(yaw_error_deg))
        
        ax.axhline(y=rmse, color='r', linestyle='--', linewidth=1.5, 
                   label=f'RMSE: {rmse:.2f} deg')
        ax.axhline(y=-rmse, color='r', linestyle='--', linewidth=1.5)
        
        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('Yaw Error (degrees)', fontsize=12)
        ax.set_title('Yaw Error vs Time', fontsize=14, fontweight='bold')
        ax.legend(loc='upper right', fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.set_xlim(left=0)
        
        # Add text box with statistics
        stats_text = f'Mean |Error|: {mean_error:.2f} deg\nMax |Error|: {max_error:.2f} deg'
        ax.text(0.02, 0.98, stats_text, transform=ax.transAxes, fontsize=9,
                verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Saved yaw error plot to: {save_path}")
        else:
            plt.show()
        
        plt.close()
    
    def generate_all_plots(self, output_dir: str = None):
        """Generate all plots and save to output directory."""
        if output_dir is None:
            output_dir = os.path.join(self.data_dir, 'plots')
        
        os.makedirs(output_dir, exist_ok=True)
        
        timestamp = self.timestamp_suffix or datetime.now().strftime('%Y%m%d_%H%M%S')
        
        self.plot_trajectory_comparison(
            os.path.join(output_dir, f'trajectory_comparison_{timestamp}.png'))
        self.plot_position_error(
            os.path.join(output_dir, f'position_error_{timestamp}.png'))
        self.plot_yaw_error(
            os.path.join(output_dir, f'yaw_error_{timestamp}.png'))
        
        print(f"\nAll plots saved to: {output_dir}")


def main():
    parser = argparse.ArgumentParser(description='Process localization evaluation data')
    parser.add_argument('--data-dir', type=str, default=None,
                        help='Directory containing evaluation data')
    parser.add_argument('--timestamp', type=str, default=None,
                        help='Specific timestamp suffix (e.g., 20251222_173321)')
    parser.add_argument('--output-dir', type=str, default=None,
                        help='Output directory for plots')
    parser.add_argument('--show', action='store_true',
                        help='Show plots interactively instead of saving')
    
    args = parser.parse_args()
    
    processor = LocalizationDataProcessor(args.data_dir)
    
    if not processor.load_data(args.timestamp):
        return 1
    
    if args.show:
        processor.plot_trajectory_comparison()
        processor.plot_position_error()
        processor.plot_yaw_error()
    else:
        processor.generate_all_plots(args.output_dir)
    
    return 0


if __name__ == '__main__':
    exit(main())

