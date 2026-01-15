#!/usr/bin/env python3
"""
Pose Evaluation Node

Functionality:
- Synchronously records Ground Truth (odom) and AMCL estimated poses.
- Calculates position and heading errors with time alignment.
- Outputs error curves and statistical metrics (RMSE, etc.).

Subscribed Topics:
- /odom: Ground Truth (in Gazebo simulation, odom is the ideal value)
- /amcl_pose: AMCL estimated pose

This node does not participate in path planning or motion control; it only evaluates the performance of the localization system.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import math
import time
import csv
import os
from datetime import datetime
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class PoseRecord:
    """Pose record"""
    timestamp: float
    x: float
    y: float
    yaw: float


@dataclass
class ErrorRecord:
    """Error record"""
    timestamp: float
    x_error: float
    y_error: float
    position_error: float  # Euclidean distance error
    yaw_error: float


class PoseEvalNode(Node):
    """Pose Evaluation Node"""
    
    # Data save path
    OUTPUT_DIR = os.path.expanduser('~/ids_roswk/evaluation_results')
    
    def __init__(self):
        super().__init__('pose_eval_node')
        
        # Data storage
        self.ground_truth_records: List[PoseRecord] = []
        self.amcl_records: List[PoseRecord] = []
        self.error_records: List[ErrorRecord] = []
        
        # Latest poses (for real-time calculation)
        self.latest_gt: Optional[PoseRecord] = None
        self.latest_amcl: Optional[PoseRecord] = None
        
        # Evaluation status
        self.is_recording = False
        self.start_time: Optional[float] = None
        
        # Create output directory
        os.makedirs(self.OUTPUT_DIR, exist_ok=True)
        
        # QoS settings
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Subscribe to odom as Ground Truth (in Gazebo simulation, odom is the ideal value)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            sensor_qos
        )
        
        # Subscribe to AMCL estimated pose
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            reliable_qos
        )
        
        # Timer: Periodically calculate and record errors
        self.eval_timer = self.create_timer(0.1, self.evaluate_error)  # 10Hz
        
        # Timer: Periodically output statistics
        self.stats_timer = self.create_timer(5.0, self.print_statistics)  # Every 5 seconds
        
        self.get_logger().info('='*60)
        self.get_logger().info('Pose Evaluation Node has been started.')
        self.get_logger().info('Ground Truth: /odom (ideal value in Gazebo simulation)')
        self.get_logger().info('Estimated Pose: /amcl_pose')
        self.get_logger().info(f'Data saving directory: {self.OUTPUT_DIR}')
        self.get_logger().info('='*60)
        self.get_logger().info('Waiting to receive pose data...')
        
        # Flag to indicate if AMCL is being used (True if an AMCL message is received)
        self.using_amcl = False

    def odom_callback(self, msg: Odometry):
        """Processes odom data as Ground Truth (in Gazebo simulation, odom is the ideal value)"""
        pose = msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        yaw = self.quaternion_to_yaw(pose.orientation)
        
        timestamp = time.time()
        
        # odom as Ground Truth
        self.latest_gt = PoseRecord(timestamp, x, y, yaw)
        
        # Start recording
        if not self.is_recording:
            self.is_recording = True
            self.start_time = timestamp
            self.get_logger().info('Start recording data (using odom as Ground Truth)')
        
        # Save Ground Truth record
        self.ground_truth_records.append(self.latest_gt)
        
        # If there is no AMCL, use odom as the estimate as well (error will be 0, for testing purposes only)
        if not self.using_amcl:
            self.latest_amcl = PoseRecord(timestamp, x, y, yaw)
            self.amcl_records.append(self.latest_amcl)

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        """Processes AMCL estimated pose"""
        if not self.using_amcl:
            self.using_amcl = True
            self.get_logger().info('AMCL pose detected, using AMCL as the estimation source.')
        
        pose = msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        yaw = self.quaternion_to_yaw(pose.orientation)
        
        timestamp = time.time()
        
        self.latest_amcl = PoseRecord(timestamp, x, y, yaw)
        
        if self.is_recording:
            self.amcl_records.append(self.latest_amcl)

    def evaluate_error(self):
        """Calculates the current error"""
        if self.latest_gt is None or self.latest_amcl is None:
            return
        
        # Time alignment check (time difference between the two poses should not exceed 0.5 seconds)
        time_diff = abs(self.latest_gt.timestamp - self.latest_amcl.timestamp)
        if time_diff > 0.5:
            return
        
        # Calculate errors
        x_error = self.latest_amcl.x - self.latest_gt.x
        y_error = self.latest_amcl.y - self.latest_gt.y
        position_error = math.sqrt(x_error**2 + y_error**2)
        yaw_error = self.normalize_angle(self.latest_amcl.yaw - self.latest_gt.yaw)
        
        timestamp = (self.latest_gt.timestamp + self.latest_amcl.timestamp) / 2
        
        error = ErrorRecord(
            timestamp=timestamp,
            x_error=x_error,
            y_error=y_error,
            position_error=position_error,
            yaw_error=yaw_error
        )
        
        self.error_records.append(error)

    def print_statistics(self):
        """Outputs statistical information"""
        if len(self.error_records) < 10:
            self.get_logger().info(f'Collecting data... GT: {len(self.ground_truth_records)}, '
                                   f'Estimate: {len(self.amcl_records)}, '
                                   f'Error Records: {len(self.error_records)}')
            return
        
        # Calculate statistical metrics
        stats = self.calculate_statistics()
        
        self.get_logger().info('-'*50)
        self.get_logger().info(f'[Localization Error Statistics] (Sample count: {stats["count"]})')
        self.get_logger().info(f'  Position RMSE: {stats["position_rmse"]:.4f} m')
        self.get_logger().info(f'  Position Mean: {stats["position_mean"]:.4f} m')
        self.get_logger().info(f'  Position Max:  {stats["position_max"]:.4f} m')
        self.get_logger().info(f'  Heading RMSE:   {math.degrees(stats["yaw_rmse"]):.2f} deg')
        self.get_logger().info(f'  Heading Mean:   {math.degrees(stats["yaw_mean"]):.2f} deg')
        self.get_logger().info(f'  Heading Max:    {math.degrees(stats["yaw_max"]):.2f} deg')
        self.get_logger().info('-'*50)

    def calculate_statistics(self) -> dict:
        """Calculates error statistical metrics"""
        if len(self.error_records) == 0:
            return {
                'count': 0,
                'position_rmse': 0.0,
                'position_mean': 0.0,
                'position_max': 0.0,
                'yaw_rmse': 0.0,
                'yaw_mean': 0.0,
                'yaw_max': 0.0,
            }
        
        n = len(self.error_records)
        
        # Position error
        position_errors = [e.position_error for e in self.error_records]
        position_rmse = math.sqrt(sum(e**2 for e in position_errors) / n)
        position_mean = sum(position_errors) / n
        position_max = max(position_errors)
        
        # Heading error
        yaw_errors = [abs(e.yaw_error) for e in self.error_records]
        yaw_rmse = math.sqrt(sum(e**2 for e in yaw_errors) / n)
        yaw_mean = sum(yaw_errors) / n
        yaw_max = max(yaw_errors)
        
        return {
            'count': n,
            'position_rmse': position_rmse,
            'position_mean': position_mean,
            'position_max': position_max,
            'yaw_rmse': yaw_rmse,
            'yaw_mean': yaw_mean,
            'yaw_max': yaw_max,
        }

    def save_results(self):
        """Saves evaluation results to files"""
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # Save error data
        error_file = os.path.join(self.OUTPUT_DIR, f'errors_{timestamp_str}.csv')
        with open(error_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'x_error', 'y_error', 'position_error', 'yaw_error'])
            for e in self.error_records:
                writer.writerow([e.timestamp, e.x_error, e.y_error, e.position_error, e.yaw_error])
        
        # Save Ground Truth data
        gt_file = os.path.join(self.OUTPUT_DIR, f'ground_truth_{timestamp_str}.csv')
        with open(gt_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'x', 'y', 'yaw'])
            for r in self.ground_truth_records:
                writer.writerow([r.timestamp, r.x, r.y, r.yaw])
        
        # Save estimated pose data
        est_file = os.path.join(self.OUTPUT_DIR, f'estimated_{timestamp_str}.csv')
        with open(est_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'x', 'y', 'yaw'])
            for r in self.amcl_records:
                writer.writerow([r.timestamp, r.x, r.y, r.yaw])
        
        # Save statistics results
        stats = self.calculate_statistics()
        stats_file = os.path.join(self.OUTPUT_DIR, f'statistics_{timestamp_str}.txt')
        with open(stats_file, 'w') as f:
            f.write('='*50 + '\n')
            f.write('Localization Performance Evaluation Report\n')
            f.write(f'Time: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}\n')
            f.write('='*50 + '\n\n')
            f.write(f'Number of samples: {stats["count"]}\n\n')
            f.write('[Position Error]\n')
            f.write(f'  RMSE: {stats["position_rmse"]:.4f} m\n')
            f.write(f'  Mean: {stats["position_mean"]:.4f} m\n')
            f.write(f'  Max:  {stats["position_max"]:.4f} m\n\n')
            f.write('[Heading Error]\n')
            f.write(f'  RMSE: {math.degrees(stats["yaw_rmse"]):.2f} deg\n')
            f.write(f'  Mean: {math.degrees(stats["yaw_mean"]):.2f} deg\n')
            f.write(f'  Max:  {math.degrees(stats["yaw_max"]):.2f} deg\n')
        
        self.get_logger().info(f'Results have been saved to: {self.OUTPUT_DIR}')
        self.get_logger().info(f'  - {os.path.basename(error_file)}')
        self.get_logger().info(f'  - {os.path.basename(gt_file)}')
        self.get_logger().info(f'  - {os.path.basename(est_file)}')
        self.get_logger().info(f'  - {os.path.basename(stats_file)}')

    def quaternion_to_yaw(self, q) -> float:
        """Converts quaternion to yaw angle"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle: float) -> float:
        """Normalizes an angle to the range [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = PoseEvalNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\nUser interrupt, saving data...')
    finally:
        # Save results
        if len(node.error_records) > 0:
            node.save_results()
            # Print final statistics
            node.get_logger().info('\n' + '='*60)
            node.get_logger().info('[Final Evaluation Results]')
            stats = node.calculate_statistics()
            node.get_logger().info(f'  Total samples: {stats["count"]}')
            node.get_logger().info(f'  Position RMSE: {stats["position_rmse"]:.4f} m')
            node.get_logger().info(f'  Heading RMSE: {math.degrees(stats["yaw_rmse"]):.2f} deg')
            node.get_logger().info('='*60)
        else:
            node.get_logger().warn('Not enough data was collected.')
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
