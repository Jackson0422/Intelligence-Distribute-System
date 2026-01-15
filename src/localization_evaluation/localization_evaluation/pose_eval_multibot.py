#!/usr/bin/env python3

"""
Multi-Robot Localization Performance Evaluation Node

Features:
- Simultaneously record Ground Truth and AMCL estimated poses for multiple robots
- Independently calculate position error and heading angle error for each robot
- Output error curves and statistical metrics for each robot separately

Subscribed Topics:
- /odom, /amcl_pose: Robot 1 (no namespace)
- /tb3_2/odom, /tb3_2/amcl_pose: Robot 2

Output files saved in ~/ids_roswk/evaluation_results/multibot/
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
from dataclasses import dataclass, field
from typing import List, Optional, Dict


@dataclass
class PoseRecord:
    """Pose Record"""
    timestamp: float
    x: float
    y: float
    yaw: float


@dataclass
class ErrorRecord:
    """Error Record"""
    timestamp: float
    x_error: float
    y_error: float
    position_error: float
    yaw_error: float


@dataclass
class RobotData:
    """Data storage for a single robot"""
    namespace: str
    ground_truth_records: List[PoseRecord] = field(default_factory=list)
    amcl_records: List[PoseRecord] = field(default_factory=list)
    error_records: List[ErrorRecord] = field(default_factory=list)
    latest_gt: Optional[PoseRecord] = None
    latest_amcl: Optional[PoseRecord] = None
    using_amcl: bool = False


class MultiRobotPoseEvalNode(Node):
    """Multi-Robot Localization Performance Evaluation Node"""
    
    # Data save path - multibot subdirectory
    OUTPUT_DIR = os.path.expanduser('~/ids_roswk/evaluation_results/multibot')
    
    # ========== Robot Namespace Configuration ==========
    # Consistent with the configuration in track_multibot.py
    # '' (empty string) = first robot, no namespace
    # 'tb3_2' = second robot
    ROBOT_NAMESPACES = ['', 'tb3_2']
    # ========================================
    
    def __init__(self):
        super().__init__('multi_robot_pose_eval_node')
        
        # Create data storage for each robot
        self.robots: Dict[str, RobotData] = {}
        
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
        
        # Create subscriptions for each robot
        for ns in self.ROBOT_NAMESPACES:
            # Identifier for display and filename
            display_name = ns if ns else 'tb3_1'
            self.robots[display_name] = RobotData(namespace=display_name)
            
            # Construct topic names based on namespace
            odom_topic = f'/{ns}/odom' if ns else '/odom'
            amcl_topic = f'/{ns}/amcl_pose' if ns else '/amcl_pose'
            
            # Subscribe to odom as Ground Truth
            self.create_subscription(
                Odometry,
                odom_topic,
                lambda msg, n=display_name: self.odom_callback(msg, n),
                sensor_qos
            )
            
            # Subscribe to amcl_pose
            self.create_subscription(
                PoseWithCovarianceStamped,
                amcl_topic,
                lambda msg, n=display_name: self.amcl_callback(msg, n),
                reliable_qos
            )
            
            self.get_logger().info(f'[{display_name}] Subscribed to topics:')
            self.get_logger().info(f'  - {odom_topic} (Ground Truth)')
            self.get_logger().info(f'  - {amcl_topic} (AMCL Estimate)')
        
        # Timer: periodically calculate errors
        self.eval_timer = self.create_timer(0.1, self.evaluate_errors)  # 10Hz
        
        # Timer: periodically output statistics
        self.stats_timer = self.create_timer(5.0, self.print_statistics)  # every 5 seconds
        
        self.get_logger().info('='*60)
        self.get_logger().info('Multi-Robot Localization Performance Evaluation Node started')
        self.get_logger().info(f'Evaluating robots: {list(self.robots.keys())}')
        self.get_logger().info(f'Data save directory: {self.OUTPUT_DIR}')
        self.get_logger().info('='*60)
        self.get_logger().info('Waiting for pose data...')
    
    def odom_callback(self, msg: Odometry, namespace: str):
        """Process odom data as Ground Truth"""
        robot = self.robots[namespace]
        
        pose = msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        yaw = self.quaternion_to_yaw(pose.orientation)
        timestamp = time.time()
        
        robot.latest_gt = PoseRecord(timestamp, x, y, yaw)
        
        # Start recording
        if not self.is_recording:
            self.is_recording = True
            self.start_time = timestamp
            self.get_logger().info('Started recording data')
        
        # Save Ground Truth record
        robot.ground_truth_records.append(robot.latest_gt)
        
        # If no AMCL, also use odom as estimate
        if not robot.using_amcl:
            robot.latest_amcl = PoseRecord(timestamp, x, y, yaw)
            robot.amcl_records.append(robot.latest_amcl)
    
    def amcl_callback(self, msg: PoseWithCovarianceStamped, namespace: str):
        """Process AMCL estimated pose"""
        robot = self.robots[namespace]
        
        if not robot.using_amcl:
            robot.using_amcl = True
            self.get_logger().info(f'[{namespace}] Detected AMCL pose, using AMCL as estimate source')
        
        pose = msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        yaw = self.quaternion_to_yaw(pose.orientation)
        timestamp = time.time()
        
        robot.latest_amcl = PoseRecord(timestamp, x, y, yaw)
        
        if self.is_recording:
            robot.amcl_records.append(robot.latest_amcl)
    
    def evaluate_errors(self):
        """Calculate current errors for each robot"""
        for ns, robot in self.robots.items():
            if robot.latest_gt is None or robot.latest_amcl is None:
                continue
            
            # Time alignment check
            time_diff = abs(robot.latest_gt.timestamp - robot.latest_amcl.timestamp)
            if time_diff > 0.5:
                continue
            
            # Calculate errors
            x_error = robot.latest_amcl.x - robot.latest_gt.x
            y_error = robot.latest_amcl.y - robot.latest_gt.y
            position_error = math.sqrt(x_error**2 + y_error**2)
            yaw_error = self.normalize_angle(robot.latest_amcl.yaw - robot.latest_gt.yaw)
            
            timestamp = (robot.latest_gt.timestamp + robot.latest_amcl.timestamp) / 2
            
            robot.error_records.append(ErrorRecord(
                timestamp=timestamp,
                x_error=x_error,
                y_error=y_error,
                position_error=position_error,
                yaw_error=yaw_error
            ))
    
    def print_statistics(self):
        """Output statistical information"""
        has_data = False
        for robot in self.robots.values():
            if len(robot.error_records) >= 10:
                has_data = True
                break
        
        if not has_data:
            total_gt = sum(len(r.ground_truth_records) for r in self.robots.values())
            total_est = sum(len(r.amcl_records) for r in self.robots.values())
            total_err = sum(len(r.error_records) for r in self.robots.values())
            self.get_logger().info(f'Collecting data... GT: {total_gt}, Estimates: {total_est}, Errors: {total_err}')
            return
        
        self.get_logger().info('-'*60)
        self.get_logger().info('【Multi-Robot Localization Error Statistics】')
        
        for ns, robot in self.robots.items():
            stats = self.calculate_statistics(robot)
            self.get_logger().info(
                f'  [{ns}] Samples: {stats["count"]}, '
                f'Position RMSE: {stats["position_rmse"]:.4f}m, '
                f'Heading RMSE: {math.degrees(stats["yaw_rmse"]):.2f}°'
            )
        
        self.get_logger().info('-'*60)
    
    def calculate_statistics(self, robot: RobotData) -> dict:
        """Calculate error statistics for a single robot"""
        if len(robot.error_records) == 0:
            return {
                'count': 0,
                'position_rmse': 0.0,
                'position_mean': 0.0,
                'position_max': 0.0,
                'yaw_rmse': 0.0,
                'yaw_mean': 0.0,
                'yaw_max': 0.0,
            }
        
        n = len(robot.error_records)
        
        # Position error
        position_errors = [e.position_error for e in robot.error_records]
        position_rmse = math.sqrt(sum(e**2 for e in position_errors) / n)
        position_mean = sum(position_errors) / n
        position_max = max(position_errors)
        
        # Heading angle error
        yaw_errors = [abs(e.yaw_error) for e in robot.error_records]
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
        """Save evaluation results for all robots"""
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        self.get_logger().info('\nSaving evaluation results...')
        
        for ns, robot in self.robots.items():
            if len(robot.error_records) == 0:
                self.get_logger().warn(f'[{ns}] No error data, skipping save')
                continue
            
            # Save error data
            error_file = os.path.join(self.OUTPUT_DIR, f'{ns}_errors_{timestamp_str}.csv')
            with open(error_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'x_error', 'y_error', 'position_error', 'yaw_error'])
                for e in robot.error_records:
                    writer.writerow([e.timestamp, e.x_error, e.y_error, e.position_error, e.yaw_error])
            
            # Save Ground Truth data
            gt_file = os.path.join(self.OUTPUT_DIR, f'{ns}_ground_truth_{timestamp_str}.csv')
            with open(gt_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'x', 'y', 'yaw'])
                for r in robot.ground_truth_records:
                    writer.writerow([r.timestamp, r.x, r.y, r.yaw])
            
            # Save estimated pose data
            est_file = os.path.join(self.OUTPUT_DIR, f'{ns}_estimated_{timestamp_str}.csv')
            with open(est_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'x', 'y', 'yaw'])
                for r in robot.amcl_records:
                    writer.writerow([r.timestamp, r.x, r.y, r.yaw])
            
            # Save statistical results
            stats = self.calculate_statistics(robot)
            stats_file = os.path.join(self.OUTPUT_DIR, f'{ns}_statistics_{timestamp_str}.txt')
            with open(stats_file, 'w') as f:
                f.write('='*50 + '\n')
                f.write(f'Localization Performance Evaluation Report - {ns}\n')
                f.write(f'Time: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}\n')
                f.write('='*50 + '\n\n')
                f.write(f'Sample Count: {stats["count"]}\n\n')
                f.write('【Position Error】\n')
                f.write(f'  RMSE: {stats["position_rmse"]:.4f} m\n')
                f.write(f'  Mean: {stats["position_mean"]:.4f} m\n')
                f.write(f'  Max:  {stats["position_max"]:.4f} m\n\n')
                f.write('【Heading Angle Error】\n')
                f.write(f'  RMSE: {math.degrees(stats["yaw_rmse"]):.2f}°\n')
                f.write(f'  Mean: {math.degrees(stats["yaw_mean"]):.2f}°\n')
                f.write(f'  Max:  {math.degrees(stats["yaw_max"]):.2f}°\n')
            
            self.get_logger().info(f'[{ns}] Data saved:')
            self.get_logger().info(f'  - {os.path.basename(error_file)}')
            self.get_logger().info(f'  - {os.path.basename(gt_file)}')
            self.get_logger().info(f'  - {os.path.basename(est_file)}')
            self.get_logger().info(f'  - {os.path.basename(stats_file)}')
    
    def quaternion_to_yaw(self, q) -> float:
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotPoseEvalNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\nUser interrupted, saving data...')
    finally:
        # Save results
        total_errors = sum(len(r.error_records) for r in node.robots.values())
        if total_errors > 0:
            node.save_results()
            
            # Print final statistics
            node.get_logger().info('\n' + '='*60)
            node.get_logger().info('【Final Evaluation Results】')
            for ns, robot in node.robots.items():
                stats = node.calculate_statistics(robot)
                if stats['count'] > 0:
                    node.get_logger().info(
                        f'  [{ns}] Samples: {stats["count"]}, '
                        f'Position RMSE: {stats["position_rmse"]:.4f}m, '
                        f'Heading RMSE: {math.degrees(stats["yaw_rmse"]):.2f}°'
                    )
            node.get_logger().info('='*60)
        else:
            node.get_logger().warn('Not enough data collected')
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
