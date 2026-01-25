#!/usr/bin/env python3
"""
Collaborative Localization Performance Evaluation Node

Evaluates the performance of decentralized collaborative localization by comparing
ground truth (odom) and collaborative localization estimates (coloc_pose)

Results saved to: evaluation_results/multibot/coloc/

Author: Distributed Intelligent Systems Course
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.exceptions import ParameterAlreadyDeclaredException
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
import math
import numpy as np
from datetime import datetime
import os

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
import rclpy.time
import rclpy.duration


class PoseEvalColoc(Node):
    """Collaborative localization performance evaluation node"""
    
    def __init__(self):
        super().__init__('pose_eval_coloc')
        
        # Declare parameters (tolerate pre-declared use_sim_time)
        self.declare_parameter('num_robots', 4)
        try:
            self.declare_parameter('use_sim_time', True)
        except ParameterAlreadyDeclaredException:
            pass
        self.declare_parameter('result_dir', '')

        self.num_robots = int(self.get_parameter('num_robots').value)
        self.use_sim_time = bool(self.get_parameter('use_sim_time').value)
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, self.use_sim_time)])

        # Validate parameter range (supports up to 20)
        if self.num_robots < 2 or self.num_robots > 20:
            self.get_logger().warning(f'num_robots parameter expected 2-20, got {self.num_robots}; using 2')
            self.num_robots = 2

        # Create result directory (defaults to package share logs/evaluation_results)
        default_dir = os.path.join(
            os.path.dirname(os.path.dirname(__file__)),
            'logs',
            'evaluation_results',
            'multibot',
            'coloc'
        )
        configured_dir = self.get_parameter('result_dir').value
        self.result_dir = configured_dir if configured_dir else default_dir
        os.makedirs(self.result_dir, exist_ok=True)
        
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'Collaborative localization evaluation node started (number of robots: {self.num_robots})')
        self.get_logger().info(f'Results saved to: {self.result_dir}')
        self.get_logger().info('=' * 80)
        
        # Robot ID list
        self.robot_ids = [f'tb3_{i}' for i in range(1, self.num_robots + 1)]
        
        # Dynamically create robot data dictionary
        self.robot_data = {}
        for robot_id in self.robot_ids:
            self.robot_data[robot_id] = {
                'odom': None,
                'coloc_pose': None,
                'last_coloc_pose': None,  # Used to detect if coloc_pose actually updated
                'errors': [],
                'count': 0,
                'skipped_count': 0  # Count of skipped duplicate samples
            }
        
        # Dynamically create subscribers
        self.odom_subs = []
        self.coloc_subs = []
        
        for robot_id in self.robot_ids:
            odom_topic = f'/{robot_id}/odom'
            coloc_topic = f'/{robot_id}/coloc_pose'

            # Use lambda to create callback with default parameter to capture robot_id
            odom_sub = self.create_subscription(
                Odometry,
                odom_topic,
                lambda msg, rid=robot_id: self.odom_callback(msg, rid),
                10
            )
            self.odom_subs.append(odom_sub)
            
            # Subscribe to collaborative localization pose
            coloc_sub = self.create_subscription(
                PoseWithCovarianceStamped,
                coloc_topic,
                lambda msg, rid=robot_id: self.coloc_callback(msg, rid),
                10
            )
            self.coloc_subs.append(coloc_sub)
            
            self.get_logger().info(f'  Subscribed to: {odom_topic} and {coloc_topic}')
        
        # Timer to print statistics (5 seconds)
        self.timer = self.create_timer(5.0, self.print_statistics)
        
        # TF listener: used to align poses in different frames before calculating errors
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # TF ready flag
        self.tf_ready = False
        self.tf_check_count = 0
        
        # Start TF check timer (check every 0.5 seconds, max 40 times = 20 seconds)
        self.tf_check_timer = self.create_timer(0.5, self._check_tf_ready)
        
        self.get_logger().info('Waiting for TF tree initialization...')
        self.get_logger().info('Waiting for collaborative localization pose data...')

    def _check_tf_ready(self):
        """Periodically check if TF is ready (async callback)"""
        if self.tf_ready:
            # Already ready, stop checking
            return
        
        self.tf_check_count += 1
        
        # Dynamically build list of TFs to check
        # Note: TF direction to check should match the one actually used in pose_in_frame()
        # pose_in_frame() uses lookup_transform(target='odom', source='map')
        required_transforms = []
        for robot_id in self.robot_ids:
            if robot_id == 'tb3_1':
                required_transforms.append(('odom', 'map'))
            else:
                required_transforms.append((f'{robot_id}/odom', 'map'))
        
        all_ready = True
        for target, source in required_transforms:
            try:
                self.tf_buffer.lookup_transform(
                    target, source,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                # TF available, but don't print immediately (avoid duplication)
            except Exception:
                all_ready = False
                break
        
        if all_ready:
            # All TFs are ready
            self.tf_ready = True
            self.tf_check_timer.cancel()  # Stop checking
            self.get_logger().info('  ✓ TF tree is ready, starting error calculation')
        elif self.tf_check_count >= 40:
            # Timeout (40 times * 0.5 seconds = 20 seconds)
            self.get_logger().warning(
                f'  ✗ TF wait timeout ({self.tf_check_count * 0.5:.1f} seconds), '
                'will continue but error calculation may fail'
            )
            self.tf_ready = True  # Force continue to avoid permanent blocking
            # Option A: Stop the node completely
            # self.get_logger().fatal('TF timeout, shutting down...')
            # rclpy.shutdown()

            self.tf_check_timer.cancel()

    def pose_in_frame(self, pose_with_cov: PoseWithCovarianceStamped, target_frame: str):
        """
        Transform PoseWithCovarianceStamped to target_frame, return (x, y, yaw) or None (TF unavailable)
        """
        src_frame = pose_with_cov.header.frame_id
        if not src_frame or src_frame == target_frame:
            x = pose_with_cov.pose.pose.position.x
            y = pose_with_cov.pose.pose.position.y
            yaw = self.quaternion_to_yaw(pose_with_cov.pose.pose.orientation)
            return x, y, yaw
        
        # Construct Pose object (not PoseStamped)
        # do_transform_pose expects Pose type, not PoseStamped
        pose = Pose()
        
        # Extract Pose data from PoseWithCovarianceStamped
        pose.position.x = pose_with_cov.pose.pose.position.x
        pose.position.y = pose_with_cov.pose.pose.position.y
        pose.position.z = pose_with_cov.pose.pose.position.z
        pose.orientation.x = pose_with_cov.pose.pose.orientation.x
        pose.orientation.y = pose_with_cov.pose.pose.orientation.y
        pose.orientation.z = pose_with_cov.pose.pose.orientation.z
        pose.orientation.w = pose_with_cov.pose.pose.orientation.w
        
        try:
            # Query latest available TF (don't use message timestamp because coloc_pose uses wall clock time while TF uses simulation time)
            tf = self.tf_buffer.lookup_transform(
                target_frame,
                src_frame,
                rclpy.time.Time(),  # Use Time() to query latest available TF
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            # do_transform_pose(Pose, TransformStamped) -> Pose
            pose_transformed = do_transform_pose(pose, tf)
            x = pose_transformed.position.x
            y = pose_transformed.position.y
            yaw = self.quaternion_to_yaw(pose_transformed.orientation)
            return x, y, yaw
        except Exception as e:
            self.get_logger().error(f'TF query failed: {target_frame} <- {src_frame}, error: {e}')
            return None
    
    def odom_callback(self, msg, robot_id):
        """Generic odometry callback"""
        self.robot_data[robot_id]['odom'] = msg
        self.calculate_error(robot_id)
    
    def coloc_callback(self, msg, robot_id):
        """Generic collaborative localization pose callback"""
        data = self.robot_data[robot_id]
        # Detect if coloc_pose actually updated
        if self._is_pose_updated(data, msg):
            data['coloc_pose'] = msg
            data['last_coloc_pose'] = msg
            self.calculate_error(robot_id)
        else:
            # coloc_pose not updated, skip this calculation
            data['skipped_count'] += 1
    
    def _is_pose_updated(self, data, new_msg):
        """Detect if coloc_pose actually updated (position or orientation changed)"""
        last_pose = data['last_coloc_pose']
        
        # First reception, count as update
        if last_pose is None:
            return True
        
        # Compare position and orientation, any change counts as update
        # Use small thresholds to avoid floating point errors
        pos_threshold = 1e-6
        ori_threshold = 1e-6
        
        dx = abs(new_msg.pose.pose.position.x - last_pose.pose.pose.position.x)
        dy = abs(new_msg.pose.pose.position.y - last_pose.pose.pose.position.y)
        dz = abs(new_msg.pose.pose.position.z - last_pose.pose.pose.position.z)
        
        dqx = abs(new_msg.pose.pose.orientation.x - last_pose.pose.pose.orientation.x)
        dqy = abs(new_msg.pose.pose.orientation.y - last_pose.pose.pose.orientation.y)
        dqz = abs(new_msg.pose.pose.orientation.z - last_pose.pose.pose.orientation.z)
        dqw = abs(new_msg.pose.pose.orientation.w - last_pose.pose.pose.orientation.w)
        
        # If position or orientation changed, count as update
        if (dx > pos_threshold or dy > pos_threshold or dz > pos_threshold or
            dqx > ori_threshold or dqy > ori_threshold or dqz > ori_threshold or dqw > ori_threshold):
            return True
        
        return False
    
    def calculate_error(self, robot_id):
        """Calculate localization error"""
        data = self.robot_data[robot_id]
        
        # If TF not ready yet, skip calculation
        if not self.tf_ready:
            return
        
        if data['odom'] is None or data['coloc_pose'] is None:
            # Temporary debug log
            if data['count'] == 0:  # Print only once
                self.get_logger().info(
                    f'[DEBUG] {robot_id}: odom={data["odom"] is not None}, '
                    f'coloc_pose={data["coloc_pose"] is not None}'
                )
            return
        
        # Extract ground truth
        gt_x = data['odom'].pose.pose.position.x
        gt_y = data['odom'].pose.pose.position.y
        gt_quat = data['odom'].pose.pose.orientation
        gt_yaw = self.quaternion_to_yaw(gt_quat)

        # Align collaborative localization estimate to ground truth frame before comparison
        # (avoid pseudo-errors caused by inconsistent map/odom coordinate systems)
        gt_frame = data['odom'].header.frame_id or 'odom'
        coloc_frame = data['coloc_pose'].header.frame_id or 'map'
        
        # Temporary debug log
        if data['count'] == 0:
            self.get_logger().info(
                f'[DEBUG] {robot_id}: Attempting TF transform {coloc_frame} -> {gt_frame}'
            )
        
        est = self.pose_in_frame(data['coloc_pose'], gt_frame)
        if est is None:
            # TF query failed
            if data['count'] == 0:  # Print only once
                self.get_logger().warning(
                    f'[DEBUG] {robot_id}: pose_in_frame returned None! '
                    f'coloc_frame={coloc_frame}, gt_frame={gt_frame}'
                )
            return
        est_x, est_y, est_yaw = est
        
        # Calculate error
        x_error = est_x - gt_x
        y_error = est_y - gt_y
        position_error = math.sqrt(x_error**2 + y_error**2)
        
        yaw_error = self.wrap_angle(est_yaw - gt_yaw)
        yaw_error_deg = math.degrees(yaw_error)
        
        # Record error
        error_data = {
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'gt_x': gt_x,
            'gt_y': gt_y,
            'gt_yaw': gt_yaw,
            'est_x': est_x,
            'est_y': est_y,
            'est_yaw': est_yaw,
            'x_error': x_error,
            'y_error': y_error,
            'position_error': position_error,
            'yaw_error': yaw_error,
            'yaw_error_deg': yaw_error_deg
        }
        
        data['errors'].append(error_data)
        data['count'] += 1
    
    def quaternion_to_yaw(self, quat):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y**2 + quat.z**2)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def wrap_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def print_statistics(self):
        """Print statistics"""
        self.get_logger().info('=' * 80)
        
        # Print valid sample counts
        sample_counts = ', '.join([f'{rid.upper()}={self.robot_data[rid]["count"]}' 
                                   for rid in self.robot_ids])
        self.get_logger().info(f'Collaborative localization evaluation - Valid samples: {sample_counts}')
        
        # Print skipped duplicate sample counts
        skipped_counts = ', '.join([f'{rid.upper()}={self.robot_data[rid]["skipped_count"]}' 
                                    for rid in self.robot_ids])
        self.get_logger().info(f'  Skipped duplicate samples: {skipped_counts}')
        
        for robot_id in self.robot_ids:
            data = self.robot_data[robot_id]
            
            if len(data['errors']) < 2:
                self.get_logger().info(f'{robot_id}: Insufficient data')
                continue
            
            # Calculate statistics
            pos_errors = [e['position_error'] for e in data['errors']]
            yaw_errors_deg = [e['yaw_error_deg'] for e in data['errors']]
            
            pos_rmse = math.sqrt(np.mean(np.square(pos_errors)))
            pos_mean = np.mean(pos_errors)
            pos_max = np.max(pos_errors)
            
            yaw_rmse = math.sqrt(np.mean(np.square(yaw_errors_deg)))
            yaw_mean = np.mean(np.abs(yaw_errors_deg))
            yaw_max = np.max(np.abs(yaw_errors_deg))
            
            self.get_logger().info(f'\n{robot_id.upper()}:')
            self.get_logger().info(f'  Position error - RMSE: {pos_rmse:.4f}m, '
                                 f'Mean: {pos_mean:.4f}m, Max: {pos_max:.4f}m')
            self.get_logger().info(f'  Heading error - RMSE: {yaw_rmse:.2f}°, '
                                 f'Mean: {yaw_mean:.2f}°, Max: {yaw_max:.2f}°')
        
        self.get_logger().info('=' * 80)
    
    def save_results(self):
        """Save results to files"""
        self.get_logger().info('Saving results...')
        
        for robot_id in self.robot_ids:
            data = self.robot_data[robot_id]
            
            if len(data['errors']) == 0:
                self.get_logger().warning(f'{robot_id}: No data to save')
                continue
            
            # Save CSV
            csv_file = os.path.join(
                self.result_dir, 
                f'{robot_id}_coloc_eval_{self.timestamp}.csv'
            )
            
            with open(csv_file, 'w') as f:
                # Write header
                f.write('timestamp,gt_x,gt_y,gt_yaw,est_x,est_y,est_yaw,'
                       'x_error,y_error,position_error,yaw_error,yaw_error_deg\n')
                
                # Write data
                for e in data['errors']:
                    f.write(f"{e['timestamp']:.6f},{e['gt_x']:.6f},{e['gt_y']:.6f},"
                           f"{e['gt_yaw']:.6f},{e['est_x']:.6f},{e['est_y']:.6f},"
                           f"{e['est_yaw']:.6f},{e['x_error']:.6f},{e['y_error']:.6f},"
                           f"{e['position_error']:.6f},{e['yaw_error']:.6f},"
                           f"{e['yaw_error_deg']:.2f}\n")
            
            self.get_logger().info(f'  Saved CSV: {csv_file}')
            
            # Save statistics report
            txt_file = os.path.join(
                self.result_dir,
                f'{robot_id}_coloc_statistics_{self.timestamp}.txt'
            )
            
            pos_errors = [e['position_error'] for e in data['errors']]
            yaw_errors_deg = [e['yaw_error_deg'] for e in data['errors']]
            
            pos_rmse = math.sqrt(np.mean(np.square(pos_errors)))
            pos_mean = np.mean(pos_errors)
            pos_max = np.max(pos_errors)
            
            yaw_rmse = math.sqrt(np.mean(np.square(yaw_errors_deg)))
            yaw_mean = np.mean(np.abs(yaw_errors_deg))
            yaw_max = np.max(np.abs(yaw_errors_deg))
            
            with open(txt_file, 'w') as f:
                f.write('=' * 50 + '\n')
                f.write(f'Collaborative Localization Evaluation Report - {robot_id}\n')
                f.write(f'Time: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}\n')
                f.write('=' * 50 + '\n\n')
                f.write(f'Valid sample count: {len(data["errors"])}\n')
                f.write(f'Skipped duplicate samples: {data["skipped_count"]}\n')
                f.write(f'Total callbacks: {len(data["errors"]) + data["skipped_count"]}\n')
                f.write(f'Valid sample ratio: {len(data["errors"])/(len(data["errors"])+data["skipped_count"])*100:.1f}%\n\n')
                f.write('【Position Error】\n')
                f.write(f'  RMSE: {pos_rmse:.4f} m\n')
                f.write(f'  Mean: {pos_mean:.4f} m\n')
                f.write(f'  Max:  {pos_max:.4f} m\n\n')
                f.write('【Heading Angle Error】\n')
                f.write(f'  RMSE: {yaw_rmse:.2f}°\n')
                f.write(f'  Mean: {yaw_mean:.2f}°\n')
                f.write(f'  Max:  {yaw_max:.2f}°\n')
            
            self.get_logger().info(f'  Saved statistics: {txt_file}')
        
        self.get_logger().info('Results saved successfully!')
    
    def shutdown(self):
        """Save results when node shuts down"""
        self.get_logger().info('\nCtrl+C detected, saving results...')
        self.save_results()
        self.get_logger().info('Node shut down')


def main(args=None):
    rclpy.init(args=args)
    node = PoseEvalColoc()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
