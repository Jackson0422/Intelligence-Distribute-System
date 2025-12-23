#!/usr/bin/env python3
"""
定位性能评估节点 (Pose Evaluation Node)

功能：
- 同步记录 Ground Truth (odom) 和 AMCL 估计位姿
- 时间对齐计算位置误差和航向角误差
- 输出误差曲线和统计指标（RMSE等）

订阅话题：
- /odom: Ground Truth (Gazebo仿真中odom是理想值)
- /amcl_pose: AMCL 估计位姿

该节点不参与路径规划或运动控制，仅评估定位系统性能。
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
    """位姿记录"""
    timestamp: float
    x: float
    y: float
    yaw: float


@dataclass
class ErrorRecord:
    """误差记录"""
    timestamp: float
    x_error: float
    y_error: float
    position_error: float  # 欧氏距离误差
    yaw_error: float


class PoseEvalNode(Node):
    """定位性能评估节点"""
    
    # 数据保存路径
    OUTPUT_DIR = os.path.expanduser('~/ids_roswk/evaluation_results')
    
    def __init__(self):
        super().__init__('pose_eval_node')
        
        # 数据存储
        self.ground_truth_records: List[PoseRecord] = []
        self.amcl_records: List[PoseRecord] = []
        self.error_records: List[ErrorRecord] = []
        
        # 最新位姿（用于实时计算）
        self.latest_gt: Optional[PoseRecord] = None
        self.latest_amcl: Optional[PoseRecord] = None
        
        # 评估状态
        self.is_recording = False
        self.start_time: Optional[float] = None
        
        # 创建输出目录
        os.makedirs(self.OUTPUT_DIR, exist_ok=True)
        
        # QoS设置
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
        
        # 订阅 odom 作为 Ground Truth（Gazebo仿真中odom是理想值）
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            sensor_qos
        )
        
        # 订阅 AMCL 估计位姿
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            reliable_qos
        )
        
        # 定时器：定期计算和记录误差
        self.eval_timer = self.create_timer(0.1, self.evaluate_error)  # 10Hz
        
        # 定时器：定期输出统计信息
        self.stats_timer = self.create_timer(5.0, self.print_statistics)  # 每5秒
        
        self.get_logger().info('='*60)
        self.get_logger().info('定位性能评估节点已启动')
        self.get_logger().info('Ground Truth: /odom (Gazebo仿真理想值)')
        self.get_logger().info('估计位姿: /amcl_pose')
        self.get_logger().info(f'数据保存目录: {self.OUTPUT_DIR}')
        self.get_logger().info('='*60)
        self.get_logger().info('等待接收位姿数据...')
        
        # 标记是否使用AMCL（如果收到AMCL消息则为True）
        self.using_amcl = False

    def odom_callback(self, msg: Odometry):
        """处理odom数据作为Ground Truth（Gazebo仿真中odom是理想值）"""
        pose = msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        yaw = self.quaternion_to_yaw(pose.orientation)
        
        timestamp = time.time()
        
        # odom 作为 Ground Truth
        self.latest_gt = PoseRecord(timestamp, x, y, yaw)
        
        # 开始记录
        if not self.is_recording:
            self.is_recording = True
            self.start_time = timestamp
            self.get_logger().info('开始记录数据 (odom作为Ground Truth)')
        
        # 保存 Ground Truth 记录
        self.ground_truth_records.append(self.latest_gt)
        
        # 如果没有AMCL，也用odom作为估计值（此时误差为0，仅用于测试）
        if not self.using_amcl:
            self.latest_amcl = PoseRecord(timestamp, x, y, yaw)
            self.amcl_records.append(self.latest_amcl)

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        """处理AMCL估计位姿"""
        if not self.using_amcl:
            self.using_amcl = True
            self.get_logger().info('检测到 AMCL 位姿，使用 AMCL 作为估计源')
        
        pose = msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        yaw = self.quaternion_to_yaw(pose.orientation)
        
        timestamp = time.time()
        
        self.latest_amcl = PoseRecord(timestamp, x, y, yaw)
        
        if self.is_recording:
            self.amcl_records.append(self.latest_amcl)

    def evaluate_error(self):
        """计算当前误差"""
        if self.latest_gt is None or self.latest_amcl is None:
            return
        
        # 时间对齐检查（两个位姿时间差不超过0.5秒）
        time_diff = abs(self.latest_gt.timestamp - self.latest_amcl.timestamp)
        if time_diff > 0.5:
            return
        
        # 计算误差
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
        """输出统计信息"""
        if len(self.error_records) < 10:
            self.get_logger().info(f'数据收集中... GT: {len(self.ground_truth_records)}, '
                                   f'估计: {len(self.amcl_records)}, '
                                   f'误差记录: {len(self.error_records)}')
            return
        
        # 计算统计指标
        stats = self.calculate_statistics()
        
        self.get_logger().info('-'*50)
        self.get_logger().info(f'【定位误差统计】 (样本数: {stats["count"]})')
        self.get_logger().info(f'  位置 RMSE:    {stats["position_rmse"]:.4f} m')
        self.get_logger().info(f'  位置 Mean:    {stats["position_mean"]:.4f} m')
        self.get_logger().info(f'  位置 Max:     {stats["position_max"]:.4f} m')
        self.get_logger().info(f'  航向角 RMSE:  {math.degrees(stats["yaw_rmse"]):.2f}°')
        self.get_logger().info(f'  航向角 Mean:  {math.degrees(stats["yaw_mean"]):.2f}°')
        self.get_logger().info(f'  航向角 Max:   {math.degrees(stats["yaw_max"]):.2f}°')
        self.get_logger().info('-'*50)

    def calculate_statistics(self) -> dict:
        """计算误差统计指标"""
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
        
        # 位置误差
        position_errors = [e.position_error for e in self.error_records]
        position_rmse = math.sqrt(sum(e**2 for e in position_errors) / n)
        position_mean = sum(position_errors) / n
        position_max = max(position_errors)
        
        # 航向角误差
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
        """保存评估结果到文件"""
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # 保存误差数据
        error_file = os.path.join(self.OUTPUT_DIR, f'errors_{timestamp_str}.csv')
        with open(error_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'x_error', 'y_error', 'position_error', 'yaw_error'])
            for e in self.error_records:
                writer.writerow([e.timestamp, e.x_error, e.y_error, e.position_error, e.yaw_error])
        
        # 保存Ground Truth数据
        gt_file = os.path.join(self.OUTPUT_DIR, f'ground_truth_{timestamp_str}.csv')
        with open(gt_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'x', 'y', 'yaw'])
            for r in self.ground_truth_records:
                writer.writerow([r.timestamp, r.x, r.y, r.yaw])
        
        # 保存估计位姿数据
        est_file = os.path.join(self.OUTPUT_DIR, f'estimated_{timestamp_str}.csv')
        with open(est_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'x', 'y', 'yaw'])
            for r in self.amcl_records:
                writer.writerow([r.timestamp, r.x, r.y, r.yaw])
        
        # 保存统计结果
        stats = self.calculate_statistics()
        stats_file = os.path.join(self.OUTPUT_DIR, f'statistics_{timestamp_str}.txt')
        with open(stats_file, 'w') as f:
            f.write('='*50 + '\n')
            f.write('定位性能评估报告\n')
            f.write(f'时间: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}\n')
            f.write('='*50 + '\n\n')
            f.write(f'样本数量: {stats["count"]}\n\n')
            f.write('【位置误差】\n')
            f.write(f'  RMSE: {stats["position_rmse"]:.4f} m\n')
            f.write(f'  Mean: {stats["position_mean"]:.4f} m\n')
            f.write(f'  Max:  {stats["position_max"]:.4f} m\n\n')
            f.write('【航向角误差】\n')
            f.write(f'  RMSE: {math.degrees(stats["yaw_rmse"]):.2f}°\n')
            f.write(f'  Mean: {math.degrees(stats["yaw_mean"]):.2f}°\n')
            f.write(f'  Max:  {math.degrees(stats["yaw_max"]):.2f}°\n')
        
        self.get_logger().info(f'结果已保存到: {self.OUTPUT_DIR}')
        self.get_logger().info(f'  - {os.path.basename(error_file)}')
        self.get_logger().info(f'  - {os.path.basename(gt_file)}')
        self.get_logger().info(f'  - {os.path.basename(est_file)}')
        self.get_logger().info(f'  - {os.path.basename(stats_file)}')

    def quaternion_to_yaw(self, q) -> float:
        """四元数转航向角"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle: float) -> float:
        """将角度归一化到 [-pi, pi]"""
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
        node.get_logger().info('\n用户中断，正在保存数据...')
    finally:
        # 保存结果
        if len(node.error_records) > 0:
            node.save_results()
            # 打印最终统计
            node.get_logger().info('\n' + '='*60)
            node.get_logger().info('【最终评估结果】')
            stats = node.calculate_statistics()
            node.get_logger().info(f'  总样本数: {stats["count"]}')
            node.get_logger().info(f'  位置 RMSE: {stats["position_rmse"]:.4f} m')
            node.get_logger().info(f'  航向角 RMSE: {math.degrees(stats["yaw_rmse"]):.2f}°')
            node.get_logger().info('='*60)
        else:
            node.get_logger().warn('没有收集到足够的数据')
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

