#!/usr/bin/env python3
"""
多机器人定位性能评估节点 (Multi-Robot Pose Evaluation Node)

功能：
- 同时记录多个机器人的 Ground Truth 和 AMCL 估计位姿
- 为每个机器人独立计算位置误差和航向角误差
- 分别输出每个机器人的误差曲线和统计指标

订阅话题：
- /odom, /amcl_pose: 机器人1（无命名空间）
- /tb3_1/odom, /tb3_1/amcl_pose: 机器人2

输出文件保存在 ~/ids_roswk/evaluation_results/multibot/
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
    position_error: float
    yaw_error: float


@dataclass
class RobotData:
    """单个机器人的数据存储"""
    namespace: str
    ground_truth_records: List[PoseRecord] = field(default_factory=list)
    amcl_records: List[PoseRecord] = field(default_factory=list)
    error_records: List[ErrorRecord] = field(default_factory=list)
    latest_gt: Optional[PoseRecord] = None
    latest_amcl: Optional[PoseRecord] = None
    using_amcl: bool = False


class MultiRobotPoseEvalNode(Node):
    """多机器人定位性能评估节点"""
    
    # 数据保存路径 - multibot子目录
    OUTPUT_DIR = os.path.expanduser('~/ids_roswk/evaluation_results/multibot')
    
    # ========== 机器人命名空间配置 ==========
    # 与 track_multibot.py 中的配置保持一致
    # '' (空字符串) = 第一个机器人，无命名空间
    # 'tb3_1' = 第二个机器人
    ROBOT_NAMESPACES = ['', 'tb3_1']
    # ========================================
    
    def __init__(self):
        super().__init__('multi_robot_pose_eval_node')
        
        # 为每个机器人创建数据存储
        self.robots: Dict[str, RobotData] = {}
        
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
        
        # 为每个机器人创建订阅
        for ns in self.ROBOT_NAMESPACES:
            # 用于显示和文件名的标识
            display_name = ns if ns else 'tb3_0'
            self.robots[display_name] = RobotData(namespace=display_name)
            
            # 根据命名空间构造话题名
            odom_topic = f'/{ns}/odom' if ns else '/odom'
            amcl_topic = f'/{ns}/amcl_pose' if ns else '/amcl_pose'
            
            # 订阅 odom 作为 Ground Truth
            self.create_subscription(
                Odometry,
                odom_topic,
                lambda msg, n=display_name: self.odom_callback(msg, n),
                sensor_qos
            )
            
            # 订阅 amcl_pose
            self.create_subscription(
                PoseWithCovarianceStamped,
                amcl_topic,
                lambda msg, n=display_name: self.amcl_callback(msg, n),
                reliable_qos
            )
            
            self.get_logger().info(f'[{display_name}] 订阅话题:')
            self.get_logger().info(f'  - {odom_topic} (Ground Truth)')
            self.get_logger().info(f'  - {amcl_topic} (AMCL估计)')
        
        # 定时器：定期计算误差
        self.eval_timer = self.create_timer(0.1, self.evaluate_errors)  # 10Hz
        
        # 定时器：定期输出统计信息
        self.stats_timer = self.create_timer(5.0, self.print_statistics)  # 每5秒
        
        self.get_logger().info('='*60)
        self.get_logger().info('多机器人定位性能评估节点已启动')
        self.get_logger().info(f'评估机器人: {list(self.robots.keys())}')
        self.get_logger().info(f'数据保存目录: {self.OUTPUT_DIR}')
        self.get_logger().info('='*60)
        self.get_logger().info('等待接收位姿数据...')
    
    def odom_callback(self, msg: Odometry, namespace: str):
        """处理odom数据作为Ground Truth"""
        robot = self.robots[namespace]
        
        pose = msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        yaw = self.quaternion_to_yaw(pose.orientation)
        timestamp = time.time()
        
        robot.latest_gt = PoseRecord(timestamp, x, y, yaw)
        
        # 开始记录
        if not self.is_recording:
            self.is_recording = True
            self.start_time = timestamp
            self.get_logger().info('开始记录数据')
        
        # 保存 Ground Truth 记录
        robot.ground_truth_records.append(robot.latest_gt)
        
        # 如果没有AMCL，也用odom作为估计值
        if not robot.using_amcl:
            robot.latest_amcl = PoseRecord(timestamp, x, y, yaw)
            robot.amcl_records.append(robot.latest_amcl)
    
    def amcl_callback(self, msg: PoseWithCovarianceStamped, namespace: str):
        """处理AMCL估计位姿"""
        robot = self.robots[namespace]
        
        if not robot.using_amcl:
            robot.using_amcl = True
            self.get_logger().info(f'[{namespace}] 检测到AMCL位姿，使用AMCL作为估计源')
        
        pose = msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        yaw = self.quaternion_to_yaw(pose.orientation)
        timestamp = time.time()
        
        robot.latest_amcl = PoseRecord(timestamp, x, y, yaw)
        
        if self.is_recording:
            robot.amcl_records.append(robot.latest_amcl)
    
    def evaluate_errors(self):
        """计算各机器人当前误差"""
        for ns, robot in self.robots.items():
            if robot.latest_gt is None or robot.latest_amcl is None:
                continue
            
            # 时间对齐检查
            time_diff = abs(robot.latest_gt.timestamp - robot.latest_amcl.timestamp)
            if time_diff > 0.5:
                continue
            
            # 计算误差
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
        """输出统计信息"""
        has_data = False
        for robot in self.robots.values():
            if len(robot.error_records) >= 10:
                has_data = True
                break
        
        if not has_data:
            total_gt = sum(len(r.ground_truth_records) for r in self.robots.values())
            total_est = sum(len(r.amcl_records) for r in self.robots.values())
            total_err = sum(len(r.error_records) for r in self.robots.values())
            self.get_logger().info(f'数据收集中... GT: {total_gt}, 估计: {total_est}, 误差: {total_err}')
            return
        
        self.get_logger().info('-'*60)
        self.get_logger().info('【多机器人定位误差统计】')
        
        for ns, robot in self.robots.items():
            stats = self.calculate_statistics(robot)
            self.get_logger().info(
                f'  [{ns}] 样本: {stats["count"]}, '
                f'位置RMSE: {stats["position_rmse"]:.4f}m, '
                f'航向RMSE: {math.degrees(stats["yaw_rmse"]):.2f}°'
            )
        
        self.get_logger().info('-'*60)
    
    def calculate_statistics(self, robot: RobotData) -> dict:
        """计算单个机器人的误差统计"""
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
        
        # 位置误差
        position_errors = [e.position_error for e in robot.error_records]
        position_rmse = math.sqrt(sum(e**2 for e in position_errors) / n)
        position_mean = sum(position_errors) / n
        position_max = max(position_errors)
        
        # 航向角误差
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
        """保存所有机器人的评估结果"""
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        self.get_logger().info('\n保存评估结果...')
        
        for ns, robot in self.robots.items():
            if len(robot.error_records) == 0:
                self.get_logger().warn(f'[{ns}] 没有误差数据，跳过保存')
                continue
            
            # 保存误差数据
            error_file = os.path.join(self.OUTPUT_DIR, f'{ns}_errors_{timestamp_str}.csv')
            with open(error_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'x_error', 'y_error', 'position_error', 'yaw_error'])
                for e in robot.error_records:
                    writer.writerow([e.timestamp, e.x_error, e.y_error, e.position_error, e.yaw_error])
            
            # 保存Ground Truth数据
            gt_file = os.path.join(self.OUTPUT_DIR, f'{ns}_ground_truth_{timestamp_str}.csv')
            with open(gt_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'x', 'y', 'yaw'])
                for r in robot.ground_truth_records:
                    writer.writerow([r.timestamp, r.x, r.y, r.yaw])
            
            # 保存估计位姿数据
            est_file = os.path.join(self.OUTPUT_DIR, f'{ns}_estimated_{timestamp_str}.csv')
            with open(est_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'x', 'y', 'yaw'])
                for r in robot.amcl_records:
                    writer.writerow([r.timestamp, r.x, r.y, r.yaw])
            
            # 保存统计结果
            stats = self.calculate_statistics(robot)
            stats_file = os.path.join(self.OUTPUT_DIR, f'{ns}_statistics_{timestamp_str}.txt')
            with open(stats_file, 'w') as f:
                f.write('='*50 + '\n')
                f.write(f'定位性能评估报告 - {ns}\n')
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
            
            self.get_logger().info(f'[{ns}] 数据已保存:')
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
    node = MultiRobotPoseEvalNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\n用户中断，正在保存数据...')
    finally:
        # 保存结果
        total_errors = sum(len(r.error_records) for r in node.robots.values())
        if total_errors > 0:
            node.save_results()
            
            # 打印最终统计
            node.get_logger().info('\n' + '='*60)
            node.get_logger().info('【最终评估结果】')
            for ns, robot in node.robots.items():
                stats = node.calculate_statistics(robot)
                if stats['count'] > 0:
                    node.get_logger().info(
                        f'  [{ns}] 样本: {stats["count"]}, '
                        f'位置RMSE: {stats["position_rmse"]:.4f}m, '
                        f'航向RMSE: {math.degrees(stats["yaw_rmse"]):.2f}°'
                    )
            node.get_logger().info('='*60)
        else:
            node.get_logger().warn('没有收集到足够的数据')
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

