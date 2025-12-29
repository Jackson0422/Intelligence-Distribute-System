#!/usr/bin/env python3
"""
协同定位性能评估节点
Collaborative Localization Performance Evaluation Node

评估去中心化协同定位的性能，对比ground truth (odom)和协同定位估计 (coloc_pose)

保存结果到: evaluation_results/multibot/coloc/

作者: Distributed Intelligent Systems Course
"""

import rclpy
from rclpy.node import Node
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
    """协同定位性能评估节点"""
    
    def __init__(self):
        super().__init__('pose_eval_coloc')
        
        # 创建保存目录
        self.timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.result_dir = os.path.expanduser('~/ids_roswk/evaluation_results/multibot/coloc')
        os.makedirs(self.result_dir, exist_ok=True)
        
        self.get_logger().info('=' * 80)
        self.get_logger().info('协同定位性能评估节点已启动')
        self.get_logger().info(f'结果保存到: {self.result_dir}')
        self.get_logger().info('=' * 80)
        
        # 机器人0的数据
        self.tb3_0_data = {
            'odom': None,
            'coloc_pose': None,
            'last_coloc_pose': None,  # 用于检测coloc_pose是否真正更新
            'errors': [],
            'count': 0,
            'skipped_count': 0  # 跳过的重复样本计数
        }
        
        # 机器人1的数据
        self.tb3_1_data = {
            'odom': None,
            'coloc_pose': None,
            'last_coloc_pose': None,  # 用于检测coloc_pose是否真正更新
            'errors': [],
            'count': 0,
            'skipped_count': 0  # 跳过的重复样本计数
        }
        
        # 订阅ground truth (odom)
        self.odom_sub_0 = self.create_subscription(
            Odometry, '/odom', self.odom_callback_0, 10
        )
        self.odom_sub_1 = self.create_subscription(
            Odometry, '/tb3_1/odom', self.odom_callback_1, 10
        )
        
        # 订阅协同定位位姿
        self.coloc_sub_0 = self.create_subscription(
            PoseWithCovarianceStamped, '/coloc_pose', self.coloc_callback_0, 10
        )
        self.coloc_sub_1 = self.create_subscription(
            PoseWithCovarianceStamped, '/tb3_1/coloc_pose', self.coloc_callback_1, 10
        )
        
        # 定时打印统计信息 (5秒)
        self.timer = self.create_timer(5.0, self.print_statistics)
        
        # TF监听器：用于把不同frame下的位姿对齐后再计算误差
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # TF就绪标志
        self.tf_ready = False
        self.tf_check_count = 0
        
        # 启动TF检查定时器（每0.5秒检查一次，最多检查40次=20秒）
        self.tf_check_timer = self.create_timer(0.5, self._check_tf_ready)
        
        self.get_logger().info('等待TF树初始化...')
        self.get_logger().info('等待协同定位位姿数据...')

    def _check_tf_ready(self):
        """定时检查TF是否准备好（异步回调）"""
        if self.tf_ready:
            # 已经就绪，不再检查
            return
        
        self.tf_check_count += 1
        
        # 注意：检查的TF方向要和pose_in_frame()里实际使用的一致
        # pose_in_frame()里是lookup_transform(target='odom', source='map')
        required_transforms = [
            ('odom', 'map'),           # tb3_0: 从map到odom
            ('tb3_1/odom', 'map'),     # tb3_1: 从map到tb3_1/odom
        ]
        
        all_ready = True
        for target, source in required_transforms:
            try:
                self.tf_buffer.lookup_transform(
                    target, source,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                # TF可用，但不立即打印（避免重复）
            except Exception:
                all_ready = False
                break
        
        if all_ready:
            # 所有TF都准备好了
            self.tf_ready = True
            self.tf_check_timer.cancel()  # 停止检查
            self.get_logger().info('  ✓ TF树已就绪，开始计算误差')
        elif self.tf_check_count >= 40:
            # 超时（40次 * 0.5秒 = 20秒）
            self.get_logger().warning(
                f'  ✗ 等待TF超时（{self.tf_check_count * 0.5:.1f}秒），'
                '将继续运行但误差计算可能失败'
            )
            self.tf_ready = True  # 强制继续，避免永久阻塞
            self.tf_check_timer.cancel()

    def pose_in_frame(self, pose_with_cov: PoseWithCovarianceStamped, target_frame: str):
        """
        将PoseWithCovarianceStamped变换到target_frame，返回(x, y, yaw)或None（TF不可用）
        """
        src_frame = pose_with_cov.header.frame_id
        if not src_frame or src_frame == target_frame:
            x = pose_with_cov.pose.pose.position.x
            y = pose_with_cov.pose.pose.position.y
            yaw = self.quaternion_to_yaw(pose_with_cov.pose.pose.orientation)
            return x, y, yaw
        
        # 构造 Pose 对象（不是 PoseStamped）
        # do_transform_pose 期望的是 Pose 类型，而不是 PoseStamped
        pose = Pose()
        
        # 从 PoseWithCovarianceStamped 提取 Pose 数据
        pose.position.x = pose_with_cov.pose.pose.position.x
        pose.position.y = pose_with_cov.pose.pose.position.y
        pose.position.z = pose_with_cov.pose.pose.position.z
        pose.orientation.x = pose_with_cov.pose.pose.orientation.x
        pose.orientation.y = pose_with_cov.pose.pose.orientation.y
        pose.orientation.z = pose_with_cov.pose.pose.orientation.z
        pose.orientation.w = pose_with_cov.pose.pose.orientation.w
        
        try:
            # 查询最新可用的TF（不使用消息时间戳，因为coloc_pose用墙钟时间而TF用仿真时间）
            tf = self.tf_buffer.lookup_transform(
                target_frame,
                src_frame,
                rclpy.time.Time(),  # 使用Time()查询最新可用TF
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            # do_transform_pose(Pose, TransformStamped) -> Pose
            pose_transformed = do_transform_pose(pose, tf)
            x = pose_transformed.position.x
            y = pose_transformed.position.y
            yaw = self.quaternion_to_yaw(pose_transformed.orientation)
            return x, y, yaw
        except Exception as e:
            self.get_logger().error(f'TF查询失败: {target_frame} <- {src_frame}, 错误: {e}')
            return None
    
    def odom_callback_0(self, msg):
        """机器人0的里程计回调"""
        self.tb3_0_data['odom'] = msg
        self.calculate_error('tb3_0')
    
    def odom_callback_1(self, msg):
        """机器人1的里程计回调"""
        self.tb3_1_data['odom'] = msg
        self.calculate_error('tb3_1')
    
    def coloc_callback_0(self, msg):
        """机器人0的协同定位位姿回调"""
        # 检测coloc_pose是否真正更新
        if self._is_pose_updated(self.tb3_0_data, msg):
            self.tb3_0_data['coloc_pose'] = msg
            self.tb3_0_data['last_coloc_pose'] = msg
            self.calculate_error('tb3_0')
        else:
            # coloc_pose没有更新，跳过本次计算
            self.tb3_0_data['skipped_count'] += 1
    
    def coloc_callback_1(self, msg):
        """机器人1的协同定位位姿回调"""
        # 检测coloc_pose是否真正更新
        if self._is_pose_updated(self.tb3_1_data, msg):
            self.tb3_1_data['coloc_pose'] = msg
            self.tb3_1_data['last_coloc_pose'] = msg
            self.calculate_error('tb3_1')
        else:
            # coloc_pose没有更新，跳过本次计算
            self.tb3_1_data['skipped_count'] += 1
    
    def _is_pose_updated(self, data, new_msg):
        """检测coloc_pose是否真正更新（位置或角度有变化）"""
        last_pose = data['last_coloc_pose']
        
        # 首次接收，算作更新
        if last_pose is None:
            return True
        
        # 比较位置和方向，有任何变化就算更新
        # 使用较小的阈值来判断（避免浮点误差）
        pos_threshold = 1e-6
        ori_threshold = 1e-6
        
        dx = abs(new_msg.pose.pose.position.x - last_pose.pose.pose.position.x)
        dy = abs(new_msg.pose.pose.position.y - last_pose.pose.pose.position.y)
        dz = abs(new_msg.pose.pose.position.z - last_pose.pose.pose.position.z)
        
        dqx = abs(new_msg.pose.pose.orientation.x - last_pose.pose.pose.orientation.x)
        dqy = abs(new_msg.pose.pose.orientation.y - last_pose.pose.pose.orientation.y)
        dqz = abs(new_msg.pose.pose.orientation.z - last_pose.pose.pose.orientation.z)
        dqw = abs(new_msg.pose.pose.orientation.w - last_pose.pose.pose.orientation.w)
        
        # 只要位置或方向有任何变化，就认为更新了
        if (dx > pos_threshold or dy > pos_threshold or dz > pos_threshold or
            dqx > ori_threshold or dqy > ori_threshold or dqz > ori_threshold or dqw > ori_threshold):
            return True
        
        return False
    
    def calculate_error(self, robot_id):
        """计算定位误差"""
        data = self.tb3_0_data if robot_id == 'tb3_0' else self.tb3_1_data
        
        # 如果TF还没准备好，暂不计算
        if not self.tf_ready:
            return
        
        if data['odom'] is None or data['coloc_pose'] is None:
            # 临时调试日志
            if data['count'] == 0:  # 只打印一次
                self.get_logger().info(
                    f'[DEBUG] {robot_id}: odom={data["odom"] is not None}, '
                    f'coloc_pose={data["coloc_pose"] is not None}'
                )
            return
        
        # 提取ground truth
        gt_x = data['odom'].pose.pose.position.x
        gt_y = data['odom'].pose.pose.position.y
        gt_quat = data['odom'].pose.pose.orientation
        gt_yaw = self.quaternion_to_yaw(gt_quat)

        # 将协同定位估计对齐到ground truth所在frame再比较（避免map/odom坐标系不一致导致的伪误差）
        gt_frame = data['odom'].header.frame_id or 'odom'
        coloc_frame = data['coloc_pose'].header.frame_id or 'map'
        
        # 临时调试日志
        if data['count'] == 0:
            self.get_logger().info(
                f'[DEBUG] {robot_id}: 尝试TF变换 {coloc_frame} -> {gt_frame}'
            )
        
        est = self.pose_in_frame(data['coloc_pose'], gt_frame)
        if est is None:
            # TF查询失败
            if data['count'] == 0:  # 只打印一次
                self.get_logger().warning(
                    f'[DEBUG] {robot_id}: pose_in_frame返回None! '
                    f'coloc_frame={coloc_frame}, gt_frame={gt_frame}'
                )
            return
        est_x, est_y, est_yaw = est
        
        # 计算误差
        x_error = est_x - gt_x
        y_error = est_y - gt_y
        position_error = math.sqrt(x_error**2 + y_error**2)
        
        yaw_error = self.wrap_angle(est_yaw - gt_yaw)
        yaw_error_deg = math.degrees(yaw_error)
        
        # 记录误差
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
        """四元数转yaw角"""
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y**2 + quat.z**2)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def wrap_angle(self, angle):
        """角度归一化到[-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def print_statistics(self):
        """打印统计信息"""
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'协同定位性能评估 - 有效样本数: TB3_0={self.tb3_0_data["count"]}, '
                              f'TB3_1={self.tb3_1_data["count"]}')
        self.get_logger().info(f'  跳过的重复样本: TB3_0={self.tb3_0_data["skipped_count"]}, '
                              f'TB3_1={self.tb3_1_data["skipped_count"]}')
        
        for robot_id in ['tb3_0', 'tb3_1']:
            data = self.tb3_0_data if robot_id == 'tb3_0' else self.tb3_1_data
            
            if len(data['errors']) < 2:
                self.get_logger().info(f'{robot_id}: 数据不足')
                continue
            
            # 计算统计量
            pos_errors = [e['position_error'] for e in data['errors']]
            yaw_errors_deg = [e['yaw_error_deg'] for e in data['errors']]
            
            pos_rmse = math.sqrt(np.mean(np.square(pos_errors)))
            pos_mean = np.mean(pos_errors)
            pos_max = np.max(pos_errors)
            
            yaw_rmse = math.sqrt(np.mean(np.square(yaw_errors_deg)))
            yaw_mean = np.mean(np.abs(yaw_errors_deg))
            yaw_max = np.max(np.abs(yaw_errors_deg))
            
            self.get_logger().info(f'\n{robot_id.upper()}:')
            self.get_logger().info(f'  位置误差 - RMSE: {pos_rmse:.4f}m, '
                                 f'Mean: {pos_mean:.4f}m, Max: {pos_max:.4f}m')
            self.get_logger().info(f'  航向误差 - RMSE: {yaw_rmse:.2f}°, '
                                 f'Mean: {yaw_mean:.2f}°, Max: {yaw_max:.2f}°')
        
        self.get_logger().info('=' * 80)
    
    def save_results(self):
        """保存结果到文件"""
        self.get_logger().info('正在保存结果...')
        
        for robot_id in ['tb3_0', 'tb3_1']:
            data = self.tb3_0_data if robot_id == 'tb3_0' else self.tb3_1_data
            
            if len(data['errors']) == 0:
                self.get_logger().warning(f'{robot_id}: 没有数据可保存')
                continue
            
            # 保存CSV
            csv_file = os.path.join(
                self.result_dir, 
                f'{robot_id}_coloc_eval_{self.timestamp}.csv'
            )
            
            with open(csv_file, 'w') as f:
                # 写入表头
                f.write('timestamp,gt_x,gt_y,gt_yaw,est_x,est_y,est_yaw,'
                       'x_error,y_error,position_error,yaw_error,yaw_error_deg\n')
                
                # 写入数据
                for e in data['errors']:
                    f.write(f"{e['timestamp']:.6f},{e['gt_x']:.6f},{e['gt_y']:.6f},"
                           f"{e['gt_yaw']:.6f},{e['est_x']:.6f},{e['est_y']:.6f},"
                           f"{e['est_yaw']:.6f},{e['x_error']:.6f},{e['y_error']:.6f},"
                           f"{e['position_error']:.6f},{e['yaw_error']:.6f},"
                           f"{e['yaw_error_deg']:.2f}\n")
            
            self.get_logger().info(f'  保存CSV: {csv_file}')
            
            # 保存统计报告
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
                f.write(f'协同定位性能评估报告 - {robot_id}\n')
                f.write(f'时间: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}\n')
                f.write('=' * 50 + '\n\n')
                f.write(f'有效样本数量: {len(data["errors"])}\n')
                f.write(f'跳过的重复样本: {data["skipped_count"]}\n')
                f.write(f'总回调次数: {len(data["errors"]) + data["skipped_count"]}\n')
                f.write(f'有效样本比例: {len(data["errors"])/(len(data["errors"])+data["skipped_count"])*100:.1f}%\n\n')
                f.write('【位置误差】\n')
                f.write(f'  RMSE: {pos_rmse:.4f} m\n')
                f.write(f'  Mean: {pos_mean:.4f} m\n')
                f.write(f'  Max:  {pos_max:.4f} m\n\n')
                f.write('【航向角误差】\n')
                f.write(f'  RMSE: {yaw_rmse:.2f}°\n')
                f.write(f'  Mean: {yaw_mean:.2f}°\n')
                f.write(f'  Max:  {yaw_max:.2f}°\n')
            
            self.get_logger().info(f'  保存统计: {txt_file}')
        
        self.get_logger().info('结果保存完成！')
    
    def shutdown(self):
        """节点关闭时保存结果"""
        self.get_logger().info('\n检测到Ctrl+C，正在保存结果...')
        self.save_results()
        self.get_logger().info('节点已关闭')


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

