#!/usr/bin/env python3
"""
去中心化协同定位代理节点
Decentralized Collaborative Localization Agent

实现基于Gossip协议的去中心化协同定位
每个机器人独立运行，通过P2P通信共享位姿信息，使用加权共识算法融合多源信息

作者: Distributed Intelligent Systems Course
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import json
import math
import numpy as np
from collections import defaultdict
from tf2_ros import TransformListener, Buffer
import rclpy.time
import rclpy.duration


class DecentralizedColocAgent(Node):
    """去中心化协同定位代理"""
    
    def __init__(self):
        super().__init__('decentralized_coloc_agent')
        
        # 声明参数
        self.declare_parameter('robot_id', 'tb3_0')
        self.declare_parameter('peer_ids', ['tb3_1'])
        self.declare_parameter('gossip_rate', 1.0)  # Hz
        self.declare_parameter('self_weight', 0.7)  # 自身权重（保留兼容性，实际不再用于平均）
        self.declare_parameter('peer_timeout', 3.0)  # 秒
        self.declare_parameter('correction_threshold', 0.001)  # 米
        # EKF协同定位新增参数
        self.declare_parameter('relative_obs_std_xy', 0.10)  # 相对观测xy噪声(m)
        self.declare_parameter('relative_obs_std_yaw', 0.087)  # 相对观测yaw噪声(rad, ~5°)
        self.declare_parameter('max_comm_range', 3.0)  # 通信距离门限(m)
        self.declare_parameter('mahalanobis_threshold', 9.0)  # 马氏距离门限(chi^2_3, p=0.05)
        
        # 获取参数
        self.robot_id = self.get_parameter('robot_id').value
        self.peer_ids = self.get_parameter('peer_ids').value
        self.gossip_rate = self.get_parameter('gossip_rate').value
        self.self_weight = self.get_parameter('self_weight').value
        self.peer_timeout = self.get_parameter('peer_timeout').value
        self.correction_threshold = self.get_parameter('correction_threshold').value
        
        # 参数验证
        if self.gossip_rate <= 0.0:
            raise ValueError(f"gossip_rate must be > 0, got {self.gossip_rate}")
        if not (0.0 <= self.self_weight <= 1.0):
            raise ValueError(f"self_weight must be in [0, 1], got {self.self_weight}")
        if self.peer_timeout <= 0.0:
            raise ValueError(f"peer_timeout must be > 0, got {self.peer_timeout}")
        
        # 状态变量
        self.amcl_pose = None  # 当前AMCL估计
        self.current_belief = None  # 当前belief
        self.peer_beliefs = defaultdict(dict)  # {peer_id: {'pose': ..., 'timestamp': ...}}
        self.peer_subs = []  # 保存订阅句柄，防止GC回收
        self.last_stats_time = 0.0  # 统计打印时间戳
        
        # TF监听器（用于获取相对位姿）
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info(f'初始化协同定位代理: {self.robot_id}')
        self.get_logger().info(f'  邻居: {self.peer_ids}')
        self.get_logger().info(f'  自身权重: {self.self_weight}')
        self.get_logger().info(f'  Gossip频率: {self.gossip_rate} Hz')
        
        # 设置通信
        self._setup_communication()
        
        # 启动Gossip定时器
        gossip_period = 1.0 / self.gossip_rate
        self.gossip_timer = self.create_timer(gossip_period, self.gossip_callback)
        
        # 统计计数器
        self.amcl_count = 0
        self.peer_msg_count = defaultdict(int)
        self.correction_count = 0
    
    def _setup_communication(self):
        """设置ROS通信"""
        
        # QoS配置
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 订阅本机器人的AMCL位姿
        if self.robot_id == 'tb3_0':
            amcl_topic = '/amcl_pose'
        else:
            amcl_topic = f'/{self.robot_id}/amcl_pose'
        
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            amcl_topic,
            self.amcl_callback,
            qos_reliable
        )
        
        # 发布协同定位结果
        if self.robot_id == 'tb3_0':
            coloc_topic = '/coloc_pose'
            belief_topic = '/coloc_belief'
        else:
            coloc_topic = f'/{self.robot_id}/coloc_pose'
            belief_topic = f'/{self.robot_id}/coloc_belief'
        
        self.coloc_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            coloc_topic,
            qos_reliable
        )
        
        # 发布belief（使用String + JSON编码）
        self.belief_pub = self.create_publisher(
            String,
            belief_topic,
            qos_best_effort
        )
        
        # 订阅邻居的belief
        for peer_id in self.peer_ids:
            if peer_id == 'tb3_0':
                peer_belief_topic = '/coloc_belief'
            else:
                peer_belief_topic = f'/{peer_id}/coloc_belief'
            
            # 保存订阅句柄，防止被垃圾回收
            sub = self.create_subscription(
                String,
                peer_belief_topic,
                lambda msg, pid=peer_id: self.peer_belief_callback(msg, pid),
                qos_best_effort
            )
            self.peer_subs.append(sub)
            
            self.get_logger().info(f'  订阅邻居belief: {peer_belief_topic}')
        
        self.get_logger().info(f'  订阅AMCL: {amcl_topic}')
        self.get_logger().info(f'  发布协同位姿: {coloc_topic}')
        self.get_logger().info(f'  发布belief: {belief_topic}')
    
    def amcl_callback(self, msg):
        """接收AMCL位姿估计"""
        self.amcl_pose = msg
        self.amcl_count += 1
        
        # 初始化belief
        if self.current_belief is None:
            self.current_belief = msg
            self.get_logger().info(f'初始化belief: ({msg.pose.pose.position.x:.3f}, '
                                 f'{msg.pose.pose.position.y:.3f})')
    
    def peer_belief_callback(self, msg, peer_id):
        """接收邻居的belief消息"""
        try:
            data = json.loads(msg.data)
            
            # 使用本地接收时间，避免时钟不同步问题
            current_time = self.get_clock().now().nanoseconds / 1e9
            
            # 解析位姿
            pose_data = {
                'position': {
                    'x': data['pose']['position']['x'],
                    'y': data['pose']['position']['y'],
                    'z': data['pose']['position']['z']
                },
                'orientation': {
                    'x': data['pose']['orientation']['x'],
                    'y': data['pose']['orientation']['y'],
                    'z': data['pose']['orientation']['z'],
                    'w': data['pose']['orientation']['w']
                },
                'covariance': data['pose']['covariance'],
                'recv_time': current_time,  # 本地接收时间（用于超时判断）
                'src_time': data['timestamp']  # 对方时间（可选，用于日志）
            }
            
            self.peer_beliefs[peer_id] = pose_data
            self.peer_msg_count[peer_id] += 1
            
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().warning(f'解析邻居{peer_id}的belief失败: {e}')
    
    def gossip_callback(self):
        """周期性Gossip回调"""
        
        if self.amcl_pose is None or self.current_belief is None:
            return
        
        # 1. 清理过期的邻居信息
        current_time = self.get_clock().now().nanoseconds / 1e9
        peers_to_remove = []
        for peer_id, belief_data in self.peer_beliefs.items():
            # 使用本地接收时间判断超时，避免时钟不同步问题
            if current_time - belief_data['recv_time'] > self.peer_timeout:
                peers_to_remove.append(peer_id)
        
        for peer_id in peers_to_remove:
            del self.peer_beliefs[peer_id]
            self.get_logger().warning(f'邻居{peer_id}超时，已移除')
        
        # 2. 广播当前belief
        self.broadcast_belief()
        
        # 3. 如果有邻居信息，执行共识融合
        if len(self.peer_beliefs) > 0:
            consensus_pose = self.compute_consensus()
            
            if consensus_pose is not None:
                # 计算修正距离
                dx = consensus_pose.pose.pose.position.x - self.current_belief.pose.pose.position.x
                dy = consensus_pose.pose.pose.position.y - self.current_belief.pose.pose.position.y
                correction_dist = math.sqrt(dx**2 + dy**2)
                
                # 总是发布融合后的位姿（不管是否超过阈值）
                self.publish_correction(consensus_pose)

                # B方案：只要有融合结果，就总是更新belief，保证广播给邻居的belief是“最新状态”
                # correction_threshold 仅用于统计“超过阈值的显著修正次数”，不再阻止belief更新
                self.current_belief = consensus_pose
                if correction_dist > self.correction_threshold:
                    self.correction_count += 1
        else:
            # 即使没有邻居信息，也发布当前belief（基于AMCL）
            self.publish_correction(self.current_belief)
        
        # 4. 定期打印统计信息（每10秒）
        # 使用时间间隔判断，避免除零问题
        if current_time - self.last_stats_time >= 10.0:
            self.print_statistics()
            self.last_stats_time = current_time
    
    def broadcast_belief(self):
        """广播当前belief"""
        if self.current_belief is None:
            return
        
        # 构造JSON消息
        belief_data = {
            'robot_id': self.robot_id,
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'pose': {
                'position': {
                    'x': self.current_belief.pose.pose.position.x,
                    'y': self.current_belief.pose.pose.position.y,
                    'z': self.current_belief.pose.pose.position.z
                },
                'orientation': {
                    'x': self.current_belief.pose.pose.orientation.x,
                    'y': self.current_belief.pose.pose.orientation.y,
                    'z': self.current_belief.pose.pose.orientation.z,
                    'w': self.current_belief.pose.pose.orientation.w
                },
                'covariance': list(self.current_belief.pose.covariance)
            }
        }
        
        msg = String()
        msg.data = json.dumps(belief_data)
        self.belief_pub.publish(msg)
    
    def compute_consensus(self):
        """
        使用EKF约束更新（替代原来的加权平均）
        状态只估计自己：x_i = [x, y, theta]
        邻居位姿作为约束，通过相对观测进行更新
        """
        # 先验：来自AMCL
        x_prior, y_prior, yaw_prior = self.extract_pose(self.amcl_pose)
        state_prior = np.array([x_prior, y_prior, yaw_prior])
        
        cov_amcl = np.array(self.amcl_pose.pose.covariance).reshape(6, 6)
        # 提取3x3子块 (x, y, yaw)
        P_prior = np.array([
            [cov_amcl[0, 0], cov_amcl[0, 1], cov_amcl[0, 5]],
            [cov_amcl[1, 0], cov_amcl[1, 1], cov_amcl[1, 5]],
            [cov_amcl[5, 0], cov_amcl[5, 1], cov_amcl[5, 5]]
        ])
        
        # 如果没有邻居，直接返回AMCL
        if len(self.peer_beliefs) == 0:
            return self.amcl_pose
        
        # 对每个邻居执行EKF更新
        state_est = state_prior.copy()
        P_est = P_prior.copy()
        
        max_range = self.get_parameter('max_comm_range').value
        maha_thresh = self.get_parameter('mahalanobis_threshold').value
        
        update_count = 0  # 成功更新次数
        
        for peer_id, belief_data in self.peer_beliefs.items():
            # 获取邻居位姿
            x_j = belief_data['position']['x']
            y_j = belief_data['position']['y']
            peer_quat = belief_data['orientation']
            theta_j = self.quaternion_to_yaw(peer_quat)
            
            peer_cov = np.array(belief_data['covariance']).reshape(6, 6)
            cov_j = np.array([
                [peer_cov[0, 0], peer_cov[0, 1], peer_cov[0, 5]],
                [peer_cov[1, 0], peer_cov[1, 1], peer_cov[1, 5]],
                [peer_cov[5, 0], peer_cov[5, 1], peer_cov[5, 5]]
            ])
            
            # 生成相对观测
            z_rel = self.generate_relative_observation(peer_id)
            if z_rel is None:
                continue
            
            # 门控1：通信距离
            if np.linalg.norm(z_rel[:2]) > max_range:
                self.get_logger().debug(f'门控拒绝{peer_id}: 距离{np.linalg.norm(z_rel[:2]):.2f}m > {max_range}m')
                continue
            
            # 预测测量：h(x_i, x_j) = 在自己坐标系下看邻居
            x_i, y_i, theta_i = state_est
            c, s = math.cos(theta_i), math.sin(theta_i)
            
            dx = x_j - x_i
            dy = y_j - y_i
            
            z_pred = np.array([
                c * dx + s * dy,
                -s * dx + c * dy,
                self.wrap_angle(theta_j - theta_i)
            ])
            
            # 计算雅可比 H_i = ∂h/∂x_i （3x3）
            H = np.array([
                [-c,        -s,         -s * dx + c * dy],
                [s,         -c,         -c * dx - s * dy],
                [0,          0,         -1]
            ])
            
            # 有效观测噪声协方差 R_eff = R + 邻居不确定性贡献
            std_xy = self.get_parameter('relative_obs_std_xy').value
            std_yaw = self.get_parameter('relative_obs_std_yaw').value
            R = np.diag([std_xy**2, std_xy**2, std_yaw**2])
            
            # 简化：把邻居协方差对角元素加到R（保守估计）
            R_eff = R + np.diag([cov_j[0, 0], cov_j[1, 1], cov_j[2, 2]])
            
            # 计算残差（innovation）
            innovation = z_rel - z_pred
            innovation[2] = self.wrap_angle(innovation[2])  # ❗wrap角度残差
            
            # 计算新息协方差
            S = H @ P_est @ H.T + R_eff
            
            # 门控2：马氏距离
            try:
                S_inv = np.linalg.inv(S)
                maha_dist = innovation.T @ S_inv @ innovation
                if maha_dist > maha_thresh:
                    self.get_logger().debug(f'门控拒绝{peer_id}: 马氏距离{maha_dist:.1f} > {maha_thresh}')
                    continue
            except np.linalg.LinAlgError:
                self.get_logger().warning(f'协方差矩阵奇异，跳过{peer_id}')
                continue
            
            # EKF更新
            K = P_est @ H.T @ S_inv
            state_est = state_est + K @ innovation
            state_est[2] = self.wrap_angle(state_est[2])  # ❗wrap yaw
            P_est = (np.eye(3) - K @ H) @ P_est
            
            update_count += 1
        
        # 记录更新信息
        if update_count > 0:
            self.get_logger().debug(f'EKF更新: {update_count}/{len(self.peer_beliefs)} 邻居')
        
        # 构造输出消息
        consensus_pose = PoseWithCovarianceStamped()
        consensus_pose.header.frame_id = self.amcl_pose.header.frame_id
        consensus_pose.header.stamp = self.get_clock().now().to_msg()
        
        consensus_pose.pose.pose.position.x = state_est[0]
        consensus_pose.pose.pose.position.y = state_est[1]
        consensus_pose.pose.pose.position.z = 0.0
        
        quat = self.yaw_to_quaternion(state_est[2])
        consensus_pose.pose.pose.orientation.x = quat[0]
        consensus_pose.pose.pose.orientation.y = quat[1]
        consensus_pose.pose.pose.orientation.z = quat[2]
        consensus_pose.pose.pose.orientation.w = quat[3]
        
        # 填充协方差（6x6）
        cov_6x6 = np.zeros((6, 6))
        cov_6x6[0:2, 0:2] = P_est[0:2, 0:2]  # x,y块
        cov_6x6[0:2, 5] = P_est[0:2, 2]      # x,y与yaw交叉
        cov_6x6[5, 0:2] = P_est[2, 0:2]
        cov_6x6[5, 5] = P_est[2, 2]          # yaw
        consensus_pose.pose.covariance = list(cov_6x6.flatten())
        
        return consensus_pose
    
    def publish_correction(self, pose):
        """发布修正后的位姿"""
        self.coloc_pub.publish(pose)
    
    def extract_pose(self, pose_msg):
        """从PoseWithCovarianceStamped提取x, y, yaw"""
        x = pose_msg.pose.pose.position.x
        y = pose_msg.pose.pose.position.y
        quat = pose_msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw({
            'x': quat.x, 'y': quat.y, 'z': quat.z, 'w': quat.w
        })
        return x, y, yaw
    
    def quaternion_to_yaw(self, quat):
        """四元数转yaw角"""
        siny_cosp = 2 * (quat['w'] * quat['z'] + quat['x'] * quat['y'])
        cosy_cosp = 1 - 2 * (quat['y']**2 + quat['z']**2)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def yaw_to_quaternion(self, yaw):
        """yaw角转四元数 (roll=0, pitch=0)"""
        return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]
    
    def wrap_angle(self, angle):
        """将角度归一化到[-π, π]"""
        return math.atan2(math.sin(angle), math.cos(angle))
    
    def _get_base_frame(self, robot_id):
        """
        获取机器人的base_footprint frame名称
        处理tb3_0的特殊命名（无前缀，后向兼容）
        
        Args:
            robot_id: 'tb3_0', 'tb3_1', 'tb3_2', 'tb3_3'
        
        Returns:
            TF frame名称（tb3_0无前缀，其他有命名空间前缀）
        """
        if robot_id == 'tb3_0':
            return 'base_footprint'
        else:
            return f'{robot_id}/base_footprint'
    
    def generate_relative_observation(self, peer_id):
        """
        从TF获取相对位姿并加噪声，模拟UWB/视觉相对传感器
        返回: (dx, dy, dtheta) 在自己坐标系下，或None（如果TF不可用）
        """
        try:
            # 获取ground truth相对变换：T_self^{-1} * T_peer
            # 使用辅助方法处理tb3_0的特殊命名
            self_frame = self._get_base_frame(self.robot_id)
            peer_frame = self._get_base_frame(peer_id)

            # 注意：这里不能用阻塞式lookup_transform(timeout=0.1)。
            # 在TF短暂不可用时会把gossip循环硬限速到~10Hz（与timeout一致），从而让gossip_rate=30Hz失效。
            # 采取“先快速检查 -> 立即查询”的方式：拿不到就跳过本次观测，不阻塞主循环。
            if not self.tf_buffer.can_transform(
                self_frame, peer_frame,
                rclpy.time.Time(),  # 最新可用
                timeout=rclpy.duration.Duration(seconds=0.0)
            ):
                return None

            transform = self.tf_buffer.lookup_transform(
                self_frame, peer_frame,
                rclpy.time.Time(),  # 最新可用
                timeout=rclpy.duration.Duration(seconds=0.0)
            )
            
            # 提取相对位姿
            dx = transform.transform.translation.x
            dy = transform.transform.translation.y
            quat = transform.transform.rotation
            dtheta = self.quaternion_to_yaw({'x': quat.x, 'y': quat.y, 'z': quat.z, 'w': quat.w})
            
            # 加高斯噪声
            std_xy = self.get_parameter('relative_obs_std_xy').value
            std_yaw = self.get_parameter('relative_obs_std_yaw').value
            
            dx_noisy = dx + np.random.normal(0, std_xy)
            dy_noisy = dy + np.random.normal(0, std_xy)
            dtheta_noisy = self.wrap_angle(dtheta + np.random.normal(0, std_yaw))
            
            return np.array([dx_noisy, dy_noisy, dtheta_noisy])
            
        except Exception as e:
            self.get_logger().debug(f'无法获取{peer_id}的相对观测: {e}')
            return None
    
    def print_statistics(self):
        """打印统计信息"""
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'协同定位统计 - {self.robot_id}')
        self.get_logger().info(f'  AMCL消息数: {self.amcl_count}')
        self.get_logger().info(f'  修正次数: {self.correction_count}')
        self.get_logger().info(f'  活跃邻居数: {len(self.peer_beliefs)}')
        for peer_id, count in self.peer_msg_count.items():
            active = '✓' if peer_id in self.peer_beliefs else '✗'
            self.get_logger().info(f'    {peer_id}: {count} 消息 [{active}]')
        
        if self.current_belief is not None:
            x, y, yaw = self.extract_pose(self.current_belief)
            self.get_logger().info(f'  当前belief: ({x:.3f}, {y:.3f}, {math.degrees(yaw):.1f}°)')
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    node = DecentralizedColocAgent()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点被用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

