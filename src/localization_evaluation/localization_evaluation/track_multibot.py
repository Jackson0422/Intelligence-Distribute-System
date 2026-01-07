#!/usr/bin/env python3
"""
多机器人轨迹发布节点 (Multi-Robot Track Publisher)

功能：
- 同时控制2-4个机器人执行各自的轨迹
- 使用RRT路径规划为每个机器人规划安全路径
- 使用多线程并行控制多个机器人
- 支持通过参数配置机器人数量（默认2个）

发布话题：
- /cmd_vel: 机器人1速度命令（无命名空间）
- /tb3_1/cmd_vel: 机器人2速度命令
- /tb3_2/cmd_vel: 机器人3速度命令（可选）
- /tb3_3/cmd_vel: 机器人4速度命令（可选）
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
import threading

from localization_evaluation.pathplan import plan_multi_waypoints, TurtleBot3WorldMap


class RobotController:
    """单个机器人控制器"""
    
    def __init__(self, node, namespace, start_pos, waypoints, seed=42):
        """
        初始化机器人控制器
        
        Args:
            node: ROS节点
            namespace: 机器人命名空间
            start_pos: 起始位置 (x, y)
            waypoints: 关键航点列表
            seed: RRT随机种子
        """
        self.node = node
        self.namespace = namespace
        self.start_x, self.start_y = start_pos
        self.current_x = self.start_x
        self.current_y = self.start_y
        self.current_yaw = 0.0
        self.key_waypoints = waypoints
        self.seed = seed
        
        # 运动参数
        self.linear_speed = 0.15   # 线速度 m/s
        self.angular_speed = 0.5   # 角速度 rad/s
        
        # 里程计数据（用于闭环控制）
        self.odom_x = self.start_x
        self.odom_y = self.start_y
        self.odom_yaw = 0.0
        self.odom_received = False
        
        # 发布者 - 根据命名空间发布到不同话题
        cmd_vel_topic = f'/{namespace}/cmd_vel' if namespace else '/cmd_vel'
        self.cmd_vel_pub = node.create_publisher(Twist, cmd_vel_topic, 10)
        
        # 订阅者 - 订阅里程计话题
        odom_topic = f'/{namespace}/odom' if namespace else '/odom'
        self.odom_sub = node.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )
        
        # 用于日志显示的名称
        self.display_name = namespace if namespace else 'tb3_0'
        
        node.get_logger().info(f'[{self.display_name}] 控制器初始化完成')
        node.get_logger().info(f'[{self.display_name}] 发布话题: {cmd_vel_topic}')
        node.get_logger().info(f'[{self.display_name}] 订阅话题: {odom_topic}')
        node.get_logger().info(f'[{self.display_name}] 起始位置: ({self.start_x:.2f}, {self.start_y:.2f})')
    
    def quaternion_to_euler(self, x, y, z, w):
        """将四元数转换为欧拉角 yaw（绕Z轴旋转）"""
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def odom_callback(self, msg):
        """里程计回调函数 - 更新机器人当前位置"""
        # 提取位置
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        
        # 提取方向（四元数转欧拉角）
        orientation = msg.pose.pose.orientation
        self.odom_yaw = self.quaternion_to_euler(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        
        self.odom_received = True
    
    def execute_trajectory(self):
        """执行轨迹"""
        self.node.get_logger().info(f'[{self.display_name}] 开始RRT路径规划...')
        self.node.get_logger().info(f'[{self.display_name}] 关键航点数: {len(self.key_waypoints)}')
        
        # ========== 输出关键航点 ==========
        self.node.get_logger().info(f'[{self.display_name}] ===== 关键航点列表 =====')
        for idx, (x, y) in enumerate(self.key_waypoints):
            self.node.get_logger().info(f'[{self.display_name}]   关键点{idx}: ({x:.2f}, {y:.2f})')
        self.node.get_logger().info(f'[{self.display_name}] ========================')
        
        # 使用RRT规划完整路径
        planned_path = plan_multi_waypoints(self.key_waypoints, seed=self.seed)
        
        if planned_path is None or len(planned_path) == 0:
            self.node.get_logger().error(f'[{self.display_name}] 路径规划失败！')
            return
        
        self.node.get_logger().info(f'[{self.display_name}] 规划完成，共 {len(planned_path)} 个航点')
        
        # ========== 输出完整规划路径 ==========
        self.node.get_logger().info(f'[{self.display_name}] ===== RRT规划路径 =====')
        for idx, (x, y) in enumerate(planned_path):
            self.node.get_logger().info(f'[{self.display_name}]   路径点{idx}: ({x:.3f}, {y:.3f})')
        self.node.get_logger().info(f'[{self.display_name}] =======================')
        
        # 执行路径
        total_waypoints = len(planned_path)
        for i, (x, y) in enumerate(planned_path):
            self.node.get_logger().info(
                f'[{self.display_name}] [{i+1}/{total_waypoints}] 目标: ({x:.2f}, {y:.2f})'
            )
            
            # ========== 显示当前位置 ==========
            self.node.get_logger().info(
                f'[{self.display_name}]   当前位置: ({self.current_x:.3f}, {self.current_y:.3f}, {self.current_yaw:.2f}rad)'
            )
            
            self.navigate_to_point(x, y)
            
            # ========== 显示到达后的位置 ==========
            self.node.get_logger().info(
                f'[{self.display_name}]   到达位置: ({self.current_x:.3f}, {self.current_y:.3f}, {self.current_yaw:.2f}rad)'
            )
            
            # ========== 计算位置误差 ==========
            error = math.sqrt((self.current_x - x)**2 + (self.current_y - y)**2)
            if error > 0.1:
                self.node.get_logger().warn(
                    f'[{self.display_name}]   ⚠️ 位置误差: {error:.3f}m (目标偏差较大!)'
                )
            else:
                self.node.get_logger().info(
                    f'[{self.display_name}]   ✓ 位置误差: {error:.3f}m'
                )
            
            self.stop_robot()
            time.sleep(0.3)
        
        self.node.get_logger().info(f'[{self.display_name}] 轨迹执行完成！')
        self.stop_robot()
    
    def navigate_to_point(self, target_x, target_y):
        """导航到目标点 (简化版：纯时间控制)"""
        # 计算需要移动的距离和角度（基于当前估计位置）
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        target_yaw = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(target_yaw - self.current_yaw)
        
        # 出发前检查障碍物
        self.check_obstacle_clearance(self.current_x, self.current_y)
        
        # 1. 先转向目标方向（时间控制）
        if abs(angle_diff) > 0.05:
            self.turn_angle(angle_diff)
        
        # 2. 直行到目标点（时间控制）
        if distance > 0.02:
            self.move_distance(distance)
        
        # 3. 更新当前位置估计
        self.current_x = target_x
        self.current_y = target_y
        self.current_yaw = target_yaw
        
        # 4. 如果里程计可用，报告实际位置和误差
        if self.odom_received:
            time.sleep(0.3)  # 等待里程计更新
            actual_x, actual_y = self.odom_x, self.odom_y
            
            self.node.get_logger().info(
                f'[{self.display_name}]   到达位置: ({actual_x:.3f}, {actual_y:.3f}, {self.odom_yaw:.2f}rad)'
            )
            
            # 计算位置误差
            error = math.sqrt((actual_x - target_x)**2 + (actual_y - target_y)**2)
            if error > 0.1:
                self.node.get_logger().warn(
                    f'[{self.display_name}]   ⚠️ 位置误差: {error:.3f}m (目标偏差较大!)'
                )
            else:
                self.node.get_logger().info(
                    f'[{self.display_name}]   ✓ 位置误差: {error:.3f}m'
                )
        
        # 到达后检查障碍物
        self.check_obstacle_clearance(self.current_x, self.current_y)
    
    def navigate_to_point_openloop(self, target_x, target_y):
        """导航到目标点 (使用航位推算 - 开环控制，备用方法)"""
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        target_yaw = math.atan2(dy, dx)
        
        # ========== 出发前检查障碍物 ==========
        self.node.get_logger().info(f'[{self.display_name}]   出发前检查:')
        self.check_obstacle_clearance(self.current_x, self.current_y)
        
        # 计算需要转的角度
        angle_diff = self.normalize_angle(target_yaw - self.current_yaw)
        
        # 1. 先转向目标方向
        if abs(angle_diff) > 0.05:
            self.turn_angle(angle_diff)
        
        # 2. 直行到目标点
        if distance > 0.02:
            self.move_distance(distance)
        
        # 更新当前位置
        self.current_x = target_x
        self.current_y = target_y
        self.current_yaw = target_yaw
        
        # ========== 到达后检查障碍物 ==========
        self.node.get_logger().info(f'[{self.display_name}]   到达后检查:')
        self.check_obstacle_clearance(self.current_x, self.current_y)
    
    def turn_angle(self, angle):
        """原地转动指定角度（弧度）"""
        twist = Twist()
        
        if angle > 0:
            twist.angular.z = self.angular_speed
        else:
            twist.angular.z = -self.angular_speed
        
        duration = abs(angle) / self.angular_speed
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        
        self.stop_robot()
        time.sleep(0.1)
        self.current_yaw = self.normalize_angle(self.current_yaw + angle)
    
    def move_distance(self, distance):
        """直行指定距离"""
        twist = Twist()
        twist.linear.x = self.linear_speed
        
        duration = distance / self.linear_speed
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        
        self.stop_robot()
        time.sleep(0.1)
    
    def normalize_angle(self, angle):
        """将角度归一化到 [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def stop_robot(self):
        """停止机器人"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        for _ in range(5):
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.02)
    
    def check_obstacle_clearance(self, x, y):
        """检查当前位置与障碍物的距离"""
        world_map = TurtleBot3WorldMap()
        
        min_distance = float('inf')
        closest_obstacle = None
        
        for idx, obs in enumerate(world_map.obstacles):
            dist = math.sqrt((x - obs.x)**2 + (y - obs.y)**2)
            actual_clearance = dist - obs.radius  # 表面距离
            
            if actual_clearance < min_distance:
                min_distance = actual_clearance
                closest_obstacle = (obs.x, obs.y, obs.radius)
        
        # 警告：距离障碍物太近
        if min_distance < 0.15:
            self.node.get_logger().error(
                f'[{self.display_name}] ⚠️⚠️ 危险！距离障碍物仅 {min_distance:.3f}m！'
                f' 障碍物位置: ({closest_obstacle[0]:.2f}, {closest_obstacle[1]:.2f}), '
                f'半径: {closest_obstacle[2]:.2f}m'
            )
        elif min_distance < 0.30:
            self.node.get_logger().warn(
                f'[{self.display_name}] ⚠️ 接近障碍物: {min_distance:.3f}m'
            )
        else:
            self.node.get_logger().info(
                f'[{self.display_name}] ✓ 安全距离: {min_distance:.3f}m'
            )
        
        return min_distance


class MultiRobotTrackPublisher(Node):
    """多机器人轨迹发布节点"""
    
    # ========== 机器人配置 ==========
    # 命名空间说明：
    # - '' (空字符串): 第一个机器人，话题为 /cmd_vel, /odom
    # - 'tb3_1': 第二个机器人，话题为 /tb3_1/cmd_vel, /tb3_1/odom
    # - 'tb3_2': 第三个机器人，话题为 /tb3_2/cmd_vel, /tb3_2/odom
    # - 'tb3_3': 第四个机器人，话题为 /tb3_3/cmd_vel, /tb3_3/odom
    ROBOTS_CONFIG = {
        '': {  # 机器人1 - 无命名空间
            'start': (-2.0, -0.5),
            'waypoints': [
                (-2.0, -0.5),   # 起点
                (-1.0, -0.5),   # 点1
                (-0.5, -0.5),   # 点2
                (0.5, -1.0),    # 点3
                (1.0, -1.5),    # 点4
                (2.1, 0.0),     # 点5
                (1.5, 1.5),     # 点6
                (-0.5, 2.2),    # 点7
                (-0.5, 0.5),    # 点8
            ],
            'seed': 42,
        },
        'tb3_1': {  # 机器人2 - tb3_1 命名空间
            'start': (0.0, 0.5),
            'waypoints': [
                (0.0, 0.5),    # 起点
                (1.0, 0.5),    # 点1
                (0.5, 1.5),   # 点2
                (-1.0,2.0),   # 点3
                (-2.0, 0.5),  # 点4
                (-1.0, -0.5),    # 点5
            ],
            'seed': 43,  # 不同的种子产生不同的路径
        },
        'tb3_2': {  # 机器人3 - tb3_2 命名空间（新增）
            'start': (-1.0, -1.5),
            'waypoints': [
                (-1.0, -1.5),   # 起点
                (-1.5, -1.5),
                (-2.0, -0.5),   # 点1
                (-2.0, 0.5),    # 点2
                (-0.5, 0.5),    # 点3
                (0.5, 0.5),    # 点4
            ],
            'seed': 42,
        },
        'tb3_3': {  # 机器人4 - tb3_3 命名空间（新增）
            'start': (2.0, 0.0),
            'waypoints': [
                (2.0, 0.0),     # 起点
                (2.0, -1.0),    # 点1
                (1.0, -2),    # 点2
                (0.0, -1.5),    # 点3
            ],
            'seed': 42,
        },
    }
    # =================================
    
    def __init__(self):
        super().__init__('multi_robot_track_publisher')
        
        # 声明参数：机器人数量（默认2，保持现有行为）
        self.declare_parameter('num_robots', 2)
        num_robots = self.get_parameter('num_robots').value
        
        # 验证参数
        if num_robots < 1 or num_robots > 4:
            self.get_logger().error(f'机器人数量必须在1-4之间，当前值: {num_robots}')
            raise ValueError(f'Invalid num_robots: {num_robots}')
        
        # 选择要运行的机器人（按字典顺序选择前N个）
        # 注意：空字符串在前，所以顺序是: '', 'tb3_1', 'tb3_2', 'tb3_3'
        all_robot_keys = ['', 'tb3_1', 'tb3_2', 'tb3_3']
        selected_keys = all_robot_keys[:num_robots]
        
        self.get_logger().info('='*60)
        self.get_logger().info(f'多机器人轨迹发布节点已启动 - 运行 {num_robots} 个机器人')
        self.get_logger().info(f'激活的机器人: {[k if k else "tb3_0" for k in selected_keys]}')
        self.get_logger().info('='*60)
        
        # 为选中的机器人创建控制器
        self.controllers = {}
        
        for name in selected_keys:
            config = self.ROBOTS_CONFIG[name]
            self.controllers[name] = RobotController(
                node=self,
                namespace=name,
                start_pos=config['start'],
                waypoints=config['waypoints'],
                seed=config['seed']
            )
        
        self.get_logger().info('='*60)
        self.get_logger().info(f'已创建 {len(self.controllers)} 个机器人控制器')
        for name in self.controllers.keys():
            display_name = name if name else 'tb3_0 (无命名空间)'
            self.get_logger().info(f'  - {display_name}')
        self.get_logger().info('='*60)
        
        # 延迟启动
        self.timer = self.create_timer(2.0, self.start_execution)
        self.started = False
    
    def start_execution(self):
        """开始执行所有机器人的轨迹"""
        if self.started:
            return
        self.started = True
        self.timer.cancel()
        
        self.get_logger().info('\n开始执行多机器人轨迹...\n')
        
        # 使用线程并行执行各机器人轨迹
        threads = []
        for name, controller in self.controllers.items():
            t = threading.Thread(
                target=controller.execute_trajectory,
                name=f'thread_{name}'
            )
            threads.append(t)
            t.start()
        
        # 等待所有线程完成
        for t in threads:
            t.join()
        
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('所有机器人轨迹执行完成！')
        self.get_logger().info('='*60)


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotTrackPublisher()
    
    # 使用多线程执行器以支持多线程回调处理
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('\n用户中断，正在停止所有机器人...')
    finally:
        # 停止所有机器人
        for controller in node.controllers.values():
            controller.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

