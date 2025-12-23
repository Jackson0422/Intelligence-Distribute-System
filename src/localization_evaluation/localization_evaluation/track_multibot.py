#!/usr/bin/env python3
"""
多机器人轨迹发布节点 (Multi-Robot Track Publisher)

功能：
- 同时控制两个机器人执行各自的轨迹
- 使用RRT路径规划为每个机器人规划安全路径
- 使用多线程并行控制多个机器人

发布话题：
- /cmd_vel: 机器人1速度命令（无命名空间）
- /tb3_1/cmd_vel: 机器人2速度命令
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time
import threading

from localization_evaluation.pathplan import plan_multi_waypoints


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
        
        # 发布者 - 根据命名空间发布到不同话题
        topic = f'/{namespace}/cmd_vel' if namespace else '/cmd_vel'
        self.cmd_vel_pub = node.create_publisher(Twist, topic, 10)
        
        # 用于日志显示的名称
        self.display_name = namespace if namespace else 'tb3_0'
        
        node.get_logger().info(f'[{self.display_name}] 控制器初始化完成')
        node.get_logger().info(f'[{self.display_name}] 发布话题: {topic}')
        node.get_logger().info(f'[{self.display_name}] 起始位置: ({self.start_x:.2f}, {self.start_y:.2f})')
    
    def execute_trajectory(self):
        """执行轨迹"""
        self.node.get_logger().info(f'[{self.display_name}] 开始RRT路径规划...')
        self.node.get_logger().info(f'[{self.display_name}] 关键航点数: {len(self.key_waypoints)}')
        
        # 使用RRT规划完整路径
        planned_path = plan_multi_waypoints(self.key_waypoints, seed=self.seed)
        
        if planned_path is None or len(planned_path) == 0:
            self.node.get_logger().error(f'[{self.display_name}] 路径规划失败！')
            return
        
        self.node.get_logger().info(f'[{self.display_name}] 规划完成，共 {len(planned_path)} 个航点')
        
        total_waypoints = len(planned_path)
        for i, (x, y) in enumerate(planned_path):
            self.node.get_logger().info(
                f'[{self.display_name}] [{i+1}/{total_waypoints}] 目标: ({x:.2f}, {y:.2f})'
            )
            self.navigate_to_point(x, y)
            self.stop_robot()
            time.sleep(0.3)
        
        self.node.get_logger().info(f'[{self.display_name}] 轨迹执行完成！')
        self.stop_robot()
    
    def navigate_to_point(self, target_x, target_y):
        """导航到目标点 (使用航位推算)"""
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        target_yaw = math.atan2(dy, dx)
        
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


class MultiRobotTrackPublisher(Node):
    """多机器人轨迹发布节点"""
    
    # ========== 机器人配置 ==========
    # 命名空间说明：
    # - '' (空字符串): 第一个机器人，话题为 /cmd_vel, /odom
    # - 'tb3_1': 第二个机器人，话题为 /tb3_1/cmd_vel, /tb3_1/odom
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
                (2.0, 0.0),    # 点6
            ],
            'seed': 43,  # 不同的种子产生不同的路径
        },
    }
    # =================================
    
    def __init__(self):
        super().__init__('multi_robot_track_publisher')
        
        # 为每个机器人创建控制器
        self.controllers = {}
        
        for name, config in self.ROBOTS_CONFIG.items():
            self.controllers[name] = RobotController(
                node=self,
                namespace=name,
                start_pos=config['start'],
                waypoints=config['waypoints'],
                seed=config['seed']
            )
        
        self.get_logger().info('='*60)
        self.get_logger().info('多机器人轨迹发布节点已启动')
        self.get_logger().info(f'控制机器人数量: {len(self.controllers)}')
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
    
    try:
        rclpy.spin(node)
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

