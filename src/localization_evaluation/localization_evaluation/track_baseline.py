#!/usr/bin/env python3
"""
TurtleBot3 固定轨迹发布节点

地图信息 (turtlebot3_world):
- 地图范围: X[-2.5, 3.0]m, Y[-2.3, 2.3]m
- 起点: (-2.0, -0.5)
- 中心9个圆柱: 3x3网格在 (±1.1, ±1.1)，半径0.15m
- 六边形障碍物:
  - Head: (3.5, 0), 半径0.8m
  - Left Hand: (1.8, 2.7), 半径0.55m
  - Right Hand: (1.8, -2.7), 半径0.55m
  - Left Foot: (-1.8, 2.7), 半径0.55m
  - Right Foot: (-1.8, -2.7), 半径0.55m
- 外围墙壁

轨迹设计: 支持固定航点和RRT路径规划两种模式
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

# 导入RRT路径规划模块
from localization_evaluation.pathplan import plan_multi_waypoints


class TrackPublisher(Node):
    # RRT随机种子（固定种子确保相同航点生成相同路径）
    RRT_SEED = 42
    
    # ========== 用户自定义关键航点 ==========
    # RRT会自动规划这些航点之间的安全路径
    KEY_WAYPOINTS = [
        (-2.0, -0.5),   # 起点
        (-1.0, -0.5),   # 点1
        (-0.5, -0.5),   # 点2
        (0.5, -1.0),    # 点3
        (1.0, -1.5),    # 点4
        (2.1, 0.0),     # 点5
        (1.5, 1.5),     # 点6
        (-0.5, 2.2),    # 点7
        (-0.5, 0.5),    # 点8
    ]
    # =========================================
    
    def __init__(self):
        super().__init__('track_publisher')
        
        # 速度命令发布者
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 运动参数
        self.linear_speed = 0.15   # 线速度 m/s (安全速度)
        self.angular_speed = 0.5   # 角速度 rad/s
        
        # 机器人起始位置和朝向
        self.current_x = -2.0
        self.current_y = -0.5
        self.current_yaw = 0.0  # 初始朝向 X+ 方向
        
        self.get_logger().info('='*60)
        self.get_logger().info('轨迹发布节点已启动 (RRT路径规划)')
        self.get_logger().info(f'起始位置: ({self.current_x}, {self.current_y})')
        self.get_logger().info('='*60)
        
        # 延迟启动
        self.timer = self.create_timer(2.0, self.execute_coverage_trajectory)
        self.started = False

    def execute_coverage_trajectory(self):
        """执行覆盖轨迹"""
        if self.started:
            return
        self.started = True
        self.timer.cancel()
        
        self.get_logger().info('\n开始执行地图覆盖轨迹...\n')
        
        waypoints = self._get_rrt_planned_waypoints()
        
        if waypoints is None or len(waypoints) == 0:
            self.get_logger().error('无法获取航点，轨迹执行终止')
            return
        
        total_waypoints = len(waypoints)
        self.get_logger().info(f'共 {total_waypoints} 个航点')
        
        for i, waypoint in enumerate(waypoints):
            if len(waypoint) == 3:
                target_x, target_y, description = waypoint
            else:
                target_x, target_y = waypoint
                description = f"航点{i}"
            
            self.get_logger().info(f'\n[{i+1}/{total_waypoints}] {description}')
            self.get_logger().info(f'    目标: ({target_x:.2f}, {target_y:.2f})')
            
            self.navigate_to_point(target_x, target_y)
            
            # 到达后短暂停顿
            self.stop_robot()
            time.sleep(0.3)
        
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('轨迹执行完成！已覆盖地图主要区域')
        self.get_logger().info('='*60)
        self.stop_robot()
    
    def _get_rrt_planned_waypoints(self):
        """使用RRT算法规划路径"""
        self.get_logger().info('使用RRT算法规划路径...')
        self.get_logger().info(f'关键航点数: {len(self.KEY_WAYPOINTS)}')
        self.get_logger().info(f'随机种子: {self.RRT_SEED}')
        
        # 打印用户定义的关键航点
        for i, (x, y) in enumerate(self.KEY_WAYPOINTS):
            self.get_logger().info(f'  关键点{i}: ({x:.2f}, {y:.2f})')
        
        # 使用RRT规划完整路径
        planned_path = plan_multi_waypoints(self.KEY_WAYPOINTS, seed=self.RRT_SEED)
        
        if planned_path is None:
            self.get_logger().error('RRT路径规划失败！')
            return None
        
        self.get_logger().info(f'RRT规划完成，生成 {len(planned_path)} 个路径点')
        
        # 转换为带描述的格式
        waypoints_with_desc = []
        for i, (x, y) in enumerate(planned_path):
            waypoints_with_desc.append((x, y, f"RRT点{i}"))
        
        return waypoints_with_desc

    def navigate_to_point(self, target_x, target_y):
        """导航到目标点 (使用航位推算)"""
        # 计算到目标点的距离和角度
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        target_yaw = math.atan2(dy, dx)
        
        # 计算需要转的角度
        angle_diff = self.normalize_angle(target_yaw - self.current_yaw)
        
        # 1. 先转向目标方向
        if abs(angle_diff) > 0.05:  # 大于约3度才转
            self.turn_angle(angle_diff)
        
        # 2. 直行到目标点
        if distance > 0.02:  # 大于2cm才移动
            self.move_distance(distance)
        
        # 更新当前位置（航位推算）
        self.current_x = target_x
        self.current_y = target_y
        self.current_yaw = target_yaw

    def turn_angle(self, angle):
        """原地转动指定角度（弧度）"""
        twist = Twist()
        
        # 确定转向方向
        if angle > 0:
            twist.angular.z = self.angular_speed
        else:
            twist.angular.z = -self.angular_speed
        
        # 计算转动时间
        duration = abs(angle) / self.angular_speed
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        
        self.stop_robot()
        time.sleep(0.1)
        
        # 更新朝向
        self.current_yaw = self.normalize_angle(self.current_yaw + angle)

    def move_distance(self, distance):
        """直行指定距离"""
        twist = Twist()
        twist.linear.x = self.linear_speed
        
        # 计算行驶时间
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
        # 发布多次确保停止
        for _ in range(5):
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.02)


def main(args=None):
    rclpy.init(args=args)
    node = TrackPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\n用户中断，正在停止机器人...')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

