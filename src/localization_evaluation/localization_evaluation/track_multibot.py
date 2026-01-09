#!/usr/bin/env python3
"""
Multi-Robot Track Publisher

Functionality:
- Simultaneously controls 2-4 robots to execute their respective trajectories.
- Uses RRT path planning to plan safe paths for each robot.
- Uses multithreading to control multiple robots in parallel.
- Supports configuring the number of robots via parameters (default is 2).

Published Topics:
- /cmd_vel: Velocity command for robot 1 (no namespace)
- /tb3_1/cmd_vel: Velocity command for robot 2
- /tb3_2/cmd_vel: Velocity command for robot 3 (optional)
- /tb3_3/cmd_vel: Velocity command for robot 4 (optional)
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
    """Single Robot Controller"""
    
    def __init__(self, node, namespace, start_pos, waypoints, seed=42):
        """
        Initializes the robot controller.
        
        Args:
            node: The ROS node.
            namespace: The robot's namespace.
            start_pos: The starting position (x, y).
            waypoints: A list of key waypoints.
            seed: The random seed for RRT.
        """
        self.node = node
        self.namespace = namespace
        self.start_x, self.start_y = start_pos
        self.current_x = self.start_x
        self.current_y = self.start_y
        self.current_yaw = 0.0
        self.key_waypoints = waypoints
        self.seed = seed
        
        # Motion parameters
        self.linear_speed = 0.15   # Linear speed in m/s
        self.angular_speed = 0.5   # Angular speed in rad/s
        
        # Odometry data (for closed-loop control)
        self.odom_x = self.start_x
        self.odom_y = self.start_y
        self.odom_yaw = 0.0
        self.odom_received = False
        
        # Publisher - publishes to different topics based on namespace
        cmd_vel_topic = f'/{namespace}/cmd_vel' if namespace else '/cmd_vel'
        self.cmd_vel_pub = node.create_publisher(Twist, cmd_vel_topic, 10)
        
        # Subscriber - subscribes to the odometry topic
        odom_topic = f'/{namespace}/odom' if namespace else '/odom'
        self.odom_sub = node.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )
        
        # Name for logging display
        self.display_name = namespace if namespace else 'tb3_0'
        
        node.get_logger().info(f'[{self.display_name}] Controller initialized.')
        node.get_logger().info(f'[{self.display_name}] Publishing to topic: {cmd_vel_topic}')
        node.get_logger().info(f'[{self.display_name}] Subscribed to topic: {odom_topic}')
        node.get_logger().info(f'[{self.display_name}] Starting position: ({self.start_x:.2f}, {self.start_y:.2f})')
    
    def quaternion_to_euler(self, x, y, z, w):
        """Converts a quaternion to a Euler angle (yaw)."""
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def odom_callback(self, msg):
        """Odometry callback function - updates the robot's current position."""
        # Extract position
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        
        # Extract orientation (quaternion to Euler)
        orientation = msg.pose.pose.orientation
        self.odom_yaw = self.quaternion_to_euler(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        
        self.odom_received = True
    
    def execute_trajectory(self):
        """Executes the trajectory."""
        self.node.get_logger().info(f'[{self.display_name}] Starting RRT path planning...')
        self.node.get_logger().info(f'[{self.display_name}] Number of key waypoints: {len(self.key_waypoints)}')
        
        # ========== Output Key Waypoints ==========
        self.node.get_logger().info(f'[{self.display_name}] ===== Key Waypoint List =====')
        for idx, (x, y) in enumerate(self.key_waypoints):
            self.node.get_logger().info(f'[{self.display_name}]   Keypoint {idx}: ({x:.2f}, {y:.2f})')
        self.node.get_logger().info(f'[{self.display_name}] ========================')
        
        # Plan the full path using RRT
        planned_path = plan_multi_waypoints(self.key_waypoints, seed=self.seed)
        
        if planned_path is None or len(planned_path) == 0:
            self.node.get_logger().error(f'[{self.display_name}] Path planning failed!')
            return
        
        self.node.get_logger().info(f'[{self.display_name}] Planning complete, {len(planned_path)} waypoints in total.')
        
        # ========== Output Full Planned Path ==========
        self.node.get_logger().info(f'[{self.display_name}] ===== RRT Planned Path =====')
        for idx, (x, y) in enumerate(planned_path):
            self.node.get_logger().info(f'[{self.display_name}]   Path point {idx}: ({x:.3f}, {y:.3f})')
        self.node.get_logger().info(f'[{self.display_name}] =======================')
        
        # Execute the path
        total_waypoints = len(planned_path)
        for i, (x, y) in enumerate(planned_path):
            self.node.get_logger().info(
                f'[{self.display_name}] [{i+1}/{total_waypoints}] Target: ({x:.2f}, {y:.2f})'
            )
            
            # ========== Display Current Position ==========
            self.node.get_logger().info(
                f'[{self.display_name}]   Current position: ({self.current_x:.3f}, {self.current_y:.3f}, {self.current_yaw:.2f}rad)'
            )
            
            self.navigate_to_point(x, y)
            
            # ========== Display Position After Arrival ==========
            self.node.get_logger().info(
                f'[{self.display_name}]   Arrival position: ({self.current_x:.3f}, {self.current_y:.3f}, {self.current_yaw:.2f}rad)'
            )
            
            # ========== Calculate Position Error ==========
            error = math.sqrt((self.current_x - x)**2 + (self.current_y - y)**2)
            if error > 0.1:
                self.node.get_logger().warn(
                    f'[{self.display_name}]   ⚠️ Position error: {error:.3f}m (Large deviation from target!)'
                )
            else:
                self.node.get_logger().info(
                    f'[{self.display_name}]   ✓ Position error: {error:.3f}m'
                )
            
            self.stop_robot()
            time.sleep(0.3)
        
        self.node.get_logger().info(f'[{self.display_name}] Trajectory execution complete!')
        self.stop_robot()
    
    def navigate_to_point(self, target_x, target_y):
        """Navigate to a target point (simplified version: time-controlled)."""
        # Calculate distance and angle to move (based on current estimated position)
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        target_yaw = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(target_yaw - self.current_yaw)
        
        # Check for obstacles before departure
        self.check_obstacle_clearance(self.current_x, self.current_y)
        
        # 1. First, turn towards the target direction (time-controlled)
        if abs(angle_diff) > 0.05:
            self.turn_angle(angle_diff)
        
        # 2. Then, move straight to the target point (time-controlled)
        if distance > 0.02:
            self.move_distance(distance)
        
        # 3. Update the current position estimate
        self.current_x = target_x
        self.current_y = target_y
        self.current_yaw = target_yaw
        
        # 4. If odometry is available, report the actual position and error
        if self.odom_received:
            time.sleep(0.3)  # Wait for odometry to update
            actual_x, actual_y = self.odom_x, self.odom_y
            
            self.node.get_logger().info(
                f'[{self.display_name}]   Arrival position: ({actual_x:.3f}, {actual_y:.3f}, {self.odom_yaw:.2f}rad)'
            )
            
            # Calculate position error
            error = math.sqrt((actual_x - target_x)**2 + (actual_y - target_y)**2)
            if error > 0.1:
                self.node.get_logger().warn(
                    f'[{self.display_name}]   ⚠️ Position error: {error:.3f}m (Large deviation from target!)'
                )
            else:
                self.node.get_logger().info(
                    f'[{self.display_name}]   ✓ Position error: {error:.3f}m'
                )
        
        # Check for obstacles upon arrival
        self.check_obstacle_clearance(self.current_x, self.current_y)
    
    def navigate_to_point_openloop(self, target_x, target_y):
        """Navigate to a target point (using dead reckoning - open-loop control, backup method)."""
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        target_yaw = math.atan2(dy, dx)
        
        # ========== Check for Obstacles Before Departure ==========
        self.node.get_logger().info(f'[{self.display_name}]   Checking before departure:')
        self.check_obstacle_clearance(self.current_x, self.current_y)
        
        # Calculate the angle to turn
        angle_diff = self.normalize_angle(target_yaw - self.current_yaw)
        
        # 1. First, turn towards the target direction
        if abs(angle_diff) > 0.05:
            self.turn_angle(angle_diff)
        
        # 2. Then, move straight to the target point
        if distance > 0.02:
            self.move_distance(distance)
        
        # Update the current position
        self.current_x = target_x
        self.current_y = target_y
        self.current_yaw = target_yaw
        
        # ========== Check for Obstacles After Arrival ==========
        self.node.get_logger().info(f'[{self.display_name}]   Checking after arrival:')
        self.check_obstacle_clearance(self.current_x, self.current_y)
    
    def turn_angle(self, angle):
        """Rotates in place by a specified angle (in radians)."""
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
        """Moves straight for a specified distance."""
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
        """Normalizes an angle to the range [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def stop_robot(self):
        """Stops the robot."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        for _ in range(5):
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.02)
    
    def check_obstacle_clearance(self, x, y):
        """Checks the distance from the current position to obstacles."""
        world_map = TurtleBot3WorldMap()
        
        min_distance = float('inf')
        closest_obstacle = None
        
        for idx, obs in enumerate(world_map.obstacles):
            dist = math.sqrt((x - obs.x)**2 + (y - obs.y)**2)
            actual_clearance = dist - obs.radius  # Surface distance
            
            if actual_clearance < min_distance:
                min_distance = actual_clearance
                closest_obstacle = (obs.x, obs.y, obs.radius)
        
        # Warning: Too close to an obstacle
        if min_distance < 0.15:
            self.node.get_logger().error(
                f'[{self.display_name}] ⚠️⚠️ Danger! Only {min_distance:.3f}m away from an obstacle!'
                f' Obstacle position: ({closest_obstacle[0]:.2f}, {closest_obstacle[1]:.2f}), '
                f'Radius: {closest_obstacle[2]:.2f}m'
            )
        elif min_distance < 0.30:
            self.node.get_logger().warn(
                f'[{self.display_name}] ⚠️ Approaching an obstacle: {min_distance:.3f}m'
            )
        else:
            self.node.get_logger().info(
                f'[{self.display_name}] ✓ Safe distance: {min_distance:.3f}m'
            )
        
        return min_distance


class MultiRobotTrackPublisher(Node):
    """Multi-Robot Track Publisher Node"""
    
    # ========== Robot Configuration ==========
    # Namespace explanation:
    # - '' (empty string): First robot, topics are /cmd_vel, /odom
    # - 'tb3_1': Second robot, topics are /tb3_1/cmd_vel, /tb3_1/odom
    # - 'tb3_2': Third robot, topics are /tb3_2/cmd_vel, /tb3_2/odom
    # - 'tb3_3': Fourth robot, topics are /tb3_3/cmd_vel, /tb3_3/odom
    ROBOTS_CONFIG = {
        '': {  # Robot 1 - no namespace
            'start': (-2.0, -0.5),
            'waypoints': [
                (-2.0, -0.5),   # Start
                (-1.0, -0.5),   # Point 1
                (-0.5, -0.5),   # Point 2
                (0.5, -1.0),    # Point 3
                (1.0, -1.5),    # Point 4
                (2.1, 0.0),     # Point 5
                (1.5, 1.5),     # Point 6
                (-0.5, 2.2),    # Point 7
                (-0.5, 0.5),    # Point 8
            ],
            'seed': 42,
        },
        'tb3_1': {  # Robot 2 - tb3_1 namespace
            'start': (0.0, 0.5),
            'waypoints': [
                (0.0, 0.5),    # Start
                (1.0, 0.5),    # Point 1
                (0.5, 1.5),   # Point 2
                (-1.0,2.0),   # Point 3
                (-2.0, 0.5),  # Point 4
                (-1.0, -0.5),    # Point 5
            ],
            'seed': 43,  # Different seeds produce different paths
        },
        'tb3_2': {  # Robot 3 - tb3_2 namespace (Added)
            'start': (-1.0, -1.5),
            'waypoints': [
                (-1.0, -1.5),   # Start
                (-1.5, -1.5),
                (-2.0, -0.5),   # Point 1
                (-2.0, 0.5),    # Point 2
                (-0.5, 0.5),    # Point 3
                (0.5, 0.5),    # Point 4
            ],
            'seed': 42,
        },
        'tb3_3': {  # Robot 4 - tb3_3 namespace (Added)
            'start': (2.0, 0.0),
            'waypoints': [
                (2.0, 0.0),     # Start
                (2.0, -1.0),    # Point 1
                (1.0, -2),    # Point 2
                (0.0, -1.5),    # Point 3
            ],
            'seed': 42,
        },
    }
    # =================================
    
    def __init__(self):
        super().__init__('multi_robot_track_publisher')
        
        # Declare parameter: number of robots (default 2, maintains existing behavior)
        self.declare_parameter('num_robots', 2)
        num_robots = self.get_parameter('num_robots').value
        
        # Validate parameter
        if num_robots < 1 or num_robots > 4:
            self.get_logger().error(f'Number of robots must be between 1 and 4, but is: {num_robots}')
            raise ValueError(f'Invalid num_robots: {num_robots}')
        
        # Select the robots to run (select the first N in dictionary order)
        # Note: The empty string comes first, so the order is: '', 'tb3_1', 'tb3_2', 'tb3_3'
        all_robot_keys = ['', 'tb3_1', 'tb3_2', 'tb3_3']
        selected_keys = all_robot_keys[:num_robots]
        
        self.get_logger().info('='*60)
        self.get_logger().info(f'Multi-robot track publisher started - running {num_robots} robots.')
        self.get_logger().info(f'Active robots: {[k if k else "tb3_0" for k in selected_keys]}')
        self.get_logger().info('='*60)
        
        # Create controllers for the selected robots
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
        self.get_logger().info(f'Created {len(self.controllers)} robot controllers.')
        for name in self.controllers.keys():
            display_name = name if name else 'tb3_0 (no namespace)'
            self.get_logger().info(f'  - {display_name}')
        self.get_logger().info('='*60)
        
        # Delayed start
        self.timer = self.create_timer(2.0, self.start_execution)
        self.started = False
    
    def start_execution(self):
        """Starts executing the trajectories for all robots."""
        if self.started:
            return
        self.started = True
        self.timer.cancel()
        
        self.get_logger().info('\nStarting multi-robot trajectory execution...\n')
        
        # Use threads to execute each robot's trajectory in parallel
        threads = []
        for name, controller in self.controllers.items():
            t = threading.Thread(
                target=controller.execute_trajectory,
                name=f'thread_{name}'
            )
            threads.append(t)
            t.start()
        
        # Wait for all threads to complete
        for t in threads:
            t.join()
        
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('All robot trajectories have been executed!')
        self.get_logger().info('='*60)


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotTrackPublisher()
    
    # Use a multi-threaded executor to support multi-threaded callback processing
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('\nUser interrupt, stopping all robots...')
    finally:
        # Stop all robots
        for controller in node.controllers.values():
            controller.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

