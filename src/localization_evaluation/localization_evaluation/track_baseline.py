#!/usr/bin/env python3
"""
TurtleBot3 Fixed Trajectory Publisher Node

Map Information (turtlebot3_world):
- Map range: X[-2.5, 3.0]m, Y[-2.3, 2.3]m
- Starting point: (-2.0, -0.5)
- 9 central cylinders: 3x3 grid at (±1.1, ±1.1), radius 0.15m
- Hexagonal obstacles:
  - Head: (3.5, 0), radius 0.8m
  - Left Hand: (1.8, 2.7), radius 0.55m
  - Right Hand: (1.8, -2.7), radius 0.55m
  - Left Foot: (-1.8, 2.7), radius 0.55m
  - Right Foot: (-1.8, -2.7), radius 0.55m
- Peripheral walls

Trajectory Design: Supports both fixed waypoints and RRT path planning modes.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

# Import the RRT path planning module
from localization_evaluation.pathplan import plan_multi_waypoints


class TrackPublisher(Node):
    # RRT random seed (a fixed seed ensures the same path is generated for the same waypoints)
    RRT_SEED = 42
    
    # ========== User-defined Key Waypoints ==========
    # RRT will automatically plan a safe path between these waypoints
    KEY_WAYPOINTS = [
        (-2.0, -0.5),   # Start
        (-1.0, -0.5),   # Point 1
        (-0.5, -0.5),   # Point 2
        (0.5, -1.0),    # Point 3
        (1.0, -1.5),    # Point 4
        (2.1, 0.0),     # Point 5
        (1.5, 1.5),     # Point 6
        (-0.5, 2.2),    # Point 7
        (-0.5, 0.5),    # Point 8
    ]
    # =========================================
    
    def __init__(self):
        super().__init__('track_publisher')
        
        # Velocity command publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Motion parameters
        self.linear_speed = 0.15   # Linear speed m/s (safe speed)
        self.angular_speed = 0.5   # Angular speed rad/s
        
        # Robot starting position and orientation
        self.current_x = -2.0
        self.current_y = -0.5
        self.current_yaw = 0.0  # Initial orientation towards X+
        
        self.get_logger().info('='*60)
        self.get_logger().info('Track Publisher Node started (RRT Path Planning)')
        self.get_logger().info(f'Starting position: ({self.current_x}, {self.current_y})')
        self.get_logger().info('='*60)
        
        # Delayed start
        self.timer = self.create_timer(2.0, self.execute_coverage_trajectory)
        self.started = False

    def execute_coverage_trajectory(self):
        """Execute the coverage trajectory"""
        if self.started:
            return
        self.started = True
        self.timer.cancel()
        
        self.get_logger().info('\nStarting to execute the map coverage trajectory...\n')
        
        waypoints = self._get_rrt_planned_waypoints()
        
        if waypoints is None or len(waypoints) == 0:
            self.get_logger().error('Could not get waypoints, trajectory execution aborted.')
            return
        
        total_waypoints = len(waypoints)
        self.get_logger().info(f'Total {total_waypoints} waypoints')
        
        for i, waypoint in enumerate(waypoints):
            if len(waypoint) == 3:
                target_x, target_y, description = waypoint
            else:
                target_x, target_y = waypoint
                description = f"Waypoint {i}"
            
            self.get_logger().info(f'\n[{i+1}/{total_waypoints}] {description}')
            self.get_logger().info(f'    Target: ({target_x:.2f}, {target_y:.2f})')
            
            self.navigate_to_point(target_x, target_y)
            
            # Brief pause after arrival
            self.stop_robot()
            time.sleep(0.3)
        
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('Trajectory execution complete! The main areas of the map have been covered.')
        self.get_logger().info('='*60)
        self.stop_robot()
    
    def _get_rrt_planned_waypoints(self):
        """Plan the path using the RRT algorithm"""
        self.get_logger().info('Planning path using RRT algorithm...')
        self.get_logger().info(f'Number of key waypoints: {len(self.KEY_WAYPOINTS)}')
        self.get_logger().info(f'Random seed: {self.RRT_SEED}')
        
        # Print user-defined key waypoints
        for i, (x, y) in enumerate(self.KEY_WAYPOINTS):
            self.get_logger().info(f'  Keypoint {i}: ({x:.2f}, {y:.2f})')
        
        # Use RRT to plan the full path
        planned_path = plan_multi_waypoints(self.KEY_WAYPOINTS, seed=self.RRT_SEED)
        
        if planned_path is None:
            self.get_logger().error('RRT path planning failed!')
            return None
        
        self.get_logger().info(f'RRT planning complete, generated {len(planned_path)} path points')
        
        # Convert to format with description
        waypoints_with_desc = []
        for i, (x, y) in enumerate(planned_path):
            waypoints_with_desc.append((x, y, f"RRT Point {i}"))
        
        return waypoints_with_desc

    def navigate_to_point(self, target_x, target_y):
        """Navigate to the target point (using dead reckoning)"""
        # Calculate the distance and angle to the target point
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        target_yaw = math.atan2(dy, dx)
        
        # Calculate the angle to turn
        angle_diff = self.normalize_angle(target_yaw - self.current_yaw)
        
        # 1. First, turn towards the target direction
        if abs(angle_diff) > 0.05:  # Only turn if the angle is greater than ~3 degrees
            self.turn_angle(angle_diff)
        
        # 2. Then, move straight to the target point
        if distance > 0.02:  # Only move if the distance is greater than 2cm
            self.move_distance(distance)
        
        # Update current position (dead reckoning)
        self.current_x = target_x
        self.current_y = target_y
        self.current_yaw = target_yaw

    def turn_angle(self, angle):
        """Rotate in place by a specified angle (in radians)"""
        twist = Twist()
        
        # Determine the direction of rotation
        if angle > 0:
            twist.angular.z = self.angular_speed
        else:
            twist.angular.z = -self.angular_speed
        
        # Calculate the duration of the turn
        duration = abs(angle) / self.angular_speed
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        
        self.stop_robot()
        time.sleep(0.1)
        
        # Update orientation
        self.current_yaw = self.normalize_angle(self.current_yaw + angle)

    def move_distance(self, distance):
        """Move straight for a specified distance"""
        twist = Twist()
        twist.linear.x = self.linear_speed
        
        # Calculate the travel time
        duration = distance / self.linear_speed
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        
        self.stop_robot()
        time.sleep(0.1)

    def normalize_angle(self, angle):
        """Normalize the angle to the range [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def stop_robot(self):
        """Stop the robot"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # Publish multiple times to ensure it stops
        for _ in range(5):
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.02)


def main(args=None):
    rclpy.init(args=args)
    node = TrackPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\nUser interrupt, stopping the robot...')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

