#!/usr/bin/env python3
"""
Tiny jitter motion node - makes robots rotate very slowly to trigger AMCL updates
while remaining essentially stationary for static convergence demonstration.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class JitterMotion(Node):
    def __init__(self):
        super().__init__('jitter_motion')
        
        self.declare_parameter('num_robots', 2)
        self.declare_parameter('angular_vel', 0.05)  # Very slow rotation (rad/s)
        
        num_robots = self.get_parameter('num_robots').value
        angular_vel = self.get_parameter('angular_vel').value
        
        # Create publishers for each robot
        self.cmd_vel_pubs = []
        for i in range(1, num_robots + 1):
            pub = self.create_publisher(Twist, f'/tb3_{i}/cmd_vel', 10)
            self.cmd_vel_pubs.append(pub)
        
        # Timer to publish jitter commands
        self.timer = self.create_timer(0.1, self.publish_jitter)  # 10Hz
        self.angular_vel = angular_vel
        self.direction = 1  # Alternate direction
        self.counter = 0
        self.get_logger().info(f'Jitter motion started for {num_robots} robots with {angular_vel} rad/s')
    
    def publish_jitter(self):
        twist = Twist()
        twist.angular.z = self.angular_vel * self.direction
        
        for pub in self.cmd_vel_pubs:
            pub.publish(twist)
        
        # Alternate direction every 3 seconds (30 callbacks at 10Hz)
        # This keeps robots roughly in place while providing motion for AMCL
        self.counter += 1
        if self.counter >= 30:
            self.direction *= -1
            self.counter = 0


def main(args=None):
    rclpy.init(args=args)
    node = JitterMotion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
