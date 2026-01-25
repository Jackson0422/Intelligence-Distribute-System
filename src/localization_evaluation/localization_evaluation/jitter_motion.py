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
        self.declare_parameter('flip_period', 10.0)  # Seconds before reversing; <=0 disables reversal
        
        num_robots = self.get_parameter('num_robots').value
        angular_vel = self.get_parameter('angular_vel').value
        flip_period = float(self.get_parameter('flip_period').value)
        
        # Create publishers for each robot
        self.cmd_vel_pubs = []
        for i in range(1, num_robots + 1):
            pub = self.create_publisher(Twist, f'/tb3_{i}/cmd_vel', 10)
            self.cmd_vel_pubs.append(pub)
        
        # Timer to publish jitter commands
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.publish_jitter)  # 10Hz
        self.angular_vel = angular_vel
        self.flip_period = flip_period
        self.direction = 1  # Alternate direction
        self.counter = 0
        self.flip_ticks = None
        if self.flip_period > 0.0:
            self.flip_ticks = max(1, int(round(self.flip_period / self.timer_period)))
        self.get_logger().info(
            f'Jitter motion started for {num_robots} robots with {angular_vel} rad/s, '
            f'flip_period={self.flip_period}s'
        )
    
    def publish_jitter(self):
        twist = Twist()
        twist.angular.z = self.angular_vel * self.direction
        
        for pub in self.cmd_vel_pubs:
            pub.publish(twist)
        
        # Alternate direction every flip_period seconds to keep robots near their spawn
        self.counter += 1
        if self.flip_ticks is not None and self.counter >= self.flip_ticks:
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
