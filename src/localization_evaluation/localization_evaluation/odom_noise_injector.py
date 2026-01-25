#!/usr/bin/env python3
"""
Odometry noise injector - adds tiny noise to odometry to trigger AMCL updates
while keeping robots stationary in Gazebo. This tricks AMCL into processing
laser scans for static convergence.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math


class OdomNoiseInjector(Node):
    def __init__(self):
        super().__init__('odom_noise_injector')
        
        self.declare_parameter('num_robots', 2)
        self.declare_parameter('noise_amplitude', 0.0001)  # Tiny noise in rad/s
        
        num_robots = self.get_parameter('num_robots').value
        self.noise_amp = self.get_parameter('noise_amplitude').value
        
        self.subscribers = []
        self.publishers = []
        self.latest_odom = {}
        
        for i in range(1, num_robots + 1):
            robot_name = f'tb3_{i}'
            
            # Subscribe to original odometry from Gazebo
            sub = self.create_subscription(
                Odometry,
                f'/{robot_name}/odom_original',
                lambda msg, name=robot_name: self.odom_callback(msg, name),
                10
            )
            self.subscribers.append(sub)
            
            # Publish modified odometry
            pub = self.create_publisher(Odometry, f'/{robot_name}/odom', 10)
            self.publishers.append((robot_name, pub))
            
            self.latest_odom[robot_name] = None
        
        # Timer to add noise to odometry
        self.timer = self.create_timer(0.1, self.publish_noisy_odom)
        self.counter = 0
        self.get_logger().info(f'Odom noise injector started for {num_robots} robots')
    
    def odom_callback(self, msg, robot_name):
        self.latest_odom[robot_name] = msg
    
    def publish_noisy_odom(self):
        self.counter += 1
        
        # Add sinusoidal noise that averages to zero
        noise = self.noise_amp * math.sin(self.counter * 0.1)
        
        for robot_name, pub in self.publishers:
            if self.latest_odom[robot_name] is not None:
                noisy_odom = self.latest_odom[robot_name]
                
                # Add tiny angular velocity noise to trigger AMCL
                noisy_odom.twist.twist.angular.z += noise
                
                pub.publish(noisy_odom)


def main(args=None):
    rclpy.init(args=args)
    node = OdomNoiseInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
