#!/usr/bin/env python3
"""
Decentralized Collaborative Localization Launch File

Launches two collaborative localization agent nodes to enable P2P collaborative localization between robots.

Usage:
1. First, launch Gazebo and AMCL:
   Terminal 1: ros2 launch localization_evaluation multibot_gazebo.launch.py
   Terminal 2: ros2 launch localization_evaluation amcl_multibot.launch.py

2. Then, launch the collaborative localization layer:
   Terminal 3: ros2 launch localization_evaluation decentralized_coloc.launch.py

3. Launch evaluation and control:
   Terminal 4: ros2 run localization_evaluation pose_eval_coloc
   Terminal 5: ros2 run localization_evaluation track_multibot

Author: Distributed Intelligent Systems Course
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    declare_num_robots = DeclareLaunchArgument(
        'num_robots',
        default_value='4',
        description='Number of robots for collaborative localization (supports 2-20)'
    )
    
    declare_gossip_rate = DeclareLaunchArgument(
        'gossip_rate',
        default_value='30.0',
        description='Gossip protocol update rate (Hz)'
    )
    
    declare_self_weight = DeclareLaunchArgument(
        'self_weight',
        default_value='0.7',
        description='Weight for own AMCL estimate (0-1)'
    )
    
    declare_peer_timeout = DeclareLaunchArgument(
        'peer_timeout',
        default_value='3.0',
        description='Timeout for peer beliefs (seconds)'
    )
    
    declare_correction_threshold = DeclareLaunchArgument(
        'correction_threshold',
        default_value='0.01',
        description='Minimum correction distance to publish (meters)'
    )
    
    # Get launch configurations
    num_robots = LaunchConfiguration('num_robots')
    gossip_rate = LaunchConfiguration('gossip_rate')
    self_weight = LaunchConfiguration('self_weight')
    peer_timeout = LaunchConfiguration('peer_timeout')
    correction_threshold = LaunchConfiguration('correction_threshold')
    
    # Build coloc agent nodes dynamically
    def build_agents(context):
        count = int(num_robots.perform(context))
        actions = []
        ids = [f'tb3_{i}' for i in range(1, count + 1)]
        for rid in ids:
            peers = [p for p in ids if p != rid]
            actions.append(Node(
                package='localization_evaluation',
                executable='decentralized_coloc_agent',
                name=f'coloc_agent_{rid}',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'robot_id': rid,
                    'peer_ids': peers,
                    'gossip_rate': gossip_rate,
                    'self_weight': self_weight,
                    'peer_timeout': peer_timeout,
                    'correction_threshold': correction_threshold
                }],
                remappings=[
                    ('/tf', '/tf'),
                    ('/tf_static', '/tf_static')
                ]
            ))
        return actions

    # Build launch description
    ld = LaunchDescription()
    
    # Add parameter declarations
    ld.add_action(declare_num_robots)
    ld.add_action(declare_gossip_rate)
    ld.add_action(declare_self_weight)
    ld.add_action(declare_peer_timeout)
    ld.add_action(declare_correction_threshold)

    ld.add_action(OpaqueFunction(function=build_agents))
    
    return ld
