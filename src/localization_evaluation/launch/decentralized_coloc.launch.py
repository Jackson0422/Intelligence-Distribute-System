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

    declare_max_peer_updates = DeclareLaunchArgument(
        'max_peer_updates',
        default_value='2',
        description='Max peer updates per cycle (0 = no limit)'
    )

    declare_ambiguity_distance = DeclareLaunchArgument(
        'ambiguity_distance',
        default_value='0.3',
        description='Ambiguity distance for peer association (meters, <=0 disables)'
    )

    declare_max_innovation_dist = DeclareLaunchArgument(
        'max_innovation_dist',
        default_value='0.5',
        description='Innovation gate for relative measurements (meters, <=0 disables)'
    )

    declare_robot_radius = DeclareLaunchArgument(
        'robot_radius',
        default_value='0.10',
        description='Approx robot radius used for LIDAR cluster gating (meters)'
    )

    declare_peer_search_radius = DeclareLaunchArgument(
        'peer_search_radius',
        default_value='0.6',
        description='Search radius around expected peer position (meters)'
    )

    declare_cluster_link_distance = DeclareLaunchArgument(
        'cluster_link_distance',
        default_value='0.12',
        description='Max gap between scan points in a cluster (meters, <=0 uses robot_radius)'
    )

    declare_min_cluster_points = DeclareLaunchArgument(
        'min_cluster_points',
        default_value='4',
        description='Minimum number of scan points for a valid peer cluster'
    )

    declare_cluster_extent_min = DeclareLaunchArgument(
        'cluster_extent_min',
        default_value='0.05',
        description='Minimum cluster extent to accept (meters, <=0 uses robot_radius)'
    )

    declare_cluster_extent_max = DeclareLaunchArgument(
        'cluster_extent_max',
        default_value='0.45',
        description='Maximum cluster extent to accept (meters, <=0 uses robot_radius)'
    )

    declare_cluster_span_max = DeclareLaunchArgument(
        'cluster_span_max',
        default_value='0.1',
        description='Max distance between first/last scan point in cluster (meters, <=0 disables)'
    )

    declare_peer_detection_log_period = DeclareLaunchArgument(
        'peer_detection_log_period',
        default_value='1.0',
        description='Min seconds between peer detection logs (<=0 logs every detection)'
    )

    declare_debug_gt = DeclareLaunchArgument(
        'debug_gt',
        default_value='false',
        description='Enable ground truth debug logging (requires /<robot>/ground_truth)'
    )
    
    declare_correction_threshold = DeclareLaunchArgument(
        'correction_threshold',
        default_value='0.01',
        description='Minimum correction distance to publish (meters)'
    )

    declare_max_correction = DeclareLaunchArgument(
        'max_correction',
        default_value='0.8',
        description='Clamp correction distance (meters, <=0 disables)'
    )
    
    # Get launch configurations
    num_robots = LaunchConfiguration('num_robots')
    gossip_rate = LaunchConfiguration('gossip_rate')
    self_weight = LaunchConfiguration('self_weight')
    peer_timeout = LaunchConfiguration('peer_timeout')
    correction_threshold = LaunchConfiguration('correction_threshold')
    max_correction = LaunchConfiguration('max_correction')
    max_peer_updates = LaunchConfiguration('max_peer_updates')
    ambiguity_distance = LaunchConfiguration('ambiguity_distance')
    max_innovation_dist = LaunchConfiguration('max_innovation_dist')
    debug_gt = LaunchConfiguration('debug_gt')
    robot_radius = LaunchConfiguration('robot_radius')
    peer_search_radius = LaunchConfiguration('peer_search_radius')
    cluster_link_distance = LaunchConfiguration('cluster_link_distance')
    min_cluster_points = LaunchConfiguration('min_cluster_points')
    cluster_extent_min = LaunchConfiguration('cluster_extent_min')
    cluster_extent_max = LaunchConfiguration('cluster_extent_max')
    cluster_span_max = LaunchConfiguration('cluster_span_max')
    peer_detection_log_period = LaunchConfiguration('peer_detection_log_period')
    
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
                    'correction_threshold': correction_threshold,
                    'max_correction': max_correction,
                    'max_peer_updates': max_peer_updates,
                    'ambiguity_distance': ambiguity_distance,
                    'max_innovation_dist': max_innovation_dist,
                    'robot_radius': robot_radius,
                    'peer_search_radius': peer_search_radius,
                    'cluster_link_distance': cluster_link_distance,
                    'min_cluster_points': min_cluster_points,
                    'cluster_extent_min': cluster_extent_min,
                    'cluster_extent_max': cluster_extent_max,
                    'cluster_span_max': cluster_span_max,
                    'peer_detection_log_period': peer_detection_log_period,
                    'debug_gt': debug_gt
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
    ld.add_action(declare_max_peer_updates)
    ld.add_action(declare_ambiguity_distance)
    ld.add_action(declare_max_correction)
    ld.add_action(declare_max_innovation_dist)
    ld.add_action(declare_robot_radius)
    ld.add_action(declare_peer_search_radius)
    ld.add_action(declare_cluster_link_distance)
    ld.add_action(declare_min_cluster_points)
    ld.add_action(declare_cluster_extent_min)
    ld.add_action(declare_cluster_extent_max)
    ld.add_action(declare_cluster_span_max)
    ld.add_action(declare_peer_detection_log_period)
    ld.add_action(declare_debug_gt)
    ld.add_action(declare_correction_threshold)

    ld.add_action(OpaqueFunction(function=build_agents))
    
    return ld
