#!/usr/bin/env python3
"""
去中心化协同定位启动文件
Decentralized Collaborative Localization Launch File

启动两个协同定位代理节点，实现机器人之间的P2P协同定位

使用方法:
1. 先启动Gazebo和AMCL:
   Terminal 1: ros2 launch localization_evaluation multibot_gazebo.launch.py
   Terminal 2: ros2 launch localization_evaluation amcl_multibot.launch.py

2. 再启动协同定位层:
   Terminal 3: ros2 launch localization_evaluation decentralized_coloc.launch.py

3. 启动评估和控制:
   Terminal 4: ros2 run localization_evaluation pose_eval_coloc
   Terminal 5: ros2 run localization_evaluation track_multibot

作者: Distributed Intelligent Systems Course
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # 声明启动参数
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
    
    # 获取启动配置
    gossip_rate = LaunchConfiguration('gossip_rate')
    self_weight = LaunchConfiguration('self_weight')
    peer_timeout = LaunchConfiguration('peer_timeout')
    correction_threshold = LaunchConfiguration('correction_threshold')
    
    # 协同定位代理 - TB3_0
    agent_tb3_0 = Node(
        package='localization_evaluation',
        executable='decentralized_coloc_agent',
        name='coloc_agent_tb3_0',
        output='screen',
        parameters=[{
            # 使用Gazebo仿真时间，保证与AMCL/TF时间戳一致
            'use_sim_time': True,
            'robot_id': 'tb3_0',
            'peer_ids': ['tb3_1'],
            'gossip_rate': gossip_rate,
            'self_weight': self_weight,
            'peer_timeout': peer_timeout,
            'correction_threshold': correction_threshold
        }],
        remappings=[
            # 确保TF话题正确
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ]
    )
    
    # 协同定位代理 - TB3_1
    agent_tb3_1 = Node(
        package='localization_evaluation',
        executable='decentralized_coloc_agent',
        name='coloc_agent_tb3_1',
        output='screen',
        parameters=[{
            # 使用Gazebo仿真时间，保证与AMCL/TF时间戳一致
            'use_sim_time': True,
            'robot_id': 'tb3_1',
            'peer_ids': ['tb3_0'],
            'gossip_rate': gossip_rate,
            'self_weight': self_weight,
            'peer_timeout': peer_timeout,
            'correction_threshold': correction_threshold
        }],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ]
    )
    
    # 构建启动描述
    ld = LaunchDescription()
    
    # 添加参数声明
    ld.add_action(declare_gossip_rate)
    ld.add_action(declare_self_weight)
    ld.add_action(declare_peer_timeout)
    ld.add_action(declare_correction_threshold)
    
    # 添加节点
    ld.add_action(agent_tb3_0)
    ld.add_action(agent_tb3_1)
    
    return ld

