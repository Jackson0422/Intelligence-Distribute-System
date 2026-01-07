#!/usr/bin/env python3
#
# Launch file for multi-robot AMCL localization
# Launches map server and AMCL nodes for 2-4 robots
#

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_localization_eval = get_package_share_directory('localization_evaluation')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    num_robots = LaunchConfiguration('num_robots', default='2')
    map_yaml_file = LaunchConfiguration(
        'map', 
        default='/opt/ros/humble/share/turtlebot3_navigation2/map/map.yaml'
    )
    autostart = LaunchConfiguration('autostart', default='true')
    
    # Parameters files
    params_file_tb3_0 = os.path.join(
        pkg_localization_eval, 
        'param', 
        'nav2_params_tb3_0.yaml'
    )
    params_file_tb3_1 = os.path.join(
        pkg_localization_eval, 
        'param', 
        'nav2_params_tb3_1.yaml'
    )
    params_file_tb3_2 = os.path.join(
        pkg_localization_eval, 
        'param', 
        'nav2_params_tb3_2.yaml'
    )
    params_file_tb3_3 = os.path.join(
        pkg_localization_eval, 
        'param', 
        'nav2_params_tb3_3.yaml'
    )
    
    # Map server node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml_file,
            'use_sim_time': use_sim_time
        }]
    )
    
    # AMCL node for robot 1 (no namespace)
    amcl_node_1 = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            params_file_tb3_0,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    
    # AMCL node for robot 2 (different node name instead of namespace)
    # Using node name 'tb3_1_amcl' without namespace to simplify parameter loading
    amcl_node_2 = Node(
        package='nav2_amcl',
        executable='amcl',
        name='tb3_1_amcl',
        output='screen',
        parameters=[params_file_tb3_1, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
            ('map', '/map'),  # Subscribe to global /map topic
            ('scan', '/tb3_1/scan'),  # Subscribe to TB3_1's scan
            ('amcl_pose', '/tb3_1/amcl_pose'),  # Publish to TB3_1's pose topic
            ('particle_cloud', '/tb3_1/particle_cloud'),  # Particle cloud
            ('initialpose', '/tb3_1/initialpose')  # Subscribe to TB3_1's initialpose
        ]
    )
    
    # AMCL node for robot 3 (tb3_2) - 新增
    amcl_node_3 = Node(
        package='nav2_amcl',
        executable='amcl',
        name='tb3_2_amcl',
        output='screen',
        parameters=[params_file_tb3_2, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
            ('map', '/map'),
            ('scan', '/tb3_2/scan'),
            ('amcl_pose', '/tb3_2/amcl_pose'),
            ('particle_cloud', '/tb3_2/particle_cloud'),
            ('initialpose', '/tb3_2/initialpose')
        ],
        condition=IfCondition(PythonExpression([num_robots, ' >= 3']))
    )
    
    # AMCL node for robot 4 (tb3_3) - 新增
    amcl_node_4 = Node(
        package='nav2_amcl',
        executable='amcl',
        name='tb3_3_amcl',
        output='screen',
        parameters=[params_file_tb3_3, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
            ('map', '/map'),
            ('scan', '/tb3_3/scan'),
            ('amcl_pose', '/tb3_3/amcl_pose'),
            ('particle_cloud', '/tb3_3/particle_cloud'),
            ('initialpose', '/tb3_3/initialpose')
        ],
        condition=IfCondition(PythonExpression([num_robots, ' >= 4']))
    )
    
    # Lifecycle manager for all nodes
    # All AMCLs now use global 'map' frame and set_initial_pose: true
    # node_names dynamically set based on num_robots
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': PythonExpression([
                "['map_server', 'amcl', 'tb3_1_amcl'] if int('", num_robots, "') == 2 else ",
                "(['map_server', 'amcl', 'tb3_1_amcl', 'tb3_2_amcl'] if int('", num_robots, "') == 3 else ",
                "['map_server', 'amcl', 'tb3_1_amcl', 'tb3_2_amcl', 'tb3_3_amcl'])"
            ]),
            'bond_timeout': 10.0,
        }]
    )
    
    # Build and return launch description
    ld = LaunchDescription()
    
    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    ))
    ld.add_action(DeclareLaunchArgument(
        'num_robots',
        default_value='2',
        description='Number of robots for AMCL (2-4)'
    ))
    ld.add_action(DeclareLaunchArgument(
        'map',
        default_value='/opt/ros/humble/share/turtlebot3_navigation2/map/map.yaml',
        description='Full path to map yaml file'
    ))
    ld.add_action(DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    ))
    
    # Add nodes
    ld.add_action(map_server_node)
    ld.add_action(amcl_node_1)
    ld.add_action(amcl_node_2)
    ld.add_action(amcl_node_3)  # 新增
    ld.add_action(amcl_node_4)  # 新增
    ld.add_action(lifecycle_manager_node)
    
    return ld

