#!/usr/bin/env python3
#
# Multi-robot Gazebo launch file for TurtleBot3
# This file launches two robots with proper TF frame separation
#

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Get package directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_turtlebot3_description = get_package_share_directory('turtlebot3_description')
    pkg_localization_eval = get_package_share_directory('localization_evaluation')
    
    # Get TURTLEBOT3_MODEL from environment
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    num_robots = LaunchConfiguration('num_robots', default='2')
    
    # URDF file for robot_state_publisher (xacro format)
    urdf_file = os.path.join(
        pkg_turtlebot3_description,
        'urdf',
        'turtlebot3_burger.urdf'
    )
    
    # Read URDF file as text
    with open(urdf_file, 'r') as f:
        urdf_content = f.read()
    
    # Process xacro for robot 1 (no namespace) - manually replace $(arg namespace)
    robot_desc_1_xacro = urdf_content.replace('$(arg namespace)', '')
    # Now process with xacro (no mappings needed as we already replaced the args)
    import tempfile
    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as tmp1:
        tmp1.write(robot_desc_1_xacro)
        tmp1_path = tmp1.name
    robot_desc_1 = xacro.process_file(tmp1_path).toxml()
    os.unlink(tmp1_path)
    
    # Process xacro for robot 2 (tb3_1 namespace) - manually replace $(arg namespace)
    robot_desc_2_xacro = urdf_content.replace('$(arg namespace)', 'tb3_1/')
    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as tmp2:
        tmp2.write(robot_desc_2_xacro)
        tmp2_path = tmp2.name
    robot_desc_2 = xacro.process_file(tmp2_path).toxml()
    os.unlink(tmp2_path)
    
    # Process xacro for robot 3 (tb3_2 namespace) - 新增
    robot_desc_3_xacro = urdf_content.replace('$(arg namespace)', 'tb3_2/')
    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as tmp3:
        tmp3.write(robot_desc_3_xacro)
        tmp3_path = tmp3.name
    robot_desc_3 = xacro.process_file(tmp3_path).toxml()
    os.unlink(tmp3_path)
    
    # Process xacro for robot 4 (tb3_3 namespace) - 新增
    robot_desc_4_xacro = urdf_content.replace('$(arg namespace)', 'tb3_3/')
    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as tmp4:
        tmp4.write(robot_desc_4_xacro)
        tmp4_path = tmp4.name
    robot_desc_4 = xacro.process_file(tmp4_path).toxml()
    os.unlink(tmp4_path)
    
    # Robot 1 position (no namespace)
    x_pose_1 = LaunchConfiguration('x_pose_1', default='-2.0')
    y_pose_1 = LaunchConfiguration('y_pose_1', default='-0.5')
    
    # Robot 2 position (tb3_1 namespace)
    x_pose_2 = LaunchConfiguration('x_pose_2', default='0.0')
    y_pose_2 = LaunchConfiguration('y_pose_2', default='0.5')
    
    # Robot 3 position (tb3_2 namespace) - 新增
    x_pose_3 = LaunchConfiguration('x_pose_3', default='-1.0')
    y_pose_3 = LaunchConfiguration('y_pose_3', default='-1.5')
    
    # Robot 4 position (tb3_3 namespace) - 新增
    x_pose_4 = LaunchConfiguration('x_pose_4', default='2.0')
    y_pose_4 = LaunchConfiguration('y_pose_4', default='0.0')
    
    # World file
    world = os.path.join(
        pkg_turtlebot3_gazebo,
        'worlds',
        'turtlebot3_world.world'
    )
    
    # SDF model file for robot 1 (original, no namespace)
    sdf_path_robot1 = os.path.join(
        pkg_turtlebot3_gazebo,
        'models',
        model_folder,
        'model.sdf'
    )
    
    # SDF model file for robot 2 (custom with tb3_1 namespace and frame prefix)
    sdf_path_robot2 = os.path.join(
        pkg_localization_eval,
        'models',
        'tb3_1',
        'model.sdf'
    )
    
    # SDF model file for robot 3 (custom with tb3_2 namespace) - 新增
    sdf_path_robot3 = os.path.join(
        pkg_localization_eval,
        'models',
        'tb3_2',
        'model.sdf'
    )
    
    # SDF model file for robot 4 (custom with tb3_3 namespace) - 新增
    sdf_path_robot4 = os.path.join(
        pkg_localization_eval,
        'models',
        'tb3_3',
        'model.sdf'
    )
    
    # Launch Gazebo server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )
    
    # Launch Gazebo client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    
    # ========== Robot 1 (no namespace) ==========
    # Robot state publisher for robot 1 (publishes TF: base_footprint -> base_link -> base_scan, etc.)
    robot_state_pub_1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc_1,
            'use_sim_time': use_sim_time
        }]
    )
    
    # Spawn robot 1 in Gazebo using original SDF model
    spawn_robot_1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'burger',
            '-file', sdf_path_robot1,
            '-x', x_pose_1,
            '-y', y_pose_1,
            '-z', '0.01',
        ],
        output='screen',
    )
    
    # ========== Robot 2 (tb3_1 namespace) ==========
    # Robot state publisher for robot 2 (publishes TF: tb3_1/base_footprint -> tb3_1/base_link -> tb3_1/base_scan, etc.)
    robot_state_pub_2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='tb3_1',
        output='screen',
        parameters=[{
            'robot_description': robot_desc_2,
            'use_sim_time': use_sim_time
        }]
    )
    
    # Spawn robot 2 in Gazebo using custom SDF model with tb3_1 frame prefix
    spawn_robot_2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tb3_1',
            '-file', sdf_path_robot2,
            '-x', x_pose_2,
            '-y', y_pose_2,
            '-z', '0.01',
        ],
        output='screen',
    )
    
    # Delay robot 2 spawn to avoid race condition
    delayed_spawn_robot_2 = TimerAction(
        period=2.0,
        actions=[spawn_robot_2]
    )
    
    # ========== Robot 3 (tb3_2 namespace) ========== 新增
    # Robot state publisher for robot 3
    robot_state_pub_3 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='tb3_2',
        output='screen',
        parameters=[{
            'robot_description': robot_desc_3,
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(PythonExpression([num_robots, ' >= 3']))
    )
    
    # Spawn robot 3 in Gazebo
    spawn_robot_3 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tb3_2',
            '-file', sdf_path_robot3,
            '-x', x_pose_3,
            '-y', y_pose_3,
            '-z', '0.01',
        ],
        output='screen',
        condition=IfCondition(PythonExpression([num_robots, ' >= 3']))
    )
    
    # Delay robot 3 spawn
    delayed_spawn_robot_3 = TimerAction(
        period=4.0,
        actions=[spawn_robot_3]
    )
    
    # ========== Robot 4 (tb3_3 namespace) ========== 新增
    # Robot state publisher for robot 4
    robot_state_pub_4 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='tb3_3',
        output='screen',
        parameters=[{
            'robot_description': robot_desc_4,
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(PythonExpression([num_robots, ' >= 4']))
    )
    
    # Spawn robot 4 in Gazebo
    spawn_robot_4 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tb3_3',
            '-file', sdf_path_robot4,
            '-x', x_pose_4,
            '-y', y_pose_4,
            '-z', '0.01',
        ],
        output='screen',
        condition=IfCondition(PythonExpression([num_robots, ' >= 4']))
    )
    
    # Delay robot 4 spawn
    delayed_spawn_robot_4 = TimerAction(
        period=6.0,
        actions=[spawn_robot_4]
    )
    
    # Build launch description
    ld = LaunchDescription()
    
    # Declare arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    ))
    ld.add_action(DeclareLaunchArgument(
        'num_robots',
        default_value='2',
        description='Number of robots to spawn (2-4)'
    ))
    ld.add_action(DeclareLaunchArgument(
        'x_pose_1', default_value='-2.0',
        description='X position of robot 1'
    ))
    ld.add_action(DeclareLaunchArgument(
        'y_pose_1', default_value='-0.5',
        description='Y position of robot 1'
    ))
    ld.add_action(DeclareLaunchArgument(
        'x_pose_2', default_value='0.0',
        description='X position of robot 2'
    ))
    ld.add_action(DeclareLaunchArgument(
        'y_pose_2', default_value='0.5',
        description='Y position of robot 2'
    ))
    ld.add_action(DeclareLaunchArgument(
        'x_pose_3', default_value='-1.0',
        description='X position of robot 3'
    ))
    ld.add_action(DeclareLaunchArgument(
        'y_pose_3', default_value='-1.5',
        description='Y position of robot 3'
    ))
    ld.add_action(DeclareLaunchArgument(
        'x_pose_4', default_value='2.0',
        description='X position of robot 4'
    ))
    ld.add_action(DeclareLaunchArgument(
        'y_pose_4', default_value='0.0',
        description='Y position of robot 4'
    ))
    
    # Add actions
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_pub_1)
    ld.add_action(spawn_robot_1)
    ld.add_action(robot_state_pub_2)
    ld.add_action(delayed_spawn_robot_2)
    ld.add_action(robot_state_pub_3)  # 新增
    ld.add_action(delayed_spawn_robot_3)  # 新增
    ld.add_action(robot_state_pub_4)  # 新增
    ld.add_action(delayed_spawn_robot_4)  # 新增
    
    return ld
