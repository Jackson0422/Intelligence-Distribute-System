#!/usr/bin/env python3
#
# Multi-robot Gazebo launch for TurtleBot3 using a shared SDF template.
#

import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_description = get_package_share_directory('turtlebot3_description')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_localization_eval = get_package_share_directory('localization_evaluation')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    num_robots = LaunchConfiguration('num_robots', default='2')

    pose_args = {
        f'x_pose_{i}': LaunchConfiguration(f'x_pose_{i}', default='')
        for i in range(1, 21)
    }
    pose_args.update({
        f'y_pose_{i}': LaunchConfiguration(f'y_pose_{i}', default='')
        for i in range(1, 21)
    })
    pose_args.update({
        f'yaw_pose_{i}': LaunchConfiguration(f'yaw_pose_{i}', default='')
        for i in range(1, 21)
    })

    # URDF template
    urdf_file = os.path.join(
        pkg_turtlebot3_description,
        'urdf',
        'turtlebot3_burger.urdf'
    )
    with open(urdf_file, 'r') as f:
        urdf_content = f.read()

    def render_urdf(namespace: str):
        """Render the URDF with the given namespace."""
        urdf_xacro = urdf_content.replace('$(arg namespace)', f'{namespace}/')
        with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as tmp:
            tmp.write(urdf_xacro)
            tmp_path = tmp.name
        rendered = xacro.process_file(tmp_path).toxml()
        os.unlink(tmp_path)
        return rendered

    # Shared SDF template (tb3_basic) with placeholder __ROBOT__
    sdf_template_path = os.path.join(
        pkg_localization_eval,
        'models',
        'tb3_basic',
        'model.sdf'
    )
    with open(sdf_template_path, 'r') as sdf_file:
        sdf_template = sdf_file.read()

    def render_sdf(robot_name: str):
        """Render SDF for a robot by replacing placeholder with its name/namespace."""
        sdf_text = sdf_template.replace('__ROBOT__', robot_name)
        with tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False) as tmp:
            tmp.write(sdf_text)
            return tmp.name

    world_file = LaunchConfiguration(
        'world',
        default=os.path.join(pkg_turtlebot3_gazebo, 'worlds', 'turtlebot3_world.world')
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    def launch_setup(context, *args, **kwargs):
        actions = []
        count = int(num_robots.perform(context))
        for idx in range(1, count + 1):
            ns = f'tb3_{idx}'
            robot_desc = render_urdf(ns)
            sdf_path = render_sdf(ns)
            x_pose = pose_args[f'x_pose_{idx}'].perform(context)
            y_pose = pose_args[f'y_pose_{idx}'].perform(context)
            yaw_pose = pose_args[f'yaw_pose_{idx}'].perform(context)

            rsp = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                namespace=ns,
                output='screen',
                parameters=[{
                    'robot_description': robot_desc,
                    'use_sim_time': use_sim_time
                }]
            )

            spawn = Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', ns,
                    '-file', sdf_path,
                    '-x', x_pose,
                    '-y', y_pose,
                    '-z', '0.01',
                    '-Y', yaw_pose,
                ],
                output='screen',
            )

            actions.append(rsp)
            actions.append(TimerAction(period=0.5 * idx, actions=[spawn]))

        return actions

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
        description='Number of robots to spawn (>=1)'
    ))
    ld.add_action(DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_turtlebot3_gazebo, 'worlds', 'turtlebot3_world.world'),
        description='World file to load in Gazebo'
    ))
    for idx in range(1, 21):
        for axis in ['x', 'y', 'yaw']:
            name = f'{axis}_pose_{idx}'
            ld.add_action(DeclareLaunchArgument(
                name, default_value='',
                description=f'Override {axis} pose for robot {idx}'
            ))

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
