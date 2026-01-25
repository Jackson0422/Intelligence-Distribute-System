#!/usr/bin/env python3
#
# Multi-robot Gazebo launch for TurtleBot3 using a shared SDF template.
#

import os
import tempfile
import math
import random
import time
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
    randomize_spawn = LaunchConfiguration('randomize_spawn', default='true')
    gt_spawn_radius = LaunchConfiguration('gt_spawn_radius', default='2.2')
    random_seed = LaunchConfiguration('random_seed', default='auto')

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
        randomize_spawn_flag = randomize_spawn.perform(context).lower() == 'true'
        seed_val = random_seed.perform(context)
        if str(seed_val).lower() == 'auto':
            seed_val = str(int(time.time() * 1000) % 1000000000)
        rng = random.Random(int(seed_val))
        max_spawn_radius = 2.2
        spawn_radius_val = min(float(gt_spawn_radius.perform(context)), max_spawn_radius)
        min_robot_spacing = 0.4
        min_robot_spacing_sq = min_robot_spacing * min_robot_spacing
        chosen_positions = []
        for idx in range(1, count + 1):
            ns = f'tb3_{idx}'
            robot_desc = render_urdf(ns)
            sdf_path = render_sdf(ns)
            x_pose = pose_args[f'x_pose_{idx}'].perform(context)
            y_pose = pose_args[f'y_pose_{idx}'].perform(context)
            yaw_pose = pose_args[f'yaw_pose_{idx}'].perform(context)
            if randomize_spawn_flag and (x_pose == '' or y_pose == ''):
                placed = False
                for _ in range(50):
                    r = spawn_radius_val * math.sqrt(rng.random())
                    theta = rng.uniform(-math.pi, math.pi)
                    x_val = r * math.cos(theta)
                    y_val = r * math.sin(theta)
                    if all(
                        (x_val - px) ** 2 + (y_val - py) ** 2 >= min_robot_spacing_sq
                        for px, py in chosen_positions
                    ):
                        chosen_positions.append((x_val, y_val))
                        placed = True
                        break
                if not placed:
                    x_val, y_val = 0.0, 0.0
                x_pose = str(x_val)
                y_pose = str(y_val)
                if yaw_pose == '':
                    yaw_pose = str(rng.uniform(-math.pi, math.pi))
            else:
                if x_pose == '':
                    x_pose = '0.0'
                if y_pose == '':
                    y_pose = '0.0'
                if yaw_pose == '':
                    yaw_pose = '0.0'

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
        'randomize_spawn',
        default_value='true',
        description='Randomize spawn poses when overrides are not provided'
    ))
    ld.add_action(DeclareLaunchArgument(
        'gt_spawn_radius',
        default_value='2.2',
        description='Max ground-truth spawn radius (m) around the world origin'
    ))
    ld.add_action(DeclareLaunchArgument(
        'random_seed',
        default_value='auto',
        description='Seed for spawn randomization (int or "auto")'
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
