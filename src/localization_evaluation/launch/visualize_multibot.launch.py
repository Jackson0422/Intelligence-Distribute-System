#!/usr/bin/env python3
"""
Bringup for visualizing the full multibot simulation: Gazebo + AMCL + RViz.

Order:
1) Gazebo spawn for all robots
2) AMCL (map server + lifecycle) with shared params and random pose options
3) RViz2 with a default Nav2 view
"""

import os
import time
import random
import math
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_localization_eval = get_package_share_directory('localization_evaluation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    num_robots = LaunchConfiguration('num_robots')
    map_yaml_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    use_shared_params = LaunchConfiguration('use_shared_params')
    shared_params_file = LaunchConfiguration('shared_params_file')
    randomize_initial_pose = LaunchConfiguration('randomize_initial_pose')
    spawn_radius = LaunchConfiguration('spawn_radius')
    random_seed = LaunchConfiguration('random_seed')
    rviz_config = LaunchConfiguration('rviz_config')
    log_ground_truth = LaunchConfiguration('log_ground_truth')
    log_dir = os.path.join(pkg_localization_eval, 'logs')
    resolved_seed = LaunchConfiguration('resolved_seed')

    def resolve_map_and_seed(context, *args, **kwargs):
        # Resolve map aliases to full paths
        map_arg = map_yaml_file.perform(context)
        alias_map = {
            'empty_map': os.path.join(pkg_localization_eval, 'maps', 'empty_map.yaml'),
            'square_walls_map': os.path.join(pkg_localization_eval, 'maps', 'square_walls_map.yaml'),
            'turtle_world': os.path.join(pkg_localization_eval, 'maps', 'turtle_world.yaml'),
            'turtle_world_big': os.path.join(pkg_localization_eval, 'maps', 'turtle_world_big.yaml'),
        }
        map_resolved = alias_map.get(map_arg, map_arg)

        # Resolve seed
        seed_val = LaunchConfiguration('random_seed').perform(context)
        if str(seed_val).lower() == 'auto':
            seed_val = str(int(time.time() * 1000) % 1000000000)
        rng = random.Random(int(seed_val))
        radius = float(LaunchConfiguration('spawn_radius').perform(context))
        sample_free = LaunchConfiguration('sample_free_space').perform(context).lower() == 'true'
        max_robots = int(LaunchConfiguration('num_robots').perform(context))
        poses = []

        def read_pgm(pgm_path):
            with open(pgm_path, 'rb') as f:
                header = f.readline().strip()
                if header != b'P5':
                    raise ValueError('Unsupported PGM format')
                line = f.readline()
                while line.startswith(b'#'):
                    line = f.readline()
                width, height = map(int, line.split())
                maxval = int(f.readline())
                data = f.read()
            import numpy as np
            img = np.frombuffer(data, dtype=np.uint8).reshape((height, width))
            return img, maxval, width, height

        if sample_free:
            try:
                cfg = yaml.safe_load(open(map_resolved, 'r'))
                res = float(cfg.get('resolution', 0.05))
                origin = cfg.get('origin', [0.0, 0.0, 0.0])
                free_thresh = float(cfg.get('free_thresh', 0.196))
                pgm_path = cfg.get('image')
                if not os.path.isabs(pgm_path):
                    pgm_path = os.path.join(os.path.dirname(map_resolved), pgm_path)
                img, maxval, width, height = read_pgm(pgm_path)
                import numpy as np
                free_mask = img > int(free_thresh * maxval)
                free_indices = np.argwhere(free_mask)
                if len(free_indices) == 0:
                    raise RuntimeError('No free cells found in map')
                for _ in range(max_robots):
                    r, c = free_indices[rng.randrange(len(free_indices))]
                    x = origin[0] + (c + 0.5) * res
                    y = origin[1] + (height - r - 0.5) * res
                    poses.append((x, y, rng.uniform(-math.pi, math.pi)))
            except Exception as e:
                print(f'[visualize_multibot] Free-space sampling failed: {e}, falling back to box sampling')
                for _ in range(max_robots):
                    poses.append((
                        rng.uniform(-radius, radius),
                        rng.uniform(-radius, radius),
                        rng.uniform(-math.pi, math.pi),
                    ))
        else:
            for _ in range(max_robots):
                poses.append((
                    rng.uniform(-radius, radius),
                    rng.uniform(-radius, radius),
                    rng.uniform(-math.pi, math.pi),
                ))
        actions = [
            SetLaunchConfiguration('resolved_seed', seed_val),
            SetLaunchConfiguration('resolved_map', map_resolved),
        ]
        for idx, (xv, yv, yawv) in enumerate(poses, start=1):
            actions.extend([
                SetLaunchConfiguration(f'x_pose_{idx}', str(xv)),
                SetLaunchConfiguration(f'y_pose_{idx}', str(yv)),
                SetLaunchConfiguration(f'yaw_pose_{idx}', str(yawv)),
            ])
        return actions

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localization_eval, 'launch', 'multibot_gazebo.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'num_robots': num_robots,
            'randomize_spawn': LaunchConfiguration('randomize_spawn'),
            'spawn_radius': LaunchConfiguration('spawn_radius'),
            'random_seed': resolved_seed,
            **{f'x_pose_{i}': LaunchConfiguration(f'x_pose_{i}') for i in range(1, 21)},
            **{f'y_pose_{i}': LaunchConfiguration(f'y_pose_{i}') for i in range(1, 21)},
            **{f'yaw_pose_{i}': LaunchConfiguration(f'yaw_pose_{i}') for i in range(1, 21)},
        }.items(),
    )

    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localization_eval, 'launch', 'amcl_multibot.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'num_robots': num_robots,
            'map': LaunchConfiguration('resolved_map'),
            'autostart': autostart,
            'use_shared_params': use_shared_params,
            'shared_params_file': shared_params_file,
            'randomize_initial_pose': randomize_initial_pose,
            'spawn_radius': spawn_radius,
            'random_seed': resolved_seed,
            'x_pose_1': LaunchConfiguration('x_pose_1'),
            'y_pose_1': LaunchConfiguration('y_pose_1'),
            'yaw_pose_1': LaunchConfiguration('yaw_pose_1'),
            'x_pose_2': LaunchConfiguration('x_pose_2'),
            'y_pose_2': LaunchConfiguration('y_pose_2'),
            'yaw_pose_2': LaunchConfiguration('yaw_pose_2'),
            'x_pose_3': LaunchConfiguration('x_pose_3'),
            'y_pose_3': LaunchConfiguration('y_pose_3'),
            'yaw_pose_3': LaunchConfiguration('yaw_pose_3'),
            'x_pose_4': LaunchConfiguration('x_pose_4'),
            'y_pose_4': LaunchConfiguration('y_pose_4'),
            'yaw_pose_4': LaunchConfiguration('yaw_pose_4'),
        }.items(),
    )

    gt_logger = Node(
        package='localization_evaluation',
        executable='ground_truth_logger',
        name='ground_truth_logger',
        output='screen',
        parameters=[{
            'robot_names': ['tb3_1', 'tb3_2', 'tb3_3', 'tb3_4'],
            'log_enabled': log_ground_truth,
            'log_dir': log_dir,
        }],
        condition=None,
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config],
    )

    ld = LaunchDescription()

    # Launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation clock'
    ))
    ld.add_action(DeclareLaunchArgument(
        'num_robots', default_value='4', description='Number of robots to spawn'
    ))
    ld.add_action(DeclareLaunchArgument(
        'map',
        default_value='turtle_world_big',
        description='Map alias (empty_map, square_walls_map, turtle_world, turtle_world_big) or full YAML path'
    ))
    ld.add_action(DeclareLaunchArgument(
        'autostart', default_value='true', description='Autostart Nav2 lifecycle nodes'
    ))
    ld.add_action(DeclareLaunchArgument(
        'use_shared_params',
        default_value='true',
        description='Reuse a single AMCL/Nav2 params file for all robots'
    ))
    ld.add_action(DeclareLaunchArgument(
        'shared_params_file',
        default_value=os.path.join(pkg_localization_eval, 'param', 'nav2_params_tb3_basic.yaml'),
        description='Template params file for all robots'
    ))
    ld.add_action(DeclareLaunchArgument(
        'randomize_initial_pose',
        default_value='true',
        description='Randomize AMCL initial pose per robot'
    ))
    ld.add_action(DeclareLaunchArgument(
        'spawn_radius',
        default_value='2.0',
        description='Half-width of square around origin for random pose'
    ))
    ld.add_action(DeclareLaunchArgument(
        'random_seed',
        default_value='auto',
        description='Seed for pose randomization (int or "auto")'
    ))
    for name, default in [
        ('x_pose_1', ''), ('y_pose_1', ''), ('yaw_pose_1', ''),
        ('x_pose_2', ''), ('y_pose_2', ''), ('yaw_pose_2', ''),
        ('x_pose_3', ''), ('y_pose_3', ''), ('yaw_pose_3', ''),
        ('x_pose_4', ''), ('y_pose_4', ''), ('yaw_pose_4', ''),
    ]:
        ld.add_action(DeclareLaunchArgument(
            name, default_value=default,
            description=f'Override {name.split("_")[0]} {name.split("_")[1]}'
        ))
    ld.add_action(DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_localization_eval, 'rviz', 'multibot_view.rviz'),
        description='RViz configuration file'
    ))
    ld.add_action(DeclareLaunchArgument(
        'log_ground_truth',
        default_value='true',
        description='Log GT/AMCL/coloc to CSV'
    ))

    # Order the startup slightly to avoid RViz complaining about missing TF/map
    ld.add_action(OpaqueFunction(function=resolve_map_and_seed))
    ld.add_action(TimerAction(period=0.0, actions=[gazebo_launch]))
    ld.add_action(TimerAction(period=2.0, actions=[amcl_launch]))
    ld.add_action(TimerAction(period=3.0, actions=[gt_logger]))
    ld.add_action(TimerAction(period=4.0, actions=[rviz_node]))

    return ld
