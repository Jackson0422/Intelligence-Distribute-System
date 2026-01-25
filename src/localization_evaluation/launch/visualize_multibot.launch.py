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
    _ = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    num_robots = LaunchConfiguration('num_robots')
    map_yaml_file = LaunchConfiguration('map')
    world_file = LaunchConfiguration('world')
    autostart = LaunchConfiguration('autostart')
    use_shared_params = LaunchConfiguration('use_shared_params')
    shared_params_file = LaunchConfiguration('shared_params_file')
    randomize_initial_pose = LaunchConfiguration('randomize_initial_pose')
    randomize_spawn = LaunchConfiguration('randomize_spawn')
    gt_spawn_radius = LaunchConfiguration('gt_spawn_radius')
    spawn_radius = LaunchConfiguration('spawn_radius')
    _ = LaunchConfiguration('random_seed')
    rviz_config = LaunchConfiguration('rviz_config')
    log_ground_truth = LaunchConfiguration('log_ground_truth')
    log_dir = os.path.join(pkg_localization_eval, 'logs')
    resolved_seed = LaunchConfiguration('resolved_seed')

    def resolve_map_and_seed(context, *args, **kwargs):
        # Get the actual value of randomize_initial_pose from launch configuration
        randomize_amcl_pose = randomize_initial_pose.perform(context).lower() == 'true'
        randomize_spawn_flag = randomize_spawn.perform(context).lower() == 'true'
        
        # Resolve map aliases to full paths
        map_arg = map_yaml_file.perform(context)
        alias_map = {
            'empty_map': os.path.join(pkg_localization_eval, 'maps', 'empty_map.yaml'),
            'square_walls_map': os.path.join(pkg_localization_eval, 'maps', 'square_walls_map.yaml'),
            'turtle_world': os.path.join(pkg_localization_eval, 'maps', 'turtle_world.yaml'),
            'turtle_world_big': os.path.join(pkg_localization_eval, 'maps', 'turtle_world_big.yaml'),
        }
        map_resolved = alias_map.get(map_arg, map_arg)
        if map_resolved == map_arg and not os.path.isabs(map_arg):
            candidate = os.path.join(pkg_localization_eval, 'maps', map_arg)
            map_resolved = candidate if os.path.exists(candidate) else map_arg

        # Resolve world aliases similarly
        world_arg = world_file.perform(context)
        world_alias = {
            'empty_world': os.path.join(pkg_localization_eval, 'worlds', 'empty.world'),
            'turtle_world': os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'worlds',
                'turtlebot3_world.world'
            ),
            'turtle_world_big': os.path.join(
                pkg_localization_eval,
                'worlds',
                'turtlebot3_world_big.world'
            ),
        }
        world_resolved = world_alias.get(world_arg, world_arg)
        if world_resolved == world_arg and not os.path.isabs(world_arg):
            candidate = os.path.join(pkg_localization_eval, 'worlds', world_arg)
            world_resolved = candidate if os.path.exists(candidate) else world_arg

        # Resolve seed
        seed_val = LaunchConfiguration('random_seed').perform(context)
        if str(seed_val).lower() == 'auto':
            seed_val = str(int(time.time() * 1000) % 1000000000)
        rng = random.Random(int(seed_val))
        gt_radius = float(LaunchConfiguration('gt_spawn_radius').perform(context))
        sample_free = LaunchConfiguration('sample_free_space').perform(context).lower() == 'true'
        max_robots = int(LaunchConfiguration('num_robots').perform(context))
        max_radius = 2.2  # hard cap on distance to keep spawns near map center
        effective_radius = min(gt_radius, max_radius)
        effective_radius_sq = effective_radius * effective_radius
        obstacle_clearance = 0.2  # meters away from obstacles
        min_robot_spacing = 0.4  # meters between robot centers
        min_robot_spacing_sq = min_robot_spacing * min_robot_spacing
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

        if randomize_spawn_flag and sample_free:
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
                edge_margin = max(1, int(0.5 / res))  # keep spawns ~0.5m from map edges
                free_mask[:edge_margin, :] = False
                free_mask[-edge_margin:, :] = False
                free_mask[:, :edge_margin] = False
                free_mask[:, -edge_margin:] = False

                # Constrain to the occupied-area bounding box (plus margin) to avoid far-out padding
                occ_thresh = 0.65
                occ_mask = img <= int(occ_thresh * maxval)
                occ_indices = np.argwhere(occ_mask)
                if len(occ_indices) > 0:
                    r_min, c_min = occ_indices.min(axis=0)
                    r_max, c_max = occ_indices.max(axis=0)
                    bbox_margin = 0  # only occupied region, no expansion
                    r_min = max(0, r_min - bbox_margin)
                    c_min = max(0, c_min - bbox_margin)
                    r_max = min(height - 1, r_max + bbox_margin)
                    c_max = min(width - 1, c_max + bbox_margin)
                    bbox_mask = np.zeros_like(free_mask, dtype=bool)
                    bbox_mask[r_min:r_max + 1, c_min:c_max + 1] = True
                    free_mask &= bbox_mask

                # Enforce obstacle clearance by removing free cells too close to occupied cells
                clearance_cells = max(1, int(math.ceil(obstacle_clearance / res)))
                try:
                    from numpy.lib.stride_tricks import sliding_window_view
                    window = sliding_window_view(occ_mask, (2 * clearance_cells + 1, 2 * clearance_cells + 1))
                    near_occ = window.any(axis=(2, 3))
                    pad = clearance_cells
                    near_padded = np.pad(near_occ, pad_width=pad, mode='constant', constant_values=False)
                    free_mask &= ~near_padded[:height, :width]
                except Exception:
                    # Fallback: simple dilation-like padding
                    dilated = occ_mask.copy()
                    for dr in range(-clearance_cells, clearance_cells + 1):
                        for dc in range(-clearance_cells, clearance_cells + 1):
                            if dr == 0 and dc == 0:
                                continue
                            shifted = np.zeros_like(occ_mask)
                            rs = slice(max(0, dr), height + min(0, dr))
                            cs = slice(max(0, dc), width + min(0, dc))
                            r_src = slice(max(0, -dr), height - max(0, dr))
                            c_src = slice(max(0, -dc), width - max(0, dc))
                            shifted[rs, cs] = occ_mask[r_src, c_src]
                            dilated |= shifted
                    free_mask &= ~dilated

                free_indices = np.argwhere(free_mask)
                if len(free_indices) == 0:
                    raise RuntimeError('No free cells found in map')

                # Precompute world positions and keep only those inside the radius cap
                filtered_positions = []
                for r, c in free_indices:
                    x = origin[0] + (c + 0.5) * res
                    y = origin[1] + (height - r - 0.5) * res
                    if x * x + y * y <= effective_radius_sq:
                        filtered_positions.append((x, y))

                if not filtered_positions:
                    raise RuntimeError(
                        f'No free cells inside gt_spawn_radius={effective_radius:.2f}m'
                    )
                usable_positions = filtered_positions

                chosen_positions = []
                for _ in range(max_robots):
                    placed = False
                    for _ in range(200):
                        x, y = usable_positions[rng.randrange(len(usable_positions))]
                        if all((x - px) ** 2 + (y - py) ** 2 >= min_robot_spacing_sq for px, py in chosen_positions):
                            chosen_positions.append((x, y))
                            poses.append((x, y, rng.uniform(-math.pi, math.pi)))
                            placed = True
                            break
                    if not placed:
                        # If spacing fails, fall back to center
                        poses.append((0.0, 0.0, rng.uniform(-math.pi, math.pi)))
            except Exception as e:
                print(f'[visualize_multibot] Free-space sampling failed: {e}, falling back to box sampling')
                limited_radius = effective_radius
                chosen_positions = []
                for _ in range(max_robots):
                    for _ in range(50):
                        x = rng.uniform(-limited_radius, limited_radius)
                        y = rng.uniform(-limited_radius, limited_radius)
                        if x * x + y * y <= effective_radius_sq and all(
                            (x - px) ** 2 + (y - py) ** 2 >= min_robot_spacing_sq for px, py in chosen_positions
                        ):
                            chosen_positions.append((x, y))
                            break
                    else:
                        x, y = 0.0, 0.0
                    poses.append((x, y, rng.uniform(-math.pi, math.pi)))
        elif randomize_spawn_flag:
            limited_radius = effective_radius
            chosen_positions = []
            for _ in range(max_robots):
                for _ in range(50):
                    x = rng.uniform(-limited_radius, limited_radius)
                    y = rng.uniform(-limited_radius, limited_radius)
                    if x * x + y * y <= effective_radius_sq and all(
                        (x - px) ** 2 + (y - py) ** 2 >= min_robot_spacing_sq for px, py in chosen_positions
                    ):
                        chosen_positions.append((x, y))
                        break
                else:
                    x, y = 0.0, 0.0
                poses.append((x, y, rng.uniform(-math.pi, math.pi)))
        else:
            # Use explicit overrides if provided, otherwise place robots on a fixed circle.
            if max_robots > 1:
                min_circle_radius = min_robot_spacing / (2.0 * math.sin(math.pi / max_robots))
            else:
                min_circle_radius = 0.0
            circle_radius = min(max_radius, max(gt_radius, min_circle_radius))
            for idx in range(1, max_robots + 1):
                x_override = LaunchConfiguration(f'x_pose_{idx}').perform(context)
                y_override = LaunchConfiguration(f'y_pose_{idx}').perform(context)
                yaw_override = LaunchConfiguration(f'yaw_pose_{idx}').perform(context)
                if x_override != '' and y_override != '':
                    try:
                        x = float(x_override)
                        y = float(y_override)
                        yaw = float(yaw_override) if yaw_override != '' else 0.0
                    except ValueError:
                        x, y, yaw = 0.0, 0.0, 0.0
                else:
                    angle = 0.0 if max_robots == 1 else (2.0 * math.pi * (idx - 1) / max_robots)
                    x = circle_radius * math.cos(angle)
                    y = circle_radius * math.sin(angle)
                    yaw = 0.0
                poses.append((x, y, yaw))
        actions = [
            SetLaunchConfiguration('resolved_seed', seed_val),
            SetLaunchConfiguration('resolved_map', map_resolved),
            SetLaunchConfiguration('resolved_world', world_resolved),
            SetLaunchConfiguration('randomize_amcl_internal', str(randomize_amcl_pose)),
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
            'gt_spawn_radius': LaunchConfiguration('gt_spawn_radius'),
            'random_seed': resolved_seed,
            'world': LaunchConfiguration('resolved_world'),
            **{f'x_pose_{i}': LaunchConfiguration(f'x_pose_{i}') for i in range(1, 21)},
            **{f'y_pose_{i}': LaunchConfiguration(f'y_pose_{i}') for i in range(1, 21)},
            **{f'yaw_pose_{i}': LaunchConfiguration(f'yaw_pose_{i}') for i in range(1, 21)},
        }.items(),
    )

    def build_amcl_launch(context, *args, **kwargs):
        # When randomize_initial_pose is true, still pass spawn poses so AMCL can randomize around them.
        randomize_amcl = LaunchConfiguration('randomize_amcl_internal').perform(context).lower() == 'true'
        
        # Build base arguments that are always passed
        base_args = {
            'use_sim_time': use_sim_time,
            'num_robots': num_robots,
            'map': LaunchConfiguration('resolved_map'),
            'autostart': autostart,
            'use_shared_params': use_shared_params,
            'shared_params_file': shared_params_file,
            'randomize_initial_pose': randomize_initial_pose,
            'spawn_radius': spawn_radius,
            'random_seed': resolved_seed,
        }
        
        if randomize_amcl:
            print(f"[visualize_multibot] AMCL randomization ENABLED - using spawn poses as base")
        else:
            print(f"[visualize_multibot] AMCL randomization DISABLED - using spawn poses directly")
        base_args.update({
            **{f'x_pose_{i}': LaunchConfiguration(f'x_pose_{i}') for i in range(1, 21)},
            **{f'y_pose_{i}': LaunchConfiguration(f'y_pose_{i}') for i in range(1, 21)},
            **{f'yaw_pose_{i}': LaunchConfiguration(f'yaw_pose_{i}') for i in range(1, 21)},
        })
        
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_localization_eval, 'launch', 'amcl_multibot.launch.py')
            ),
            launch_arguments=base_args.items(),
        )]
    
    amcl_launch = OpaqueFunction(function=build_amcl_launch)

    gt_logger = Node(
        package='localization_evaluation',
        executable='ground_truth_logger',
        name='ground_truth_logger',
        output='screen',
        parameters=[{
            'robot_names': [f'tb3_{i}' for i in range(1, 21)],
            'use_sim_time': use_sim_time,
            'log_enabled': log_ground_truth,
            'log_dir': log_dir,
        }],
        condition=None,
    )

    # Jitter motion node - provides tiny rotation to trigger AMCL updates
    jitter_node = Node(
        package='localization_evaluation',
        executable='jitter_motion',
        name='jitter_motion',
        output='screen',
        parameters=[{
            'num_robots': num_robots,
            'angular_vel': 0.25,  # Faster rotation for quicker convergence (rad/s)
            'flip_period': 15.0,  # Seconds before reversing direction
            'use_sim_time': use_sim_time,
        }],
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
        default_value='turtle_world',
        description='Map alias (empty_map, square_walls_map, turtle_world, turtle_world_big) or full YAML path'
    ))
    ld.add_action(DeclareLaunchArgument(
        'world',
        default_value='turtle_world',
        description='World alias (empty_world, turtle_world) or full world path'
    ))
    ld.add_action(DeclareLaunchArgument(
        'autostart', default_value='true', description='Autostart Nav2 lifecycle nodes'
    ))
    ld.add_action(DeclareLaunchArgument(
        'randomize_spawn',
        default_value='true',
        description='Randomize Gazebo spawn pose per robot'
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
        'gt_spawn_radius',
        default_value='2.2',
        description='Max ground-truth spawn radius around origin (meters)'
    ))
    ld.add_action(DeclareLaunchArgument(
        'spawn_radius',
        default_value='1.0',
        description='Max AMCL randomization radius from spawn pose (meters)'
    ))
    ld.add_action(DeclareLaunchArgument(
        'random_seed',
        default_value='auto',
        description='Seed for pose randomization (int or "auto")'
    ))
    ld.add_action(DeclareLaunchArgument(
        'sample_free_space',
        default_value='true',
        description='Sample robot spawns from free space in the map (requires map PGM/YAML)'
    ))
    for idx in range(1, 21):
        for axis in ['x', 'y', 'yaw']:
            name = f'{axis}_pose_{idx}'
            ld.add_action(DeclareLaunchArgument(
                name,
                default_value='',
                description=f'Override {axis} pose for robot {idx}'
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
    ld.add_action(TimerAction(period=3.5, actions=[jitter_node]))  # Start jitter after GT logger
    ld.add_action(TimerAction(period=4.0, actions=[rviz_node]))

    return ld
