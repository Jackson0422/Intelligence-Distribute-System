#!/usr/bin/env python3
import os
import copy
import random
import time
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_amcl_params(params_file: str) -> dict:
    """Extract AMCL ros__parameters from a YAML file whose top-level key is the node name."""
    if not os.path.exists(params_file):
        return {}

    with open(params_file, "r", encoding="utf-8") as param_stream:
        data = yaml.safe_load(param_stream) or {}

    if "amcl" in data and "ros__parameters" in data["amcl"]:
        return data["amcl"]["ros__parameters"]

    # Fallback: grab the first node block that has ros__parameters
    for value in data.values():
        if isinstance(value, dict) and "ros__parameters" in value:
            return value["ros__parameters"]
    return {}


def _load_robot_initial_pose(robot_name: str, pkg_dir: str) -> dict:
    """Fetch initial_pose for a robot from its specific nav2_params file if present."""
    candidate = os.path.join(pkg_dir, 'param', f'nav2_params_{robot_name}.yaml')
    if not os.path.exists(candidate):
        return {}
    with open(candidate, "r", encoding="utf-8") as param_stream:
        data = yaml.safe_load(param_stream) or {}
    for value in data.values():
        if isinstance(value, dict):
            init_pose = value.get('ros__parameters', {}).get('initial_pose')
            if init_pose:
                return init_pose
    return {}


def generate_launch_description():
    # Get package directory
    pkg_localization_eval = get_package_share_directory('localization_evaluation')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    num_robots = LaunchConfiguration('num_robots', default='4')
    map_yaml_file = LaunchConfiguration(
        'map',
        default=os.path.join(pkg_localization_eval, 'maps', 'turtle_world_big.yaml')
    )
    autostart = LaunchConfiguration('autostart', default='true')
    use_shared_params = LaunchConfiguration('use_shared_params', default='true')
    shared_params_file = LaunchConfiguration(
        'shared_params_file',
        default=os.path.join(pkg_localization_eval, 'param', 'nav2_params_tb3_basic.yaml')
    )
    randomize_initial_pose = LaunchConfiguration('randomize_initial_pose', default='true')
    spawn_radius = LaunchConfiguration('spawn_radius', default='2.0')
    random_seed = LaunchConfiguration('random_seed', default='auto')
    # Allow up to 20 robots without KeyErrors on overrides; extend if you need more.
    pose_overrides = {
        f'x_pose_{i}': LaunchConfiguration(f'x_pose_{i}', default='')
        for i in range(1, 21)
    }
    pose_overrides.update({
        f'y_pose_{i}': LaunchConfiguration(f'y_pose_{i}', default='')
        for i in range(1, 21)
    })
    pose_overrides.update({
        f'yaw_pose_{i}': LaunchConfiguration(f'yaw_pose_{i}', default='')
        for i in range(1, 21)
    })

    def launch_setup(context, *args, **kwargs):
        actions = []

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
        actions.append(map_server_node)

        lifecycle_nodes = ['map_server']

        robot_count = int(num_robots.perform(context))
        shared_params_enabled = use_shared_params.perform(context).lower() == 'true'
        base_params = {}
        if shared_params_enabled:
            base_params = _load_amcl_params(shared_params_file.perform(context))

        # Deterministic random generator so repeated runs can be reproduced
        seed_val = random_seed.perform(context)
        try:
            seed = int(seed_val)
        except ValueError:
            seed = int(time.time() * 1000)
        rng = random.Random(seed)
        spread = float(spawn_radius.perform(context))
        randomize_pose = randomize_initial_pose.perform(context).lower() == 'true'

        for robot_idx in range(1, robot_count + 1):
            robot_name = f'tb3_{robot_idx}'

            # Prefer a shared template; otherwise, fall back to per-robot YAML if it exists.
            params_source = None
            if shared_params_enabled and base_params:
                amcl_params = copy.deepcopy(base_params)
                amcl_params['base_frame_id'] = f'{robot_name}/base_footprint'
                amcl_params['odom_frame_id'] = f'{robot_name}/odom'
                amcl_params['scan_topic'] = f'/{robot_name}/scan'
                amcl_params['map_topic'] = 'map'

                overrides = (
                    pose_overrides.get(f'x_pose_{robot_idx}', LaunchConfiguration('', default='')).perform(context),
                    pose_overrides.get(f'y_pose_{robot_idx}', LaunchConfiguration('', default='')).perform(context),
                    pose_overrides.get(f'yaw_pose_{robot_idx}', LaunchConfiguration('', default='')).perform(context),
                )

                print(f"[amcl_multibot] {robot_name}: overrides={overrides}, randomize={randomize_pose}")
                
                initial_pose = None
                if overrides[0] != '' and overrides[1] != '' and overrides[2] != '':
                    print(f"[amcl_multibot] {robot_name}: Using OVERRIDE pose x={overrides[0]}, y={overrides[1]}")
                    initial_pose = {
                        'x': float(overrides[0]),
                        'y': float(overrides[1]),
                        'z': 0.0,
                        'yaw': float(overrides[2]),
                    }
                else:
                    initial_pose = _load_robot_initial_pose(robot_name, pkg_localization_eval)
                    if randomize_pose:
                        # Set initial pose FARTHER from true position to demonstrate convergence
                        wrong_x = rng.uniform(-2.5, 2.5)  # Up to 2.5m error
                        wrong_y = rng.uniform(-2.5, 2.5)
                        wrong_yaw = rng.uniform(-3.14, 3.14)  # Full rotation uncertainty
                        initial_pose = {
                            'x': wrong_x,
                            'y': wrong_y,
                            'z': 0.0,
                            'yaw': wrong_yaw,
                        }
                        print(f"[amcl_multibot] {robot_name}: FAR WRONG pose x={wrong_x:.2f}, y={wrong_y:.2f}, yaw={wrong_yaw:.2f}")
                    elif not initial_pose:
                        initial_pose = base_params.get('initial_pose', {})
                
                # When randomizing, use LARGE covariance for wide particle spread
                if randomize_pose:
                    # Large covariance so particles explore widely before converging
                    amcl_params['set_initial_pose'] = True
                    amcl_params['always_reset_initial_pose'] = False
                    # LARGE covariance - particles spread over ~10m diameter initially
                    amcl_params['initial_cov_xx'] = 9.0     # 3m std dev
                    amcl_params['initial_cov_yy'] = 9.0     # 3m std dev
                    amcl_params['initial_cov_aa'] = 3.14    # ~180 deg std dev
                    # Adaptive particle count with minimum 500, maximum 3000
                    amcl_params['max_particles'] = 3000     # Maximum particles for wide search
                    amcl_params['min_particles'] = 500      # Minimum particles to maintain
                    # Force updates on every scan
                    amcl_params['update_min_d'] = 0.0
                    amcl_params['update_min_a'] = 0.0
                    amcl_params['resample_interval'] = 1    # Resample every update for faster convergence
                    # KLD sampling - set pf_err very high to keep particle count fixed
                    amcl_params['pf_err'] = 0.99            # Very loose - effectively disables KLD collapse
                    amcl_params['pf_z'] = 0.99
                    # Aggressive sensor model for fast convergence with limited particles
                    amcl_params['z_hit'] = 0.98             # Very high weight on accurate measurements
                    amcl_params['z_rand'] = 0.02            # Low random noise
                    amcl_params['sigma_hit'] = 0.05         # Tighter sensor noise model
                    amcl_params['laser_likelihood_max_dist'] = 3.0  # Wider search range
                    # Aggressive recovery to continuously add random particles
                    amcl_params['recovery_alpha_slow'] = 0.005      # Add random particles more aggressively
                    amcl_params['recovery_alpha_fast'] = 0.15       # Fast recovery from wrong estimates
                    # TF timing tolerance for simulation
                    amcl_params['transform_tolerance'] = 2.0        # Lenient timing for sim time synchronization
                    print(f"[amcl_multibot] {robot_name}: Adaptive particles min=500, max=3000 - wide search with KLD sampling")
                    
                # Set initial_pose in params
                    amcl_params['initial_pose'] = initial_pose
                
                params_source = amcl_params
            else:
                candidate = os.path.join(
                    pkg_localization_eval,
                    'param',
                    f'nav2_params_{robot_name}.yaml'
                )
                params_source = candidate if os.path.exists(candidate) else shared_params_file

            amcl_node = Node(
                package='nav2_amcl',
                executable='amcl',
                name=f'{robot_name}_amcl',
                output='screen',
                parameters=[params_source, {'use_sim_time': use_sim_time}],
                remappings=[
                    ('/tf', '/tf'),
                    ('/tf_static', '/tf_static'),
                    ('map', '/map'),
                    ('scan', f'/{robot_name}/scan'),
                    ('amcl_pose', f'/{robot_name}/amcl_pose'),
                    ('particle_cloud', f'/{robot_name}/particle_cloud'),
                    ('initialpose', f'/{robot_name}/initialpose')
                ]
            )

            actions.append(amcl_node)
            lifecycle_nodes.append(f'{robot_name}_amcl')

        lifecycle_manager_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes,
                'bond_timeout': 10.0,
            }]
        )
        actions.append(lifecycle_manager_node)
        return actions

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
        default_value='4',
        description='Number of robots for AMCL (>=1)'
    ))
    ld.add_action(DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_localization_eval, 'maps', 'turtle_world.yaml'),
        description='Full path to map yaml file'
    ))
    ld.add_action(DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    ))
    ld.add_action(DeclareLaunchArgument(
        'use_shared_params',
        default_value='true',
        description='Use a single template nav2_params file for every robot'
    ))
    ld.add_action(DeclareLaunchArgument(
        'shared_params_file',
        default_value=os.path.join(pkg_localization_eval, 'param', 'nav2_params_exp.yaml'),
        description='Template params file to reuse for all robots'
    ))
    ld.add_action(DeclareLaunchArgument(
        'randomize_initial_pose',
        default_value='true',
        description='Assign random AMCL initial pose per robot (within spawn_radius)'
    ))
    ld.add_action(DeclareLaunchArgument(
        'spawn_radius',
        default_value='2.0',
        description='Half-width of the square used to randomize initial pose (meters)'
    ))
    ld.add_action(DeclareLaunchArgument(
        'random_seed',
        default_value='auto',
        description='Seed for initial pose randomization'
    ))
    for idx in range(1, 21):
        for axis in ['x', 'y', 'yaw']:
            name = f'{axis}_pose_{idx}'
            ld.add_action(DeclareLaunchArgument(
                name,
                default_value='',
                description=f'Override {axis} pose for robot {idx}'
            ))

    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
