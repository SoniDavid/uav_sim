"""
kalman_filter.launch.py
=======================
Launches the full quadrotor simulation with sensor noise simulation and
Kalman filter state estimation.

Node pipeline:
  dynamics_node  →  sensor_noise_node  →  kalman_filter_node
       ↓                                         ↑
  kinematics_node                       /uav/noisy_state
       ↓
  controller  ←  reference/trajectory

Tunable parameters (all overridable from the command line):
  -- Sensor noise (sensor_noise_node) --
  sigma_pos     std dev of position sensor noise [m]       default 0.05
  sigma_ang     std dev of IMU angle noise [rad]            default 0.01
  noise_seed    RNG seed (0 = random)                       default 42

  -- Kalman filter (kalman_filter_node) --
  q_pos         process noise — position/angle states       default 1e-4
  q_vel         process noise — velocity/rate states        default 1e-2
  r_pos         measurement noise — position [m²]           default 0.01
  r_ang         measurement noise — angles [rad²]           default 0.005

  -- Simulation / controller --
  controller    'sf_controller' | 'nonlinear_pd_controller' | 'lqr_controller'
  traj_type     'none' | 'waypoints' | 'circle' | 'lemniscate' | 'helix'
  ref_x/y/z     static setpoint or trajectory centre [m]
  radius        orbit radius [m]  (circle / lemniscate / helix)
  speed         angular velocity [rad/s]
  climb_rate    vertical rate for helix [m/s]
  ref_max_vel   max ramp velocity for static setpoint [m/s]

Example:
  ros2 launch quadrotor_sim kalman_filter.launch.py \\
      traj_type:=circle radius:=0.5 speed:=0.4 \\
      sigma_pos:=0.08 sigma_ang:=0.02 \\
      q_pos:=1e-3 r_pos:=0.02
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_ANALYTICAL_TYPES = {'circle', 'lemniscate', 'helix'}


def _dynamic_nodes(context, *args, **kwargs):
    pkg = get_package_share_directory('quadrotor_sim')
    nodes = []

    traj_type = LaunchConfiguration('traj_type').perform(context)

    if traj_type in _ANALYTICAL_TYPES:
        nodes.append(Node(
            package='quadrotor_sim',
            executable='analytical_traj_node',
            name='uav_analytical_traj',
            output='screen',
            parameters=[{
                'traj_type':  traj_type,
                'radius':     LaunchConfiguration('radius'),
                'height':     LaunchConfiguration('ref_z'),
                'speed':      LaunchConfiguration('speed'),
                'center_x':   LaunchConfiguration('ref_x'),
                'center_y':   LaunchConfiguration('ref_y'),
                'climb_rate': LaunchConfiguration('climb_rate'),
            }],
        ))
    else:
        nodes.append(Node(
            package='quadrotor_sim',
            executable='reference_node',
            name='quadrotor_reference',
            output='screen',
            parameters=[{
                'ref_x':       LaunchConfiguration('ref_x'),
                'ref_y':       LaunchConfiguration('ref_y'),
                'ref_z':       LaunchConfiguration('ref_z'),
                'ref_max_vel': LaunchConfiguration('ref_max_vel'),
            }],
        ))
        if traj_type == 'waypoints':
            traj_file = LaunchConfiguration('trajectory_file').perform(context)
            if not traj_file:
                traj_file = os.path.join(pkg, 'config', 'trajectory.yaml')
            nodes.append(Node(
                package='quadrotor_sim',
                executable='trajectory_runner',
                name='trajectory_runner',
                output='screen',
                parameters=[{
                    'trajectory_file': traj_file,
                    'loop': LaunchConfiguration('loop'),
                }],
            ))

    return nodes


def _kalman_nodes(context, *args, **kwargs):
    tuning = LaunchConfiguration('kf_tuning').perform(context)
    
    # Default values (Manual override if provided)
    q_pos = LaunchConfiguration('q_pos').perform(context)
    q_vel = LaunchConfiguration('q_vel').perform(context)
    r_pos = LaunchConfiguration('r_pos').perform(context)
    r_ang = LaunchConfiguration('r_ang').perform(context)

    # Tuning Profiles
    if tuning == 'sharp':
        # Optimized for MIMO: Low lag, allows some noise
        q_pos, q_vel = '1e-4', '1e-5'
        r_pos, r_ang = '0.0025', '0.0001'
    elif tuning == 'smooth':
        # Optimized for Decoupled SF: Heavy smoothing, accepts some lag
        q_pos, q_vel = '1e-7', '1e-6'
        r_pos, r_ang = '0.05', '0.01'
    
    return [
        Node(
            package='quadrotor_sim',
            executable='kalman_filter_node',
            name='kalman_filter',
            output='screen',
            parameters=[{
                'q_pos': float(q_pos),
                'q_vel': float(q_vel),
                'r_pos': float(r_pos),
                'r_ang': float(r_ang),
            }],
        )
    ]


def generate_launch_description():
    pkg = get_package_share_directory('quadrotor_sim')
    default_traj = os.path.join(pkg, 'config', 'trajectory.yaml')

    # ── Simulation args ────────────────────────────────────────────────────────
    args_sim = [
        DeclareLaunchArgument('controller',      default_value='sf_controller',
                              description="'sf_controller' | 'sf_mimo_controller' | 'nonlinear_pd_controller' | 'lqr_controller'"),
        DeclareLaunchArgument('traj_type',       default_value='none',
                              description="'none' | 'waypoints' | 'circle' | 'lemniscate' | 'helix'"),
        DeclareLaunchArgument('ref_x',           default_value='0.0',  description='Setpoint / centre x (m)'),
        DeclareLaunchArgument('ref_y',           default_value='0.0',  description='Setpoint / centre y (m)'),
        DeclareLaunchArgument('ref_z',           default_value='1.0',  description='Setpoint / orbit height (m)'),
        DeclareLaunchArgument('ref_max_vel',     default_value='0.5',  description='Max ramp velocity (m/s)'),
        DeclareLaunchArgument('radius',          default_value='1.0',  description='Orbit radius (m)'),
        DeclareLaunchArgument('speed',           default_value='0.3',  description='Angular velocity (rad/s)'),
        DeclareLaunchArgument('climb_rate',      default_value='0.1',  description='Helix climb rate (m/s)'),
        DeclareLaunchArgument('trajectory_file', default_value=default_traj,
                              description='Path to YAML trajectory file'),
        DeclareLaunchArgument('loop',            default_value='false', description='Loop waypoint trajectory'),
    ]

    # ── Sensor noise args ──────────────────────────────────────────────────────
    args_noise = [
        DeclareLaunchArgument('sigma_pos',  default_value='0.05',
                              description='Position sensor noise std dev (m)'),
        DeclareLaunchArgument('sigma_ang',  default_value='0.01',
                              description='IMU angle noise std dev (rad)'),
        DeclareLaunchArgument('noise_seed', default_value='42',
                              description='RNG seed for noise (0 = random)'),
    ]

    # ── Kalman filter args ─────────────────────────────────────────────────────
    args_kf = [
        DeclareLaunchArgument('kf_tuning', default_value='none',
                              description="'sharp' (MIMO) | 'smooth' (Decoupled) | 'none' (manual)"),
        DeclareLaunchArgument('q_pos', default_value='0.00001',
                              description='Manual KF process noise — position/angle states'),
        DeclareLaunchArgument('q_vel', default_value='0.000001',
                              description='Manual KF process noise — velocity/rate states'),
        DeclareLaunchArgument('r_pos', default_value='0.05',
                              description='Manual KF measurement noise — position (m^2)'),
        DeclareLaunchArgument('r_ang', default_value='0.5',
                              description='Manual KF measurement noise — angles (rad^2)'),
    ]

    # ── Nodes ──────────────────────────────────────────────────────────────────
    node_dynamics = Node(
        package='quadrotor_sim',
        executable='dynamics_node',
        name='quadrotor_dynamics',
        output='screen',
    )

    node_kinematics = Node(
        package='quadrotor_sim',
        executable='kinematics_node',
        name='quadrotor_kinematics',
        output='screen',
    )

    node_controller = Node(
        package='quadrotor_sim',
        executable=LaunchConfiguration('controller'),
        name='quadrotor_controller',
        output='screen',
        remappings=[
            ('/uav/state', '/uav/kf_state')
        ],
    )

    node_sensor_noise = Node(
        package='quadrotor_sim',
        executable='sensor_noise_node',
        name='sensor_noise',
        output='screen',
        parameters=[{
            'sigma_pos': LaunchConfiguration('sigma_pos'),
            'sigma_ang': LaunchConfiguration('sigma_ang'),
            'seed':      LaunchConfiguration('noise_seed'),
        }],
    )

    node_foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'send_buffer_limit': 10000000,
        }],
    )

    return LaunchDescription(
        args_sim + args_noise + args_kf + [
            node_dynamics,
            node_kinematics,
            node_controller,
            node_sensor_noise,
            # node_foxglove,
            OpaqueFunction(function=_dynamic_nodes),
            OpaqueFunction(function=_kalman_nodes),
        ]
    )
