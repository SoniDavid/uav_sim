from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


# Maps ctrl argument value → (executable, node_name).
# Add new entries here when new controllers are implemented.
_CTRL_MAP = {
    'fxt_pd':    ('fxt_pd_controller',    'hexa_fxt_pd_controller'),
    'simple_pd': ('simple_pd_controller', 'hexa_simple_pd_controller'),
}

# Trajectory types handled by the analytical trajectory node.
_ANALYTICAL_TYPES = {'circle', 'lemniscate', 'helix'}


def _dynamic_nodes(context, *args, **kwargs):
    pkg = get_package_share_directory('hexa_tilt_sim')
    nodes = []

    # ── Controller selection ───────────────────────────────────────────────────
    ctrl = LaunchConfiguration('ctrl').perform(context)
    if ctrl not in _CTRL_MAP:
        raise RuntimeError(
            f"Unknown ctrl={ctrl!r}. Valid options: {list(_CTRL_MAP.keys())}")
    executable, node_name = _CTRL_MAP[ctrl]
    nodes.append(Node(
        package='hexa_tilt_sim',
        executable=executable,
        name=node_name,
        output='screen',
    ))

    # ── Trajectory / reference selection ──────────────────────────────────────
    traj_type = LaunchConfiguration('traj_type').perform(context)

    if traj_type in _ANALYTICAL_TYPES:
        # Analytical trajectory node publishes /hexa/reference directly
        # (includes velocity feedforward; reference_node is not needed)
        nodes.append(Node(
            package='hexa_tilt_sim',
            executable='analytical_traj_node',
            name='hexa_analytical_traj',
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
        # Static setpoint or YAML-waypoint mode — use reference_node
        nodes.append(Node(
            package='hexa_tilt_sim',
            executable='reference_node',
            name='hexa_reference',
            output='screen',
            parameters=[{
                'ref_x':       LaunchConfiguration('ref_x'),
                'ref_y':       LaunchConfiguration('ref_y'),
                'ref_z':       LaunchConfiguration('ref_z'),
                'ref_psi':     LaunchConfiguration('ref_psi'),
                'ref_max_vel': LaunchConfiguration('ref_max_vel'),
            }],
        ))
        if traj_type == 'waypoints':
            traj_file = LaunchConfiguration('trajectory_file').perform(context)
            if not traj_file:
                traj_file = os.path.join(pkg, 'config', 'trajectory.yaml')
            nodes.append(Node(
                package='hexa_tilt_sim',
                executable='trajectory_runner',
                name='hexa_trajectory_runner',
                output='screen',
                parameters=[{
                    'trajectory_file': traj_file,
                    'loop': LaunchConfiguration('loop'),
                }],
            ))

    return nodes


def generate_launch_description():
    pkg = get_package_share_directory('hexa_tilt_sim')
    default_traj = os.path.join(pkg, 'config', 'trajectory.yaml')

    # ── Launch arguments ───────────────────────────────────────────────────────
    arg_ctrl      = DeclareLaunchArgument(
                        'ctrl', default_value='fxt_pd',
                        description=f"Controller: {list(_CTRL_MAP.keys())}")
    arg_traj_type = DeclareLaunchArgument(
                        'traj_type', default_value='none',
                        description=(
                            "Trajectory source: 'none' (static setpoint) | "
                            "'waypoints' (YAML file) | "
                            "'circle' | 'lemniscate' | 'helix'"))

    # Static setpoint / waypoint parameters
    arg_ref_x       = DeclareLaunchArgument('ref_x',       default_value='0.0',  description='Target x (m) / orbit centre x')
    arg_ref_y       = DeclareLaunchArgument('ref_y',       default_value='0.0',  description='Target y (m) / orbit centre y')
    arg_ref_z       = DeclareLaunchArgument('ref_z',       default_value='1.2',  description='Target z (m) / orbit height')
    arg_ref_psi     = DeclareLaunchArgument('ref_psi',     default_value='0.0',  description='Target yaw (rad)')
    arg_ref_max_vel = DeclareLaunchArgument('ref_max_vel', default_value='0.5',  description='Max reference velocity (m/s)')
    arg_traj_file   = DeclareLaunchArgument('trajectory_file', default_value=default_traj,
                                            description='Path to YAML trajectory file (traj_type=waypoints)')
    arg_loop        = DeclareLaunchArgument('loop',        default_value='false',
                                            description='Loop waypoint trajectory when complete')

    # Analytical trajectory parameters
    arg_radius      = DeclareLaunchArgument('radius',      default_value='1.0',  description='Orbit radius (m)')
    arg_speed       = DeclareLaunchArgument('speed',       default_value='0.3',  description='Angular velocity (rad/s)')
    arg_climb_rate  = DeclareLaunchArgument('climb_rate',  default_value='0.1',  description='Vertical climb rate for helix (m/s)')

    # ── Static nodes (always launched) ────────────────────────────────────────
    urdf_path = os.path.join(pkg, 'urdf', 'hexa_tilt.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    node_robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='hexa_robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    node_dynamics = Node(
        package='hexa_tilt_sim',
        executable='dynamics_node',
        name='hexa_dynamics',
        output='screen',
    )

    node_kinematics = Node(
        package='hexa_tilt_sim',
        executable='kinematics_node',
        name='hexa_kinematics',
        output='screen',
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

    return LaunchDescription([
        arg_ctrl,
        arg_traj_type,
        arg_ref_x,
        arg_ref_y,
        arg_ref_z,
        arg_ref_psi,
        arg_ref_max_vel,
        arg_traj_file,
        arg_loop,
        arg_radius,
        arg_speed,
        arg_climb_rate,
        node_robot_state_pub,
        node_dynamics,
        node_kinematics,
        OpaqueFunction(function=_dynamic_nodes),
        node_foxglove,
    ])
