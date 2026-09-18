from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


# Trajectory types handled by the analytical trajectory node.
_ANALYTICAL_TYPES = {'circle', 'lemniscate', 'helix'}


def _dynamic_nodes(context, *args, **kwargs):
    """Return controller, reference/trajectory nodes, and optional URDF publisher."""
    pkg = get_package_share_directory('quadrotor_sim')
    nodes = []

    # ── Trajectory / reference selection ──────────────────────────────────────
    traj_type = LaunchConfiguration('traj_type').perform(context)

    if traj_type in _ANALYTICAL_TYPES:
        # Analytical trajectory node publishes /uav/reference directly
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
        # Static setpoint or YAML-waypoint mode — use reference_node
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

    # ── Optional URDF / robot_state_publisher ─────────────────────────────────
    urdf_file = LaunchConfiguration('urdf_file').perform(context)
    if urdf_file:
        with open(urdf_file, 'r') as f:
            robot_description = f.read()
        nodes.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ))

    return nodes


def generate_launch_description():
    pkg = get_package_share_directory('quadrotor_sim')
    default_traj = os.path.join(pkg, 'config', 'trajectory.yaml')

    # ── Launch arguments ───────────────────────────────────────────────────────
    arg_traj_type = DeclareLaunchArgument(
                        'traj_type', default_value='none',
                        description=(
                            "Trajectory source: 'none' (static setpoint) | "
                            "'waypoints' (YAML file) | "
                            "'circle' | 'lemniscate' | 'helix'"))

    # Static setpoint / waypoint parameters
    arg_ref_x        = DeclareLaunchArgument('ref_x',       default_value='0.0',  description='Target x (m) / orbit centre x')
    arg_ref_y        = DeclareLaunchArgument('ref_y',       default_value='0.0',  description='Target y (m) / orbit centre y')
    arg_ref_z        = DeclareLaunchArgument('ref_z',       default_value='1.0',  description='Target z (m) / orbit height')
    arg_ref_max_vel  = DeclareLaunchArgument('ref_max_vel', default_value='0.5',  description='Max reference velocity (m/s)')
    arg_controller   = DeclareLaunchArgument('controller',  default_value='sf_controller',
                                             description="'sf_controller' | 'sf_mimo_controller' | 'nonlinear_pd_controller' | 'lqr_controller'")
    arg_traj_file    = DeclareLaunchArgument('trajectory_file', default_value=default_traj,
                                             description='Path to YAML trajectory file (traj_type=waypoints)')
    arg_loop         = DeclareLaunchArgument('loop',        default_value='false',
                                             description='Loop waypoint trajectory when complete')
    arg_urdf         = DeclareLaunchArgument('urdf_file',   default_value='',
                                             description='Absolute path to drone URDF (enables robot_state_publisher)')

    # Analytical trajectory parameters
    arg_radius       = DeclareLaunchArgument('radius',      default_value='1.0',  description='Orbit radius (m)')
    arg_speed        = DeclareLaunchArgument('speed',       default_value='0.3',  description='Angular velocity (rad/s)')
    arg_climb_rate   = DeclareLaunchArgument('climb_rate',  default_value='0.1',  description='Vertical climb rate for helix (m/s)')

    # ── Static nodes (always launched) ────────────────────────────────────────
    node_controller = Node(
        package='quadrotor_sim',
        executable=LaunchConfiguration('controller'),
        name='quadrotor_controller',
        output='screen',
    )

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
        arg_traj_type,
        arg_ref_x,
        arg_ref_y,
        arg_ref_z,
        arg_ref_max_vel,
        arg_controller,
        arg_traj_file,
        arg_loop,
        arg_urdf,
        arg_radius,
        arg_speed,
        arg_climb_rate,
        node_controller,
        node_dynamics,
        node_kinematics,
        OpaqueFunction(function=_dynamic_nodes),
        node_foxglove,
    ])
