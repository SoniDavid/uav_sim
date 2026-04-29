from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def _optional_nodes(context, *args, **kwargs):
    """Return trajectory runner and/or robot_state_publisher based on launch args."""
    pkg = get_package_share_directory('quadrotor_sim')
    nodes = []

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

    if LaunchConfiguration('use_trajectory').perform(context).lower() == 'true':
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


def generate_launch_description():
    pkg = get_package_share_directory('quadrotor_sim')
    default_traj = os.path.join(pkg, 'config', 'trajectory.yaml')

    return LaunchDescription([
        # ── Reference / trajectory parameters ────────────────────────
        DeclareLaunchArgument('ref_x',       default_value='0.0',  description='Initial target x (m)'),
        DeclareLaunchArgument('ref_y',       default_value='0.0',  description='Initial target y (m)'),
        DeclareLaunchArgument('ref_z',       default_value='1.0',  description='Initial target z (m)'),
        DeclareLaunchArgument('ref_max_vel', default_value='0.5',  description='Max reference velocity (m/s)'),

        # ── Controller parameter ──────────────────────────────────────
        DeclareLaunchArgument('controller', default_value='cascaded',
                             description="'cascaded'|'lqr'|'constrained_lqr'|'constrained_lqr_qp'|'gain_scheduled_lqr'"),

        # ── Optional trajectory runner ────────────────────────────────
        DeclareLaunchArgument('use_trajectory',  default_value='false',
                             description='Launch trajectory runner alongside simulator'),
        DeclareLaunchArgument('trajectory_file', default_value=default_traj,
                             description='Path to trajectory YAML file'),
        DeclareLaunchArgument('loop',            default_value='false',
                             description='Loop trajectory when complete'),
        DeclareLaunchArgument('urdf_file',       default_value='',
                             description='Absolute path to drone URDF (enables robot_state_publisher)'),

        # ── Reference node ────────────────────────────────────────────
        Node(
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
        ),

        # ── Controller node ───────────────────────────────────────────
        Node(
            package='quadrotor_sim',
            executable='controller_node',
            name='quadrotor_controller',
            output='screen',
            parameters=[{
                'controller': LaunchConfiguration('controller'),
            }],
        ),

        # ── Dynamics node ─────────────────────────────────────────────
        Node(
            package='quadrotor_sim',
            executable='dynamics_node',
            name='quadrotor_dynamics',
            output='screen',
        ),

        # ── Optional: trajectory runner + robot_state_publisher ───────
        OpaqueFunction(function=_optional_nodes),

        # ── Foxglove bridge ───────────────────────────────────────────
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{
                'port': 8765,
                'address': '0.0.0.0',
                'send_buffer_limit': 10000000,
            }],
        ),
    ])
