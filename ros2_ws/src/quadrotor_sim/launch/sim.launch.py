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

    # ── Launch arguments ───────────────────────────────────────────────────────
    arg_ref_x        = DeclareLaunchArgument('ref_x',       default_value='0.0',  description='Initial target x (m)')
    arg_ref_y        = DeclareLaunchArgument('ref_y',       default_value='0.0',  description='Initial target y (m)')
    arg_ref_z        = DeclareLaunchArgument('ref_z',       default_value='1.0',  description='Initial target z (m)')
    arg_ref_max_vel  = DeclareLaunchArgument('ref_max_vel', default_value='0.5',  description='Max reference velocity (m/s)')
    arg_controller   = DeclareLaunchArgument('controller',  default_value='sf_controller',
                                             description="'sf_controller' | 'nonlinear_pd_controller' | 'lqr_controller'")
    arg_use_traj     = DeclareLaunchArgument('use_trajectory',  default_value='false',
                                             description='Launch trajectory runner alongside simulator')
    arg_traj_file    = DeclareLaunchArgument('trajectory_file', default_value=default_traj,
                                             description='Path to trajectory YAML file')
    arg_loop         = DeclareLaunchArgument('loop',        default_value='false',
                                             description='Loop trajectory when complete')
    arg_urdf         = DeclareLaunchArgument('urdf_file',   default_value='',
                                             description='Absolute path to drone URDF (enables robot_state_publisher)')

    # ── Nodes ──────────────────────────────────────────────────────────────────
    node_reference = Node(
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
    )

    node_controller = Node(
        package='quadrotor_sim',
        executable=LaunchConfiguration('controller'),
        name='quadrotor_controller',
        output='screen',
    )

    node_dynamics = Node(           # simulation only — remove for real drone
        package='quadrotor_sim',
        executable='dynamics_node',
        name='quadrotor_dynamics',
        output='screen',
    )

    node_kinematics = Node(         # sim + real drone
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

    # ── Active launch items ────────────────────────────────────────────────────
    return LaunchDescription([
        arg_ref_x,
        arg_ref_y,
        arg_ref_z,
        arg_ref_max_vel,
        arg_controller,
        arg_use_traj,
        arg_traj_file,
        arg_loop,
        arg_urdf,
        node_reference,
        node_controller,
        node_dynamics,
        node_kinematics,
        OpaqueFunction(function=_optional_nodes),
        node_foxglove,
    ])
