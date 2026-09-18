"""
Gazebo (Harmonic / gz-sim8) simulation of the hexa_s550, driven by the existing
FxT-PD controller through two thin adapters.

Pipeline:
  reference/traj (Py) -> fxt_pd_controller -> /hexa/control (wrench)
      -> wrench_to_actuators -> /hexa/command/motor_speed (Actuators)
      -> [ros_gz_bridge] -> Gazebo plant (6x MulticopterMotorModel)
      -> OdometryPublisher -> /model/hexa/odometry -> [ros_gz_bridge]
      -> odom_to_state -> /hexa/state (18-D) -> controller (closes loop)

Requires: ros-humble-ros-gzharmonic, ros-humble-actuator-msgs.

Example:
  ros2 launch hexa_tilt_sim gz_sim.launch.py                       # hover at z=1.2
  ros2 launch hexa_tilt_sim gz_sim.launch.py traj_type:=circle radius:=1.0 speed:=0.3
  ros2 launch hexa_tilt_sim gz_sim.launch.py headless:=true
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


_CTRL_MAP = {
    'fxt_pd':    ('fxt_pd_controller',    'hexa_fxt_pd_controller'),
    'simple_pd': ('simple_pd_controller', 'hexa_simple_pd_controller'),
}
_ANALYTICAL_TYPES = {'circle', 'lemniscate', 'helix'}


def _control_and_reference(context, *args, **kwargs):
    """Controller + reference/trajectory selection (mirrors sim.launch.py)."""
    nodes = []
    ctrl = LaunchConfiguration('ctrl').perform(context)
    if ctrl not in _CTRL_MAP:
        raise RuntimeError(f"Unknown ctrl={ctrl!r}. Valid: {list(_CTRL_MAP.keys())}")
    executable, node_name = _CTRL_MAP[ctrl]
    nodes.append(Node(package='hexa_tilt_sim', executable=executable, name=node_name, output='screen'))

    traj_type = LaunchConfiguration('traj_type').perform(context)
    if traj_type in _ANALYTICAL_TYPES:
        nodes.append(Node(
            package='hexa_tilt_sim', executable='analytical_traj_node', name='hexa_analytical_traj',
            output='screen',
            parameters=[{
                'traj_type': traj_type,
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
            package='hexa_tilt_sim', executable='reference_node', name='hexa_reference', output='screen',
            parameters=[{
                'ref_x': LaunchConfiguration('ref_x'), 'ref_y': LaunchConfiguration('ref_y'),
                'ref_z': LaunchConfiguration('ref_z'), 'ref_psi': LaunchConfiguration('ref_psi'),
                'ref_max_vel': LaunchConfiguration('ref_max_vel'),
            }],
        ))
    return nodes


def generate_launch_description():
    pkg = get_package_share_directory('hexa_tilt_sim')
    world = os.path.join(pkg, 'worlds', 'hexa.sdf')
    model_sdf = os.path.join(pkg, 'sdf', 'fah_hexa', 'model.sdf')
    bridge_cfg = os.path.join(pkg, 'config', 'bridge_hexa.yaml')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    args = [
        DeclareLaunchArgument('ctrl', default_value='fxt_pd'),
        DeclareLaunchArgument('traj_type', default_value='none',
                              description="none | circle | lemniscate | helix"),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('linear_twist_world', default_value='false',
                              description="set true if Gz reports odom linear vel in world frame"),
        DeclareLaunchArgument('spawn_z', default_value='0.20'),
        # reference / trajectory params
        DeclareLaunchArgument('ref_x', default_value='0.0'),
        DeclareLaunchArgument('ref_y', default_value='0.0'),
        DeclareLaunchArgument('ref_z', default_value='1.2'),
        DeclareLaunchArgument('ref_psi', default_value='0.0'),
        DeclareLaunchArgument('ref_max_vel', default_value='0.5'),
        DeclareLaunchArgument('radius', default_value='1.0'),
        DeclareLaunchArgument('speed', default_value='0.3'),
        DeclareLaunchArgument('climb_rate', default_value='0.1'),
    ]

    # Let Gz resolve model:// URIs from our sdf/ dir (meshes/sensors).
    set_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg, 'sdf'))

    def _gz(context, *a, **k):
        headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
        flags = '-r -v4 -s --headless-rendering ' if headless else '-r -v4 '
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': flags + world, 'gz_version': '8'}.items(),
        )]
    gz_sim = OpaqueFunction(function=_gz)

    spawn = Node(
        package='ros_gz_sim', executable='create', name='spawn_hexa', output='screen',
        arguments=['-world', 'hexa_world', '-name', 'hexa', '-file', model_sdf,
                   '-z', LaunchConfiguration('spawn_z')],
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='hexa_bridge', output='screen',
        parameters=[{'config_file': bridge_cfg}],
    )

    wrench_adapter = Node(
        package='hexa_tilt_sim', executable='wrench_to_actuators', name='wrench_to_actuators', output='screen')
    odom_adapter = Node(
        package='hexa_tilt_sim', executable='odom_to_state', name='odom_to_state', output='screen',
        parameters=[{'linear_twist_world': LaunchConfiguration('linear_twist_world')}])

    foxglove = Node(
        package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge',
        parameters=[{'port': 8765, 'address': '0.0.0.0', 'send_buffer_limit': 10000000}],
    )

    return LaunchDescription(args + [
        set_resource_path,
        gz_sim,
        spawn,
        bridge,
        wrench_adapter,
        odom_adapter,
        foxglove,
        OpaqueFunction(function=_control_and_reference),
    ])
