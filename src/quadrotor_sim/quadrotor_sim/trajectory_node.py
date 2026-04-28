"""
Trajectory runner node — publishes waypoints to /uav/cmd_pose from a YAML file.

The simulator's built-in reference smoother (ref_max_vel) handles the smooth
transition between waypoints, so this node simply sends the next position target
after the hold_time for the current waypoint has elapsed.

Usage (after colcon build):
    ros2 run quadrotor_sim trajectory_runner \\
        --ros-args -p trajectory_file:=/path/to/trajectory.yaml

Or launch alongside the simulator:
    ros2 launch quadrotor_sim sim.launch.py

Then in a second terminal:
    ros2 run quadrotor_sim trajectory_runner \\
        --ros-args -p trajectory_file:=$(ros2 pkg prefix quadrotor_sim)/share/quadrotor_sim/config/trajectory.yaml

YAML format:
    trajectory:
      - position: [x, y, z]   # target in metres
        hold_time: 5.0         # seconds to hold before advancing
"""

import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class TrajectoryRunnerNode(Node):

    def __init__(self):
        super().__init__('trajectory_runner')

        self.declare_parameter('trajectory_file', '')
        self.declare_parameter('loop', False)      # repeat trajectory when done

        traj_file = self.get_parameter('trajectory_file').value
        if not traj_file:
            self.get_logger().error('trajectory_file parameter is required')
            raise RuntimeError('trajectory_file not set')

        with open(traj_file, 'r') as f:
            doc = yaml.safe_load(f)

        raw = doc.get('trajectory', [])
        if not raw:
            self.get_logger().error('No trajectory waypoints found in file')
            raise RuntimeError('Empty trajectory')

        self.waypoints = []
        for wp in raw:
            pos = wp['position']
            hold = float(wp.get('hold_time', 5.0))
            self.waypoints.append((float(pos[0]), float(pos[1]), float(pos[2]), hold))

        self.loop = self.get_parameter('loop').value
        self.wp_idx = 0
        self.wp_start_time = None

        self.pub = self.create_publisher(PoseStamped, '/uav/cmd_pose', 10)
        self.timer = self.create_timer(0.1, self._tick)  # check every 100 ms

        self.get_logger().info(
            f'Loaded {len(self.waypoints)} waypoints from {traj_file}. '
            f'Loop={self.loop}.'
        )
        self._send_current()

    def _send_current(self):
        x, y, z, _ = self.waypoints[self.wp_idx]
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)
        self.wp_start_time = self.get_clock().now()
        self.get_logger().info(
            f'[wp {self.wp_idx+1}/{len(self.waypoints)}] '
            f'target=({x:.2f}, {y:.2f}, {z:.2f})  '
            f'hold={self.waypoints[self.wp_idx][3]:.1f}s'
        )

    def _tick(self):
        if self.wp_start_time is None:
            return

        elapsed = (self.get_clock().now() - self.wp_start_time).nanoseconds * 1e-9
        hold_time = self.waypoints[self.wp_idx][3]

        if elapsed >= hold_time:
            self.wp_idx += 1
            if self.wp_idx >= len(self.waypoints):
                if self.loop:
                    self.wp_idx = 0
                else:
                    self.get_logger().info('Trajectory complete.')
                    self.timer.cancel()
                    return
            self._send_current()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = TrajectoryRunnerNode()
        rclpy.spin(node)
    except (RuntimeError, KeyboardInterrupt):
        pass
    finally:
        rclpy.shutdown()
