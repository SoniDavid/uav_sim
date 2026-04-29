#!/usr/bin/env python3
"""
Quadrotor reference node.

Manages the position reference: accepts target pose commands, applies a
velocity-limited ramp (ref_max_vel), and publishes the smoothed 12-state
reference for the controller at 50 Hz.

Subscribed topics:
  /uav/cmd_pose        geometry_msgs/PoseStamped  — new target position

Published topics:
  /uav/reference       std_msgs/Float64MultiArray  — smoothed 12-state reference
  /uav/reference/pose  geometry_msgs/PoseStamped   — same, as pose for Foxglove

Parameters:
  ref_x        float  initial target x (m)  [default 0.0]
  ref_y        float  initial target y (m)  [default 0.0]
  ref_z        float  initial target z (m)  [default 1.0]
  ref_max_vel  float  max reference speed (m/s) [default 0.5]

Send a new target at any time:
  ros2 topic pub --once /uav/cmd_pose geometry_msgs/msg/PoseStamped \\
    '{header: {frame_id: world}, pose: {position: {x: 2.0, y: 1.0, z: 1.5}}}'
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray

from quadrotor_sim_scripts.params import DT_PUB, DT_SIM, SIM_STEPS_PER_PUB


class QuadrotorReferenceNode(Node):

    def __init__(self):
        super().__init__('quadrotor_reference')

        self.declare_parameter('ref_x',       0.0)
        self.declare_parameter('ref_y',       0.0)
        self.declare_parameter('ref_z',       1.0)
        self.declare_parameter('ref_max_vel', 0.5)

        p = self.get_parameter
        self._active_target = np.zeros(12)
        self._active_target[0] = p('ref_x').value
        self._active_target[1] = p('ref_y').value
        self._active_target[2] = p('ref_z').value
        self._ref_max_vel = p('ref_max_vel').value

        # Smoothed reference state — start at target so first publish is already correct
        self._x_ref = self._active_target.copy()

        # Smoothing step: equivalent to SIM_STEPS_PER_PUB × DT_SIM = DT_PUB
        self._step = DT_SIM * SIM_STEPS_PER_PUB  # == DT_PUB

        self.pub_ref     = self.create_publisher(Float64MultiArray, '/uav/reference',      10)
        self.pub_ref_viz = self.create_publisher(PoseStamped,       '/uav/reference/pose', 10)

        self.sub_cmd = self.create_subscription(
            PoseStamped, '/uav/cmd_pose', self._cmd_pose_callback, 10)

        self.timer = self.create_timer(DT_PUB, self._tick)

        self.get_logger().info(
            f'Reference node started. '
            f'Initial target: {self._active_target[:3]}, '
            f'max_vel={self._ref_max_vel} m/s. '
            f'Publish /uav/cmd_pose to change target.'
        )

    def _cmd_pose_callback(self, msg: PoseStamped):
        self._active_target[0] = msg.pose.position.x
        self._active_target[1] = msg.pose.position.y
        self._active_target[2] = msg.pose.position.z
        self.get_logger().info(
            f'New target: x={self._active_target[0]:.2f}, '
            f'y={self._active_target[1]:.2f}, '
            f'z={self._active_target[2]:.2f}'
        )

    def _tick(self):
        delta = self._active_target - self._x_ref
        v = self._ref_max_vel
        dt = self._step

        # Rate-limit position (x, y, z) and yaw (index 8)
        for i in [0, 1, 2]:
            self._x_ref[i] += np.clip(delta[i], -v * dt, v * dt)
        self._x_ref[8] += np.clip(delta[8], -v * dt, v * dt)

        ref_msg = Float64MultiArray()
        ref_msg.data = [float(x) for x in self._x_ref]
        self.pub_ref.publish(ref_msg)

        now = self.get_clock().now().to_msg()
        viz_msg = PoseStamped()
        viz_msg.header.stamp = now
        viz_msg.header.frame_id = 'world'
        viz_msg.pose.position.x = float(self._x_ref[0])
        viz_msg.pose.position.y = float(self._x_ref[1])
        viz_msg.pose.position.z = float(self._x_ref[2])
        viz_msg.pose.orientation.w = 1.0
        self.pub_ref_viz.publish(viz_msg)


def main(args=None):
    rclpy.init(args=args)
    node = QuadrotorReferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
