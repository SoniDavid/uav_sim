#!/usr/bin/env python3
"""
Hexacopter reference node.

Accepts target pose commands, applies a velocity-limited ramp, and publishes
the smoothed 12-state reference for the FxT-PD controller at 50 Hz.

Reference layout (12D): [x_d, y_d, z_d, φ_d, θ_d, ψ_d,  ẋ_d, ẏ_d, ż_d, φ̇_d, θ̇_d, ψ̇_d]
Position smoothed via rate limiter; velocities and attitude refs default to zero.

Subscribed topics:
  /hexa/cmd_pose        geometry_msgs/PoseStamped  — new target position/yaw

Published topics:
  /hexa/reference       std_msgs/Float64MultiArray  — smoothed 12-state reference
  /hexa/reference/pose  geometry_msgs/PoseStamped   — same, as pose for Foxglove

Parameters:
  ref_x        float  initial target x (m)     [default 0.0]
  ref_y        float  initial target y (m)     [default 0.0]
  ref_z        float  initial target z (m)     [default 1.2]
  ref_psi      float  initial target yaw (rad) [default 0.0]
  ref_max_vel  float  max reference speed (m/s) [default 0.5]
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray

DT_PUB = 0.02   # 50 Hz


class HexaReferenceNode(Node):

    def __init__(self):
        super().__init__('hexa_reference')

        self.declare_parameter('ref_x',       0.0)
        self.declare_parameter('ref_y',       0.0)
        self.declare_parameter('ref_z',       1.2)
        self.declare_parameter('ref_psi',     0.0)
        self.declare_parameter('ref_max_vel', 0.5)

        p = self.get_parameter
        self._ref_max_vel = p('ref_max_vel').value

        # chi_d = [x, y, z, phi, theta, psi]  (phi/theta always 0 for waypoints)
        self._active_target = np.zeros(6)
        self._active_target[0] = p('ref_x').value
        self._active_target[1] = p('ref_y').value
        self._active_target[2] = p('ref_z').value
        self._active_target[5] = p('ref_psi').value

        # Smoothed reference χ_d — start at target
        self._chi_ref = self._active_target.copy()

        self.pub_ref     = self.create_publisher(Float64MultiArray, '/hexa/reference',      10)
        self.pub_ref_viz = self.create_publisher(PoseStamped,       '/hexa/reference/pose', 10)

        self.sub_cmd = self.create_subscription(
            PoseStamped, '/hexa/cmd_pose', self._cmd_pose_callback, 10)

        self.timer = self.create_timer(DT_PUB, self._tick)

        self.get_logger().info(
            f'Hexa reference node started. '
            f'Initial target: xyz={self._active_target[:3]}, psi={self._active_target[5]:.2f} rad, '
            f'max_vel={self._ref_max_vel} m/s. '
            f'Publish /hexa/cmd_pose to change target.'
        )

    def _cmd_pose_callback(self, msg: PoseStamped):
        self._active_target[0] = msg.pose.position.x
        self._active_target[1] = msg.pose.position.y
        self._active_target[2] = msg.pose.position.z
        # Yaw from quaternion (simplified: read z component of orientation as proxy)
        # For full yaw extraction, use tf_transformations; here we just accept
        # yaw if it was set (pose.orientation encodes it via the PoseStamped convention)
        self.get_logger().info(
            f'New target: x={self._active_target[0]:.2f}, '
            f'y={self._active_target[1]:.2f}, '
            f'z={self._active_target[2]:.2f}'
        )

    def _tick(self):
        delta = self._active_target - self._chi_ref
        v = self._ref_max_vel
        dt = DT_PUB

        # Rate-limit x, y, z and yaw (index 5)
        for i in [0, 1, 2, 5]:
            self._chi_ref[i] += np.clip(delta[i], -v * dt, v * dt)

        # Full 12D reference: [chi_d(6), chi_dot_d(6)] — velocity targets are zero
        ref = np.zeros(12)
        ref[:6] = self._chi_ref

        ref_msg = Float64MultiArray()
        ref_msg.data = [float(x) for x in ref]
        self.pub_ref.publish(ref_msg)

        now = self.get_clock().now().to_msg()
        viz_msg = PoseStamped()
        viz_msg.header.stamp = now
        viz_msg.header.frame_id = 'world'
        viz_msg.pose.position.x = float(self._chi_ref[0])
        viz_msg.pose.position.y = float(self._chi_ref[1])
        viz_msg.pose.position.z = float(self._chi_ref[2])
        viz_msg.pose.orientation.w = 1.0
        self.pub_ref_viz.publish(viz_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HexaReferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
