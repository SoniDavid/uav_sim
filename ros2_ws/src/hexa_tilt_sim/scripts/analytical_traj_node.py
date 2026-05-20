#!/usr/bin/env python3
"""Analytical trajectory generator for hexa_tilt_sim.

Publishes /hexa/reference (12D) directly — bypasses reference_node so that
velocity feedforward is included in the reference.

Supported trajectory types (traj_type parameter):
  circle      — flat circle in the XY plane at constant height
  lemniscate  — figure-8 (Lissajous 1:2) in the XY plane at constant height
  helix       — ascending/descending circle

Parameters
----------
traj_type   : str   — 'circle' | 'lemniscate' | 'helix'  (default: 'circle')
radius      : float — characteristic radius in metres      (default: 1.0)
height      : float — z setpoint in metres                 (default: 1.2)
speed       : float — angular velocity in rad/s            (default: 0.3)
center_x    : float — orbit centre x in metres            (default: 0.0)
center_y    : float — orbit centre y in metres            (default: 0.0)
climb_rate  : float — vertical speed for helix (m/s)      (default: 0.1)
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped


class AnalyticalTrajNode(Node):
    DT = 0.02   # 50 Hz

    def __init__(self):
        super().__init__('hexa_analytical_traj')

        self.declare_parameter('traj_type',  'circle')
        self.declare_parameter('radius',      1.0)
        self.declare_parameter('height',      1.2)
        self.declare_parameter('speed',       0.3)
        self.declare_parameter('center_x',    0.0)
        self.declare_parameter('center_y',    0.0)
        self.declare_parameter('climb_rate',  0.1)

        self.ttype  = self.get_parameter('traj_type').value
        self.R      = self.get_parameter('radius').value
        self.z0     = self.get_parameter('height').value
        self.omega  = self.get_parameter('speed').value
        self.cx     = self.get_parameter('center_x').value
        self.cy     = self.get_parameter('center_y').value
        self.vz     = self.get_parameter('climb_rate').value

        self.t = 0.0

        self.pub_ref      = self.create_publisher(Float64MultiArray, '/hexa/reference',      10)
        self.pub_ref_pose = self.create_publisher(PoseStamped,       '/hexa/reference/pose', 10)

        self.create_timer(self.DT, self._cb)
        self.get_logger().info(
            f'Analytical trajectory [{self.ttype}] | '
            f'R={self.R} m | ω={self.omega} rad/s | z={self.z0} m'
        )

    # ------------------------------------------------------------------
    def _compute(self, t):
        """Return (x, y, z, vx, vy, vz) for the current trajectory type."""
        R, w, cx, cy, z0 = self.R, self.omega, self.cx, self.cy, self.z0

        if self.ttype == 'circle':
            x  = cx + R * math.cos(w * t)
            y  = cy + R * math.sin(w * t)
            z  = z0
            vx = -R * w * math.sin(w * t)
            vy =  R * w * math.cos(w * t)
            vz = 0.0

        elif self.ttype == 'lemniscate':
            # Lissajous figure-8: x = R·cos(ωt),  y = R/2·sin(2ωt)
            # Velocities:         ẋ = −Rω·sin(ωt), ẏ = Rω·cos(2ωt)
            x  = cx + R * math.cos(w * t)
            y  = cy + (R / 2.0) * math.sin(2.0 * w * t)
            z  = z0
            vx = -R * w * math.sin(w * t)
            vy =  R * w * math.cos(2.0 * w * t)
            vz = 0.0

        elif self.ttype == 'helix':
            x  = cx + R * math.cos(w * t)
            y  = cy + R * math.sin(w * t)
            z  = z0 + self.vz * t
            vx = -R * w * math.sin(w * t)
            vy =  R * w * math.cos(w * t)
            vz = self.vz

        else:
            x, y, z   = cx, cy, z0
            vx, vy, vz = 0.0, 0.0, 0.0

        return x, y, z, vx, vy, vz

    # ------------------------------------------------------------------
    def _cb(self):
        x, y, z, vx, vy, vz = self._compute(self.t)
        self.t += self.DT

        # 12D reference: [x_d, y_d, z_d, φ_d, θ_d, ψ_d, ẋ_d, ẏ_d, ż_d, ṗ_d, q̇_d, ṙ_d]
        ref_msg = Float64MultiArray()
        ref_msg.data = [x, y, z, 0.0, 0.0, 0.0, vx, vy, vz, 0.0, 0.0, 0.0]
        self.pub_ref.publish(ref_msg)

        pose_msg = PoseStamped()
        pose_msg.header.stamp    = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.w = 1.0
        self.pub_ref_pose.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(AnalyticalTrajNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
