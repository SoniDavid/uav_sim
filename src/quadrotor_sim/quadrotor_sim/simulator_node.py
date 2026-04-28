"""
ROS2 node for the quadrotor mathematical simulator.

Runs the full nonlinear dynamics at 1 kHz internally and publishes
all relevant topics at 50 Hz for visualization in Foxglove Studio.

Topics published:
  /uav/pose        geometry_msgs/PoseStamped  — position + orientation (quat)
  /uav/euler       geometry_msgs/Vector3Stamped — roll, pitch, yaw (rad)
  /uav/state       std_msgs/Float64MultiArray  — full 12-state vector
  /uav/motors      std_msgs/Float64MultiArray  — [T1,T2,T3,T4] in Newtons
  /uav/motor_omega std_msgs/Float64MultiArray  — [ω1,ω2,ω3,ω4] in rad/s
  /uav/reference   geometry_msgs/PoseStamped  — current smoothed target

Topics subscribed:
  /uav/cmd_pose    geometry_msgs/PoseStamped  — set new target position in-flight

TF:  world → base_link  (for Foxglove 3D panel)

Send a new target at any time:
  ros2 topic pub --once /uav/cmd_pose geometry_msgs/msg/PoseStamped \
    '{header: {frame_id: world}, pose: {position: {x: 2.0, y: 1.0, z: 1.5}}}'

Or click "Publish > Pose" in the Foxglove 3D panel.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3Stamped, TransformStamped
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from .params import OMEGA_HOVER, DT_SIM, DT_PUB, SIM_STEPS_PER_PUB
from .dynamics import rk4_step, euler_to_quaternion
from .mixer import thrust_torques_to_omega_sq, omega_sq_to_omega_des, compute_thrust_torques
from .controller import (compute_cascaded_control, compute_control,
                         compute_constrained_control, compute_constrained_control_qp,
                         design_lqr_scheduled, design_lqr)


def _float_array_msg(data):
    msg = Float64MultiArray()
    msg.data = [float(v) for v in data]
    return msg


class QuadrotorSimulatorNode(Node):

    def __init__(self):
        super().__init__('quadrotor_simulator')

        # ROS2 parameters
        self.declare_parameter('ref_x', 0.0)
        self.declare_parameter('ref_y', 0.0)
        self.declare_parameter('ref_z', 1.0)
        self.declare_parameter('ref_max_vel', 0.5)
        self.declare_parameter('controller', 'cascaded')  # options: 'cascaded', 'lqr', 'constrained_lqr'

        # Simulation state: [x,y,z, xd,yd,zd, phi,theta,psi, phid,thetad,psid, ω1,ω2,ω3,ω4]
        self.state = np.zeros(16)
        self.state[12:16] = OMEGA_HOVER   # warm-start motors at hover speed

        self.sim_time = 0.0

        # Active target: updated via /uav/cmd_pose subscription
        p = self.get_parameter
        self.active_target = np.zeros(12)
        self.active_target[0] = p('ref_x').value
        self.active_target[1] = p('ref_y').value
        self.active_target[2] = p('ref_z').value
        self.ref_max_vel = p('ref_max_vel').value

        # Smoothed reference (ramps toward active_target at ref_max_vel)
        self.x_ref = np.zeros(12)

        # Publishers
        self.pub_pose    = self.create_publisher(PoseStamped,       '/uav/pose',        10)
        self.pub_euler   = self.create_publisher(Vector3Stamped,    '/uav/euler',       10)
        self.pub_state   = self.create_publisher(Float64MultiArray, '/uav/state',       10)
        self.pub_motors  = self.create_publisher(Float64MultiArray, '/uav/motors',      10)
        self.pub_omega   = self.create_publisher(Float64MultiArray, '/uav/motor_omega', 10)
        self.pub_ref     = self.create_publisher(PoseStamped,       '/uav/reference',   10)

        # Subscriber: in-flight reference commands
        self.sub_cmd = self.create_subscription(
            PoseStamped, '/uav/cmd_pose', self._cmd_pose_callback, 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        # Static identity transform so Foxglove 3D has a stable root frame
        self._static_broadcaster = StaticTransformBroadcaster(self)
        world_tf = TransformStamped()
        world_tf.header.stamp = self.get_clock().now().to_msg()
        world_tf.header.frame_id = 'map'
        world_tf.child_frame_id  = 'world'
        world_tf.transform.rotation.w = 1.0
        self._static_broadcaster.sendTransform(world_tf)

        # Timer runs at 50 Hz; inside we run 20 RK4 steps (1 kHz)
        self.timer = self.create_timer(DT_PUB, self._step_callback)

        # Precompute LQR gain for optional LQR modes
        try:
            self.K, _, _, _ = design_lqr()
            self.R = np.diag([11.5, 330.0, 330.0, 46.0])  # Used by QP solver
        except Exception as e:
            self.get_logger().warn(f'Failed to design LQR: {e}; LQR modes will be disabled')
            self.K = None
            self.R = None

        # Saturation tracking
        self.T_commanded_history = []
        self.T_applied_history = []
        self.saturation_count = 0
        self.publish_rate_limit = 0

        # TF publish divider: every 5th cycle (10 Hz) to reduce Foxglove 3D flickering
        self._tf_cycle = 0

        self.get_logger().info(
            f'Quadrotor simulator started. '
            f'Initial target: {self.active_target[:3]}, '
            f'controller={self.get_parameter("controller").value}. '
            f'Send /uav/cmd_pose to change target in-flight.'
        )

    # ------------------------------------------------------------------ #

    def _cmd_pose_callback(self, msg: PoseStamped):
        """Update the active target from an incoming pose command."""
        self.active_target[0] = msg.pose.position.x
        self.active_target[1] = msg.pose.position.y
        self.active_target[2] = msg.pose.position.z
        self.get_logger().info(
            f'New target received: x={self.active_target[0]:.2f}, '
            f'y={self.active_target[1]:.2f}, z={self.active_target[2]:.2f}'
        )

    def _smooth_reference(self):
        """Rate-limit the reference to avoid actuator saturation."""
        delta = self.active_target - self.x_ref
        v = self.ref_max_vel
        for pi in [0, 1, 2]:
            self.x_ref[pi] += np.clip(delta[pi], -v * DT_SIM, v * DT_SIM)
        self.x_ref[8] += np.clip(delta[8], -v * DT_SIM, v * DT_SIM)

    def _step_callback(self):
        last_T, last_tau_phi, last_tau_theta, last_tau_psi = 0.0, 0.0, 0.0, 0.0

        for _ in range(SIM_STEPS_PER_PUB):
            self._smooth_reference()

            # read controller param each cycle so it can be changed at runtime
            controller = self.get_parameter('controller').value
            if controller == 'lqr' and self.K is not None:
                T, tau_phi, tau_theta, tau_psi = compute_control(self.K, self.state[:12], self.x_ref)
            elif controller == 'constrained_lqr' and self.K is not None:
                T, tau_phi, tau_theta, tau_psi = compute_constrained_control(self.K, self.state[:12], self.x_ref)
            elif controller == 'constrained_lqr_qp' and self.K is not None:
                T, tau_phi, tau_theta, tau_psi = compute_constrained_control_qp(self.K, self.R, self.state[:12], self.x_ref)
            elif controller == 'gain_scheduled_lqr' and self.K is not None:
                K_scheduled = design_lqr_scheduled(self.K, self.state[:12], self.x_ref)
                T, tau_phi, tau_theta, tau_psi = compute_control(K_scheduled, self.state[:12], self.x_ref)
                from .params import T_MAX, ARM, KQ, OMEGA_MAX
                TAU_ROLL_MAX = ARM * T_MAX
                TAU_PSI_MAX = 4.0 * KQ * OMEGA_MAX**2
                T = np.clip(T, 0.0, T_MAX)
                tau_phi = np.clip(tau_phi, -TAU_ROLL_MAX, TAU_ROLL_MAX)
                tau_theta = np.clip(tau_theta, -TAU_ROLL_MAX, TAU_ROLL_MAX)
                tau_psi = np.clip(tau_psi, -TAU_PSI_MAX, TAU_PSI_MAX)
            else:
                # default: cascaded nonlinear PD
                T, tau_phi, tau_theta, tau_psi = compute_cascaded_control(self.state[:12], self.x_ref)

            self.T_commanded_history.append(T)

            omega_sq = thrust_torques_to_omega_sq(T, tau_phi, tau_theta, tau_psi)
            u_cmd    = omega_sq_to_omega_des(omega_sq)

            self.state = rk4_step(self.state, u_cmd, DT_SIM)
            self.sim_time += DT_SIM

            last_T, last_tau_phi, last_tau_theta, last_tau_psi = T, tau_phi, tau_theta, tau_psi

        _, _, _, _, T_i = compute_thrust_torques(self.state[12:16])
        T_applied = np.sum(T_i)
        self.T_applied_history.append(T_applied)

        T_cmd_avg = np.mean(self.T_commanded_history[-SIM_STEPS_PER_PUB:]) if self.T_commanded_history else 0.0
        T_app_avg = np.mean(self.T_applied_history[-SIM_STEPS_PER_PUB:]) if self.T_applied_history else 0.0
        saturation_delta = T_cmd_avg - T_app_avg

        if saturation_delta > 0.01:
            self.saturation_count += 1

        self.publish_rate_limit += 1
        if self.publish_rate_limit >= 100:
            self.publish_rate_limit = 0
            controller = self.get_parameter('controller').value
            if self.saturation_count > 0:
                self.get_logger().warn(
                    f'[{controller}] Thrust saturation: '
                    f'cmd={T_cmd_avg:.3f}N, applied={T_app_avg:.3f}N, '
                    f'delta={saturation_delta:.3f}N, events={self.saturation_count}'
                )
            else:
                self.get_logger().info(
                    f'[{controller}] No saturation: '
                    f'cmd={T_cmd_avg:.3f}N, applied={T_app_avg:.3f}N'
                )
            self.saturation_count = 0

        self._publish_all(last_T, last_tau_phi, last_tau_theta, last_tau_psi)

    def _publish_all(self, T, tau_phi, tau_theta, tau_psi):
        now = self.get_clock().now().to_msg()
        s = self.state

        qx, qy, qz, qw = euler_to_quaternion(s[6], s[7], s[8])

        # /uav/pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = 'world'
        pose_msg.pose.position.x = float(s[0])
        pose_msg.pose.position.y = float(s[1])
        pose_msg.pose.position.z = float(s[2])
        pose_msg.pose.orientation.x = float(qx)
        pose_msg.pose.orientation.y = float(qy)
        pose_msg.pose.orientation.z = float(qz)
        pose_msg.pose.orientation.w = float(qw)
        self.pub_pose.publish(pose_msg)

        # /uav/euler
        euler_msg = Vector3Stamped()
        euler_msg.header.stamp = now
        euler_msg.header.frame_id = 'world'
        euler_msg.vector.x = float(s[6])   # phi  (roll)
        euler_msg.vector.y = float(s[7])   # theta (pitch)
        euler_msg.vector.z = float(s[8])   # psi  (yaw)
        self.pub_euler.publish(euler_msg)

        # /uav/state (12 elements)
        self.pub_state.publish(_float_array_msg(s[:12]))

        # /uav/motors — individual thrusts [T1,T2,T3,T4] in Newtons
        _, _, _, _, T_i = compute_thrust_torques(s[12:16])
        self.pub_motors.publish(_float_array_msg(T_i))

        # /uav/motor_omega — rotor speeds [ω1,ω2,ω3,ω4] in rad/s
        self.pub_omega.publish(_float_array_msg(s[12:16]))

        # /uav/reference — current smoothed target
        ref_msg = PoseStamped()
        ref_msg.header.stamp = now
        ref_msg.header.frame_id = 'world'
        ref_msg.pose.position.x = float(self.x_ref[0])
        ref_msg.pose.position.y = float(self.x_ref[1])
        ref_msg.pose.position.z = float(self.x_ref[2])
        ref_msg.pose.orientation.w = 1.0
        self.pub_ref.publish(ref_msg)

        # TF world → base_link at 10 Hz (every 5th call) to reduce Foxglove 3D flickering
        self._tf_cycle = (self._tf_cycle + 1) % 5
        if self._tf_cycle == 0:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now
            tf_msg.header.frame_id = 'world'
            tf_msg.child_frame_id = 'base_link'
            tf_msg.transform.translation.x = float(s[0])
            tf_msg.transform.translation.y = float(s[1])
            tf_msg.transform.translation.z = float(s[2])
            tf_msg.transform.rotation.x = float(qx)
            tf_msg.transform.rotation.y = float(qy)
            tf_msg.transform.rotation.z = float(qz)
            tf_msg.transform.rotation.w = float(qw)
            self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = QuadrotorSimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
