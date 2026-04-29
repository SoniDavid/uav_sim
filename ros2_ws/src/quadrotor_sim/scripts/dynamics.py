"""
Full nonlinear quadrotor dynamics.

State vector (16 elements):
  [0:3]   = [x, y, z]               inertial position (m)
  [3:6]   = [x_dot, y_dot, z_dot]   inertial velocity (m/s)
  [6:9]   = [phi, theta, psi]        Euler angles ZYX (rad)
  [9:12]  = [phi_dot, theta_dot, psi_dot]  Euler angle rates (rad/s)
  [12:16] = [omega1, omega2, omega3, omega4]  rotor speeds (rad/s)

Control input (4 elements):
  u = [u1, u2, u3, u4]  desired motor speeds in rad/s (omega_des)
  Motor ODE: omega_dot = (u - omega) / TM  (first-order filter)
"""

import numpy as np
from quadrotor_sim_scripts.params import MASS, G, IXX, IYY, IZZ, JR, KT, KQ, TM, ARM


def _kinematic_matrix_inv(phi, theta):
    """
    J_theta_inv maps body angular rates [p, q, r] → Euler angle rates [phi_dot, theta_dot, psi_dot].

    ZYX Euler convention:
      J_inv = [[1,  sin(phi)*tan(theta),  cos(phi)*tan(theta)],
               [0,  cos(phi),            -sin(phi)           ],
               [0,  sin(phi)/cos(theta),  cos(phi)/cos(theta)]]
    """
    if abs(theta) > np.radians(88):
        # Near gimbal lock — clamp to avoid division by zero
        theta = np.sign(theta) * np.radians(88)

    sp, cp = np.sin(phi), np.cos(phi)
    st, ct = np.sin(theta), np.cos(theta)
    tt = st / ct

    return np.array([
        [1.0, sp * tt,  cp * tt],
        [0.0, cp,      -sp      ],
        [0.0, sp / ct,  cp / ct ],
    ])


def state_derivative(state, u_cmd):
    """
    Full 16-dimensional ODE right-hand side.

    Args:
        state:  np.ndarray (16,)
        u_cmd:  np.ndarray (4,) desired motor speeds in rad/s

    Returns:
        ds: np.ndarray (16,) time derivative of state
    """
    x, y, z             = state[0:3]
    xd, yd, zd          = state[3:6]
    phi, theta, psi     = state[6:9]
    p, q, r             = state[9:12]
    omega               = state[12:16]

    omega_dot = (u_cmd - omega) / TM

    T_i = KT * omega**2
    Q_i = KQ * omega**2

    T_total   = np.sum(T_i)
    tau_phi   = ARM * (T_i[1] + T_i[2] - T_i[0] - T_i[3])
    tau_theta = ARM * (T_i[0] + T_i[1] - T_i[2] - T_i[3])
    tau_psi   = -Q_i[0] + Q_i[1] - Q_i[2] + Q_i[3]

    Omega_r = omega[0] - omega[1] + omega[2] - omega[3]

    sp, cp = np.sin(phi), np.cos(phi)
    st, ct = np.sin(theta), np.cos(theta)
    spsi, cpsi = np.sin(psi), np.cos(psi)

    acc_scale = T_total / MASS
    x_ddot = acc_scale * (cp * st * cpsi + sp * spsi)
    y_ddot = acc_scale * (cp * st * spsi - sp * cpsi)
    z_ddot = acc_scale * (cp * ct) - G

    J_inv = _kinematic_matrix_inv(phi, theta)
    euler_rates = J_inv @ np.array([p, q, r])

    phi_ddot   = ((IYY - IZZ) * q * r + JR * q * Omega_r + tau_phi)   / IXX
    theta_ddot = ((IZZ - IXX) * p * r - JR * p * Omega_r + tau_theta) / IYY
    psi_ddot   = ((IXX - IYY) * p * q                    + tau_psi)   / IZZ

    ds = np.empty(16)
    ds[0:3]   = [xd, yd, zd]
    ds[3:6]   = [x_ddot, y_ddot, z_ddot]
    ds[6:9]   = euler_rates
    ds[9:12]  = [phi_ddot, theta_ddot, psi_ddot]
    ds[12:16] = omega_dot

    return ds


def rk4_step(state, u_cmd, dt):
    """Single RK4 integration step with fixed control input."""
    k1 = state_derivative(state, u_cmd)
    k2 = state_derivative(state + 0.5 * dt * k1, u_cmd)
    k3 = state_derivative(state + 0.5 * dt * k2, u_cmd)
    k4 = state_derivative(state + dt * k3, u_cmd)
    return state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def euler_to_quaternion(phi, theta, psi):
    """ZYX Euler angles to quaternion (qx, qy, qz, qw)."""
    cy, sy = np.cos(psi / 2), np.sin(psi / 2)
    cp, sp = np.cos(theta / 2), np.sin(theta / 2)
    cr, sr = np.cos(phi / 2), np.sin(phi / 2)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw
