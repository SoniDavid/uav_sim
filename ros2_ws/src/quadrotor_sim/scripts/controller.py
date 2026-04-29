"""
LQR controller for the quadrotor.

Linearizes the dynamics at hover (phi=theta=psi=0, T=mg) and solves
the Continuous Algebraic Riccati Equation to obtain an optimal gain matrix K.

Control law: [delta_T, tau_phi, tau_theta, tau_psi] = -K * (x - x_ref)
Total thrust: T = T_hover + delta_T

Extensions:
- compute_constrained_control_qp: QP-based projection respecting R weighting
- compute_gain_scheduled_control: re-linearize + recompute K online at current operating point
"""

import numpy as np
from scipy.linalg import solve_continuous_are
from quadrotor_sim_scripts.params import MASS, G, IXX, IYY, IZZ, T_HOVER, KT, OMEGA_MAX, ARM, KQ

T_MAX = 4.0 * KT * OMEGA_MAX**2   # physical thrust ceiling ≈ 0.736 N

# Actuator / torque limits (used by constrained LQR projection)
TAU_ROLL_MAX = ARM * T_MAX                   # roll/pitch torque ceiling (approx)
TAU_PSI_MAX  = 4.0 * KQ * OMEGA_MAX**2       # yaw torque ceiling (approx)


def compute_constrained_control(K, state_12, x_ref_12):
        """
        Constrained LQR-style control (simple projection onto actuator bounds).

        This function computes the nominal LQR command `delta_u = -K(x-x_ref)` and
        then projects each input onto conservative bounds derived from physical
        limits (thrust and torque ceilings). It returns the same tuple as
        `compute_control`: `(T_total, tau_phi, tau_theta, tau_psi)`.

        Notes:
        - This is a simple, per-input projection (box constraints). For better
            performance you can solve a small QP (e.g., to respect coupling via R)
            or implement a constrained MPC.
        - The cascaded PD controller is left intact and still recommended for
            aggressive maneuvers; use this constrained LQR when you want a linear
            feedback law that respects actuator bounds.
        """
        error = state_12 - x_ref_12
        delta_u = -K @ error   # [delta_T, tau_phi, tau_theta, tau_psi]

        delta_T_min = -T_HOVER
        delta_T_max = T_MAX - T_HOVER

        delta_u[0] = np.clip(delta_u[0], delta_T_min, delta_T_max)
        delta_u[1] = np.clip(delta_u[1], -TAU_ROLL_MAX, TAU_ROLL_MAX)
        delta_u[2] = np.clip(delta_u[2], -TAU_ROLL_MAX, TAU_ROLL_MAX)
        delta_u[3] = np.clip(delta_u[3], -TAU_PSI_MAX, TAU_PSI_MAX)

        T_total = np.clip(T_HOVER + delta_u[0], 0.0, T_MAX)
        return T_total, delta_u[1], delta_u[2], delta_u[3]


def _qp_box_solver(H, c, bounds, u_nom=None, max_iter=100, tol=1e-6):
    """
    Solve minimization: min_u  0.5*u^T*H*u + c^T*u
                        s.t.   bounds_min <= u <= bounds_max

    Uses projected gradient descent (no external QP solver required).
    """
    bounds_min, bounds_max = bounds
    if u_nom is not None:
        u = np.clip(u_nom, bounds_min, bounds_max)
    else:
        u = np.zeros(len(c))

    for iteration in range(max_iter):
        grad = H @ u + c
        u_old = u.copy()

        step_size = 0.1 / (1.0 + np.linalg.norm(H, 2))
        u = u - step_size * grad

        bounds_min, bounds_max = bounds
        u = np.clip(u, bounds_min, bounds_max)

        if np.linalg.norm(u - u_old) < tol:
            break

    return u


def compute_constrained_control_qp(K, R, state_12, x_ref_12):
    """
    Constrained LQR using QP projection that respects the R weighting.

    Solves: min_u  0.5*(u-u_nom)^T * R^{-1} * (u-u_nom)
            s.t.   thrust and torque box constraints
    """
    error = state_12 - x_ref_12
    u_nom = -K @ error

    bounds_min = np.array([-T_HOVER, -TAU_ROLL_MAX, -TAU_ROLL_MAX, -TAU_PSI_MAX])
    bounds_max = np.array([T_MAX - T_HOVER, TAU_ROLL_MAX, TAU_ROLL_MAX, TAU_PSI_MAX])

    R_inv = np.linalg.inv(R)
    H = 2.0 * R_inv
    c = -2.0 * R_inv @ u_nom
    bounds = (bounds_min, bounds_max)

    delta_u = _qp_box_solver(H, c, bounds, u_nom=u_nom)

    T_total = np.clip(T_HOVER + delta_u[0], 0.0, T_MAX)
    return T_total, delta_u[1], delta_u[2], delta_u[3]


def build_hover_linear_model(x_op=None):
    """Returns (A, B): linearized state-space matrices at hover."""
    A = np.zeros((12, 12))
    B = np.zeros((12, 4))

    A[0, 3]  = 1.0
    A[1, 4]  = 1.0
    A[2, 5]  = 1.0
    A[6, 9]  = 1.0
    A[7, 10] = 1.0
    A[8, 11] = 1.0

    A[3, 7] =  G
    A[4, 6] = -G

    B[5,  0] = 1.0 / MASS
    B[9,  1] = 1.0 / IXX
    B[10, 2] = 1.0 / IYY
    B[11, 3] = 1.0 / IZZ

    return A, B


def design_lqr(Q=None, R=None):
    """Solve CARE and return LQR gain matrix K (4×12)."""
    A, B = build_hover_linear_model()

    if Q is None:
        q_diag = np.array([4.0,  4.0,  4.0,
                            4.0,  4.0,  4.0,
                           14.0, 14.0,  4.0,
                            3.7,  3.7,  3.7])
        Q = np.diag(q_diag)

    if R is None:
        r_diag = np.array([11.5, 330.0, 330.0, 46.0])
        R = np.diag(r_diag)

    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P

    A_cl = A - B @ K
    eigs = np.linalg.eigvals(A_cl)
    assert all(e.real < 0 for e in eigs), "LQR closed-loop system is NOT stable!"

    return K, A, B, P


def design_lqr_scheduled(K_nom, x_current, x_ref, velocity_scale=1.0):
    """Gain-scheduling: scale LQR gain based on normalized error magnitude."""
    error = x_current - x_ref

    q_diag = np.array([4.0,  4.0,  4.0,
                       4.0,  4.0,  4.0,
                      14.0, 14.0,  4.0,
                       3.7,  3.7,  3.7])
    Q_wts = np.sqrt(q_diag)
    normalized_error = np.abs(error) / (Q_wts + 1e-8)
    max_normalized_error = np.max(normalized_error)

    gain_scale = np.clip(1.0 - 0.7 * max(0, max_normalized_error - 1.0) / 3.0, 0.3, 1.0)

    return gain_scale * K_nom


# ─────────────────────────────────────────────────────────────────────────────
# Cascaded nonlinear controller (position PD + attitude PD with tilt compensation)
# ─────────────────────────────────────────────────────────────────────────────

KP_XY = 2.0
KD_XY = 1.5
KP_Z  = 4.0
KD_Z  = 3.0

KP_ATT = 300.0
KD_ATT = 30.0
KP_PSI = 100.0
KD_PSI = 15.0


def compute_cascaded_control(state_12, x_ref_12):
    """
    Cascaded nonlinear PD controller with tilt compensation.

    Outer loop: position error → desired acceleration → desired attitude
    Inner loop: attitude error → torques

    Returns:
        (T_total, tau_phi, tau_theta, tau_psi)
    """
    x, y, z             = state_12[0:3]
    xd, yd, zd          = state_12[3:6]
    phi, theta, psi     = state_12[6:9]
    phid, thetad, psid  = state_12[9:12]

    x_r, y_r, z_r          = x_ref_12[0:3]
    xd_r, yd_r, zd_r       = x_ref_12[3:6]
    phi_r, theta_r, psi_r  = x_ref_12[6:9]

    ax_d = KP_XY * (x_r - x) + KD_XY * (xd_r - xd)
    ay_d = KP_XY * (y_r - y) + KD_XY * (yd_r - yd)
    az_d = KP_Z  * (z_r - z) + KD_Z  * (zd_r - zd)

    cos_tilt = max(np.cos(phi) * np.cos(theta), 0.1)
    T_total = MASS * (az_d + G) / cos_tilt
    T_total = np.clip(T_total, 0.0, T_MAX)

    T_ref = max(T_total, 0.01)
    theta_d = np.arcsin(np.clip(MASS * ax_d / T_ref, -0.5, 0.5))
    phi_d   = np.arcsin(np.clip(-MASS * ay_d / (T_ref * np.cos(theta_d)), -0.5, 0.5))
    psi_d   = psi_r

    tau_phi   = IXX * (KP_ATT * (phi_d   - phi)   - KD_ATT * phid)
    tau_theta = IYY * (KP_ATT * (theta_d - theta) - KD_ATT * thetad)
    tau_psi   = IZZ * (KP_PSI * (psi_d   - psi)   - KD_PSI * psid)

    return T_total, tau_phi, tau_theta, tau_psi


def compute_control(K, state_12, x_ref_12):
    """LQR feedback control (no saturation handling)."""
    error = state_12 - x_ref_12
    delta_u = -K @ error
    T_total = np.clip(T_HOVER + delta_u[0], 0.0, T_MAX)
    return T_total, delta_u[1], delta_u[2], delta_u[3]
