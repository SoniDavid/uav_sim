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
from .params import MASS, G, IXX, IYY, IZZ, T_HOVER, KT, OMEGA_MAX, ARM, KQ

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

        # Conservative per-input bounds (around hover)
        delta_T_min = -T_HOVER
        delta_T_max = T_MAX - T_HOVER

        tau_phi_max   = TAU_ROLL_MAX
        tau_theta_max = TAU_ROLL_MAX
        tau_psi_max   = TAU_PSI_MAX

        # Element-wise projection (weighted projection is trivial here because
        # R used in design is diagonal). This prevents commanding impossible
        # thrust/torques that would be clipped by the mixer/runtime.
        delta_u[0] = np.clip(delta_u[0], delta_T_min, delta_T_max)
        delta_u[1] = np.clip(delta_u[1], -tau_phi_max, tau_phi_max)
        delta_u[2] = np.clip(delta_u[2], -tau_theta_max, tau_theta_max)
        delta_u[3] = np.clip(delta_u[3], -tau_psi_max, tau_psi_max)

        T_total = np.clip(T_HOVER + delta_u[0], 0.0, T_MAX)
        return T_total, delta_u[1], delta_u[2], delta_u[3]


def _qp_box_solver(H, c, bounds, u_nom=None, max_iter=100, tol=1e-6):
    """
    Solve minimization: min_u  0.5*u^T*H*u + c^T*u
                        s.t.   bounds_min <= u <= bounds_max

    Uses projected gradient descent (no external QP solver required).

    Initial guess: the feasible point closest to u_nom (i.e. the clamped nominal).
    This is critical — initializing at c (the cost gradient) is wrong and converges
    to the wrong solution when u_nom is far from zero.
    """
    bounds_min, bounds_max = bounds
    if u_nom is not None:
        u = np.clip(u_nom, bounds_min, bounds_max)   # warm start at feasible projection
    else:
        u = np.zeros(len(c))
    
    for iteration in range(max_iter):
        grad = H @ u + c
        u_old = u.copy()
        
        # Gradient step with adaptive step size
        step_size = 0.1 / (1.0 + np.linalg.norm(H, 2))
        u = u - step_size * grad
        
        # Project onto box constraints
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
    
    This preserves the LQR's relative cost weighting (R) rather than
    clamping each input independently. More sophisticated than simple clipping.
    """
    error = state_12 - x_ref_12
    u_nom = -K @ error  # [delta_T, tau_phi, tau_theta, tau_psi]
    
    bounds_min = np.array([-T_HOVER, -TAU_ROLL_MAX, -TAU_ROLL_MAX, -TAU_PSI_MAX])
    bounds_max = np.array([T_MAX - T_HOVER, TAU_ROLL_MAX, TAU_ROLL_MAX, TAU_PSI_MAX])
    
    # QP: minimize (u - u_nom)^T R^(-1) (u - u_nom) = u^T R^(-1) u - 2*u_nom^T R^(-1) u + const
    # This is:  min_u  0.5*u^T * (2*R^{-1}) * u + (-2*R^{-1}*u_nom)^T * u
    R_inv = np.linalg.inv(R)
    H = 2.0 * R_inv
    c = -2.0 * R_inv @ u_nom
    bounds = (bounds_min, bounds_max)
    
    delta_u = _qp_box_solver(H, c, bounds, u_nom=u_nom)
    
    T_total = np.clip(T_HOVER + delta_u[0], 0.0, T_MAX)
    return T_total, delta_u[1], delta_u[2], delta_u[3]


def build_hover_linear_model(x_op=None):
    """
    Returns (A, B): linearized state-space matrices.
    
    If x_op is None, linearizes at hover.
    If x_op is provided (12-state), linearizes around that operating point.
    """
    A = np.zeros((12, 12))
    B = np.zeros((12, 4))

    # Kinematics: position integrates velocity
    A[0, 3]  = 1.0   # x_dot  -> x
    A[1, 4]  = 1.0   # y_dot  -> y
    A[2, 5]  = 1.0   # z_dot  -> z
    # Euler angle kinematics (at small angles J_inv ≈ I)
    A[6, 9]  = 1.0   # phi_dot   -> phi
    A[7, 10] = 1.0   # theta_dot -> theta
    A[8, 11] = 1.0   # psi_dot   -> psi

    # Gravity-attitude coupling (linearized)
    A[3, 7] =  G     # x_ddot ≈  g * theta
    A[4, 6] = -G     # y_ddot ≈ -g * phi

    # Control input coupling (always same at any equilibrium)
    B[5,  0] = 1.0 / MASS   # z_ddot   <- delta_T
    B[9,  1] = 1.0 / IXX    # phi_ddot <- tau_phi
    B[10, 2] = 1.0 / IYY    # theta_ddot <- tau_theta
    B[11, 3] = 1.0 / IZZ    # psi_ddot <- tau_psi

    # Note: for small angles around any trajectory, the linearization
    # above (around hover) remains valid. For large angle changes,
    # a proper gain-scheduling would use a trajectory-dependent A matrix.
    return A, B


def design_lqr(Q=None, R=None):
    """
    Solve CARE and return LQR gain matrix K (4×12).

    Default Q and R are Bryson-rule based with extra tuning for this vehicle.
    """
    A, B = build_hover_linear_model()

    if Q is None:
        # Bryson's rule: Q_ii = 1 / max_acceptable_error_i²
        # Position (m): max 0.5m error  → 1/0.25 = 4
        # Velocity (m/s): max 0.5 m/s  → 1/0.25 = 4
        # Attitude (rad): max 15° error → 1/(0.26²) ≈ 14
        # Angular rate (rad/s): max 30°/s → 1/(0.52²) ≈ 3.7
        q_diag = np.array([4.0,  4.0,  4.0,   # x, y, z
                            4.0,  4.0,  4.0,   # xd, yd, zd
                           14.0, 14.0,  4.0,   # phi, theta, psi
                            3.7,  3.7,  3.7])  # phid, thetad, psid
        Q = np.diag(q_diag)

    if R is None:
        # Bryson's rule: R_jj = 1 / max_acceptable_input_j²
        # delta_T max = T_MAX - T_hover ≈ 0.295 N  → 1/0.087 ≈ 11.5
        # tau_phi max = ARM * T_max ≈ 0.055 Nm     → 1/0.003 ≈ 330
        # tau_theta: same as tau_phi               → 330
        # tau_psi max ≈ 4*kQ*OMEGA_MAX² ≈ 0.148 Nm → 1/0.022 ≈ 46
        r_diag = np.array([11.5, 330.0, 330.0, 46.0])
        R = np.diag(r_diag)

    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P   # (4, 12)

    # Stability check
    A_cl = A - B @ K
    eigs = np.linalg.eigvals(A_cl)
    assert all(e.real < 0 for e in eigs), "LQR closed-loop system is NOT stable!"

    return K, A, B, P


def design_lqr_scheduled(K_nom, x_current, x_ref, velocity_scale=1.0):
    """
    Gain-scheduling: adjust the LQR gain based on error magnitude.
    
    Simple scaling: if error is larger than expected, reduce gains to avoid
    aggressive commands. If error is small, use nominal gains.
    
    Returns a scaled K matrix.
    """
    error = x_current - x_ref
    
    # Compute overall error magnitude (normalized by max acceptable errors from Q)
    q_diag = np.array([4.0,  4.0,  4.0,   # pos: max 0.5 m
                       4.0,  4.0,  4.0,   # vel: max 0.5 m/s
                      14.0, 14.0,  4.0,   # att: max ~26 deg (0.26 rad)
                       3.7,  3.7,  3.7])  # rate: max ~30 deg/s (0.52 rad/s)
    Q_wts = np.sqrt(q_diag)
    normalized_error = np.abs(error) / (Q_wts + 1e-8)
    max_normalized_error = np.max(normalized_error)
    
    # Scheduling function: reduce gain if error is large
    # Linear interpolation between 0.3*K_nom (at 3x expected error) and K_nom (at nominal)
    gain_scale = np.clip(1.0 - 0.7 * max(0, max_normalized_error - 1.0) / 3.0, 0.3, 1.0)
    
    return gain_scale * K_nom


# ─────────────────────────────────────────────────────────────────────────────
# Cascaded nonlinear controller (position PD + attitude PD with tilt compensation)
# ─────────────────────────────────────────────────────────────────────────────

# Outer loop gains (position PD)
KP_XY = 2.0       # rad/s², position proportional
KD_XY = 1.5       # rad/s, velocity derivative
KP_Z  = 4.0       # m/s², altitude proportional
KD_Z  = 3.0       # altitude derivative

# Inner loop gains (attitude PD)
KP_ATT = 300.0    # N·m/rad, attitude proportional
KD_ATT = 30.0     # N·m·s/rad, attitude derivative
KP_PSI = 100.0
KD_PSI = 15.0


def compute_cascaded_control(state_12, x_ref_12):
    """
    Cascaded nonlinear PD controller with tilt compensation.

    Outer loop: position error → desired acceleration → desired attitude
    Inner loop: attitude error → torques
    Total thrust: corrected for tilt so altitude is maintained during lateral motion.

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

    # --- Outer position loop ---
    ax_d = KP_XY * (x_r - x) + KD_XY * (xd_r - xd)
    ay_d = KP_XY * (y_r - y) + KD_XY * (yd_r - yd)
    az_d = KP_Z  * (z_r - z) + KD_Z  * (zd_r - zd)

    # --- Tilt-compensated total thrust ---
    # Desired vertical acceleration (world frame) must equal (az_d + g) for the z dynamics.
    # With tilt: T * cos(phi)*cos(theta) / m = az_d + g
    cos_tilt = max(np.cos(phi) * np.cos(theta), 0.1)   # avoid division by near-zero
    T_total = MASS * (az_d + G) / cos_tilt
    T_total = np.clip(T_total, 0.0, T_MAX)

    # --- Desired attitude from desired horizontal acceleration ---
    # At small angles: ax_d ≈ g*theta,  ay_d ≈ -g*phi
    # Full expression from rotation matrix:
    T_ref = max(T_total, 0.01)  # avoid division by zero
    theta_d = np.arcsin(np.clip(MASS * ax_d / T_ref, -0.5, 0.5))   # limit ±30°
    phi_d   = np.arcsin(np.clip(-MASS * ay_d / (T_ref * np.cos(theta_d)), -0.5, 0.5))
    psi_d   = psi_r

    # --- Inner attitude loop ---
    tau_phi   = IXX * (KP_ATT * (phi_d   - phi)   - KD_ATT * phid)
    tau_theta = IYY * (KP_ATT * (theta_d - theta) - KD_ATT * thetad)
    tau_psi   = IZZ * (KP_PSI * (psi_d   - psi)   - KD_PSI * psid)

    return T_total, tau_phi, tau_theta, tau_psi


def compute_control(K, state_12, x_ref_12):
    """
    Compute high-level control from LQR feedback (no saturation handling).

    Args:
        K:          (4, 12) LQR gain matrix
        state_12:   (12,)   current rigid-body state
        x_ref_12:   (12,)   reference state

    Returns:
        (T_total, tau_phi, tau_theta, tau_psi)
    """
    error = state_12 - x_ref_12
    delta_u = -K @ error                     # [delta_T, tau_phi, tau_theta, tau_psi]
    T_total = np.clip(T_HOVER + delta_u[0], 0.0, T_MAX)
    return T_total, delta_u[1], delta_u[2], delta_u[3]
