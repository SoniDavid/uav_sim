#!/usr/bin/env python3
"""
LQR redesign for quadrotor — discrete-time DARE on decoupled subsystems.

Strategy: mirror the SF controller's design philosophy using LQR.
  - Same target poles as SF (outer -2.5±0.5j, inner -15±2j, alt -4±0.3j, yaw -3±0.3j)
  - Discrete-time DARE at Ts = 0.02 s (50 Hz) accounts for sampling properly
  - Q (diagonal) found numerically so DARE gives the same K as SF pole placement
  - R set by Bryson's rule: R_i = 1 / u_max_i²

Root cause of original failure:
  Q_psi=4, R_psi=46  →  k_psi = sqrt(4/46) = 0.295  (yaw bandwidth ~63 rad/s = 10 Hz)
  SF target k_psi = 0.000665                           (yaw bandwidth  ~3 rad/s = 0.5 Hz)
  The yaw was 443× too aggressive; attitude channels 2–3× too aggressive.

Run:  python3 docs/lqr_redesign.py
"""
import numpy as np
from numpy.linalg import eigvals
from scipy import linalg, optimize, signal

# ─── Physical constants (params.hpp) ──────────────────────────────────────
m    = 0.045
g    = 9.81
Ixx  = 7.921e-5
Iyy  = 13.604e-5
Izz  = 7.317e-5
ARM  = 0.075
KT   = 2.64e-8
KQ   = 5.4e-9
OM   = 2640.0
Ts   = 0.02

T_MAX       = 4 * KT * OM**2
T_HOVER     = m * g
TAU_RP_MAX  = ARM * T_MAX
TAU_PSI_MAX = 4 * KQ * OM**2

print(f"T_MAX = {T_MAX:.4f} N   T_HOVER = {T_HOVER:.4f} N")
print(f"TAU_RP_MAX = {TAU_RP_MAX:.5f} Nm   TAU_PSI_MAX = {TAU_PSI_MAX:.5f} Nm\n")

# ─── Target poles (identical to SF controller) ────────────────────────────
P_ALT    = np.array([-4+0.3j,    -4-0.3j])
P_YROLL  = np.array([-2.5+0.5j,  -2.5-0.5j,  -15+2j,  -15-2j])
P_XPITCH = np.array([-2.5+0.5j,  -2.5-0.5j,  -15+2j,  -15-2j])
P_YAW    = np.array([-3+0.3j,    -3-0.3j])

# ─── Decoupled subsystem matrices ─────────────────────────────────────────
# Altitude  [z, vz],          u = ΔF
A_alt = np.array([[0.,1.],[0.,0.]])
B_alt = np.array([[0.],[1./m]])

# y-roll    [y, vy, φ, φ̇],  u = τ_φ   (ÿ = -g·φ)
A_yr  = np.array([[0.,1.,0.,0.],[0.,0.,-g,0.],[0.,0.,0.,1.],[0.,0.,0.,0.]])
B_yr  = np.array([[0.],[0.],[0.],[1./Ixx]])

# x-pitch   [x, vx, θ, θ̇],  u = τ_θ   (ẍ = +g·θ)
A_xp  = np.array([[0.,1.,0.,0.],[0.,0.,g,0.],[0.,0.,0.,1.],[0.,0.,0.,0.]])
B_xp  = np.array([[0.],[0.],[0.],[1./Iyy]])

# Yaw       [ψ, ψ̇],          u = τ_ψ
A_yaw = np.array([[0.,1.],[0.,0.]])
B_yaw = np.array([[0.],[1./Izz]])

# ─── R values — Bryson's rule: R = 1 / u_max² ────────────────────────────
R_F    = 1.0 / (T_MAX - T_HOVER)**2   # ΔF_max ≈ 0.295 N
R_phi  = 1.0 / TAU_RP_MAX**2          # τ_φ_max ≈ 0.0552 Nm
R_tht  = 1.0 / TAU_RP_MAX**2
R_psi  = 1.0 / TAU_PSI_MAX**2         # τ_ψ_max ≈ 0.1505 Nm

print(f"R_F={R_F:.2f}  R_phi={R_phi:.1f}  R_theta={R_tht:.1f}  R_psi={R_psi:.2f}\n")


# ─── Helpers ──────────────────────────────────────────────────────────────
def zoh(A, B, Ts):
    """Zero-order-hold discretization."""
    n, m_in = A.shape[0], B.shape[1]
    M = np.zeros((n + m_in, n + m_in))
    M[:n, :n] = A * Ts
    M[:n, n:] = B * Ts
    eM = linalg.expm(M)
    return eM[:n, :n], eM[:n, n:]


def dare_gain(A_d, B_d, Q, R):
    """DARE solution → feedback gain matrix (1 × n)."""
    P = linalg.solve_discrete_are(A_d, B_d, Q, R)
    return linalg.solve(R + B_d.T @ P @ B_d, B_d.T @ P @ A_d)[0]


def ct_poles_from_disc(A_cl_d, Ts):
    """Convert discrete closed-loop poles to continuous-time equivalents."""
    return np.log(eigvals(A_cl_d)) / Ts


def fit_q_to_K_target(A_ct, B_ct, K_target, Ts, R_scalar):
    """
    Find diagonal Q (all positive) such that DARE(A_d, B_d, Q, R) ≈ K_target.
    R = R_scalar * I_1.  Uses Nelder-Mead on log(q_i).
    """
    A_d, B_d = zoh(A_ct, B_ct, Ts)
    R = np.array([[R_scalar]])
    n = A_ct.shape[0]

    def cost(lq):
        Q = np.diag(np.exp(lq))
        try:
            K = dare_gain(A_d, B_d, Q, R)
            return float(np.sum((K - K_target) ** 2))
        except Exception:
            return 1e10

    # Initial guess: from double-integrator CARE identity  k ≈ sqrt(q/r)
    lq0 = np.log(np.maximum(K_target ** 2 * R_scalar, 1e-20))

    res = optimize.minimize(
        cost, lq0, method='Nelder-Mead',
        options={'maxiter': 200_000, 'xatol': 1e-14, 'fatol': 1e-14, 'adaptive': True}
    )

    Q_opt = np.diag(np.exp(res.x))
    K_final = dare_gain(A_d, B_d, Q_opt, R)
    err = np.max(np.abs(K_final - K_target))
    if err > 1e-4:
        print(f"  [WARN] max K error = {err:.2e} — optimizer may not have fully converged")

    A_cl_d = A_d - B_d.reshape(-1, 1) @ K_final.reshape(1, -1)
    return K_final, Q_opt, ct_poles_from_disc(A_cl_d, Ts)


# ─── Step 1: compute SF pole-placement targets ────────────────────────────
print("=" * 60)
print("STEP 1 — Pole-placement targets (must match K_SF in controller.hpp)")
print("=" * 60)

pp_alt  = signal.place_poles(A_alt, B_alt, P_ALT)
pp_yr   = signal.place_poles(A_yr,  B_yr,  P_YROLL)
pp_xp   = signal.place_poles(A_xp,  B_xp,  P_XPITCH)
pp_yaw  = signal.place_poles(A_yaw, B_yaw, P_YAW)

K_alt_t = pp_alt.gain_matrix[0]
K_yr_t  = pp_yr.gain_matrix[0]
K_xp_t  = pp_xp.gain_matrix[0]
K_yaw_t = pp_yaw.gain_matrix[0]

print(f"  Altitude : k_z={K_alt_t[0]:.7f}  k_vz={K_alt_t[1]:.7f}")
print(f"  y-roll   : {np.round(K_yr_t, 7)}")
print(f"  x-pitch  : {np.round(K_xp_t, 7)}")
print(f"  Yaw      : k_psi={K_yaw_t[0]:.7f}  k_psidot={K_yaw_t[1]:.7f}\n")

# ─── Step 2: fit DARE Q to each subsystem ────────────────────────────────
print("=" * 60)
print("STEP 2 — Fitting discrete-time DARE Q/R (Ts = 0.02 s)")
print("=" * 60)

K_alt,  Q_alt,  pct_alt  = fit_q_to_K_target(A_alt, B_alt, K_alt_t,  Ts, R_F)
K_yr,   Q_yr,   pct_yr   = fit_q_to_K_target(A_yr,  B_yr,  K_yr_t,   Ts, R_phi)
K_xp,   Q_xp,   pct_xp   = fit_q_to_K_target(A_xp,  B_xp,  K_xp_t,   Ts, R_tht)
K_yaw,  Q_yaw,  pct_yaw  = fit_q_to_K_target(A_yaw, B_yaw, K_yaw_t,  Ts, R_psi)

def print_subsystem(name, K, pct):
    print(f"  {name}:")
    print(f"    K     = {np.round(K, 8)}")
    pct_real = np.round(np.real(pct), 3)
    pct_imag = np.round(np.imag(pct), 3)
    pole_strs = [f"{r:+.2f}{i:+.2f}j" for r, i in zip(pct_real, pct_imag)]
    print(f"    poles = {pole_strs}")
    bw = np.max(np.abs(pct))
    print(f"    max |pole| = {bw:.2f} rad/s  (Nyquist = 157 rad/s at 50 Hz)")

print_subsystem("Altitude ",  K_alt,  pct_alt)
print_subsystem("y-roll   ",  K_yr,   pct_yr)
print_subsystem("x-pitch  ",  K_xp,   pct_xp)
print_subsystem("Yaw      ",  K_yaw,  pct_yaw)

# ─── Step 3: saturation thresholds ───────────────────────────────────────
print()
print("=" * 60)
print("STEP 3 — Saturation thresholds")
print("=" * 60)
# Position saturation: largest gain from position directly to torque
# K_yr[0] = y→τ_φ gain, K_xp[0] = x→τ_θ gain (when phi=0, theta=0)
print(f"  y_err  → τ_φ sat at |y|   > {TAU_RP_MAX/abs(K_yr[0]):.2f} m   (SF = 4.59 m, need > 2 m)")
print(f"  x_err  → τ_θ sat at |x|   > {TAU_RP_MAX/abs(K_xp[0]):.2f} m   (SF = 2.67 m, need > 2 m)")
print(f"  φ_err  → τ_φ sat at |φ|   > {np.degrees(TAU_RP_MAX/abs(K_yr[2])):.0f} deg (attitude saturation)")
print(f"  θ_err  → τ_θ sat at |θ|   > {np.degrees(TAU_RP_MAX/abs(K_xp[2])):.0f} deg (attitude saturation)")
print(f"  ψ_err  → τ_ψ sat at |ψ|   > {np.degrees(TAU_PSI_MAX/abs(K_yaw[0])):.0f} deg (need > 90°)")
print(f"  z_err  → ΔF  sat at |z|   > {(T_MAX-T_HOVER)/K_alt[0]:.2f} m   (thrust headroom)")

# ─── Step 4: assemble full 4×12 K_LQR ───────────────────────────────────
# State order: [x=0, y=1, z=2, vx=3, vy=4, vz=5, phi=6, theta=7, psi=8,
#               phi_dot=9, theta_dot=10, psi_dot=11]
K_LQR = np.zeros((4, 12))

# Row 0 (ΔF):   altitude
K_LQR[0, 2]  = K_alt[0]    # z
K_LQR[0, 5]  = K_alt[1]    # vz

# Row 1 (τ_φ):  y-roll
K_LQR[1, 1]  = K_yr[0]     # y   (negative: loop closes via ÿ = -g·φ)
K_LQR[1, 4]  = K_yr[1]     # vy
K_LQR[1, 6]  = K_yr[2]     # φ
K_LQR[1, 9]  = K_yr[3]     # φ̇

# Row 2 (τ_θ):  x-pitch
K_LQR[2, 0]  = K_xp[0]     # x
K_LQR[2, 3]  = K_xp[1]     # vx
K_LQR[2, 7]  = K_xp[2]     # θ
K_LQR[2, 10] = K_xp[3]     # θ̇

# Row 3 (τ_ψ):  yaw
K_LQR[3, 8]  = K_yaw[0]    # ψ
K_LQR[3, 11] = K_yaw[1]    # ψ̇

# ─── Step 5: print C++ constant ───────────────────────────────────────────
print()
print("=" * 60)
print("STEP 4 — C++ K_LQR for controller.hpp")
print("=" * 60)
print("// Q diagonal (per subsystem):")
print(f"//   Altitude : {np.diag(Q_alt)}")
print(f"//   y-roll   : {np.diag(Q_yr)}")
print(f"//   x-pitch  : {np.diag(Q_xp)}")
print(f"//   Yaw      : {np.diag(Q_yaw)}")
print(f"// R (Bryson): [{R_F:.2f}, {R_phi:.1f}, {R_tht:.1f}, {R_psi:.2f}]")
print()
print("static constexpr double K_LQR[4][12] = {")
labels = ["ΔF ", "τ_φ", "τ_θ", "τ_ψ"]
for i in range(4):
    row = ", ".join(f"{v:17.13f}" for v in K_LQR[i])
    print(f"    {{ {row} }},  // {labels[i]}")
print("};")

# ─── Step 6: compare with old gains ──────────────────────────────────────
K_OLD = np.array([
    [0,          0,          0.5897678246, 0,           0,           0.6331707441, 0,            0,           0,           0,            0,           0          ],
    [0,         -0.0348155312, 0,          0,          -0.0428074386, 0,           0.0873990440,  0,           0,           0.0041080481,  0,           0          ],
    [0.0348155312, 0,          0,          0.0432794948, 0,           0,           0,             0.0931243276, 0,          0,             0.0053261215, 0          ],
    [0,          0,          0,           0,           0,           0,           0,             0,           0.2948839123, 0,            0,           0.0161413945],
])
print()
print("=" * 60)
print("COMPARISON: new vs old K_LQR (non-zero entries only)")
print("=" * 60)
ch_labels = ['x','y','z','vx','vy','vz','φ','θ','ψ','φ̇','θ̇','ψ̇']
row_labels = ['ΔF ','τ_φ','τ_θ','τ_ψ']
for i in range(4):
    for j in range(12):
        if abs(K_LQR[i,j]) > 1e-12 or abs(K_OLD[i,j]) > 1e-12:
            print(f"  K[{row_labels[i]}][{ch_labels[j]:2s}]:  old={K_OLD[i,j]:+.7f}   new={K_LQR[i,j]:+.7f}  "
                  f"  ratio={K_LQR[i,j]/K_OLD[i,j]:.3f}" if abs(K_OLD[i,j])>1e-12 else
                  f"  K[{row_labels[i]}][{ch_labels[j]:2s}]:  old={K_OLD[i,j]:+.7f}   new={K_LQR[i,j]:+.7f}")
