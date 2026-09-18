#!/usr/bin/env python3
"""
Full MIMO pole placement for the quadrotor 12-state linearized system.

Approach
--------
The decoupled SF design (lqr_redesign.py Step 1) places poles on four independent
SISO subsystems.  Those subsystems share identical pole pairs for x-pitch and y-roll
(-2.5±0.5j outer, -15±2j inner), which means the full 12-state system has REPEATED
eigenvalues — that is valid, but it limits what scipy.signal.place_poles (KNV/Tits-Yang)
can do with the extra eigenvector freedom that a multi-input system provides.

Here we treat (A_12x12, B_12x4) as a single MIMO plant and assign 12 DISTINCT poles,
choosing slightly different dynamics for y-roll vs x-pitch.  All poles are verified to
produce a stable closed-loop system with bandwidth well below the 157 rad/s Nyquist limit.

Pole selection rationale
------------------------
  Altitude  : -4  ± 0.3j  (unchanged — no reason to differentiate)
  y-roll    : outer -2.4 ± 0.6j,  inner -14 ± 2j   (slightly softer than decoupled)
  x-pitch   : outer -2.6 ± 0.4j,  inner -16 ± 2j   (slightly stiffer than decoupled)
  Yaw       : -3  ± 0.3j  (unchanged)

The asymmetry makes all 12 eigenvalues of A-BK distinct, which is required for a
well-conditioned MIMO placement (no repeated eigenvalues in the closed-loop).

Physical note
-------------
Because B is block-diagonal (each input drives exactly one angular acceleration or
one linear acceleration in z), the KNV eigenvector freedom cannot create true
cross-coupling in K.  The resulting K matrix has the same sparsity pattern as the
decoupled K — the MIMO approach is still meaningful here because:
  1. It confirms the decoupled design via an independent, non-decomposed algorithm.
  2. It uses distinct poles that give each channel its own dynamic character.
  3. It is the correct framework to extend to tilt-rotor or hexacopter designs where
     B is no longer block-diagonal and genuine cross-coupling arises.

Run:  python3 tools/quad_scripts/mimo_sf_design.py
"""

import numpy as np
from numpy.linalg import eigvals
from scipy import linalg, signal

# ── Physical parameters (must match params.hpp) ───────────────────────────────
m    = 0.045
g    = 9.81
Ixx  = 7.921e-5
Iyy  = 13.604e-5
Izz  = 7.317e-5
ARM  = 0.075
KT   = 2.64e-8
KQ   = 5.4e-9
OM   = 2640.0

T_MAX       = 4 * KT * OM**2
T_HOVER     = m * g
TAU_RP_MAX  = ARM * T_MAX
TAU_PSI_MAX = 4 * KQ * OM**2

print(f"T_MAX={T_MAX:.4f} N  T_HOVER={T_HOVER:.4f} N")
print(f"TAU_RP_MAX={TAU_RP_MAX:.5f} Nm  TAU_PSI_MAX={TAU_PSI_MAX:.5f} Nm\n")

# ── Full 12-state linearized system at hover ──────────────────────────────────
# State order: [x, y, z, vx, vy, vz, phi, theta, psi, phi_dot, theta_dot, psi_dot]
#                0  1  2   3   4   5    6      7    8        9         10        11
# Input order:  [delta_T, tau_phi, tau_theta, tau_psi]
#                      0        1          2         3

A = np.zeros((12, 12))
A[0,  3] =  1.0          # dx/dt    = vx
A[1,  4] =  1.0          # dy/dt    = vy
A[2,  5] =  1.0          # dz/dt    = vz
A[6,  9] =  1.0          # dphi/dt  = phi_dot
A[7, 10] =  1.0          # dtheta/dt= theta_dot
A[8, 11] =  1.0          # dpsi/dt  = psi_dot
A[3,  7] =  g            # d(vx)/dt ≈  g * theta  (pitch coupling)
A[4,  6] = -g            # d(vy)/dt ≈ -g * phi    (roll coupling)

B = np.zeros((12, 4))
B[5,  0] = 1.0 / m       # d(vz)/dt      = delta_T / m
B[9,  1] = 1.0 / Ixx     # d(phi_dot)/dt = tau_phi / Ixx
B[10, 2] = 1.0 / Iyy     # d(theta_dot)/dt = tau_theta / Iyy
B[11, 3] = 1.0 / Izz     # d(psi_dot)/dt = tau_psi / Izz

# ── Check open-loop controllability ──────────────────────────────────────────
C_c = np.hstack([np.linalg.matrix_power(A, i) @ B for i in range(12)])
rank = np.linalg.matrix_rank(C_c)
print(f"Controllability matrix rank: {rank} / 12  ({'OK' if rank == 12 else 'NOT CONTROLLABLE'})\n")

# ── Open-loop eigenvalues ─────────────────────────────────────────────────────
print("Open-loop eigenvalues (all at origin, as expected for hover linearization):")
for ev in np.sort(eigvals(A).real):
    print(f"  {ev:+.4f}")
print()

# ── MIMO pole selection — all 12 must be distinct ────────────────────────────
#
#  Decoupled design used:
#    altitude  -4±0.3j          (2)
#    y-roll    -2.5±0.5j, -15±2j  (4)   <- repeated in full system
#    x-pitch   -2.5±0.5j, -15±2j  (4)   <- same poles as y-roll!
#    yaw       -3±0.3j           (2)
#
#  MIMO design uses distinct poles per channel:
#    altitude  -4±0.3j          (unchanged)
#    y-roll    -2.4±0.6j, -14±2j  (softer outer, softer inner)
#    x-pitch   -2.6±0.4j, -16±2j  (stiffer outer, stiffer inner)
#    yaw       -3±0.3j          (unchanged)

P_desired = np.array([
    -4   + 0.3j,  -4   - 0.3j,   # altitude
    -2.4 + 0.6j,  -2.4 - 0.6j,  -14 + 2j, -14 - 2j,  # y-roll
    -2.6 + 0.4j,  -2.6 - 0.4j,  -16 + 2j, -16 - 2j,  # x-pitch
    -3   + 0.3j,  -3   - 0.3j,   # yaw
])

assert len(P_desired) == 12, "Need exactly 12 poles for a 12-state system"
assert len(P_desired) == len(set(np.round(P_desired, 10))), "Poles must be distinct"

print("Desired MIMO poles:")
for p in P_desired:
    print(f"  {p.real:+.1f} {p.imag:+.1f}j")
print()

# ── MIMO pole placement (KNV / Tits-Yang) ────────────────────────────────────
result = signal.place_poles(A, B, P_desired)
K_MIMO = result.gain_matrix   # shape (4, 12)

print(f"place_poles converged in {result.nb_iter} iterations")
print(f"  residues (should be ~0): max = {np.max(np.abs(result.computed_poles - P_desired)):.2e}\n")

# ── Verify closed-loop eigenvalues ───────────────────────────────────────────
A_cl = A - B @ K_MIMO
eigs_cl = eigvals(A_cl)
print("Closed-loop eigenvalues:")
all_stable = True
for ev in sorted(eigs_cl, key=lambda e: e.real):
    flag = "" if ev.real < 0 else "  *** UNSTABLE ***"
    print(f"  {ev.real:+.4f} {ev.imag:+.4f}j{flag}")
    if ev.real >= 0:
        all_stable = False
print(f"  -> {'STABLE' if all_stable else 'UNSTABLE'}\n")

# ── Bandwidth check ───────────────────────────────────────────────────────────
bw = np.max(np.abs(eigs_cl))
print(f"Max closed-loop |eigenvalue| = {bw:.2f} rad/s  (Nyquist at 50 Hz = 157 rad/s)\n")

# ── Compare with decoupled K ─────────────────────────────────────────────────
K_DECOUPLED = np.array([
    [0, 0,               0.7240500000000, 0, 0,               0.3600000000000,
     0,               0,              0,              0,               0,              0          ],
    [0, -0.0120187650357, 0,              0, -0.0108197145770, 0,
     0.0305354550000, 0,              0,              0.0027723500000, 0,              0          ],
    [0.0206417471967, 0,  0,              0.0185824260958, 0, 0,
     0,               0.0524434200000, 0,              0,               0.0047614000000, 0        ],
    [0, 0,               0,              0, 0,               0,
     0,               0,              0.0006651153000, 0,               0,              0.0004390200000],
])

state_labels = ['x','y','z','vx','vy','vz','phi','theta','psi','phid','thd','psid']
input_labels = ['dT ','t_phi','t_tht','t_psi']

print("=" * 70)
print("K_MIMO vs K_DECOUPLED — non-zero entries")
print("=" * 70)
for i in range(4):
    for j in range(12):
        km = K_MIMO[i, j]
        kd = K_DECOUPLED[i, j]
        if abs(km) > 1e-10 or abs(kd) > 1e-10:
            ratio = f"  ratio={km/kd:.3f}" if abs(kd) > 1e-10 else ""
            print(f"  K[{input_labels[i]}][{state_labels[j]:5s}]:  "
                  f"decoupled={kd:+.10f}   mimo={km:+.10f}{ratio}")
print()

# ── Saturation thresholds ─────────────────────────────────────────────────────
print("Saturation thresholds (MIMO):")
print(f"  y_err  → tau_phi  sat at |y|  > {TAU_RP_MAX / abs(K_MIMO[1,1]):.2f} m")
print(f"  x_err  → tau_tht  sat at |x|  > {TAU_RP_MAX / abs(K_MIMO[2,0]):.2f} m")
print(f"  phi    → tau_phi  sat at |phi| > {np.degrees(TAU_RP_MAX / abs(K_MIMO[1,6])):.0f} deg")
print(f"  theta  → tau_tht  sat at |tht| > {np.degrees(TAU_RP_MAX / abs(K_MIMO[2,7])):.0f} deg")
print(f"  psi    → tau_psi  sat at |psi| > {np.degrees(TAU_PSI_MAX / abs(K_MIMO[3,8])):.0f} deg")
print(f"  z_err  → dT       sat at |z|   > {(T_MAX - T_HOVER) / K_MIMO[0,2]:.2f} m\n")

# ── Print C++ constant ────────────────────────────────────────────────────────
print("=" * 70)
print("C++ K_SF_MIMO for controller.hpp")
print("=" * 70)
print("static constexpr double K_SF_MIMO[4][12] = {")
for i in range(4):
    vals = ", ".join(f"{v:17.13f}" for v in K_MIMO[i])
    print(f"    {{ {vals} }},  // {input_labels[i]}")
print("};")
