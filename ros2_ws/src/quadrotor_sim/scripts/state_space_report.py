"""
State-space system report — equivalent to the MATLAB m-file.

Prints the linearized A, B, C, D matrices with state labels, computes
the LQR gain matrix K, verifies closed-loop stability, and generates a
figure containing the state-space block diagram and the step response.

Usage (from the ros2_ws directory after sourcing install/setup.bash):
    python3 -m quadrotor_sim_scripts.state_space_report

Output: /tmp/state_space_report.png
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

from quadrotor_sim_scripts.params import (
    MASS, G, IXX, IYY, IZZ, T_HOVER, KT, OMEGA_MAX, KQ, ARM, DT_SIM, OMEGA_HOVER
)
from quadrotor_sim_scripts.controller import build_hover_linear_model, design_lqr
from quadrotor_sim_scripts.dynamics import rk4_step
from quadrotor_sim_scripts.mixer import thrust_torques_to_omega_sq, omega_sq_to_omega_des

STATE_LABELS = [
    'x  (m)',    'y  (m)',    'z  (m)',
    'ẋ  (m/s)', 'ẏ  (m/s)', 'ż  (m/s)',
    'φ  (rad)', 'θ  (rad)', 'ψ  (rad)',
    'φ̇ (r/s)', 'θ̇ (r/s)', 'ψ̇ (r/s)',
]
INPUT_LABELS = ['δT  (N)', 'τ_φ (N·m)', 'τ_θ (N·m)', 'τ_ψ (N·m)']


def print_matrix(name, M, row_labels=None, col_labels=None, fmt='+9.4f'):
    n, m = M.shape
    print(f"\n{'─'*60}")
    print(f"  {name}  ({n}×{m})")
    print(f"{'─'*60}")
    if col_labels:
        header = ' ' * 14 + ''.join(f'{l:>12}' for l in col_labels)
        print(header)
    for i, row in enumerate(M):
        lbl = f'  {row_labels[i]:12}' if row_labels else f'  row {i:2d}       '
        vals = ''.join(f'{v:{fmt}}' for v in row)
        print(lbl + vals)


def run():
    print("=" * 60)
    print("  QUADROTOR STATE-SPACE REPORT  —  IRS Tarea 3")
    print("=" * 60)
    print(f"\n  m  = {MASS:.4f} kg        g  = {G:.4f} m/s²")
    print(f"  Ixx= {IXX:.2e} kg·m²   Iyy= {IYY:.2e} kg·m²   Izz= {IZZ:.2e} kg·m²")
    print(f"  kT = {KT:.2e} N/(r/s)² kQ = {KQ:.2e} N·m/(r/s)²")
    print(f"  ARM= {ARM:.4f} m        T_hover = {T_HOVER:.4f} N")
    print(f"  ω_hover = {OMEGA_HOVER:.2f} rad/s     ω_max = {OMEGA_MAX:.1f} rad/s")
    T_MAX = 4 * KT * OMEGA_MAX**2
    print(f"  T_max   = {T_MAX:.4f} N")

    print("\n\n  Linearization point: hover  (φ=θ=ψ=0, T = mg)")
    print("  State:  x = [x y z  ẋ ẏ ż  φ θ ψ  φ̇ θ̇ ψ̇]ᵀ  (12×1)")
    print("  Input:  u = [δT  τ_φ  τ_θ  τ_ψ]ᵀ             (4×1)")
    print("  Output: y = x  (full state feedback, C = I₁₂)")

    A, B = build_hover_linear_model()
    C = np.eye(12)
    D = np.zeros((12, 4))

    print_matrix("A  —  system matrix", A,
                 row_labels=[l.split()[0] for l in STATE_LABELS],
                 col_labels=[l.split()[0] for l in STATE_LABELS])
    print_matrix("B  —  input matrix", B,
                 row_labels=[l.split()[0] for l in STATE_LABELS],
                 col_labels=['δT', 'τ_φ', 'τ_θ', 'τ_ψ'])

    eigs_open = np.linalg.eigvals(A)
    print(f"\n  Open-loop eigenvalues of A:")
    for i, e in enumerate(sorted(eigs_open, key=lambda x: x.real)):
        marker = '⚠ UNSTABLE' if e.real > 0 else ''
        print(f"    λ{i+1:02d} = {e.real:+8.4f} {'+' if e.imag>=0 else '-'} {abs(e.imag):.4f}j  {marker}")
    print("  → Open-loop system has poles at origin (marginally stable)")

    K, _, _, P = design_lqr()
    A_cl = A - B @ K
    eigs_cl = np.linalg.eigvals(A_cl)

    print_matrix("K  —  LQR gain matrix  (4×12)", K,
                 row_labels=['δT', 'τ_φ', 'τ_θ', 'τ_ψ'],
                 col_labels=[l.split()[0] for l in STATE_LABELS],
                 fmt='+9.3f')

    print(f"\n  Q diagonal (Bryson's rule):")
    q = [4, 4, 4, 4, 4, 4, 14, 14, 4, 3.7, 3.7, 3.7]
    for lbl, v in zip(STATE_LABELS, q):
        print(f"    Q[{lbl}] = {v}")
    r = [11.5, 330.0, 330.0, 46.0]
    print(f"\n  R diagonal (Bryson's rule):")
    for lbl, v in zip(INPUT_LABELS, r):
        print(f"    R[{lbl}] = {v}")

    print(f"\n  Closed-loop eigenvalues of  A - B·K:")
    all_stable = True
    for i, e in enumerate(sorted(eigs_cl, key=lambda x: x.real)):
        if e.real >= 0:
            all_stable = False
        marker = '⚠ UNSTABLE' if e.real >= 0 else '✓'
        print(f"    λ{i+1:02d} = {e.real:+8.4f} {'+' if e.imag>=0 else '-'} {abs(e.imag):.4f}j  {marker}")
    if all_stable:
        print("  → Closed-loop system is STABLE (all Re(λ) < 0) ✓")
    else:
        print("  → WARNING: closed-loop system is UNSTABLE!")

    print("\n  Simulating step response: z: 0 m → 1 m ...")
    state = np.zeros(16)
    state[12:16] = OMEGA_HOVER
    x_ref = np.zeros(12); x_ref[2] = 1.0

    t_end, dt = 8.0, DT_SIM
    n = int(t_end / dt)
    t_arr = np.linspace(0, t_end, n)
    z_arr = np.zeros(n)
    phi_arr = np.zeros(n)
    T_arr = np.zeros(n)

    from quadrotor_sim_scripts.controller import compute_cascaded_control
    for i in range(n):
        T, tau_phi, tau_theta, tau_psi = compute_cascaded_control(state[:12], x_ref)
        omega_sq = thrust_torques_to_omega_sq(T, tau_phi, tau_theta, tau_psi)
        u = omega_sq_to_omega_des(omega_sq)
        state = rk4_step(state, u, dt)
        z_arr[i] = state[2]
        phi_arr[i] = state[6]
        T_arr[i] = T

    tol = 0.02
    settled = np.where(np.abs(z_arr - 1.0) <= tol)[0]
    t_settle = t_arr[settled[0]] if len(settled) else float('nan')
    print(f"    Settling time (2%): {t_settle:.2f} s")
    print(f"    Max |φ| during step: {np.degrees(np.abs(phi_arr).max()):.3f}°")

    _plot(A, B, K, eigs_cl, t_arr, z_arr, phi_arr, T_arr, t_settle)


def _fmt_cell(v):
    if abs(v) < 1e-10:
        return '0'
    if abs(v) >= 1000:
        return f'{v:.0f}'
    if abs(v) >= 10:
        return f'{v:.1f}'
    if abs(v) >= 0.01:
        return f'{v:.3f}'
    return f'{v:.2e}'


def _matrix_heatmap(fig, gs_slot, M, row_labels, col_labels, title):
    import matplotlib.colors as mcolors

    ax = fig.add_subplot(gs_slot)
    ax.set_facecolor('#0d0d1a')

    nrows, ncols = M.shape
    nonzero = np.abs(M[M != 0])
    if len(nonzero) == 0:
        vmax = 1.0
        norm = mcolors.Normalize(vmin=-1, vmax=1)
    else:
        vmax = nonzero.max()
        vmin_nz = nonzero.min()
        if vmax / vmin_nz > 100:
            norm = mcolors.SymLogNorm(linthresh=vmin_nz, vmin=-vmax, vmax=vmax)
        else:
            norm = mcolors.Normalize(vmin=-vmax, vmax=vmax)

    im = ax.imshow(M, cmap='RdBu_r', norm=norm, aspect='auto')

    for r in range(nrows):
        for c in range(ncols):
            v = M[r, c]
            txt = _fmt_cell(v)
            norm_val = abs(norm(v) * 2 - 1)
            color = '#0d0d1a' if norm_val > 0.55 else 'white'
            fs = 6 if ncols > 6 else 7
            ax.text(c, r, txt, ha='center', va='center', fontsize=fs,
                    color=color, fontweight='bold')

    ax.set_xticks(range(ncols))
    ax.set_xticklabels(col_labels, rotation=45, ha='right', fontsize=6.5, color='white')
    ax.set_yticks(range(nrows))
    ax.set_yticklabels(row_labels, fontsize=6.5, color='white')
    ax.tick_params(colors='white', length=0)
    ax.spines[:].set_color('#555')
    ax.set_title(title, color='white', fontsize=9, pad=5)

    cb = plt.colorbar(im, ax=ax, fraction=0.03, pad=0.02)
    cb.ax.yaxis.set_tick_params(color='white', labelcolor='white')
    return ax


def _plot(A, B, K, eigs_cl, t, z, phi, T_thrust, t_settle):
    fig = plt.figure(figsize=(18, 11), facecolor='#1a1a2e')
    fig.suptitle(
        r'Quadrotor UAV — Espacio de Estados: $\dot{x}=Ax+Bu$,  $y=Cx$  |  IRS Tarea 3',
        fontsize=13, fontweight='bold', color='white', y=0.99)

    short_states = [l.split()[0] for l in STATE_LABELS]
    short_inputs = ['δT', 'τ_φ', 'τ_θ', 'τ_ψ']

    gs = gridspec.GridSpec(2, 2, figure=fig, hspace=0.55, wspace=0.35,
                           left=0.06, right=0.97, top=0.94, bottom=0.07,
                           width_ratios=[3, 1])

    _matrix_heatmap(fig, gs[0, 0], A, short_states, short_states,
                    'Matriz A  (12×12) — sistema:  ẋ = Ax + Bu')
    _matrix_heatmap(fig, gs[0, 1], B, short_states, short_inputs,
                    'Matriz B  (12×4) — entradas')

    ax1 = fig.add_subplot(gs[1, 0])
    ax1.set_facecolor('#0d0d1a')
    ax1.tick_params(colors='white'); ax1.spines[:].set_color('#555')
    for lbl in ax1.get_xticklabels() + ax1.get_yticklabels():
        lbl.set_color('white')
    ax1.plot(t, z, color='#2ecc71', linewidth=1.8, label='z(t)')
    ax1.axhline(1.0, color='white', linestyle='--', linewidth=0.9, alpha=0.6, label='z_ref = 1 m')
    ax1.axhline(0.98, color='gray', linestyle=':', linewidth=0.8)
    ax1.axhline(1.02, color='gray', linestyle=':', linewidth=0.8, label='±2% banda')
    if not np.isnan(t_settle):
        ax1.axvline(t_settle, color='#f39c12', linestyle='--', linewidth=1,
                    label=f't_s = {t_settle:.2f} s')
    ax1.set_xlabel('Tiempo (s)', color='white'); ax1.set_ylabel('Altitud z (m)', color='white')
    ax1.set_title('Respuesta al escalón: z(0)=0 → z_ref=1 m', color='white', fontsize=10)
    ax1.legend(fontsize=8, facecolor='#1a1a2e', labelcolor='white')
    ax1.grid(alpha=0.2, color='gray')

    ax2 = fig.add_subplot(gs[1, 1])
    ax2.set_facecolor('#0d0d1a')
    ax2.tick_params(colors='white'); ax2.spines[:].set_color('#555')
    for lbl in ax2.get_xticklabels() + ax2.get_yticklabels():
        lbl.set_color('white')
    ax2.axvline(0, color='white', linewidth=0.8, alpha=0.5)
    ax2.axhline(0, color='white', linewidth=0.8, alpha=0.5)
    ax2.fill_betweenx([-60, 60], 0, 2, alpha=0.08, color='red', label='Zona inestable')
    re = eigs_cl.real; im = eigs_cl.imag
    ax2.scatter(re, im, color='#2ecc71', s=60, zorder=4, label='Polos λᵢ (A−BK)')
    for r_, i_ in zip(re, im):
        ax2.annotate(f'{r_:.2f}', (r_, i_), textcoords='offset points',
                     xytext=(4, 3), fontsize=6, color='#aaddff')
    ax2.set_xlabel('Re(λ)', color='white'); ax2.set_ylabel('Im(λ)', color='white')
    ax2.set_title('Polos en lazo cerrado  A − B·K', color='white', fontsize=10)
    ax2.legend(fontsize=8, facecolor='#1a1a2e', labelcolor='white')
    ax2.grid(alpha=0.2, color='gray')
    margin = 1
    ax2.set_xlim(re.min() - margin, re.max() + margin)
    ax2.set_ylim(im.min() - margin, im.max() + margin)

    out = '/tmp/state_space_report.png'
    plt.savefig(out, dpi=150, bbox_inches='tight', facecolor='#1a1a2e')
    print(f"\n  Figure saved → {out}")
    plt.show()


if __name__ == '__main__':
    run()
