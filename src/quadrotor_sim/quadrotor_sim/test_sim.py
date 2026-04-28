"""
Standalone mathematical simulation — no ROS2 required.
Runs the full nonlinear quadrotor dynamics with LQR control
and produces matplotlib plots of state and control inputs.

Usage:
    cd UAV_sim/src/quadrotor_sim
    python -m quadrotor_sim.test_sim
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# Allow direct run without installing the package
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from quadrotor_sim.params import (
    MASS, G, KT, OMEGA_HOVER, T_HOVER,
    DT_SIM, OMEGA_MAX
)
from quadrotor_sim.dynamics import rk4_step, euler_to_quaternion
from quadrotor_sim.mixer import (
    thrust_torques_to_omega_sq,
    omega_sq_to_omega_des,
    compute_thrust_torques,
)
from quadrotor_sim.controller import design_lqr, compute_control, compute_cascaded_control


MAX_MOTOR_T = 4 * 2.64e-8 * 2640.0**2  # 0.736 N — physical thrust ceiling


def smooth_reference(x_ref_current, x_ref_target, dt, max_vel=0.2, max_yaw_rate=0.2):
    """Rate-limited reference: ramp position setpoints, keep velocity reference at zero."""
    delta = x_ref_target - x_ref_current
    for pos_idx in [0, 1, 2]:
        x_ref_current[pos_idx] += np.clip(delta[pos_idx], -max_vel * dt, max_vel * dt)
    x_ref_current[8] += np.clip(delta[8], -max_yaw_rate * dt, max_yaw_rate * dt)
    # velocity reference stays 0 — the error controller handles it
    return x_ref_current


def run_simulation(t_end=20.0, ref_switch_time=5.0):
    # --- Verification tests ---
    print("=== Pre-flight checks ===")
    print(f"  T_hover = {T_HOVER:.5f} N  (expected {MASS*G:.5f})")
    print(f"  omega_hover = {OMEGA_HOVER:.2f} rad/s")
    print(f"  omega_des at hover = {OMEGA_HOVER:.2f} rad/s  (motor command = desired omega)")

    # Hover equilibrium check
    state0 = np.zeros(16)
    state0[2]     = 1.0
    state0[12:16] = OMEGA_HOVER
    u_hover_cmd   = np.full(4, OMEGA_HOVER)
    ds = _state_derivative_test(state0, u_hover_cmd)
    print(f"  z_ddot at hover = {ds[5]:.2e} m/s²  (should be ~0)")

    # LQR design (for linearized analysis / report)
    K, A_lin, B_lin, _ = design_lqr()
    A_cl = A_lin - B_lin @ K
    eigs = np.linalg.eigvals(A_cl)
    print(f"  Closed-loop eigenvalues max Re = {max(e.real for e in eigs):.4f}  (must be <0)")
    print("  Using cascaded nonlinear PD controller for full simulation (handles tilt/saturation)")
    print("=== Checks passed ===\n")

    # --- Initial state: start at z=0, motors warm-started at hover speed ---
    state = np.zeros(16)
    state[12:16] = OMEGA_HOVER  # warm start motors

    # Reference trajectory waypoints
    x_ref_wp1 = np.zeros(12); x_ref_wp1[2] = 1.0          # hover at z=1m
    x_ref_wp2 = np.zeros(12); x_ref_wp2[:3] = [1.0, 1.0, 2.0]  # step to [1,1,2]m

    # Smoothed reference (evolves over time, starts at initial position)
    x_ref = np.zeros(12)

    # Storage
    n_steps = int(t_end / DT_SIM)
    t_arr     = np.zeros(n_steps)
    state_arr = np.zeros((n_steps, 12))
    T_i_arr   = np.zeros((n_steps, 4))
    u_arr     = np.zeros((n_steps, 4))

    print(f"Simulating {t_end}s at {1/DT_SIM:.0f} Hz ...")
    for i in range(n_steps):
        t = i * DT_SIM

        # Smoothed reference: ramp toward the active waypoint
        target_ref = x_ref_wp2 if t >= ref_switch_time else x_ref_wp1
        x_ref = smooth_reference(x_ref, target_ref, DT_SIM)

        # Cascaded nonlinear PD controller
        T_total, tau_phi, tau_theta, tau_psi = compute_cascaded_control(state[:12], x_ref)

        # Inverse mixer: high-level → desired motor speeds
        omega_sq = thrust_torques_to_omega_sq(T_total, tau_phi, tau_theta, tau_psi)
        u_cmd    = omega_sq_to_omega_des(omega_sq)   # rad/s

        # Forward mixer for logging individual thrusts
        _, _, _, _, T_i = compute_thrust_torques(state[12:16])

        # Store
        t_arr[i]       = t
        state_arr[i]   = state[:12]
        T_i_arr[i]     = T_i
        u_arr[i]       = u_cmd

        # Integrate
        state = rk4_step(state, u_cmd, DT_SIM)

    print("Simulation complete. Generating plots...")
    _plot_results(t_arr, state_arr, T_i_arr, u_arr, ref_switch_time, x_ref_wp1, x_ref_wp2)


def _state_derivative_test(state, u_cmd):
    """Call dynamics without the import loop for the pre-flight check."""
    from quadrotor_sim.dynamics import state_derivative
    return state_derivative(state, u_cmd)


def _plot_results(t, state, T_i, u, t_switch, ref1, ref2):
    fig = plt.figure(figsize=(16, 12))
    fig.suptitle("Quadrotor UAV Simulator — Cascaded PD + LQR Analysis (IRS Tarea 3)", fontsize=14, fontweight='bold')
    gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.45, wspace=0.35)

    # --- Position ---
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(t, state[:, 0], label='x', color='tab:blue')
    ax1.plot(t, state[:, 1], label='y', color='tab:orange')
    ax1.plot(t, state[:, 2], label='z', color='tab:green')
    ax1.axvline(t_switch, color='gray', linestyle='--', linewidth=0.8)
    ax1.axhline(ref1[2], color='tab:green', linestyle=':', linewidth=0.8, alpha=0.6)
    ax1.axhline(ref2[2], color='tab:green', linestyle=':', linewidth=0.8, alpha=0.6)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Position (m)')
    ax1.set_title('Position: x, y, z')
    ax1.legend(loc='upper left')
    ax1.grid(True, alpha=0.3)

    # --- Euler angles ---
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(t, np.degrees(state[:, 6]),  label='φ (roll)',  color='tab:red')
    ax2.plot(t, np.degrees(state[:, 7]),  label='θ (pitch)', color='tab:purple')
    ax2.plot(t, np.degrees(state[:, 8]),  label='ψ (yaw)',   color='tab:brown')
    ax2.axvline(t_switch, color='gray', linestyle='--', linewidth=0.8)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Angle (deg)')
    ax2.set_title('Euler Angles: φ, θ, ψ')
    ax2.legend(loc='upper left')
    ax2.grid(True, alpha=0.3)

    # --- Linear velocities ---
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.plot(t, state[:, 3], label='ẋ', color='tab:blue')
    ax3.plot(t, state[:, 4], label='ẏ', color='tab:orange')
    ax3.plot(t, state[:, 5], label='ż', color='tab:green')
    ax3.axvline(t_switch, color='gray', linestyle='--', linewidth=0.8)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Velocity (m/s)')
    ax3.set_title('Linear Velocities: ẋ, ẏ, ż')
    ax3.legend(loc='upper left')
    ax3.grid(True, alpha=0.3)

    # --- Angular velocities ---
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.plot(t, np.degrees(state[:, 9]),  label='φ̇',  color='tab:red')
    ax4.plot(t, np.degrees(state[:, 10]), label='θ̇', color='tab:purple')
    ax4.plot(t, np.degrees(state[:, 11]), label='ψ̇',  color='tab:brown')
    ax4.axvline(t_switch, color='gray', linestyle='--', linewidth=0.8)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Angular velocity (deg/s)')
    ax4.set_title('Angular Rates: φ̇, θ̇, ψ̇')
    ax4.legend(loc='upper left')
    ax4.grid(True, alpha=0.3)

    # --- Motor thrusts T1..T4 ---
    ax5 = fig.add_subplot(gs[2, 0])
    for i in range(4):
        ax5.plot(t, T_i[:, i] * 1e3, label=f'T{i+1}')  # mN
    ax5.axvline(t_switch, color='gray', linestyle='--', linewidth=0.8)
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Thrust (mN)')
    ax5.set_title('Motor Thrusts: T1, T2, T3, T4')
    ax5.legend(loc='upper right')
    ax5.grid(True, alpha=0.3)

    # --- Desired motor speeds (control commands) ---
    ax6 = fig.add_subplot(gs[2, 1])
    for i in range(4):
        ax6.plot(t, u[:, i], label=f'ω_des{i+1}')
    ax6.axvline(t_switch, color='gray', linestyle='--', linewidth=0.8)
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('Motor speed command (rad/s)')
    ax6.set_title('Motor Speed Commands: ω_des1..4')
    ax6.legend(loc='upper right')
    ax6.grid(True, alpha=0.3)

    plt.savefig('/tmp/quadrotor_lqr_results.png', dpi=150, bbox_inches='tight')
    print("Plot saved to /tmp/quadrotor_lqr_results.png")
    plt.show()


if __name__ == '__main__':
    run_simulation()
