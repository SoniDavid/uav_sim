import numpy as np
from quadrotor_sim_scripts.params import KT, KQ, ARM, OMEGA_MAX

# Mixer matrix: maps [ω1², ω2², ω3², ω4²] → [T, τ_φ, τ_θ, τ_ψ]
# Motor layout (X-config, viewed from above):
#   Motor 1: front-left  (-x, +y) CW   → negative yaw torque
#   Motor 2: front-right (+x, +y) CCW  → positive yaw torque
#   Motor 3: rear-right  (+x, -y) CW   → negative yaw torque
#   Motor 4: rear-left   (-x, -y) CCW  → positive yaw torque
A_MIX = np.array([
    [ KT,       KT,       KT,       KT      ],  # total thrust
    [-ARM*KT,   ARM*KT,   ARM*KT,  -ARM*KT  ],  # τ_φ (roll)
    [ ARM*KT,   ARM*KT,  -ARM*KT,  -ARM*KT  ],  # τ_θ (pitch)
    [-KQ,       KQ,      -KQ,       KQ      ],  # τ_ψ (yaw)
], dtype=float)

A_MIX_INV = np.linalg.inv(A_MIX)


def thrust_torques_to_omega_sq(T, tau_phi, tau_theta, tau_psi):
    """Inverse mixer: (T, τ_φ, τ_θ, τ_ψ) → individual ωᵢ² (clipped to valid range)."""
    v = np.array([T, tau_phi, tau_theta, tau_psi])
    omega_sq = A_MIX_INV @ v
    return np.clip(omega_sq, 0.0, OMEGA_MAX**2)


def omega_sq_to_omega_des(omega_sq):
    """Convert desired ωᵢ² to desired motor speeds (rad/s), clipped to [0, OMEGA_MAX]."""
    omega = np.sqrt(np.maximum(omega_sq, 0.0))
    return np.clip(omega, 0.0, OMEGA_MAX)


def compute_thrust_torques(omega):
    """Forward mixer: rotor speeds → (T, τ_φ, τ_θ, τ_ψ, T_i array)."""
    T_i = KT * omega**2
    Q_i = KQ * omega**2
    T         = np.sum(T_i)
    tau_phi   = ARM * (T_i[1] + T_i[2] - T_i[0] - T_i[3])
    tau_theta = ARM * (T_i[0] + T_i[1] - T_i[2] - T_i[3])
    tau_psi   = -Q_i[0] + Q_i[1] - Q_i[2] + Q_i[3]
    return T, tau_phi, tau_theta, tau_psi, T_i
