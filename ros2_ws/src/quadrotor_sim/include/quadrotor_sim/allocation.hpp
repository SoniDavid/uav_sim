#pragma once
#include <array>
#include <cmath>
#include <algorithm>
#include "quadrotor_sim/params.hpp"

namespace qsim {

// Control allocation matrix Γ (4×4).
// Maps the wrench vector [T, τ_φ, τ_θ, τ_ψ] to squared motor speeds [ω₁², ω₂², ω₃², ω₄²].
// This is the inverse of the forward effectiveness matrix Γ_fwd (ω² → wrench).
// Motor layout (X-config): 1=front-left CW, 2=front-right CCW, 3=rear-right CW, 4=rear-left CCW
static constexpr double GAMMA_INV[4][4] = {
    { 9.4696969697e+06, -1.2626262626e+08,  1.2626262626e+08, -4.6296296296e+07},
    { 9.4696969697e+06,  1.2626262626e+08,  1.2626262626e+08,  4.6296296296e+07},
    { 9.4696969697e+06,  1.2626262626e+08, -1.2626262626e+08, -4.6296296296e+07},
    { 9.4696969697e+06, -1.2626262626e+08, -1.2626262626e+08,  4.6296296296e+07}
};

// Control allocation: desired wrench [T, τ_φ, τ_θ, τ_ψ] → desired squared motor speeds [ω²].
// Applies Γ⁻¹ and saturates to [0, ω_max²].
inline std::array<double, 4> allocate_motor_speeds(double T, double tau_phi,
                                                     double tau_theta, double tau_psi)
{
    const double v[4] = {T, tau_phi, tau_theta, tau_psi};
    constexpr double omega_max_sq = OMEGA_MAX * OMEGA_MAX;
    std::array<double, 4> out;
    for (int i = 0; i < 4; ++i) {
        double s = 0.0;
        for (int j = 0; j < 4; ++j) s += GAMMA_INV[i][j] * v[j];
        out[i] = std::clamp(s, 0.0, omega_max_sq);
    }
    return out;
}

inline std::array<double, 4> omega_sq_to_omega_des(const std::array<double, 4>& osq)
{
    std::array<double, 4> out;
    for (int i = 0; i < 4; ++i)
        out[i] = std::clamp(std::sqrt(std::max(osq[i], 0.0)), 0.0, OMEGA_MAX);
    return out;
}

// Forward effectiveness model: individual motor speeds → wrench [T, τ_φ, τ_θ, τ_ψ].
inline void compute_wrench(const double* omega,
                            double& T, double& tau_phi,
                            double& tau_theta, double& tau_psi,
                            double* T_i_out = nullptr)
{
    double T_i[4], Q_i[4];
    for (int i = 0; i < 4; ++i) {
        T_i[i] = KT * omega[i] * omega[i];
        Q_i[i] = KQ * omega[i] * omega[i];
    }
    T         = T_i[0] + T_i[1] + T_i[2] + T_i[3];
    tau_phi   = ARM * (T_i[1] + T_i[2] - T_i[0] - T_i[3]);
    tau_theta = ARM * (T_i[0] + T_i[1] - T_i[2] - T_i[3]);
    tau_psi   = -Q_i[0] + Q_i[1] - Q_i[2] + Q_i[3];
    if (T_i_out)
        for (int i = 0; i < 4; ++i) T_i_out[i] = T_i[i];
}

} // namespace qsim
