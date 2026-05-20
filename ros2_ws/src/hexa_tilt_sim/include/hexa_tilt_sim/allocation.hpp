#pragma once
#include <array>
#include <cmath>
#include <algorithm>
#include "hexa_tilt_sim/params.hpp"

namespace hTsim {

// Inverse allocation: desired 6D wrench → desired squared motor speeds.
// Ω²_des[i] = sum_j  M_PHI_INV[i][j] * F_des[j]
// Saturates to [0, OMEGA_MAX²] before sqrt.
inline std::array<double, 6> allocate_motor_speeds(const double F_des[6])
{
    constexpr double omega_max_sq = OMEGA_MAX * OMEGA_MAX;
    std::array<double, 6> omega_sq;
    for (int i = 0; i < 6; ++i) {
        double s = 0.0;
        for (int j = 0; j < 6; ++j)
            s += M_PHI_INV[i][j] * F_des[j];
        omega_sq[i] = std::clamp(s, 0.0, omega_max_sq);
    }
    return omega_sq;
}

inline std::array<double, 6> omega_sq_to_omega_des(const std::array<double, 6>& osq)
{
    std::array<double, 6> out;
    for (int i = 0; i < 6; ++i)
        out[i] = std::clamp(std::sqrt(std::max(osq[i], 0.0)), 0.0, OMEGA_MAX);
    return out;
}

// Forward model: current motor speeds → wrench (for kinematics node visualization)
inline void compute_wrench(const double* omega6, double F_out[6])
{
    for (int i = 0; i < 6; ++i) {
        F_out[i] = 0.0;
        for (int j = 0; j < 6; ++j)
            F_out[i] += M_PHI[i][j] * (omega6[j] * omega6[j]);
    }
}

// Hover motor speeds (from M_PHI_INV @ [0, 0, m*g, 0, 0, 0]ᵀ)
inline std::array<double, 6> hover_motor_speeds()
{
    double F[6] = {0.0, 0.0, MASS * G, 0.0, 0.0, 0.0};
    auto osq = allocate_motor_speeds(F);
    return omega_sq_to_omega_des(osq);
}

} // namespace hTsim
