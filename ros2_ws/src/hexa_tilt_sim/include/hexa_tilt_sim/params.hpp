#pragma once
#include <cmath>

namespace hTsim {

// ── Physical parameters (Table II, FxTDiff_PD.pdf) ────────────────────────────
constexpr double MASS     = 1.74;        // kg  (FPVDrone S550)
constexpr double G        = 9.81;        // m/s²
constexpr double KT       = 7.86e-6;     // N/(rad/s)²
constexpr double KQ       = 1.06e-7;     // N·m/(rad/s)²

// Inertia tensor (diagonal, kg·m²)
constexpr double JXX      = 0.03;
constexpr double JYY      = 0.03;
constexpr double JZZ      = 0.048;

// Motor first-order time constant (s)
constexpr double TM       = 0.01;

// Motor speed limits (rad/s)
constexpr double OMEGA_MAX   = 1000.0;
constexpr double OMEGA_HOVER = 620.0;    // approx. hover per motor

// Simulation and publish rates
constexpr double DT_SIM    = 0.001;      // 1 kHz dynamics
constexpr double DT_PUB    = 0.02;       // 50 Hz kinematics/viz
constexpr double CTRL_FREQ = 250.0;     // Original hardware frequency
// constexpr double CTRL_FREQ = 1000.0;    // Testing frequency
constexpr double DT_CTRL   = 1.0/CTRL_FREQ; 

// ── Motor allocation matrix M_φ (6×6, Eq. 10, scaled 1e-5) ────────────────────
// Rows: [f_x, f_y, f_z, τ_φ, τ_θ, τ_ψ]
// Cols: motors 1–6
// M_phi[i][j] maps Ω_j² → force/torque component i
static constexpr double M_PHI[6][6] = {
    {-0.23e-5,  0.23e-5,  0.00e-5, -0.23e-5,  0.23e-5,  0.00e-5},
    { 0.13e-5,  0.13e-5, -0.27e-5,  0.13e-5,  0.13e-5, -0.27e-5},
    { 0.74e-5,  0.74e-5,  0.74e-5,  0.74e-5,  0.74e-5,  0.74e-5},
    { 0.17e-5,  0.17e-5,  0.00e-5, -0.17e-5, -0.17e-5,  0.00e-5},
    {-0.10e-5,  0.10e-5,  0.20e-5,  0.10e-5, -0.10e-5, -0.20e-5},
    { 0.08e-5, -0.08e-5,  0.08e-5, -0.08e-5,  0.08e-5, -0.08e-5}
};

// Precomputed inverse M_φ⁻¹ (computed offline via numpy, verified M*M⁻¹=I)
// Rows: motors 1–6; cols: [f_x, f_y, f_z, τ_φ, τ_θ, τ_ψ]
// Near-zero entries (< 1e-8) set to 0.0 for numerical cleanliness
static constexpr double M_PHI_INV[6][6] = {
    {-1.0869565217e+05,  6.2500000000e+04,  2.2804054054e+04,  1.4705882353e+05, -8.3333333333e+04,  2.0833333333e+05},
    { 1.0869565217e+05,  6.2500000000e+04,  2.2804054054e+04,  1.4705882353e+05,  8.3333333333e+04, -2.0833333333e+05},
    { 0.0000000000e+00, -1.2500000000e+05,  2.1959459459e+04,  0.0000000000e+00,  1.6666666667e+05,  2.0833333333e+05},
    {-1.0869565217e+05,  6.2500000000e+04,  2.2804054054e+04, -1.4705882353e+05,  8.3333333333e+04, -2.0833333333e+05},
    { 1.0869565217e+05,  6.2500000000e+04,  2.2804054054e+04, -1.4705882353e+05, -8.3333333333e+04,  2.0833333333e+05},
    { 0.0000000000e+00, -1.2500000000e+05,  2.1959459459e+04,  0.0000000000e+00, -1.6666666667e+05, -2.0833333333e+05}
};

// Derived constants
inline const double T_HOVER = MASS * G;
inline const double T_MAX   = 6.0 * 0.74e-5 * OMEGA_MAX * OMEGA_MAX; // z-channel only

// ── FxT Differentiator gains (Table I, FxTDiff_PD.pdf) ────────────────────────
// Applied per axis: x, y, z, φ, θ, ψ
constexpr double DIFF_G1[6]  = {5.0,  5.0,  5.0,  5.0,  5.0,  5.0};
constexpr double DIFF_G2[6]  = {6.0,  6.0, 10.0, 16.0, 16.0, 14.0};
constexpr double DIFF_Q1[6]  = {5.0,  5.0,  5.0,  5.0,  5.0,  5.0};
constexpr double DIFF_Q2[6]  = {6.0,  6.0, 10.0, 16.0, 16.0, 14.0};
constexpr double DIFF_W1     = 0.6;   // ϖ₁ ∈ (0,1)
constexpr double DIFF_RHO1   = 1.2;   // ϱ₁ > 1
constexpr double DIFF_W2     = 2.0 * DIFF_W1   - 1.0;   // = 0.2
constexpr double DIFF_RHO2   = 2.0 * DIFF_RHO1 - 1.0;   // = 1.4

// ── PD controller gains (Table I, FxTDiff_PD.pdf) ─────────────────────────────
// Applied per axis: x, y, z, φ, θ, ψ
constexpr double KP[6] = { 6.0,  6.0, 20.0, 15.0, 12.0, 20.0};
constexpr double KD[6] = { 6.0,  6.0, 10.0, 12.0, 12.0, 10.0};

} // namespace hTsim
