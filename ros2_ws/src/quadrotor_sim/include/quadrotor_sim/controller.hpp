#pragma once
#include <array>
#include <cassert>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include "quadrotor_sim/params.hpp"

namespace qsim {

// LQR gain matrix K (4×12) — discrete-time DARE at Ts=0.02 s (50 Hz).
// Design via docs/lqr_redesign.py: Q found numerically (per decoupled subsystem) so
// that DARE gives the same K as SF pole placement.  Same target poles as SF:
//   altitude -4±0.3j, y-roll outer -2.5±0.5j / inner -15±2j,
//   x-pitch outer -2.5±0.5j / inner -15±2j, yaw -3±0.3j.
// R (Bryson, per channel): [1/(ΔF_max)²=11.5, 1/(τ_RP_max)²=328, 328, 1/(τ_psi_max)²=44]
// Q diagonal (per subsystem):
//   altitude  : [7.167, 0.881]
//   y-roll    : [0.1263, 0.0369, 0.3170, 0.00262]
//   x-pitch   : [0.3726, 0.1090, 0.9352, 0.00773]
//   yaw       : [2.21e-5, 4.77e-6]
// Root cause of old failure: Q_psi=4/R_psi=46 → k_psi=0.295 (443× too large, yaw BW=63 rad/s).
//   New k_psi=0.000665 → yaw BW=3 rad/s, all channels below 33 rad/s (<<157 rad/s Nyquist).
// Saturation thresholds: y>4.6 m, x>2.7 m, ψ>12968°, z>0.41 m.
// State order: [x,y,z, vx,vy,vz, phi,theta,psi, phi_dot,theta_dot,psi_dot] (Euler rates)
static constexpr double K_LQR[4][12] = {
    {  0.0000000000000,  0.0000000000000,  0.7240500000000,  0.0000000000000,  0.0000000000000,  0.3600000000000,
       0.0000000000000,  0.0000000000000,  0.0000000000000,  0.0000000000000,  0.0000000000000,  0.0000000000000},
    {  0.0000000000000, -0.0120187650357,  0.0000000000000,  0.0000000000000, -0.0108197145770,  0.0000000000000,
       0.0305354550000,  0.0000000000000,  0.0000000000000,  0.0027723500000,  0.0000000000000,  0.0000000000000},
    {  0.0206417471967,  0.0000000000000,  0.0000000000000,  0.0185824260958,  0.0000000000000,  0.0000000000000,
       0.0000000000000,  0.0524434200000,  0.0000000000000,  0.0000000000000,  0.0047614000000,  0.0000000000000},
    {  0.0000000000000,  0.0000000000000,  0.0000000000000,  0.0000000000000,  0.0000000000000,  0.0000000000000,
       0.0000000000000,  0.0000000000000,  0.0006651153000,  0.0000000000000,  0.0000000000000,  0.0004390200000}
};

// State feedback (pole placement) gain matrix K (4×12) — designed per decoupled subsystem.
// Outer position poles -2.5±0.5j, inner attitude poles -15±2j, altitude -4±0.3j, yaw -3±0.3j.
// Gains sized so torque saturation only at position errors > 2.6 m (was 0.15 m with old gains).
// State order: [x,y,z, vx,vy,vz, phi,theta,psi, phi_dot,theta_dot,psi_dot] (Euler rates)
static constexpr double K_SF[4][12] = {
    {  0.0000000000000,  0.0000000000000,  0.7240500000000,  0.0000000000000,  0.0000000000000,  0.3600000000000,
       0.0000000000000,  0.0000000000000,  0.0000000000000,  0.0000000000000,  0.0000000000000,  0.0000000000000},
    {  0.0000000000000, -0.0120187650357,  0.0000000000000,  0.0000000000000, -0.0108197145770,  0.0000000000000,
       0.0305354550000,  0.0000000000000,  0.0000000000000,  0.0027723500000,  0.0000000000000,  0.0000000000000},
    {  0.0206417471967,  0.0000000000000,  0.0000000000000,  0.0185824260958,  0.0000000000000,  0.0000000000000,
       0.0000000000000,  0.0524434200000,  0.0000000000000,  0.0000000000000,  0.0047614000000,  0.0000000000000},
    {  0.0000000000000,  0.0000000000000,  0.0000000000000,  0.0000000000000,  0.0000000000000,  0.0000000000000,
       0.0000000000000,  0.0000000000000,  0.0006651153000,  0.0000000000000,  0.0000000000000,  0.0004390200000}
};

// Cascaded PD gains
constexpr double KP_XY  = 2.0;
constexpr double KD_XY  = 1.5;
constexpr double KP_Z   = 4.0;
constexpr double KD_Z   = 3.0;
constexpr double KP_ATT = 300.0;
constexpr double KD_ATT = 30.0;
constexpr double KP_PSI = 100.0;
constexpr double KD_PSI = 15.0;

// Cascaded nonlinear PD controller with tilt compensation
// s[0:3]=pos, s[3:6]=vel, s[6:9]=euler, s[9:12]=body_rates
// r[0:3]=pos_ref, r[3:6]=vel_ref, r[6:9]=euler_ref
// Returns: [T_total, tau_phi, tau_theta, tau_psi]
inline std::array<double, 4> compute_cascaded_control(const double* s, const double* r)
{
    const double x=s[0], y=s[1], z=s[2];
    const double xd=s[3], yd=s[4], zd=s[5];
    const double phi=s[6], theta=s[7], psi=s[8];
    const double phid=s[9], thetad=s[10], psid=s[11];

    const double x_r=r[0], y_r=r[1], z_r=r[2];
    const double xd_r=r[3], yd_r=r[4], zd_r=r[5];
    const double psi_r=r[8];

    const double ax_d = KP_XY * (x_r - x) + KD_XY * (xd_r - xd);
    const double ay_d = KP_XY * (y_r - y) + KD_XY * (yd_r - yd);
    const double az_d = KP_Z  * (z_r - z) + KD_Z  * (zd_r - zd);

    const double cos_tilt = std::max(std::cos(phi) * std::cos(theta), 0.1);
    const double T_total  = std::clamp(MASS * (az_d + G) / cos_tilt, 0.0, T_MAX);

    const double T_ref    = std::max(T_total, 0.01);
    const double theta_d  = std::asin(std::clamp(MASS * ax_d / T_ref, -0.5, 0.5));
    const double phi_d    = std::asin(std::clamp(-MASS * ay_d / (T_ref * std::cos(theta_d)), -0.5, 0.5));

    // Convert body rates [p,q,r] → Euler rates [phi_dot, theta_dot, psi_dot]
    const double tt         = std::tan(theta);
    const double cos_phi    = std::cos(phi);
    const double sin_phi    = std::sin(phi);
    const double phi_dot    = phid + sin_phi * tt * thetad + cos_phi * tt * psid;
    const double theta_dot  = cos_phi * thetad - sin_phi * psid;
    const double cos_theta  = std::cos(theta);
    const double psi_dot    = std::abs(cos_theta) > 0.035  // guard: |theta| < ~88°
                              ? (sin_phi * thetad + cos_phi * psid) / cos_theta
                              : 0.0;

    const double tau_phi   = IXX * (KP_ATT * (phi_d   - phi)   - KD_ATT * phi_dot);
    const double tau_theta = IYY * (KP_ATT * (theta_d - theta) - KD_ATT * theta_dot);
    const double tau_psi   = IZZ * (KP_PSI * (psi_r   - psi)   - KD_PSI * psi_dot);

    return {T_total, tau_phi, tau_theta, tau_psi};
}

// LQR feedback: u = T_hover - K*(x_euler - x_ref), with tilt compensation on thrust.
// Body rates (p,q,r) from the dynamics are converted to Euler rates before forming
// the error, matching the linearization assumption in which state[9:12] = Euler rates.
// Torques are clamped to physical actuator limits to prevent saturation-induced divergence.
inline std::array<double, 4> compute_lqr_control(const double* s, const double* r)
{
    // Convert body rates s[9:12]=(p,q,r) → Euler rates (phi_dot, theta_dot, psi_dot)
    const double phi   = s[6], theta = s[7];
    const double p = s[9], q = s[10], rb = s[11];
    const double sp = std::sin(phi), cp = std::cos(phi);
    const double ct = std::cos(theta);
    const double tt = std::tan(theta);
    const double phi_dot   = p + sp * tt * q + cp * tt * rb;
    const double theta_dot = cp * q - sp * rb;
    const double psi_dot   = std::abs(ct) > 0.035 ? (sp * q + cp * rb) / ct : 0.0;

    double err[12];
    for (int i = 0; i < 9;  ++i) err[i] = s[i] - r[i];
    err[9]  = phi_dot   - r[9];
    err[10] = theta_dot - r[10];
    err[11] = psi_dot   - r[11];

    double delta_u[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 12; ++j)
            delta_u[i] -= K_LQR[i][j] * err[j];

    // Clamp torques to physical actuator limits
    constexpr double TAU_RP_MAX = ARM * T_MAX;           // roll/pitch ceiling ≈ 0.055 Nm
    constexpr double TAU_PSI_MAX = 4.0 * KQ * OMEGA_MAX * OMEGA_MAX; // yaw ceiling ≈ 0.150 Nm
    delta_u[1] = std::clamp(delta_u[1], -TAU_RP_MAX,  TAU_RP_MAX);
    delta_u[2] = std::clamp(delta_u[2], -TAU_RP_MAX,  TAU_RP_MAX);
    delta_u[3] = std::clamp(delta_u[3], -TAU_PSI_MAX, TAU_PSI_MAX);

    const double cos_tilt = std::max(cp * ct, 0.1);
    const double T = std::clamp((T_HOVER + delta_u[0]) / cos_tilt, 0.0, T_MAX);

    return {T, delta_u[1], delta_u[2], delta_u[3]};
}

// State feedback controller: u = T_hover - K_SF*(x_euler - x_ref), with tilt compensation.
// K_SF multiply uses Eigen for clarity; the scalar loop is kept for cross-check in debug builds.
inline std::array<double, 4> compute_sf_control(const double* s, const double* r)
{
    const double phi   = s[6], theta = s[7];
    const double p = s[9], q = s[10], rb = s[11];
    const double sp = std::sin(phi), cp = std::cos(phi);
    const double ct = std::cos(theta);
    const double tt = std::tan(theta);
    const double phi_dot   = p + sp * tt * q + cp * tt * rb;
    const double theta_dot = cp * q - sp * rb;
    const double psi_dot   = std::abs(ct) > 0.035 ? (sp * q + cp * rb) / ct : 0.0;

    Eigen::Matrix<double, 12, 1> err;
    for (int i = 0; i < 9; ++i) err(i) = s[i] - r[i];
    err(9)  = phi_dot   - r[9];
    err(10) = theta_dot - r[10];
    err(11) = psi_dot   - r[11];

    // K_SF as a fixed-size Eigen matrix (row-major from the constexpr array)
    using K4x12 = Eigen::Matrix<double, 4, 12, Eigen::RowMajor>;
    const Eigen::Map<const K4x12> K(K_SF[0]);

    const Eigen::Vector4d delta_u_vec = -(K * err);

#ifndef NDEBUG
    // Verify Eigen result matches scalar loop to machine precision
    double delta_u_loop[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 12; ++j)
            delta_u_loop[i] -= K_SF[i][j] * err(j);
    for (int i = 0; i < 4; ++i)
        assert(std::abs(delta_u_vec(i) - delta_u_loop[i]) < 1e-12);
#endif

    constexpr double TAU_RP_MAX  = ARM * T_MAX;
    constexpr double TAU_PSI_MAX = 4.0 * KQ * OMEGA_MAX * OMEGA_MAX;
    const double du1 = std::clamp(delta_u_vec(1), -TAU_RP_MAX,  TAU_RP_MAX);
    const double du2 = std::clamp(delta_u_vec(2), -TAU_RP_MAX,  TAU_RP_MAX);
    const double du3 = std::clamp(delta_u_vec(3), -TAU_PSI_MAX, TAU_PSI_MAX);

    const double cos_tilt = std::max(cp * ct, 0.1);
    const double T = std::clamp((T_HOVER + delta_u_vec(0)) / cos_tilt, 0.0, T_MAX);

    return {T, du1, du2, du3};
}

} // namespace qsim
