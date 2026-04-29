#pragma once
#include <array>
#include <cmath>
#include <algorithm>
#include "quadrotor_sim/params.hpp"

namespace qsim {

// LQR gain matrix K (4×12) — solved offline with scipy.linalg.solve_continuous_are
// Q = diag([4,4,4,4,4,4,14,14,4,3.7,3.7,3.7]),  R = diag([11.5,330,330,46])
// Control law: u = -K * (x - x_ref)  where u = [delta_T, tau_phi, tau_theta, tau_psi]
static constexpr double K_LQR[4][12] = {
    { 0,              0,              0.5897678246196, 0,              0,              0.6331707440906,
      0,              0,              0,               0,              0,              0             },
    { 0,             -0.1100963765126, 0,              0,             -0.1610125477337, 0,
      0.6149864371432, 0,             0,               0.1063463556663, 0,             0             },
    { 0.1100963765126, 0,             0,               0.1610714641041, 0,             0,
      0,               0.6158318544550, 0,             0,               0.1066755676951, 0            },
    { 0,              0,              0,               0,              0,              0,
      0,              0,              0.2948839123098, 0,              0,              0.2836863336864}
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

// LQR feedback: u = T_hover - K*(x - x_ref)
inline std::array<double, 4> compute_lqr_control(const double* s, const double* r)
{
    double err[12];
    for (int i = 0; i < 12; ++i) err[i] = s[i] - r[i];

    double delta_u[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 12; ++j)
            delta_u[i] -= K_LQR[i][j] * err[j];

    return {std::clamp(T_HOVER + delta_u[0], 0.0, T_MAX),
            delta_u[1], delta_u[2], delta_u[3]};
}

} // namespace qsim
