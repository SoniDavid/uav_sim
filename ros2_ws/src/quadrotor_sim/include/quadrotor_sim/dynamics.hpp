#pragma once
#include <array>
#include <cmath>
#include "quadrotor_sim/params.hpp"

namespace qsim {

using State = std::array<double, 16>;
using Ctrl  = std::array<double, 4>;

// Maps body angular rates [p, q, r] → Euler rates [phi_dot, theta_dot, psi_dot] (ZYX convention)
inline std::array<double, 3> euler_rates(double phi, double theta,
                                          double p, double q, double r)
{
    constexpr double GIMBAL_LIMIT = 88.0 * M_PI / 180.0;
    if (std::abs(theta) > GIMBAL_LIMIT)
        theta = std::copysign(GIMBAL_LIMIT, theta);

    const double sp = std::sin(phi), cp = std::cos(phi);
    const double st = std::sin(theta), ct = std::cos(theta);
    const double tt = st / ct;

    return {p + sp * tt * q + cp * tt * r,
            cp * q - sp * r,
            (sp * q + cp * r) / ct};
}

inline State state_derivative(const State& s, const Ctrl& u)
{
    const double xd  = s[3], yd  = s[4], zd  = s[5];
    const double phi = s[6], theta = s[7], psi = s[8];
    const double p   = s[9], q    = s[10], r  = s[11];
    const double w1  = s[12], w2  = s[13], w3 = s[14], w4 = s[15];

    // Motor first-order dynamics: omega_dot = (u - omega) / TM
    const double w1d = (u[0] - w1) / TM;
    const double w2d = (u[1] - w2) / TM;
    const double w3d = (u[2] - w3) / TM;
    const double w4d = (u[3] - w4) / TM;

    // Thrust and torque from rotor speeds (X-config)
    const double T1 = KT * w1*w1, T2 = KT * w2*w2;
    const double T3 = KT * w3*w3, T4 = KT * w4*w4;
    const double Q1 = KQ * w1*w1, Q2 = KQ * w2*w2;
    const double Q3 = KQ * w3*w3, Q4 = KQ * w4*w4;

    const double T_total   = T1 + T2 + T3 + T4;
    const double tau_phi   = ARM * (T2 + T3 - T1 - T4);
    const double tau_theta = ARM * (T1 + T2 - T3 - T4);
    const double tau_psi   = -Q1 + Q2 - Q3 + Q4;
    const double Omega_r   = w1 - w2 + w3 - w4;

    const double sp = std::sin(phi), cp = std::cos(phi);
    const double st = std::sin(theta), ct = std::cos(theta);
    const double spsi = std::sin(psi), cpsi = std::cos(psi);

    const double asc   = T_total / MASS;
    const double x_ddot = asc * (cp * st * cpsi + sp * spsi);
    const double y_ddot = asc * (cp * st * spsi - sp * cpsi);
    const double z_ddot = asc * (cp * ct) - G;

    const auto er = euler_rates(phi, theta, p, q, r);

    const double p_dot = ((IYY - IZZ) * q * r + JR * q * Omega_r + tau_phi)   / IXX;
    const double q_dot = ((IZZ - IXX) * p * r - JR * p * Omega_r + tau_theta) / IYY;
    const double r_dot = ((IXX - IYY) * p * q                    + tau_psi)   / IZZ;

    return {xd, yd, zd,
            x_ddot, y_ddot, z_ddot,
            er[0], er[1], er[2],
            p_dot, q_dot, r_dot,
            w1d, w2d, w3d, w4d};
}

inline State rk4_step(const State& s, const Ctrl& u, double dt)
{
    const auto k1 = state_derivative(s, u);

    State s2;
    for (int i = 0; i < 16; ++i) s2[i] = s[i] + 0.5 * dt * k1[i];
    const auto k2 = state_derivative(s2, u);

    State s3;
    for (int i = 0; i < 16; ++i) s3[i] = s[i] + 0.5 * dt * k2[i];
    const auto k3 = state_derivative(s3, u);

    State s4;
    for (int i = 0; i < 16; ++i) s4[i] = s[i] + dt * k3[i];
    const auto k4 = state_derivative(s4, u);

    State out;
    for (int i = 0; i < 16; ++i)
        out[i] = s[i] + (dt / 6.0) * (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);
    return out;
}

inline void euler_to_quaternion(double phi, double theta, double psi,
                                 double& qx, double& qy, double& qz, double& qw)
{
    const double cy = std::cos(psi/2),   sy = std::sin(psi/2);
    const double cp = std::cos(theta/2), sp = std::sin(theta/2);
    const double cr = std::cos(phi/2),   sr = std::sin(phi/2);
    qw = cr*cp*cy + sr*sp*sy;
    qx = sr*cp*cy - cr*sp*sy;
    qy = cr*sp*cy + sr*cp*sy;
    qz = cr*cp*sy - sr*sp*cy;
}

} // namespace qsim
