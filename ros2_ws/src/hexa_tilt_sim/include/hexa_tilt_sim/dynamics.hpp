#pragma once
#include <array>
#include <cmath>
#include <algorithm>
#include "hexa_tilt_sim/params.hpp"

namespace hTsim {

// State:   [x,y,z, φ,θ,ψ,  u,v,w, p,q,r,  Ω₁..Ω₆]  (18 elements)
//           0 1 2  3 4 5   6 7 8  9 10 11  12..17
// Control: [Ω_des₁..Ω_des₆]                           (6 elements)
using State = std::array<double, 18>;
using Ctrl  = std::array<double, 6>;

// Euler-to-quaternion (shared with kinematics node)
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

// Compute the 6D force/torque wrench produced by current motor speeds.
// F_out[0:3] = [fx, fy, fz], F_out[3:6] = [τφ, τθ, τψ]
inline void motor_wrench(const double* omega6, double F_out[6])
{
    for (int i = 0; i < 6; ++i) {
        F_out[i] = 0.0;
        for (int j = 0; j < 6; ++j)
            F_out[i] += M_PHI[i][j] * (omega6[j] * omega6[j]);
    }
}

inline State state_derivative(const State& s, const Ctrl& u)
{
    // Unpack χ = [x,y,z, φ,θ,ψ]
    const double phi   = s[3], theta = s[4], psi = s[5];
    // Unpack body velocities v = [u_b, v_b, w_b, p, q, r]
    const double u_b = s[6], v_b = s[7], w_b = s[8];
    const double p   = s[9], q   = s[10], r  = s[11];
    // Motor speeds
    const double* omega = &s[12];

    //  Motor first-order dynamics: Ω̇ᵢ = (u[i] - Ωᵢ) / TM 
    double omega_dot[6];
    for (int i = 0; i < 6; ++i)
        omega_dot[i] = (u[i] - omega[i]) / TM;

    //  Motor force/torque (body frame) 
    double FM[6];  // [fx, fy, fz, τφ, τθ, τψ]
    for (int i = 0; i < 6; ++i) {
        FM[i] = 0.0;
        for (int j = 0; j < 6; ++j)
            FM[i] += M_PHI[i][j] * (omega[j] * omega[j]);
    }

    //  Trig shortcuts 
    const double sp = std::sin(phi), cp = std::cos(phi);
    const double st = std::sin(theta), ct = std::cos(theta);
    const double tt = std::abs(ct) > 1e-4 ? st / ct : std::copysign(1e4, st);
    const double spsi = std::sin(psi), cpsi = std::cos(psi);

    //  Kinematic equations: χ̇ = R_ib(χ)·v 
    // Translational (R₁ · v_lin)
    const double x_dot =  u_b*cpsi*ct
                        + v_b*(cpsi*st*sp - spsi*cp)
                        + w_b*(cpsi*st*cp + spsi*sp);
    const double y_dot =  u_b*spsi*ct
                        + v_b*(spsi*st*sp + cpsi*cp)
                        + w_b*(spsi*st*cp - cpsi*sp);
    const double z_dot = -u_b*st + v_b*ct*sp + w_b*ct*cp;

    // Rotational (R₂ · omega_ang)
    const double phi_dot   = p + sp*tt*q + cp*tt*r;
    const double theta_dot = cp*q - sp*r;
    const double psi_dot   = std::abs(ct) > 0.035
                             ? (sp*q + cp*r) / ct : 0.0;

    // Gravity in body frame (Eq. 8) 
    // Fg = [m*g*Sθ, -m*g*Cθ*Sφ, -m*g*Cθ*Cφ, 0, 0, 0]
    const double fg_x = MASS * G * st;
    const double fg_y = -MASS * G * ct * sp;
    const double fg_z = -MASS * G * ct * cp;

    //  Coriolis: Φ(v)ᵀ·M·v 
    // Linear part: m·[v_b*r - w_b*q, w_b*p - u_b*r, u_b*q - v_b*p]
    const double cor_x = MASS * (v_b*r - w_b*q);
    const double cor_y = MASS * (w_b*p - u_b*r);
    const double cor_z = MASS * (u_b*q - v_b*p);
    // Angular part: [(Jyy-Jzz)*q*r, (Jzz-Jxx)*p*r, (Jxx-Jyy)*p*q]
    const double cor_p = (JYY - JZZ) * q * r;
    const double cor_q = (JZZ - JXX) * p * r;
    const double cor_r = (JXX - JYY) * p * q;

    //  Body-frame acceleration: v̇ = M⁻¹·(F_M + F_g + Φᵀ·M·v) 
    const double u_dot = (FM[0] + fg_x + cor_x) / MASS;
    const double v_dot = (FM[1] + fg_y + cor_y) / MASS;
    const double w_dot = (FM[2] + fg_z + cor_z) / MASS;
    const double p_dot = (FM[3] + cor_p) / JXX;
    const double q_dot = (FM[4] + cor_q) / JYY;
    const double r_dot = (FM[5] + cor_r) / JZZ;

    return {x_dot, y_dot, z_dot,
            phi_dot, theta_dot, psi_dot,
            u_dot, v_dot, w_dot,
            p_dot, q_dot, r_dot,
            omega_dot[0], omega_dot[1], omega_dot[2],
            omega_dot[3], omega_dot[4], omega_dot[5]};
}

inline State rk4_step(const State& s, const Ctrl& u, double dt)
{
    const auto k1 = state_derivative(s, u);

    State s2;
    for (int i = 0; i < 18; ++i) s2[i] = s[i] + 0.5*dt*k1[i];
    const auto k2 = state_derivative(s2, u);

    State s3;
    for (int i = 0; i < 18; ++i) s3[i] = s[i] + 0.5*dt*k2[i];
    const auto k3 = state_derivative(s3, u);

    State s4;
    for (int i = 0; i < 18; ++i) s4[i] = s[i] + dt*k3[i];
    const auto k4 = state_derivative(s4, u);

    State out;
    for (int i = 0; i < 18; ++i)
        out[i] = s[i] + (dt/6.0)*(k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);
    return out;
}

} // namespace hTsim
