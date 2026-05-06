#pragma once
#include <array>
#include <cmath>
#include "hexa_tilt_sim/params.hpp"

namespace hTsim {

// Feedback-linearization + PD controller (Eqs. 17–22, FxTDiff_PD.pdf).
//
// Control law (with χ̈_d = 0 for setpoint tracking):
//   u_a   = Kp·e + Kd·ė         (PD on position/attitude error)
//   f(x)  = M⁻¹·(F_g + Φᵀ·M·v) (nonlinear dynamics without motor input)
//   F_des = M·(u_a - f(x))      (feedback linearization cancels nonlinearities)
//         = M·u_a - F_g - Φ_term
//
// Inputs:
//   chi_hat[6]     : estimated χ from FxT differentiator
//   chi_dot_hat[6] : estimated χ̇ from FxT differentiator
//   v_hat[6]       : estimated body velocity [u,v,w,p,q,r] — used for Coriolis
//                    (under small-angle assumption body ≈ inertial, so v_hat ≈ chi_dot_hat)
//   chi_d[6]       : desired χ from reference
//   chi_dot_d[6]   : desired χ̇ from reference (zero for static setpoints)
//
// Output:
//   F_des[6] = [fx_des, fy_des, fz_des, τφ_des, τθ_des, τψ_des]
inline std::array<double, 6> compute_fxt_pd_control(
    const double chi_hat[6],
    const double chi_dot_hat[6],
    const double v_body[6],
    const double chi_d[6],
    const double chi_dot_d[6])
{
    //  PD errors 
    double e[6], e_dot[6];
    for (int i = 0; i < 6; ++i) {
        e[i]     = chi_d[i]     - chi_hat[i];
        e_dot[i] = chi_dot_d[i] - chi_dot_hat[i];
    }

    //  PD auxiliary controller: u_a = Kp·e + Kd·ė 
    double u_a[6];
    for (int i = 0; i < 6; ++i)
        u_a[i] = KP[i] * e[i] + KD[i] * e_dot[i];

    //  Gravity in body frame (Eq. 8) 
    const double phi   = chi_hat[3], theta = chi_hat[4];
    const double sp = std::sin(phi), cp = std::cos(phi);
    const double st = std::sin(theta), ct = std::cos(theta);

    const double Fg[6] = {
        MASS * G * st,
       -MASS * G * ct * sp,
       -MASS * G * ct * cp,
        0.0, 0.0, 0.0
    };

    //  Coriolis: Φ(v)ᵀ·M·v 
    const double u_b = v_body[0], v_b = v_body[1], w_b = v_body[2];
    const double p   = v_body[3], q   = v_body[4], r   = v_body[5];

    const double Phi[6] = {
        MASS * (v_b*r - w_b*q),
        MASS * (w_b*p - u_b*r),
        MASS * (u_b*q - v_b*p),
        (JYY - JZZ) * q * r,
        (JZZ - JXX) * p * r,
        (JXX - JYY) * p * q
    };

    //  Feedback linearization: F_des = M·u_a - F_g - Φ 
    // M = diag(m, m, m, Jxx, Jyy, Jzz)
    constexpr double M_diag[6] = {MASS, MASS, MASS, JXX, JYY, JZZ};

    std::array<double, 6> F_des;
    for (int i = 0; i < 6; ++i)
        F_des[i] = M_diag[i] * u_a[i] - Fg[i] - Phi[i];

    return F_des;
}

} // namespace hTsim
