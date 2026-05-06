#pragma once
#include <cmath>
#include "hexa_tilt_sim/params.hpp"

namespace hTsim {

// Fixed-time differentiator (Eqs. 11–16, FxTDiff_PD.pdf).
// Applied independently to each of the 6 state axes χ = [x,y,z,φ,θ,ψ].
// Given position measurement γ₁, estimates position γ̂₁ and velocity γ̂₂.
//
// Equations:
//   γ̂₁̇ = γ̂₂ + G1·sig^ϖ₁(γ̃) + Q1·sig^ϱ₁(γ̃)
//   γ̂₂̇ =      G2·sig^ϖ₂(γ̃) + Q2·sig^ϱ₂(γ̃)
//   γ̃  = γ₁ - γ̂₁   (estimation error)
//   sig^a(x) = |x|^a · sign(x)

struct FxtDifferentiator
{
    double hat1[6]{};  // estimated position/attitude χ̂
    double hat2[6]{};  // estimated velocity χ̂̇

    // chi_dot_init: initial velocity estimate for hat2 (world-frame).
    // Pass nullptr to default to zero (legacy behaviour — causes a startup transient).
    void reset(const double chi[6], const double chi_dot_init[6] = nullptr)
    {
        for (int i = 0; i < 6; ++i) {
            hat1[i] = chi[i];
            hat2[i] = chi_dot_init ? chi_dot_init[i] : 0.0;
        }
    }

    // Euler integration step at rate dt (seconds).
    // chi_meas[6]: measured χ from state[0:6]
    void update(const double chi_meas[6], double dt)
    {
        for (int i = 0; i < 6; ++i) {
            const double err = chi_meas[i] - hat1[i];
            const double s_w1   = sig(err, DIFF_W1);
            const double s_rho1 = sig(err, DIFF_RHO1);
            const double s_w2   = sig(err, DIFF_W2);
            const double s_rho2 = sig(err, DIFF_RHO2);

            const double d_hat1 = hat2[i]
                                  + DIFF_G1[i] * s_w1
                                  + DIFF_Q1[i] * s_rho1;
            const double d_hat2 = DIFF_G2[i] * s_w2
                                  + DIFF_Q2[i] * s_rho2;

            hat1[i] += dt * d_hat1;
            hat2[i] += dt * d_hat2;
        }
    }

private:
    static double sig(double x, double alpha)
    {
        // sig^α(x) = |x|^α · sign(x)
        if (x == 0.0) return 0.0;
        return std::copysign(std::pow(std::abs(x), alpha), x);
    }
};

} // namespace hTsim
