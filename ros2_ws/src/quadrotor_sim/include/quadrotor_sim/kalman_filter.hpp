#pragma once
#include <Eigen/Dense>

// Discrete Kalman filter — linearized quadrotor model
// State:       xk = [x,ẋ, y,ẏ, z,ż, φ,φ̇, θ,θ̇, ψ,ψ̇]ᵀ  (12×1)
// Measurement: yk = [x, y, z, φ, θ, ψ]ᵀ                 (6×1)
// Control:     uk = [T-mg, τ_φ, τ_θ, τ_ψ]ᵀ              (4×1)  (thrust linearized around hover)
//
// A is initialized as a constant-velocity base (block-diagonal [1,dt;0,1]×6).
// The caller must inject gravity-coupling entries via set_A_entry() after init()
// and supply the discrete B matrix via init_B() before calling step(y, Bu).

class DiscreteKF
{
public:
    static constexpr int NX = 12;
    static constexpr int NY = 6;
    static constexpr int NU = 4;

    using VecX  = Eigen::Matrix<double, NX, 1>;
    using VecY  = Eigen::Matrix<double, NY, 1>;
    using VecU  = Eigen::Matrix<double, NU, 1>;
    using MatXX = Eigen::Matrix<double, NX, NX>;
    using MatXU = Eigen::Matrix<double, NX, NU>;
    using MatYX = Eigen::Matrix<double, NY, NX>;
    using MatYY = Eigen::Matrix<double, NY, NY>;

    DiscreteKF() { reset(); }

    void init(double dt,
              const MatXX & Q,
              const MatYY & R,
              const MatXX & P0,
              const VecX  & x0)
    {
        // Base: constant-velocity blocks [1,dt;0,1] repeated 6 times.
        // Caller adds gravity coupling via set_A_entry() after this returns.
        A_.setZero();
        for (int i = 0; i < 6; ++i) {
            A_(2*i,   2*i)   = 1.0;
            A_(2*i,   2*i+1) = dt;
            A_(2*i+1, 2*i+1) = 1.0;
        }

        // C: selects even-indexed states (positions/angles)
        C_.setZero();
        for (int i = 0; i < 6; ++i)
            C_(i, 2*i) = 1.0;

        B_.setZero();
        Q_ = Q;
        R_ = R;
        P_ = P0;
        x_ = x0;
        initialized_ = true;
    }

    // Inject the discrete B matrix (caller computes from physical params).
    void init_B(const MatXU & B) { B_ = B; }

    // Allow the caller to patch individual A entries (e.g., gravity coupling).
    void set_A_entry(int r, int c, double v) { A_(r, c) = v; }

    // Run one KF step with control input u (linearized around hover).
    // Bu = B * u is computed by caller; pass VecX::Zero() if no control.
    const VecX & step(const VecY & y, const VecX & Bu = VecX::Zero())
    {
        if (!initialized_) return x_;

        // Prediction (model-based: A*x + B*u)
        VecX  xp = A_ * x_ + Bu;
        MatXX Pp = A_ * P_ * A_.transpose() + Q_;

        // Kalman gain  L = Pp*Cᵀ*(C*Pp*Cᵀ + R)⁻¹
        MatYY S  = C_ * Pp * C_.transpose() + R_;
        Eigen::Matrix<double, NX, NY> L = Pp * C_.transpose() * S.inverse();

        // Correction (Joseph form for numerical stability)
        MatXX ILC = MatXX::Identity() - L * C_;
        x_ = xp + L * (y - C_ * xp);
        P_ = ILC * Pp * ILC.transpose() + L * R_ * L.transpose();

        return x_;
    }

    const VecX & state() const { return x_; }
    const MatXU & B()    const { return B_; }

    void reset()
    {
        x_.setZero();
        P_.setIdentity();
        B_.setZero();
        initialized_ = false;
    }

private:
    MatXX A_, Q_, P_;
    MatXU B_;
    MatYX C_;
    MatYY R_;
    VecX  x_;
    bool  initialized_{false};
};
