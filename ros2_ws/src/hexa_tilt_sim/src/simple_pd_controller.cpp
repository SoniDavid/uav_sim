#include <array>
#include <mutex>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "hexa_tilt_sim/params.hpp"

using namespace hTsim;

// Simple PD controller with direct state-feedback (no FxT differentiator).
//
// Uses actual body velocities from /hexa/state, converts them to world-frame
// via the full rotation matrix, then computes body-frame desired force via R^T.
// Intended to verify that allocation, dynamics, and kinematics are correct.
//
// State layout (18D):  [x,y,z, φ,θ,ψ,  u,v,w, p,q,r,  Ω₁..Ω₆]
// Reference layout (12D): [x_d,y_d,z_d, φ_d,θ_d,ψ_d,  ẋ_d,ẏ_d,ż_d, φ̇_d,θ̇_d,ψ̇_d]
// Control output (6D):    [fx, fy, fz, τφ, τθ, τψ]
class SimplePdController : public rclcpp::Node
{
public:
    SimplePdController() : Node("hexa_simple_pd_controller")
    {
        state_.fill(0.0);
        ref_.fill(0.0);
        ref_[2] = 1.2;

        pub_control_ = create_publisher<std_msgs::msg::Float64MultiArray>("/hexa/control", 10);

        sub_state_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/hexa/state", 10,
            [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (msg->data.size() < 18) return;
                std::lock_guard<std::mutex> lk(mutex_);
                for (int i = 0; i < 18; ++i) state_[i] = msg->data[i];
            });

        sub_ref_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/hexa/reference", 10,
            [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (msg->data.size() < 12) return;
                std::lock_guard<std::mutex> lk(mutex_);
                for (size_t i = 0; i < 12; ++i) ref_[i] = msg->data[i];
            });

        timer_ = create_wall_timer(
            std::chrono::microseconds(static_cast<long>(DT_CTRL * 1e6)),
            std::bind(&SimplePdController::control_step, this));

        RCLCPP_INFO(get_logger(), "Simple PD controller started at %.0f Hz.", 1.0 / DT_CTRL);
    }

private:
    // Conservative gains: max lateral force ≈ m*(Kp*1m + Kd*2m/s) ≈ 11 N, well below saturation
    static constexpr double KP_XY  = 1.5;
    static constexpr double KD_XY  = 2.5;
    static constexpr double KP_Z   = 8.0;
    static constexpr double KD_Z   = 4.0;
    static constexpr double KP_ATT = 8.0;
    static constexpr double KD_ATT = 4.0;
    static constexpr double KP_YAW = 8.0;
    static constexpr double KD_YAW = 4.0;

    void control_step()
    {
        std::array<double, 18> s;
        std::array<double, 12> r;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            s = state_;
            r = ref_;
        }

        //  Unpack state 
        const double phi   = s[3], theta = s[4], psi = s[5];
        const double u_b   = s[6], v_b   = s[7], w_b = s[8];
        const double p     = s[9], q     = s[10], rv  = s[11];

        const double sp   = std::sin(phi),   cp   = std::cos(phi);
        const double st   = std::sin(theta), ct   = std::cos(theta);
        const double spsi = std::sin(psi),   cpsi = std::cos(psi);

        //  World-frame velocity: v_world = R_body2world * v_body 
        // (identical to kinematic equations in dynamics_node)
        const double x_dot =  u_b*cpsi*ct
                            + v_b*(cpsi*st*sp - spsi*cp)
                            + w_b*(cpsi*st*cp + spsi*sp);
        const double y_dot =  u_b*spsi*ct
                            + v_b*(spsi*st*sp + cpsi*cp)
                            + w_b*(spsi*st*cp - cpsi*sp);
        const double z_dot = -u_b*st + v_b*ct*sp + w_b*ct*cp;

        //  Desired world-frame linear acceleration (PD) 
        const double ax = KP_XY*(r[0] - s[0]) + KD_XY*(r[6]  - x_dot);
        const double ay = KP_XY*(r[1] - s[1]) + KD_XY*(r[7]  - y_dot);
        const double az = KP_Z *(r[2] - s[2]) + KD_Z *(r[8]  - z_dot);

        //  Body-frame force: F_body = R^T * m*(a_des + [0,0,g]) 
        // Newton's law in world frame: m*ẍ_world = R*F_body + [0,0,-m*g]
        // → F_body = R^T * m * [ax, ay, az+g]      (z positive up)
        const double bx = MASS * ax;
        const double by = MASS * ay;
        const double bz = MASS * (az + G);

        // R_body2world columns become R^T rows:
        const double fx_des =  cpsi*ct             * bx +  spsi*ct             * by + (-st)  * bz;
        const double fy_des = (cpsi*st*sp - spsi*cp)* bx + (spsi*st*sp + cpsi*cp)* by + ct*sp * bz;
        const double fz_des = (cpsi*st*cp + spsi*sp)* bx + (spsi*st*cp - cpsi*sp)* by + ct*cp * bz;

        //  Attitude PD using body angular rates directly 
        // Inertia-scaled to match units expected by the allocation matrix
        const double tau_phi   = JXX * (KP_ATT*(r[3] - phi)   + KD_ATT*(0.0 - p));
        const double tau_theta = JYY * (KP_ATT*(r[4] - theta)  + KD_ATT*(0.0 - q));
        const double tau_psi   = JZZ * (KP_YAW*(r[5] - psi)   + KD_YAW*(0.0 - rv));

        std_msgs::msg::Float64MultiArray msg;
        msg.data = {fx_des, fy_des, fz_des, tau_phi, tau_theta, tau_psi};
        pub_control_->publish(msg);
    }

    std::array<double, 18> state_;
    std::array<double, 12> ref_;
    std::mutex mutex_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    pub_control_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_state_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_ref_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplePdController>());
    rclcpp::shutdown();
    return 0;
}
