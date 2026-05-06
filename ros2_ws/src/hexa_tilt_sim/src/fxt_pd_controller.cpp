#include <array>
#include <mutex>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "hexa_tilt_sim/params.hpp"
#include "hexa_tilt_sim/differentiator.hpp"
#include "hexa_tilt_sim/controller.hpp"
#include "hexa_tilt_sim/allocation.hpp"

using namespace hTsim;

// FxT Differentiator + Feedback Linearization PD Controller (250 Hz).
// Subscribes to /hexa/state (18D) and /hexa/reference (12D).
// Publishes /hexa/control (6D wrench).
//
// Diagnostic topics:
//   /hexa/vel_est   — differentiator velocity estimate hat2  (6D world-frame)
//   /hexa/vel_world — actual world-frame velocity from state (6D world-frame)
//   Compare these two in Foxglove to diagnose differentiator lag.
//
// Reference layout (12D): [x_d, y_d, z_d, φ_d, θ_d, ψ_d,  ẋ_d, ẏ_d, ż_d, φ̇_d, θ̇_d, ψ̇_d]
// State layout   (18D): [x,y,z, φ,θ,ψ,  u,v,w, p,q,r,  Ω₁..Ω₆]
class FxtPdController : public rclcpp::Node
{
public:
    FxtPdController() : Node("hexa_fxt_pd_controller"), initialized_(false), log_cycle_(0)
    {
        state_.fill(0.0);
        ref_.fill(0.0);
        ref_[2] = 1.2;

        pub_control_   = create_publisher<std_msgs::msg::Float64MultiArray>("/hexa/control",    10);
        pub_vel_est_   = create_publisher<std_msgs::msg::Float64MultiArray>("/hexa/vel_est",    10);
        pub_vel_world_ = create_publisher<std_msgs::msg::Float64MultiArray>("/hexa/vel_world",  10);

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
            std::bind(&FxtPdController::control_step, this));

        RCLCPP_INFO(get_logger(), "FxT-PD controller started at %.0f Hz.", 1.0 / DT_CTRL);
    }

private:
    // Compute world-frame velocity [ẋ,ẏ,ż,φ̇,θ̇,ψ̇] from body velocity and Euler angles.
    // Identical kinematics to dynamics_node — exact, no small-angle approximation.
    static void world_velocity(const double chi[6], const double v_body[6],
                               double chi_dot_out[6])
    {
        const double phi = chi[3], theta = chi[4], psi = chi[5];
        const double sp = std::sin(phi),   cp = std::cos(phi);
        const double st = std::sin(theta), ct = std::cos(theta);
        const double tt = std::abs(ct) > 1e-4 ? st / ct : std::copysign(1e4, st);
        const double spsi = std::sin(psi), cpsi = std::cos(psi);

        const double u_b = v_body[0], v_b = v_body[1], w_b = v_body[2];
        const double p   = v_body[3], q   = v_body[4], r   = v_body[5];

        chi_dot_out[0] =  u_b*cpsi*ct + v_b*(cpsi*st*sp - spsi*cp) + w_b*(cpsi*st*cp + spsi*sp);
        chi_dot_out[1] =  u_b*spsi*ct + v_b*(spsi*st*sp + cpsi*cp) + w_b*(spsi*st*cp - cpsi*sp);
        chi_dot_out[2] = -u_b*st      + v_b*ct*sp                  + w_b*ct*cp;

        chi_dot_out[3] = p + sp*tt*q + cp*tt*r;
        chi_dot_out[4] = cp*q - sp*r;
        chi_dot_out[5] = std::abs(ct) > 0.035 ? (sp*q + cp*r) / ct : 0.0;
    }

    void control_step()
    {
        std::array<double, 18> s;
        std::array<double, 12> r;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            s = state_;
            r = ref_;
        }

        const double chi_meas[6] = {s[0], s[1], s[2], s[3], s[4], s[5]};
        const double v_body[6]   = {s[6], s[7], s[8], s[9], s[10], s[11]};

        // Actual world-frame velocity (ground truth for diagnostic comparison)
        double chi_dot_actual[6];
        world_velocity(chi_meas, v_body, chi_dot_actual);

        // Initialize differentiator with actual velocity so hat2 starts accurate.
        // This eliminates the startup transient where hat2=0 but the drone is moving.
        if (!initialized_) {
            diff_.reset(chi_meas, chi_dot_actual);
            initialized_ = true;
        }

        diff_.update(chi_meas, DT_CTRL);

        const double chi_d[6]     = {r[0], r[1], r[2], r[3], r[4], r[5]};
        const double chi_dot_d[6] = {r[6], r[7], r[8], r[9], r[10], r[11]};

        auto F_des = compute_fxt_pd_control(
            diff_.hat1, diff_.hat2, v_body, chi_d, chi_dot_d);

        //  Diagnostic: check how close hat2 is to actual velocity 
        // If these diverge in Foxglove, the differentiator lag is the instability source.
        const double vel_err_xy = std::hypot(diff_.hat2[0] - chi_dot_actual[0],
                                             diff_.hat2[1] - chi_dot_actual[1]);

        //  Saturation check: warn when commanded forces exceed actuator limits 
        // Max fz ≈ 6 × 0.74e-5 × 1000² = 44.4 N; max |fx|,|fy| ≈ 4.6 N each
        constexpr double FZ_MAX   =  6.0 * 0.74e-5 * OMEGA_MAX * OMEGA_MAX;
        constexpr double FXY_MAX  =  2.0 * 0.23e-5 * OMEGA_MAX * OMEGA_MAX; // 2 motors contribute
        const bool sat_z  = std::abs(F_des[2]) > FZ_MAX;
        const bool sat_xy = std::abs(F_des[0]) > FXY_MAX || std::abs(F_des[1]) > FXY_MAX;

        // Clamp fz to prevent immediate saturation on startup (large KP_z × large e_z).
        // The allocation can still saturate on x/y but those are lower magnitude.
        F_des[2] = std::clamp(F_des[2], 0.0, FZ_MAX);

        std_msgs::msg::Float64MultiArray ctrl_msg;
        ctrl_msg.data.assign(F_des.begin(), F_des.end());
        pub_control_->publish(ctrl_msg);

        // Publish differentiator estimate
        std_msgs::msg::Float64MultiArray vel_est_msg;
        vel_est_msg.data.assign(diff_.hat2, diff_.hat2 + 6);
        pub_vel_est_->publish(vel_est_msg);

        // Publish actual world-frame velocity (compare with vel_est in Foxglove)
        std_msgs::msg::Float64MultiArray vel_world_msg;
        vel_world_msg.data.assign(chi_dot_actual, chi_dot_actual + 6);
        pub_vel_world_->publish(vel_world_msg);

        // Log at ~2 s intervals
        if (++log_cycle_ >= 500) {
            log_cycle_ = 0;
            if (sat_z || sat_xy)
                RCLCPP_WARN(get_logger(),
                    "SATURATION — fz=%.1f/%.1f N, fx=%.1f/%.1f N, fy=%.1f/%.1f N | "
                    "vel_err_xy=%.3f m/s",
                    F_des[2], FZ_MAX, F_des[0], FXY_MAX, F_des[1], FXY_MAX, vel_err_xy);
            else
                RCLCPP_INFO(get_logger(),
                    "fz=%.2f N | pos=[%.2f, %.2f, %.2f] | vel_err_xy=%.3f m/s",
                    F_des[2], s[0], s[1], s[2], vel_err_xy);
        }
    }

    std::array<double, 18> state_;
    std::array<double, 12> ref_;
    std::mutex mutex_;
    FxtDifferentiator diff_;
    bool initialized_;
    int  log_cycle_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    pub_control_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    pub_vel_est_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    pub_vel_world_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_state_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_ref_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    auto node = std::make_shared<FxtPdController>();
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
