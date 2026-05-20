#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include "quadrotor_sim/kalman_filter.hpp"
#include "quadrotor_sim/params.hpp"

// Input-augmented Kalman filter node using the same linearized model as the sf/LQR controller.
//
// Subscribes:  /uav/noisy_state  (6D — from sensor_noise_node)
//              /uav/control      (4D — [T, τ_φ, τ_θ, τ_ψ])
// Publishes:   /uav/kf_state     (12D — estimated state, same layout as /uav/state[0..11])
//              /uav/kf_euler     (Vector3Stamped — estimated φ, θ, ψ)
//
// KF internal state order: [x,ẋ, y,ẏ, z,ż, φ,φ̇, θ,θ̇, ψ,ψ̇]
//
// Linearized quadrotor model (same physics as sf/LQR controller):
//
//   A includes constant-velocity base plus gravity coupling:
//     ẍ ≈ g*θ   →  A(ẋ, θ) = g*dt        (index 1,8)
//     ÿ ≈ -g*φ  →  A(ẏ, φ) = -g*dt       (index 3,6)
//
//   B (discrete, ZOH at dt=0.02s):
//     input u_lin = [T - m*g, τ_φ, τ_θ, τ_ψ]   (thrust linearized around hover)
//     B[ż ][T-mg ] = dt/m
//     B[φ̇ ][τ_φ  ] = dt/Ixx
//     B[θ̇ ][τ_θ  ] = dt/Iyy
//     B[ψ̇ ][τ_ψ  ] = dt/Izz
//
// ROS2 parameters:
//   q_pos   process noise — position/angle states  (default 1e-4)
//   q_vel   process noise — velocity/rate states   (default 1e-5)
//   r_pos   measurement noise — position (m²)      (default 0.0025 = sigma_pos²)
//   r_ang   measurement noise — angle (rad²)       (default 0.0001 = sigma_ang²)

using namespace qsim;

class KalmanFilterNode : public rclcpp::Node
{
public:
    KalmanFilterNode() : Node("kalman_filter_node")
    {
        declare_parameter("q_pos", 1e-4);
        declare_parameter("q_vel", 1e-5);
        declare_parameter("r_pos", 0.0025);   // sigma_pos=0.05 → 0.05²
        declare_parameter("r_ang", 0.0001);   // sigma_ang=0.01 → 0.01²

        const double dt    = DT_PUB;   // 0.02 s — same rate as controller
        const double q_pos = get_parameter("q_pos").as_double();
        const double q_vel = get_parameter("q_vel").as_double();
        const double r_pos = get_parameter("r_pos").as_double();
        const double r_ang = get_parameter("r_ang").as_double();

        // ── Process noise Q ─────────────────────────────────────────────────
        DiscreteKF::MatXX Q = DiscreteKF::MatXX::Zero();
        Q.diagonal() << q_pos, q_vel, q_pos, q_vel, q_pos, q_vel,
                        q_pos, q_vel, q_pos, q_vel, q_pos, q_vel;

        // ── Measurement noise R ─────────────────────────────────────────────
        DiscreteKF::MatYY R = DiscreteKF::MatYY::Zero();
        R.diagonal() << r_pos, r_pos, r_pos, r_ang, r_ang, r_ang;

        kf_.init(dt, Q, R, DiscreteKF::MatXX::Identity(), DiscreteKF::VecX::Zero());

        // ── Gravity coupling in A (same as sf/LQR subsystem model) ──────────
        // KF state indices: x=0,ẋ=1, y=2,ẏ=3, z=4,ż=5, φ=6,φ̇=7, θ=8,θ̇=9, ψ=10,ψ̇=11
        kf_.set_A_entry(1, 8,  G * dt);    // ẍ ≈ +g*θ
        kf_.set_A_entry(3, 6, -G * dt);    // ÿ ≈ -g*φ

        // ── Discrete B matrix (ZOH, first-order terms) ───────────────────────
        // input = [T-mg, τ_φ, τ_θ, τ_ψ]
        DiscreteKF::MatXU B = DiscreteKF::MatXU::Zero();
        B(5, 0) = dt / MASS;          // (T-mg)/m → ż
        B(4, 0) = dt * dt / (2.0 * MASS);  // (T-mg)/m → z  (second-order, small)
        B(7, 1) = dt / IXX;           // τ_φ → φ̇
        B(6, 1) = dt * dt / (2.0 * IXX);   // τ_φ → φ   (second-order, small)
        B(9, 2) = dt / IYY;           // τ_θ → θ̇
        B(8, 2) = dt * dt / (2.0 * IYY);   // τ_θ → θ   (second-order, small)
        B(11, 3) = dt / IZZ;          // τ_ψ → ψ̇
        B(10, 3) = dt * dt / (2.0 * IZZ);  // τ_ψ → ψ   (second-order, small)
        kf_.init_B(B);

        // ── Publishers / subscribers ─────────────────────────────────────────
        pub_kf_state_ = create_publisher<std_msgs::msg::Float64MultiArray>("/uav/kf_state", 10);
        pub_kf_euler_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/uav/kf_euler", 10);

        sub_ctrl_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/uav/control", 10,
            [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (msg->data.size() < 4) return;
                std::lock_guard<std::mutex> lk(ctrl_mutex_);
                // Linearize thrust around hover: u_lin[0] = T - m*g
                u_cmd_ << msg->data[0] - MASS * G,
                           msg->data[1],
                           msg->data[2],
                           msg->data[3];
            });

        sub_meas_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/uav/noisy_state", 10,
            [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) { cb(msg); });

        u_cmd_.setZero();

        RCLCPP_INFO(get_logger(),
            "KF node (input-augmented): q_pos=%.2e q_vel=%.2e r_pos=%.2e r_ang=%.2e",
            q_pos, q_vel, r_pos, r_ang);
    }

private:
    void cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 6) return;
        const auto & d = msg->data;

        // Measurement vector: noisy position + noisy angles (6×1)
        DiscreteKF::VecY y;
        y << d[0], d[1], d[2],   // x, y, z  (noisy)
             d[3], d[4], d[5];   // φ, θ, ψ  (noisy)

        // Compute B*u with latest cached control command
        DiscreteKF::VecX Bu;
        {
            std::lock_guard<std::mutex> lk(ctrl_mutex_);
            Bu = kf_.B() * u_cmd_;
        }

        const auto & x = kf_.step(y, Bu);
        const auto now = get_clock()->now();

        // Publish estimated state in /uav/state convention:
        // [x̂,ŷ,ẑ, ẋ̂,ẏ̂,ż̂, φ̂,θ̂,ψ̂, φ̇̂,θ̇̂,ψ̇̂]
        std_msgs::msg::Float64MultiArray kf_msg;
        kf_msg.data = {
            x(0), x(2), x(4),    // position  x̂, ŷ, ẑ
            x(1), x(3), x(5),    // velocity  ẋ̂, ẏ̂, ż̂
            x(6), x(8), x(10),   // angles    φ̂, θ̂, ψ̂
            x(7), x(9), x(11)    // ang rates φ̇̂, θ̇̂, ψ̇̂
        };
        pub_kf_state_->publish(kf_msg);

        geometry_msgs::msg::Vector3Stamped euler_msg;
        euler_msg.header.stamp    = now;
        euler_msg.header.frame_id = "world";
        euler_msg.vector.x = x(6);    // φ̂
        euler_msg.vector.y = x(8);    // θ̂
        euler_msg.vector.z = x(10);   // ψ̂
        pub_kf_euler_->publish(euler_msg);
    }

    DiscreteKF kf_;
    DiscreteKF::VecU u_cmd_;
    std::mutex ctrl_mutex_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_ctrl_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_meas_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    pub_kf_state_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr  pub_kf_euler_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KalmanFilterNode>());
    rclcpp::shutdown();
    return 0;
}
