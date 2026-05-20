#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <random>
#include <mutex>
#include <array>

#include "quadrotor_sim/params.hpp"

using namespace qsim;

// Simulates a noisy sensor layer on top of the perfect simulation state.
// Enforces partial observability: only position and angle measurements are
// published. Velocities and angular rates are NOT included — they must be
// estimated by the Kalman filter from sequential position/angle observations.
//
// Subscribes:  /uav/state        (16D — perfect RK4 state)
// Publishes:   /uav/noisy_state  (6D  — only what real sensors measure)
//
// Output layout (6D):
//   [0..2]  x+n, y+n, z+n    — GPS/position sensor  (sigma_pos)
//   [3..5]  φ+n, θ+n, ψ+n   — IMU angles            (sigma_ang)
//
// Velocities (ẋ, ẏ, ż) and angular rates (p, q, r) are intentionally absent.
// The normal simulation path (/uav/state) is unaffected.
//
// ROS2 parameters (all tunable at launch or runtime):
//   sigma_pos  [m]     std dev of position sensor noise   (default 0.05)
//   sigma_ang  [rad]   std dev of IMU angle noise          (default 0.01)
//   seed       [-]     RNG seed; 0 = random device         (default 42)

class SensorNoiseNode : public rclcpp::Node
{
public:
    SensorNoiseNode() : Node("sensor_noise_node"), state_received_(false)
    {
        declare_parameter("sigma_pos", 0.05);
        declare_parameter("sigma_ang", 0.01);
        declare_parameter("seed",      42);

        sigma_pos_ = get_parameter("sigma_pos").as_double();
        sigma_ang_ = get_parameter("sigma_ang").as_double();
        const int seed = get_parameter("seed").as_int();

        rng_.seed(seed == 0 ? std::random_device{}() : static_cast<uint32_t>(seed));

        pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/uav/noisy_state", 10);

        sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/uav/state", 10,
            [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) { 
                if (msg->data.size() < 12) return;
                std::lock_guard<std::mutex> lk(mutex_);
                for (int i = 0; i < 12; ++i) state12_[i] = msg->data[i];
                state_received_ = true;
            });

        timer_ = create_wall_timer(
            std::chrono::microseconds(static_cast<long>(DT_PUB * 1e6)),
            std::bind(&SensorNoiseNode::tick, this));

        // Allow live parameter changes (e.g. ros2 param set)
        param_cb_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> & params) {
                for (const auto & p : params) {
                    if (p.get_name() == "sigma_pos") sigma_pos_ = p.as_double();
                    if (p.get_name() == "sigma_ang") sigma_ang_ = p.as_double();
                }
                return rcl_interfaces::msg::SetParametersResult{}.set__successful(true);
            });

        RCLCPP_INFO(get_logger(),
            "Sensor noise node started at %.0f Hz: sigma_pos=%.4f m  sigma_ang=%.4f rad  seed=%d",
            1.0 / DT_PUB, sigma_pos_, sigma_ang_, seed);
    }

private:
    void tick()
    {
        std::array<double, 12> d;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            if (!state_received_) return;
            d = state12_;
        }

        std_msgs::msg::Float64MultiArray out;
        out.data = {
            d[0] + sigma_pos_ * dist_(rng_),   // x  + noise
            d[1] + sigma_pos_ * dist_(rng_),   // y  + noise
            d[2] + sigma_pos_ * dist_(rng_),   // z  + noise
            d[6] + sigma_ang_ * dist_(rng_),   // φ  + noise
            d[7] + sigma_ang_ * dist_(rng_),   // θ  + noise
            d[8] + sigma_ang_ * dist_(rng_),   // ψ  + noise
        };
        pub_->publish(out);
    }

    double sigma_pos_, sigma_ang_;
    std::mt19937 rng_;
    std::normal_distribution<double> dist_{0.0, 1.0};
    
    std::array<double, 12> state12_;
    std::mutex mutex_;
    bool state_received_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr  pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorNoiseNode>());
    rclcpp::shutdown();
    return 0;
}
