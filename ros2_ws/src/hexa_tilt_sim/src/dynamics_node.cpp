#include <array>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "hexa_tilt_sim/params.hpp"
#include "hexa_tilt_sim/dynamics.hpp"
#include "hexa_tilt_sim/allocation.hpp"

using namespace hTsim;

class HexaDynamicsNode : public rclcpp::Node
{
public:
    HexaDynamicsNode() : Node("hexa_dynamics")
    {
        // Initialize at rest with motors pre-spun at hover speed
        state_.fill(0.0);
        const auto omega_hover = hover_motor_speeds();
        for (int i = 0; i < 6; ++i) state_[12 + i] = omega_hover[i];

        // Initial control command: hover thrust
        double F_hover[6] = {0.0, 0.0, MASS * G, 0.0, 0.0, 0.0};
        const auto osq = allocate_motor_speeds(F_hover);
        u_cmd_ = omega_sq_to_omega_des(osq);

        pub_state_ = create_publisher<std_msgs::msg::Float64MultiArray>("/hexa/state", 10);

        sub_control_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/hexa/control", 10,
            [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (msg->data.size() < 6) return;
                double F[6];
                for (int i = 0; i < 6; ++i) F[i] = msg->data[i];
                const auto osq = allocate_motor_speeds(F);
                std::lock_guard<std::mutex> lk(ctrl_mutex_);
                u_cmd_ = omega_sq_to_omega_des(osq);
            });

        timer_ = create_wall_timer(
            std::chrono::microseconds(static_cast<long>(DT_SIM * 1e6)),
            std::bind(&HexaDynamicsNode::step, this));

        RCLCPP_INFO(get_logger(), "Hexacopter dynamics node started at %.0f Hz.", 1.0 / DT_SIM);
    }

private:
    void step()
    {
        Ctrl u;
        {
            std::lock_guard<std::mutex> lk(ctrl_mutex_);
            u = u_cmd_;
        }

        state_ = rk4_step(state_, u, DT_SIM);

        std_msgs::msg::Float64MultiArray msg;
        msg.data.assign(state_.begin(), state_.end());
        pub_state_->publish(msg);
    }

    State state_;
    Ctrl  u_cmd_;
    std::mutex ctrl_mutex_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_state_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_control_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    auto node = std::make_shared<HexaDynamicsNode>();
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
