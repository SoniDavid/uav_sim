#include <array>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "quadrotor_sim/params.hpp"
#include "quadrotor_sim/dynamics.hpp"
#include "quadrotor_sim/allocation.hpp"

using namespace qsim;

class QuadrotorDynamicsNode : public rclcpp::Node
{
public:
    QuadrotorDynamicsNode() : Node("quadrotor_dynamics")
    {
        state_.fill(0.0);
        for (int i = 12; i < 16; ++i) state_[i] = OMEGA_HOVER;

        auto osq = allocate_motor_speeds(T_HOVER, 0.0, 0.0, 0.0);
        u_cmd_ = omega_sq_to_omega_des(osq);

        pub_state_ = create_publisher<std_msgs::msg::Float64MultiArray>("/uav/state", 10);

        sub_control_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/uav/control", 10,
            [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (msg->data.size() < 4) return;
                auto osq = allocate_motor_speeds(
                    msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
                std::lock_guard<std::mutex> lk(ctrl_mutex_);
                u_cmd_ = omega_sq_to_omega_des(osq);
            });

        timer_ = create_wall_timer(
            std::chrono::microseconds(static_cast<long>(DT_SIM * 1e6)),
            std::bind(&QuadrotorDynamicsNode::step, this));

        RCLCPP_INFO(get_logger(), "Quadrotor dynamics node started at %.0f Hz.", 1.0 / DT_SIM);
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
    auto node = std::make_shared<QuadrotorDynamicsNode>();
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
