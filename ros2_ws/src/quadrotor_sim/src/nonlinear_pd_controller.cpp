#include <array>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "quadrotor_sim/params.hpp"
#include "quadrotor_sim/controller.hpp"

using namespace qsim;

class NonlinearPDControllerNode : public rclcpp::Node
{
public:
    NonlinearPDControllerNode() : Node("nonlinear_pd_controller"), state_received_(false), log_cycle_(0)
    {
        ref12_.fill(0.0);
        ref12_[2] = 1.0;  // default: hover at 1 m
        state12_.fill(0.0);

        pub_control_ = create_publisher<std_msgs::msg::Float64MultiArray>("/uav/control", 10);

        sub_ref_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/uav/reference", 10,
            [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (msg->data.size() < 12) return;
                std::lock_guard<std::mutex> lk(mutex_);
                for (int i = 0; i < 12; ++i) ref12_[i] = msg->data[i];
            });

        sub_state_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/uav/state", 10,
            [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (msg->data.size() < 12) return;
                std::lock_guard<std::mutex> lk(mutex_);
                for (int i = 0; i < 12; ++i) state12_[i] = msg->data[i];
                state_received_ = true;
            });

        timer_ = create_wall_timer(
            std::chrono::microseconds(static_cast<long>(DT_PUB * 1e6)),
            std::bind(&NonlinearPDControllerNode::tick, this));

        RCLCPP_INFO(get_logger(), "Nonlinear PD controller node started at %.0f Hz.", 1.0 / DT_PUB);
    }

private:
    void tick()
    {
        std::array<double, 12> state12, ref12;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            if (!state_received_) return;
            state12 = state12_;
            ref12   = ref12_;
        }

        const auto u = compute_cascaded_control(state12.data(), ref12.data());

        std_msgs::msg::Float64MultiArray msg;
        msg.data = {u[0], u[1], u[2], u[3]};
        pub_control_->publish(msg);

        t_history_.push_back(u[0]);
        if (++log_cycle_ >= 100) {
            log_cycle_ = 0;
            double T_avg = 0.0;
            for (double v : t_history_) T_avg += v;
            T_avg /= static_cast<double>(t_history_.size());
            t_history_.clear();
            if (T_avg > T_MAX * 0.95) {
                RCLCPP_WARN(get_logger(),
                    "[nonlinear_pd] Thrust near saturation: avg=%.3f N (T_MAX=%.3f N)", T_avg, T_MAX);
            } else {
                RCLCPP_INFO(get_logger(), "[nonlinear_pd] Thrust avg=%.3f N", T_avg);
            }
        }
    }

    std::array<double, 12> ref12_, state12_;
    std::mutex mutex_;
    bool state_received_;
    int log_cycle_;
    std::vector<double> t_history_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_control_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_ref_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_state_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    auto node = std::make_shared<NonlinearPDControllerNode>();
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
