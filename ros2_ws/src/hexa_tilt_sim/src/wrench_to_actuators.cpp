// wrench_to_actuators : bridge the controller's 6-D wrench to Gazebo motor speeds.
//
// The FxT-PD controller publishes a desired body wrench [fx,fy,fz,tphi,ttheta,tpsi]
// on /hexa/control (exactly as in the analytical-sim pipeline). In the analytical
// sim, dynamics_node converts that wrench to desired motor speeds internally via
// M_PHI_INV. In the Gazebo pipeline the plant lives in Gz, so we do the SAME
// conversion here (reusing allocation.hpp) and publish an actuator_msgs/Actuators
// motor-speed vector, which the bridge forwards to the MulticopterMotorModel plugins.
//
//   sub : /hexa/control            std_msgs/Float64MultiArray  (6-D wrench)
//   pub : /hexa/command/motor_speed actuator_msgs/Actuators     (velocity[6], rad/s)

#include <array>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "actuator_msgs/msg/actuators.hpp"

#include "hexa_tilt_sim/params.hpp"
#include "hexa_tilt_sim/allocation.hpp"

using namespace hTsim;

class WrenchToActuators : public rclcpp::Node
{
public:
    WrenchToActuators() : Node("wrench_to_actuators")
    {
        // Reliable QoS: ros_gz_bridge subscribes reliably, and a reliable sub cannot
        // receive from a best-effort pub — using SensorDataQoS here silently drops all
        // motor commands at the bridge.
        pub_ = create_publisher<actuator_msgs::msg::Actuators>("/hexa/command/motor_speed", 10);

        sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/hexa/control", 10,
            [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (msg->data.size() < 6) return;
                got_cmd_ = true;
                double F[6];
                for (int i = 0; i < 6; ++i) F[i] = msg->data[i];

                // Identical conversion to dynamics_node: wrench -> Omega^2 -> Omega_des
                const auto osq   = allocate_motor_speeds(F);
                const auto omega = omega_sq_to_omega_des(osq);

                actuator_msgs::msg::Actuators out;
                out.header.stamp = get_clock()->now();
                out.velocity.assign(omega.begin(), omega.end());  // 6 rad/s commands
                pub_->publish(out);
            });

        // Publish a hover command until the controller comes online, so the vehicle
        // does not free-fall during startup.
        const auto omega_hover = hover_motor_speeds();
        hover_.velocity.assign(omega_hover.begin(), omega_hover.end());
        startup_timer_ = create_wall_timer(
            std::chrono::milliseconds(20),
            [this]() {
                if (pub_->get_subscription_count() == 0) return;
                if (got_cmd_) { startup_timer_->cancel(); return; }
                hover_.header.stamp = get_clock()->now();
                pub_->publish(hover_);
            });

        RCLCPP_INFO(get_logger(), "wrench_to_actuators up: /hexa/control -> /hexa/command/motor_speed");
    }

private:
    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr startup_timer_;
    actuator_msgs::msg::Actuators hover_;
    bool got_cmd_ = false;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WrenchToActuators>());
    rclcpp::shutdown();
    return 0;
}
