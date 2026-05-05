#include <array>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "quadrotor_sim/params.hpp"
#include "quadrotor_sim/dynamics.hpp"
#include "quadrotor_sim/allocation.hpp"

using namespace qsim;

// Subscribes to /uav/state (16D) and publishes derived geometric quantities.
// On a real drone, replace dynamics_node with a sensor/EKF node that publishes
// /uav/state in the same format — this node runs unchanged.
class KinematicsNode : public rclcpp::Node
{
public:
    KinematicsNode() : Node("kinematics_node"), state_received_(false), tf_div_(0)
    {
        state_.fill(0.0);

        pub_pose_   = create_publisher<geometry_msgs::msg::PoseStamped>("/uav/pose",         10);
        pub_euler_  = create_publisher<geometry_msgs::msg::Vector3Stamped>("/uav/euler",     10);
        pub_motors_ = create_publisher<std_msgs::msg::Float64MultiArray>("/uav/motors",      10);
        pub_omega_  = create_publisher<std_msgs::msg::Float64MultiArray>("/uav/motor_omega", 10);

        sub_state_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/uav/state", 10,
            [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (msg->data.size() < 16) return;
                std::lock_guard<std::mutex> lk(mutex_);
                for (int i = 0; i < 16; ++i) state_[i] = msg->data[i];
                state_received_ = true;
            });

        tf_broadcaster_        = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

        geometry_msgs::msg::TransformStamped world_tf;
        world_tf.header.stamp    = get_clock()->now();
        world_tf.header.frame_id = "map";
        world_tf.child_frame_id  = "world";
        world_tf.transform.rotation.w = 1.0;
        static_tf_broadcaster_->sendTransform(world_tf);

        timer_ = create_wall_timer(
            std::chrono::microseconds(static_cast<long>(DT_PUB * 1e6)),
            std::bind(&KinematicsNode::publish_viz, this));

        RCLCPP_INFO(get_logger(), "Kinematics node started at %.0f Hz.", 1.0 / DT_PUB);
    }

private:
    void publish_viz()
    {
        std::array<double, 16> state;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            if (!state_received_) return;
            state = state_;
        }

        const auto now = get_clock()->now();
        const double phi = state[6], theta = state[7], psi = state[8];

        double qx, qy, qz, qw;
        euler_to_quaternion(phi, theta, psi, qx, qy, qz, qw);

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp    = now;
        pose_msg.header.frame_id = "world";
        pose_msg.pose.position.x = state[0];
        pose_msg.pose.position.y = state[1];
        pose_msg.pose.position.z = state[2];
        pose_msg.pose.orientation.x = qx;
        pose_msg.pose.orientation.y = qy;
        pose_msg.pose.orientation.z = qz;
        pose_msg.pose.orientation.w = qw;
        pub_pose_->publish(pose_msg);

        geometry_msgs::msg::Vector3Stamped euler_msg;
        euler_msg.header.stamp    = now;
        euler_msg.header.frame_id = "world";
        euler_msg.vector.x = phi;
        euler_msg.vector.y = theta;
        euler_msg.vector.z = psi;
        pub_euler_->publish(euler_msg);

        double T, tau_phi, tau_theta, tau_psi, T_i[4];
        compute_wrench(&state[12], T, tau_phi, tau_theta, tau_psi, T_i);

        std_msgs::msg::Float64MultiArray motors_msg;
        motors_msg.data = {T_i[0], T_i[1], T_i[2], T_i[3]};
        pub_motors_->publish(motors_msg);

        std_msgs::msg::Float64MultiArray omega_msg;
        omega_msg.data = {state[12], state[13], state[14], state[15]};
        pub_omega_->publish(omega_msg);

        // TF at 10 Hz (every 5th viz publish)
        if (++tf_div_ >= 5) {
            tf_div_ = 0;
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp      = now;
            tf_msg.header.frame_id   = "world";
            tf_msg.child_frame_id    = "base_link";
            tf_msg.transform.translation.x = state[0];
            tf_msg.transform.translation.y = state[1];
            tf_msg.transform.translation.z = state[2];
            tf_msg.transform.rotation.x = qx;
            tf_msg.transform.rotation.y = qy;
            tf_msg.transform.rotation.z = qz;
            tf_msg.transform.rotation.w = qw;
            tf_broadcaster_->sendTransform(tf_msg);
        }
    }

    std::array<double, 16> state_;
    std::mutex mutex_;
    bool state_received_;
    int tf_div_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr   pub_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_euler_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr   pub_motors_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr   pub_omega_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_state_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>       tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    auto node = std::make_shared<KinematicsNode>();
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
