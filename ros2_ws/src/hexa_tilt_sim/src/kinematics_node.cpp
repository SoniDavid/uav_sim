#include <array>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "hexa_tilt_sim/params.hpp"
#include "hexa_tilt_sim/dynamics.hpp"
#include "hexa_tilt_sim/allocation.hpp"

using namespace hTsim;

// Subscribes to /hexa/state (18D) and publishes derived geometric quantities.
class HexaKinematicsNode : public rclcpp::Node
{
public:
    HexaKinematicsNode() : Node("hexa_kinematics"), state_received_(false), tf_div_(0)
    {
        state_.fill(0.0);
        path_msg_.header.frame_id = "world";

        pub_pose_   = create_publisher<geometry_msgs::msg::PoseStamped>("/hexa/pose",         10);
        pub_euler_  = create_publisher<geometry_msgs::msg::Vector3Stamped>("/hexa/euler",     10);
        pub_motors_ = create_publisher<std_msgs::msg::Float64MultiArray>("/hexa/motors",      10);
        pub_omega_  = create_publisher<std_msgs::msg::Float64MultiArray>("/hexa/motor_omega", 10);
        pub_path_   = create_publisher<nav_msgs::msg::Path>("/hexa/path",                     10);

        sub_state_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/hexa/state", 10,
            [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (msg->data.size() < 18) return;
                std::lock_guard<std::mutex> lk(mutex_);
                for (int i = 0; i < 18; ++i) state_[i] = msg->data[i];
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
            std::bind(&HexaKinematicsNode::publish_viz, this));

        RCLCPP_INFO(get_logger(), "Hexa kinematics node started at %.0f Hz.", 1.0 / DT_PUB);
    }

private:
    void publish_viz()
    {
        std::array<double, 18> state;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            if (!state_received_) return;
            state = state_;
        }

        const auto now = get_clock()->now();

        // χ = state[0:6] = [x, y, z, φ, θ, ψ]
        const double phi = state[3], theta = state[4], psi = state[5];

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

        // Accumulate trail and publish
        path_msg_.header.stamp = now;
        path_msg_.poses.push_back(pose_msg);
        pub_path_->publish(path_msg_);

        geometry_msgs::msg::Vector3Stamped euler_msg;
        euler_msg.header.stamp    = now;
        euler_msg.header.frame_id = "world";
        euler_msg.vector.x = phi;
        euler_msg.vector.y = theta;
        euler_msg.vector.z = psi;
        pub_euler_->publish(euler_msg);

        // Ω = state[12:18]; forward model → 6D wrench, then extract individual contributions
        // Publish per-motor z-force as "thrust" (f_z_i = M_PHI[2][i] * Ω_i²)
        std_msgs::msg::Float64MultiArray motors_msg;
        motors_msg.data.resize(6);
        for (int i = 0; i < 6; ++i)
            motors_msg.data[i] = M_PHI[2][i] * state[12+i] * state[12+i];
        pub_motors_->publish(motors_msg);

        std_msgs::msg::Float64MultiArray omega_msg;
        omega_msg.data.assign(&state[12], &state[18]);
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

    std::array<double, 18> state_;
    std::mutex mutex_;
    bool state_received_;
    int  tf_div_;

    nav_msgs::msg::Path path_msg_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr   pub_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_euler_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr   pub_motors_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr   pub_omega_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr               pub_path_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_state_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>       tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    auto node = std::make_shared<HexaKinematicsNode>();
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
