// odom_to_state : bridge Gazebo odometry back into the analytical-sim state format.
//
// The Gazebo plant's state is published (via OdometryPublisher + ros_gz bridge) as
// nav_msgs/Odometry. The controllers and viz expect the 18-D /hexa/state vector that
// dynamics_node produces. This node performs that translation so nothing downstream
// changes.
//
//   sub : /model/hexa/odometry   nav_msgs/Odometry
//   pub : /hexa/state            std_msgs/Float64MultiArray (18-D)
//
// State layout (see dynamics.hpp): [x,y,z, phi,theta,psi, u,v,w, p,q,r, Omega1..6]
// Motor-speed slots [12:18] are zero-filled: the controllers never read them
// (verified), and true rotor speeds are not exposed by odometry.
//
// Twist-frame note (REP-145): nav_msgs/Odometry.twist is defined in the child (body)
// frame, matching [u,v,w,p,q,r]. Gz OdometryPublisher normally follows this, but if a
// given Gz version reports linear velocity in the world frame, set the ROS param
// linear_twist_world:=true to rotate it into the body frame.

#include <array>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::array;

namespace {

// Quaternion (x,y,z,w) -> aerospace ZYX Euler (phi=roll, theta=pitch, psi=yaw).
// Inverse of euler_to_quaternion() in dynamics.hpp.
void quat_to_euler(double qx, double qy, double qz, double qw,
                   double& phi, double& theta, double& psi)
{
    phi = std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
    const double s = std::clamp(2.0 * (qw * qy - qz * qx), -1.0, 1.0);
    theta = std::asin(s);
    psi = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
}

// v_body = R(q)^T * v_world , with q = body->world rotation.
void world_to_body(double qx, double qy, double qz, double qw,
                   double vx, double vy, double vz,
                   double& bx, double& by, double& bz)
{
    // R^T rows (== R columns). Standard rotation matrix from unit quaternion.
    const double r00 = 1 - 2 * (qy * qy + qz * qz), r01 = 2 * (qx * qy + qz * qw), r02 = 2 * (qx * qz - qy * qw);
    const double r10 = 2 * (qx * qy - qz * qw), r11 = 1 - 2 * (qx * qx + qz * qz), r12 = 2 * (qy * qz + qx * qw);
    const double r20 = 2 * (qx * qz + qy * qw), r21 = 2 * (qy * qz - qx * qw), r22 = 1 - 2 * (qx * qx + qy * qy);
    // R^T * v : use columns of R (i.e. transpose multiply)
    bx = r00 * vx + r10 * vy + r20 * vz;
    by = r01 * vx + r11 * vy + r21 * vz;
    bz = r02 * vx + r12 * vy + r22 * vz;
}

}  // namespace

class OdomToState : public rclcpp::Node
{
public:
    OdomToState() : Node("odom_to_state")
    {
        linear_twist_world_ = declare_parameter<bool>("linear_twist_world", false);

        pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/hexa/state", 10);
        sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/model/hexa/odometry", rclcpp::SensorDataQoS(),
            std::bind(&OdomToState::on_odom, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
                    "odom_to_state up: /model/hexa/odometry -> /hexa/state (linear_twist_world=%s)",
                    linear_twist_world_ ? "true" : "false");
    }

private:
    void on_odom(nav_msgs::msg::Odometry::SharedPtr o)
    {
        const auto& q = o->pose.pose.orientation;
        double phi, theta, psi;
        quat_to_euler(q.x, q.y, q.z, q.w, phi, theta, psi);

        double u = o->twist.twist.linear.x, v = o->twist.twist.linear.y, w = o->twist.twist.linear.z;
        if (linear_twist_world_)
            world_to_body(q.x, q.y, q.z, q.w, u, v, w, u, v, w);

        array<double, 18> s{};
        s[0] = o->pose.pose.position.x;
        s[1] = o->pose.pose.position.y;
        s[2] = o->pose.pose.position.z;
        s[3] = phi;  s[4] = theta;  s[5] = psi;
        s[6] = u;    s[7] = v;      s[8] = w;
        s[9]  = o->twist.twist.angular.x;
        s[10] = o->twist.twist.angular.y;
        s[11] = o->twist.twist.angular.z;
        // s[12..17] = 0 (rotor speeds not observed; unused by controllers)

        std_msgs::msg::Float64MultiArray msg;
        msg.data.assign(s.begin(), s.end());
        pub_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    bool linear_twist_world_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToState>());
    rclcpp::shutdown();
    return 0;
}
