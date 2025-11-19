#include "rclcpp/rclcpp.hpp"
#include "swerve_interfaces/msg/chassis_speeds.hpp"

class KinematicsNode : public rclcpp::Node
{
public:
    KinematicsNode() : Node("kinematics")
    {
        this->chassis_speeds_subscriber = this->create_subscription<swerve_interfaces::msg::ChassisSpeeds>("swerve/desired_speeds", 10, std::bind(&KinematicsNode::desired_speeds_callback, this, std::placeholders::_1));
    }

private:
    void desired_speeds_callback(swerve_interfaces::msg::ChassisSpeeds desired_speeds)
    {
        RCLCPP_INFO(this->get_logger(), "x: %.2f y: %.2f theta: %.2f", desired_speeds.x, desired_speeds.y, desired_speeds.theta);
    }

    rclcpp::Subscription<swerve_interfaces::msg::ChassisSpeeds>::SharedPtr chassis_speeds_subscriber;
};