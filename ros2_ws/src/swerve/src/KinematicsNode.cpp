#include "rclcpp/rclcpp.hpp"
#include "util/Constants.h"
#include "swerve_interfaces/msg/chassis_speeds.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <memory>

class KinematicsNode : public rclcpp::Node
{
public:
    KinematicsNode() : Node("kinematics")
    {
        this->chassis_speeds_subscriber = this->create_subscription<swerve_interfaces::msg::ChassisSpeeds>(
            "swerve/desired_speeds",
            10,
            std::bind(&KinematicsNode::desired_speeds_callback, this, std::placeholders::_1));
        this->imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
            "sensors/imu",
            10,
            std::bind(&KinematicsNode::imu_callback, this, std::placeholders::_1));
        this->orientation = geometry_msgs::msg::Quaternion();
    }

private:
    void desired_speeds_callback(swerve_interfaces::msg::ChassisSpeeds desired_speeds)
    {
        float vx = desired_speeds.x * constants::Swerve::MAX_LINEAR_VELOCITY;
        float vy = desired_speeds.y * constants::Swerve::MAX_LINEAR_VELOCITY;
        float w = desired_speeds.theta * constants::Swerve::MAX_ANGULAR_VELOCITY;
        RCLCPP_INFO(this->get_logger(), "x: %.2f y: %.2f theta: %.2f", vx, vy, w);
    }

    void imu_callback(sensor_msgs::msg::Imu imu_data)
    {
        auto imu_orientation = imu_data.orientation;
        this->orientation.set__w(imu_orientation.w);
        this->orientation.set__x(imu_orientation.x);
        this->orientation.set__y(imu_orientation.y);
        this->orientation.set__z(imu_orientation.z);
    }

    rclcpp::Subscription<swerve_interfaces::msg::ChassisSpeeds>::SharedPtr chassis_speeds_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber;
    geometry_msgs::msg::Quaternion orientation;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinematicsNode>());
    rclcpp::shutdown();
    return 0;
}