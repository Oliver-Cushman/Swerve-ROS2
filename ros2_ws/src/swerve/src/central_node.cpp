#include "swerve_interfaces/msg/gamepad_state.hpp"
#include "swerve_interfaces/msg/chassis_speeds.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include "util/constants.h"

class CentralNode : public rclcpp::Node
{
public:
    CentralNode() : Node("central")
    {
        this->input_subscriber = this->create_subscription<swerve_interfaces::msg::GamepadState>("teleop/gamepad_input", 10, std::bind(&CentralNode::input_callback, this, std::placeholders::_1));
        this->desired_speeds_publisher = this->create_publisher<swerve_interfaces::msg::ChassisSpeeds>("swerve/desired_speeds", 10);
    }

private:
    void input_callback(const swerve_interfaces::msg::GamepadState gamepad_state)
    {
        auto desired_speeds = swerve_interfaces::msg::ChassisSpeeds();
        // Switch x and y, field coordinate system
        desired_speeds.x = gamepad_state.left_y * constants::Swerve::MAX_LINEAR_VELOCITY;
        desired_speeds.y = gamepad_state.left_x * constants::Swerve::MAX_LINEAR_VELOCITY;
        desired_speeds.theta = gamepad_state.right_x * constants::Swerve::MAX_ANGULAR_VELOCITY;
        this->desired_speeds_publisher->publish(desired_speeds);
    }

    rclcpp::Subscription<swerve_interfaces::msg::GamepadState>::SharedPtr input_subscriber;
    rclcpp::Publisher<swerve_interfaces::msg::ChassisSpeeds>::SharedPtr desired_speeds_publisher;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CentralNode>());
    rclcpp::shutdown();
    return 0;
}