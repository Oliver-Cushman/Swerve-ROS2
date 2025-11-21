#include "rclcpp/rclcpp.hpp"
#include "util/Constants.h"
#include "util/math/GeometryUtil.h"
#include "swerve_interfaces/msg/chassis_speeds.hpp"
#include "swerve_interfaces/msg/swerve_module_state.hpp"
#include "swerve_interfaces/msg/swerve_module_state_arr.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <memory>
#include <cmath>

using namespace geometry_util;
using namespace swerve_interfaces::msg;

class KinematicsNode : public rclcpp::Node
{
public:
    KinematicsNode() : Node("kinematics"), desired_module_states(4)
    {
        this->chassis_speeds_subscriber = this->create_subscription<ChassisSpeeds>(
            "swerve/desired_speeds",
            10,
            std::bind(&KinematicsNode::desired_speeds_callback, this, std::placeholders::_1));
        this->imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
            "sensors/imu",
            10,
            std::bind(&KinematicsNode::imu_callback, this, std::placeholders::_1));
        this->module_states_publisher = this->create_publisher<SwerveModuleStateArr>(
            "swerve/desired_module_states",
            10);
        this->yaw = Rotation2d();
    }

private:
    void desired_speeds_callback(ChassisSpeeds desired_speeds)
    {
        float vx = desired_speeds.x * constants::Swerve::MAX_LINEAR_VELOCITY;
        float vy = desired_speeds.y * constants::Swerve::MAX_LINEAR_VELOCITY;
        float w = desired_speeds.theta * constants::Swerve::MAX_ANGULAR_VELOCITY;
        bool field_relative = desired_speeds.field_relative;

        Vector2d lv = Vector2d(vx, vy);
        if (field_relative) {
            lv = lv.rotate_by(this->yaw * -1);
        }
        Vector3d w_vector = Vector3d(0, 0, w);

        for (int i = 0; i < 4; i++)
        {
            Translation2d module_translation = constants::Swerve::SWERVE_MOD_TRANSLATIONS[i];
            Vector3d r = Vector3d(module_translation.x, module_translation.y, 0);
            Vector3d angular = r.cross(w_vector);
            Vector2d module_vector = lv + Vector2d(angular.x, angular.y);
            Rotation2d module_angle = module_vector.angle();
            SwerveModuleState module_state = SwerveModuleState();
            module_state.set__angle(module_angle.angle);
            module_state.set__velocity(module_vector.magnitude());
            this->desired_module_states[i] = module_state;
        }

        SwerveModuleStateArr module_states_msg = SwerveModuleStateArr();
        module_states_msg.set__states(this->desired_module_states);
        this->module_states_publisher->publish(module_states_msg);
    }

    void imu_callback(sensor_msgs::msg::Imu imu_data)
    {
        float w = imu_data.orientation.w;
        float x = imu_data.orientation.x;
        float y = imu_data.orientation.y;
        float z = imu_data.orientation.z;

        this->yaw = Rotation2d(2.0f * (w * z + x * y), w * w + x * x - y * y - z * z);
    }

    rclcpp::Subscription<ChassisSpeeds>::SharedPtr chassis_speeds_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber;
    rclcpp::Publisher<SwerveModuleStateArr>::SharedPtr module_states_publisher;
    Rotation2d yaw;
    std::vector<SwerveModuleState> desired_module_states;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinematicsNode>());
    rclcpp::shutdown();
    return 0;
}