#pragma once

#include <vector>
#include <cmath>

#include <rclcpp/duration.hpp>
#include <lab_msgs/msg/pose2_d.hpp>

namespace mid_level_controller {
    class VehicleModel {
    public:
        static void step(
                const rclcpp::Duration& elapsed,
                double motor_throttle,
                double steering_servo,
                double battery_voltage,
                lab_msgs::msg::Pose2D &pose,
                double &speed);

        static void step(
                const rclcpp::Duration& elapsed,
                double motor_throttle,
                double steering_servo,
                double battery_voltage,
                lab_msgs::msg::Pose2D &pose,
                double &speed,
                double &d_px, double &d_py, double &d_yaw,
                double &d_speed);
    };
}