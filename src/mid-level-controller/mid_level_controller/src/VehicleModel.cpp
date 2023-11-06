#include <array>
#include "mid_level_controller/VehicleModel.hpp"

namespace mid_level_controller {

    void VehicleModel::step(
            const rclcpp::Duration& elapsed,
            double motor_throttle,
            double steering_servo,
            double battery_voltage,
            lab_msgs::msg::Pose2D &pose,
            double &speed) {
        // Set deltas to 0
        double d_px{0}, d_py{0}, d_yaw{0}, d_speed{0};
        // Call overloaded step function
        step(
                elapsed,
                motor_throttle,
                steering_servo,
                battery_voltage,
                pose,
                speed,
                d_px, d_py, d_yaw,
                d_speed
        );
    }


    void VehicleModel::step(
            const rclcpp::Duration& elapsed,
            double motor_throttle,
            double steering_servo,
            double battery_voltage,
            lab_msgs::msg::Pose2D &pose,
            double &speed,
            double &d_px, double &d_py, double &d_yaw,
            double &d_speed) {
        // Dynamics Parameters
        const std::array<double, 10> p = {1.004582, -0.142938, 0.195236, 3.560576, -2.190728,
                                          -9.726828, 2.515565, 1.321199, 0.032208, -0.012863};
        // Steering Servo Delta
        const double delta = steering_servo + p[9 - 1];

        const double f = motor_throttle;

        d_px = p[1 - 1] * speed * (1 + p[2 - 1] * delta * delta) * cos(pose.yaw + p[3 - 1] * delta + p[10 - 1]);
        d_py = p[1 - 1] * speed * (1 + p[2 - 1] * delta * delta) * sin(pose.yaw + p[3 - 1] * delta + p[10 - 1]);
        d_yaw = p[4 - 1] * speed * delta;
        d_speed = p[5 - 1] * speed +
                  (p[6 - 1] + p[7 - 1] * battery_voltage) * ((f >= 0) ? (1.0) : (-1.0)) * pow(fabs(f), p[8 - 1]);

        auto elapsed_in_seconds = elapsed.seconds();
        pose.x += elapsed_in_seconds * d_px;
        pose.y += elapsed_in_seconds * d_py;
        pose.yaw += elapsed_in_seconds * d_yaw;
        speed += elapsed_in_seconds * d_speed;
    }
}