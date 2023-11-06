#pragma once

#include <lab_msgs/msg/trajectory_point.hpp>
#include <rclcpp/time.hpp>


namespace mid_level_controller {
    struct InterPolatedTrajectoryPoint {
        double position_x = 0;
        double position_y = 0;
        double velocity_x = 0;
        double velocity_y = 0;
        double acceleration_x = 0;
        double acceleration_y = 0;
        double yaw = 0;
        double speed = 0;
        double curvature = 0;
    };

    class TrajectoryInterpolation {
    public:
        static auto
        interpolate(lab_msgs::msg::TrajectoryPoint start, lab_msgs::msg::TrajectoryPoint end, const rclcpp::Time& now)
        -> InterPolatedTrajectoryPoint;
    };
}