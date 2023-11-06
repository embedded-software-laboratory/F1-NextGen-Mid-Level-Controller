#pragma once

#include <lab_msgs/msg/path_point.hpp>

namespace mid_level_controller {
    struct PathInterpolation {

        lab_msgs::msg::Pose2D pose{};

        double velocity_x;

        double velocity_y;

        double acceleration_x;

        double acceleration_y;

        double speed;

        double curvature;

        PathInterpolation(double s_queried, lab_msgs::msg::PathPoint start_point, lab_msgs::msg::PathPoint end_point);
    };
}