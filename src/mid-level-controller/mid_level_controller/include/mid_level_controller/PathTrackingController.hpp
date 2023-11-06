#pragma once

#include <lab_msgs/msg/vehicle_state.hpp>
#include <lab_msgs/msg/vehicle_command_path_tracking.hpp>
#include <lab_msgs/msg/path_point.hpp>

namespace mid_level_controller {
    class PathTrackingController {

        static lab_msgs::msg::Pose2D find_reference_pose(
                const std::vector<lab_msgs::msg::PathPoint> &path,
                double x,
                double y
        );

    public:
        PathTrackingController() = default;

        static double control_steering_servo(
                const lab_msgs::msg::VehicleState &vehicleState,
                const lab_msgs::msg::VehicleCommandPathTracking &commandPathTracking
        );
    };
}