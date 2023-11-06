#pragma once

#include <map>
#include <memory>

#include <lab_msgs/msg/vehicle_command_trajectory.hpp>
#include <lab_msgs/msg/vehicle_command_direct.hpp>
#include <lab_msgs/msg/vehicle_command_speed_curvature.hpp>
#include <lab_msgs/msg/vehicle_command_path_tracking.hpp>

#include "MpcController.hpp"
#include "TrajectoryInterpolation.hpp"

namespace mid_level_controller {

#define TRAJECTORY_TRACKING_STATISTICS_BUFFER_SIZE 1500

    enum class ControllerState {
        Stop,
        Direct,
        SpeedCurvature,
        Trajectory,
        PathTracking
    };

    class Controller {
        MpcController mpcController;

        lab_msgs::msg::VehicleState m_vehicleState{};

        lab_msgs::msg::VehicleCommandDirect m_vehicleCommandDirect{};

        lab_msgs::msg::VehicleCommandSpeedCurvature m_vehicleCommandSpeedCurvature{};

        lab_msgs::msg::VehicleCommandTrajectory m_vehicleCommandTrajectory{};

        lab_msgs::msg::VehicleCommandPathTracking m_vehicleCommandPathTracking{};

        ControllerState state = ControllerState::Stop;

        const rclcpp::Duration command_timeout{1, 0};

        double speed_throttle_error_integral = 0;

        double trajectory_tracking_statistics_longitudinal_errors[TRAJECTORY_TRACKING_STATISTICS_BUFFER_SIZE]{};

        double trajectory_tracking_statistics_lateral_errors[TRAJECTORY_TRACKING_STATISTICS_BUFFER_SIZE]{};

        size_t trajectory_tracking_statistics_index = 0;

        double speedController(double speed_measured, double speed_target);

        [[nodiscard]] std::shared_ptr<InterPolatedTrajectoryPoint> interpolateTrajectory(const rclcpp::Time &t_now) const;

        void trajectoryTrackingStatisticsUpdate(const rclcpp::Time &t_now);

    public:
        Controller();

        void updateVehicleState(lab_msgs::msg::VehicleState vehicleState);

        void getControlSignals(const rclcpp::Time &t_now, double &out_motor_throttle, double &out_steering_servo);

        void getStopSignals(double &motor_throttle, double &steering_servo) const;

        void reset();

        void updateDirectCommand(lab_msgs::msg::VehicleCommandDirect msg);

        void updateSpeedCurvatureCommand(lab_msgs::msg::VehicleCommandSpeedCurvature msg);

        void updateTrajectoryCommand(lab_msgs::msg::VehicleCommandTrajectory msg);

        void updatePathTrackingCommand(lab_msgs::msg::VehicleCommandPathTracking msg);
    };
}