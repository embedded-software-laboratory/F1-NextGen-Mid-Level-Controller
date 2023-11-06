#pragma once

#include "casadi_mpc_fn.h"
#include <map>
#include <string>
#include <array>
#include <vector>
#include "VehicleModel.hpp"

#include <lab_msgs/msg/vehicle_state.hpp>
#include <lab_msgs/msg/vehicle_command_trajectory.hpp>

#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>

namespace mid_level_controller {

#define MPC_DELAY_COMPENSATION_STEPS (3)

    class MpcController {
        bool stop_vehicle_{false};

        std::map<std::string, std::vector<casadi_real> > casadi_vars;

        std::map<std::string, std::array<casadi_int, 2> > casadi_vars_size;

        std::vector<casadi_real *> casadi_arguments;

        std::vector<casadi_real *> casadi_results;

        const size_t MPC_prediction_steps = 6;

        const size_t MPC_control_steps = 3;

        const rclcpp::Duration dt_control_loop = rclcpp::Duration(0, 2e+7);

        const rclcpp::Duration dt_MPC = rclcpp::Duration(0, 5e+7);


        std::vector<double> dynamics_parameters = {1.004582, -0.142938, 0.195236, 3.560576, -2.190728, -9.726828, 2.515565, 1.321199, 0.032208,
                                                   -0.012863};

        double battery_voltage_lowpass_filtered = 8;


        double motor_output_history[MPC_DELAY_COMPENSATION_STEPS]{};

        double steering_output_history[MPC_DELAY_COMPENSATION_STEPS]{};

        lab_msgs::msg::VehicleState delayCompensationPrediction(
                const lab_msgs::msg::VehicleState &vehicleState
        );

        bool interpolateReferenceTrajectory(
                const rclcpp::Time& t_now,
                const lab_msgs::msg::VehicleCommandTrajectory &commandTrajectory,
                std::vector<double> &out_mpc_reference_trajectory_x,
                std::vector<double> &out_mpc_reference_trajectory_y
        ) const;

        void optimizeControlInputs(
                const lab_msgs::msg::VehicleState &vehicleState_predicted_start,
                const std::vector<double> &mpc_reference_trajectory_x,
                const std::vector<double> &mpc_reference_trajectory_y,
                double &out_motor_throttle,
                double &out_steering_servo
        );

        void resetOptimizer();

    public:
        MpcController();

        void update(
                const rclcpp::Time& t_now,
                const lab_msgs::msg::VehicleState &vehicleState,
                const lab_msgs::msg::VehicleCommandTrajectory &commandTrajectory,
                double &out_motor_throttle,
                double &out_steering_servo
        );

        [[nodiscard]] auto getStopVehicle() const -> bool;

    };
}