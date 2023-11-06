#pragma once

#include <rclcpp/rclcpp.hpp>
#include <lab_msgs/msg/pose2_d.hpp>
#include <lab_msgs/msg/vehicle_observation.hpp>
#include <lab_msgs/msg/vehicle_state.hpp>

namespace mid_level_controller {

#define LOCALIZATION_BUFFER_SIZE (512)

    struct LocalizationState {

        rclcpp::Time t = rclcpp::Time(0, 0, RCL_ROS_TIME);
        double imu_yaw = 0;
        double odometer_distance = 0;
        bool has_valid_observation = false;
        lab_msgs::msg::VehicleObservation vehicleObservation;
        lab_msgs::msg::Pose2D pose;

        LocalizationState() {
            pose.x = 0; // best estimate of the pose
            pose.y = 0;
            pose.yaw = 0;
        }
    };

    class Localization {
    private:
        LocalizationState state_buffer[LOCALIZATION_BUFFER_SIZE];

        size_t state_buffer_index = 0;

        LocalizationState &get_state(size_t i) {
            assert(i < LOCALIZATION_BUFFER_SIZE);
            return state_buffer[(i + state_buffer_index) % LOCALIZATION_BUFFER_SIZE];
        }

        void write_next_state(const LocalizationState &state) {
            state_buffer[state_buffer_index] = state;
            state_buffer_index = (state_buffer_index + 1) % LOCALIZATION_BUFFER_SIZE;
        }

    public:
        lab_msgs::msg::Pose2D update(
                const rclcpp::Time &now,
                const rclcpp::Duration &elapsed,
                lab_msgs::msg::VehicleState vehicleState,
                lab_msgs::msg::VehicleObservation sample_vehicleObservation,
                const rclcpp::Duration &sample_vehicleObservation_age
        );

        void reset();

    };
}