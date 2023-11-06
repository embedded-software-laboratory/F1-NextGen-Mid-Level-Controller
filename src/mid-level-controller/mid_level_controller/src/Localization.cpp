#include "mid_level_controller/Localization.hpp"
#include <cmath>

namespace mid_level_controller {
    void filter_update_step(const LocalizationState &previous, LocalizationState &current) {
        lab_msgs::msg::Pose2D new_pose = previous.pose;

        // Dead reckoning update
        {
            double delta_s = current.odometer_distance - previous.odometer_distance;
            double delta_yaw = remainder(current.imu_yaw - previous.imu_yaw, 2 * M_PI);

            // ignore signal discontinuities
            if (!(-0.5 < delta_s && delta_s < 0.5)) {
                delta_s = 0;
            }

            if (!(-1.5 < delta_yaw && delta_yaw < 1.5)) {
                delta_yaw = 0;
            }

            new_pose.yaw = new_pose.yaw + delta_yaw;
            new_pose.x = new_pose.x + delta_s * cos(new_pose.yaw);
            new_pose.y = new_pose.y + delta_s * sin(new_pose.yaw);
        }

        // IPS update
        if (current.has_valid_observation) {
            const double s = 0.3;

            double dx = current.vehicleObservation.pose.x - new_pose.x;
            double dy = current.vehicleObservation.pose.y - new_pose.y;
            double dyaw = current.vehicleObservation.pose.yaw - new_pose.yaw;
            dyaw = remainder(dyaw, 2 * M_PI);

            new_pose.x = new_pose.x + s * dx;
            new_pose.y = new_pose.y + s * dy;
            new_pose.yaw = new_pose.yaw + s * dyaw;
        }

        new_pose.yaw = remainder(new_pose.yaw, 2 * M_PI);
        current.pose = new_pose;
    }

    lab_msgs::msg::Pose2D Localization::update(const rclcpp::Time &now, const rclcpp::Duration &elapsed,
                                               lab_msgs::msg::VehicleState vehicleState,
                                               lab_msgs::msg::VehicleObservation sample_vehicleObservation,
                                               const rclcpp::Duration &sample_vehicleObservation_age) {
        // Save new sensor data, update current step
        {
            LocalizationState localizationStateNew;
            localizationStateNew.t = now;
            localizationStateNew.imu_yaw = vehicleState.imu_yaw;
            localizationStateNew.odometer_distance = vehicleState.odometer_distance;
            write_next_state(localizationStateNew);

            filter_update_step(get_state(LOCALIZATION_BUFFER_SIZE - 2), get_state(LOCALIZATION_BUFFER_SIZE - 1));
        }

        // Check for new observation. Reprocess if necessary
        if (sample_vehicleObservation_age < rclcpp::Duration(10, 0)) {
            int reprocessing_start_index = -1;

            // search for state index corresponding to the given observation
            for (int i = LOCALIZATION_BUFFER_SIZE - 1; i > 0; i--) {
                LocalizationState &state_i = get_state(i);

                // found the corresponding state
                if (state_i.t <= rclcpp::Time(sample_vehicleObservation.timings.create_stamp.nanoseconds, RCL_ROS_TIME) &&
                    rclcpp::Time(sample_vehicleObservation.timings.create_stamp.nanoseconds, RCL_ROS_TIME) <
                    (state_i.t + elapsed)) {
                    // have we already done this on a previous update()? then skip it
                    if (state_i.has_valid_observation)
                        break;

                    state_i.has_valid_observation = true;
                    state_i.vehicleObservation = sample_vehicleObservation;

                    // we have a new observation, need to do reprocessing
                    reprocessing_start_index = i;
                    break;
                }
            }

            // reprocessing
            if (reprocessing_start_index > 0) {
                for (int i = reprocessing_start_index; i < LOCALIZATION_BUFFER_SIZE; ++i) {
                    filter_update_step(get_state(i - 1), get_state(i));
                }
            }
        }

        // output latest pose
        return get_state(LOCALIZATION_BUFFER_SIZE - 1).pose;
    }

    void Localization::reset() {
        for (auto &i: state_buffer) {
            i.pose = lab_msgs::msg::Pose2D();
        }
    }

}  // namespace mid_level_controller