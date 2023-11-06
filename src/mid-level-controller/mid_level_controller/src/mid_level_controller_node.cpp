#include <rclcpp/rclcpp.hpp>

#include <lab_msgs/msg/vehicle_state.hpp>
#include <lab_msgs/msg/vehicle_observation.hpp>
#include <lab_msgs/msg/vehicle_command_direct.hpp>
#include <lab_msgs/msg/vehicle_command_speed_curvature.hpp>
#include <lab_msgs/msg/vehicle_command_trajectory.hpp>
#include <lab_msgs/msg/vehicle_command_path_tracking.hpp>
#include <lab_msgs/msg/pose2_d.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "mid_level_controller/Controller.hpp"
#include "mid_level_controller/Localization.hpp"
#include "mid_level_controller/SensorCalibration.hpp"

extern "C" {
#include "mid_level_controller/bcm2835.h"
#include "mid_level_controller/low_level_controller/spi.h"
}

using namespace std::chrono_literals;

class MidLevelControllerNode : public rclcpp::Node {
private:
    // Vehicle Data
    int vehicle_id_{0};
    lab_msgs::msg::VehicleObservation newest_vehicle_observation;

    // Publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_odom_base_link_publisher_;
    rclcpp::Publisher<lab_msgs::msg::VehicleState>::SharedPtr vehicle_state_publisher_;

    // Subscriber
    rclcpp::Subscription<lab_msgs::msg::VehicleObservation>::SharedPtr vehicle_observation_subscriber_;
    rclcpp::Subscription<lab_msgs::msg::VehicleCommandDirect>::SharedPtr vehicle_direct_command_subscriber_;
    rclcpp::Subscription<lab_msgs::msg::VehicleCommandSpeedCurvature>::SharedPtr
            vehicle_speed_curvature_command_subscriber_;
    rclcpp::Subscription<lab_msgs::msg::VehicleCommandTrajectory>::SharedPtr vehicle_trajectory_command_subscriber_;
    rclcpp::Subscription<lab_msgs::msg::VehicleCommandPathTracking>::SharedPtr vehicle_path_tracking_command_subscriber_;

    // Components
    std::shared_ptr<mid_level_controller::Localization> localization_;
    std::shared_ptr<mid_level_controller::Controller> controller_;

    rclcpp::TimerBase::SharedPtr timer_;

    void onVehicleObservation(const lab_msgs::msg::VehicleObservation::SharedPtr msg) {
        this->newest_vehicle_observation = *msg;
    }

    void onVehicleCommandDirect(const lab_msgs::msg::VehicleCommandDirect::SharedPtr msg) {
        this->controller_->updateDirectCommand(*msg);
    }

    void onVehicleCommandSpeedCurvature(const lab_msgs::msg::VehicleCommandSpeedCurvature::SharedPtr msg) {
        this->controller_->updateSpeedCurvatureCommand(*msg);
    }

    void onVehicleCommandTrajectory(const lab_msgs::msg::VehicleCommandTrajectory::SharedPtr msg) {
        this->controller_->updateTrajectoryCommand(*msg);
    }

    void onVehicleCommandPathTracking(const lab_msgs::msg::VehicleCommandPathTracking::SharedPtr msg) {
        this->controller_->updatePathTrackingCommand(*msg);
    }

    void onTimer() {
        // Current time and time since the last update
        const rclcpp::Time now = this->now();
        const rclcpp::Duration elapsed = rclcpp::Duration(0, 20000000ull);

        // Step 1: Get IPS observation
        auto vehicle_observation = this->newest_vehicle_observation;
        auto vehicle_observation_age =
                now - rclcpp::Time(vehicle_observation.timings.valid_after_stamp.nanoseconds, RCL_ROS_TIME);

        // Step 2: Get control signals from controller based on received commands and stop flag
        double motor_throttle = 0;
        double steering_servo = 0;
        this->controller_->getControlSignals(now, motor_throttle, steering_servo);

        // Step 3: Convert control signals for low level controller
        int n_transmission_attempts = 1;
        int transmission_successful = 1;

        // Motor deadband, to prevent small stall currents when standing still
        uint8_t motor_mode = SPI_MOTOR_MODE_BRAKE;
        if (motor_throttle > 0.05)
            motor_mode = SPI_MOTOR_MODE_FORWARD;
        if (motor_throttle < -0.05)
            motor_mode = SPI_MOTOR_MODE_REVERSE;

        spi_mosi_data_t spi_mosi_data;
        memset(&spi_mosi_data, 0, sizeof(spi_mosi_data_t));

        // Convert actuator input to low level controller units
        spi_mosi_data.motor_mode = motor_mode;
        spi_mosi_data.motor_pwm = int16_t(fabs(motor_throttle) * 400.0);
        // servo allows a range of 2400 (see servo_timer.c)
        spi_mosi_data.servo_command = int16_t(steering_servo * (-1200.0));

        // vehicle ID for LED flashing
        spi_mosi_data.vehicle_id = static_cast<uint8_t>(this->vehicle_id_);

        // Step 4: Exchange data with low level controller
        spi_miso_data_t spi_miso_data;

        spi_transfer(spi_mosi_data, &spi_miso_data, &n_transmission_attempts, &transmission_successful);

        if (transmission_successful && (spi_miso_data.status_flags & 1))  // IMU status
        {
            RCLCPP_ERROR(this->get_logger(), "Data transmission via TWI from IMU failed.");
        }

        // Step 5: Convert data to vehicle state
        lab_msgs::msg::VehicleState vehicle_state = SensorCalibration::convert(spi_miso_data);
        vehicle_state.is_real = true;

        if (transmission_successful) {
            // Step 6: Update Localization with observation and vehicle state, get pose and update vehicle state
            vehicle_state.pose = this->localization_->update(now, elapsed, vehicle_state, vehicle_observation,
                                                             vehicle_observation_age);

            vehicle_state.motor_throttle = motor_throttle;
            vehicle_state.steering_servo = steering_servo;
            vehicle_state.vehicle_id = this->vehicle_id_;
            vehicle_state.ips_update_age_nanoseconds = vehicle_observation_age.nanoseconds();

            vehicle_state.timings.create_stamp.nanoseconds = now.nanoseconds();
            vehicle_state.timings.valid_after_stamp.nanoseconds = (now +
                                                                   rclcpp::Duration(0, 60000000ull)).nanoseconds();

            // Step 7: Update controller with vehicle state
            this->controller_->updateVehicleState(vehicle_state);

            // Step 8: Publish vehicle state
            this->vehicle_state_publisher_->publish(vehicle_state);

            // Step 9: Publish odometry
            nav_msgs::msg::Odometry odom;
            odom.header.frame_id = "odom";
            odom.header.stamp = this->now();
            odom.child_frame_id = "camera_link";
            odom.pose.pose.position.x = vehicle_state.pose.x;
            odom.pose.pose.position.y = vehicle_state.pose.y;
            odom.pose.pose.position.z = 0;
            odom.pose.pose.orientation.x = 0.0;
            odom.pose.pose.orientation.y = 0.0;
            odom.pose.pose.orientation.z = sin(vehicle_state.pose.yaw / 2.0);
            odom.pose.pose.orientation.w = cos(vehicle_state.pose.yaw  / 2.0);

            odom.pose.covariance[0]  = 0.2;  ///< x
            odom.pose.covariance[7]  = 0.2;  ///< y
            odom.pose.covariance[35] = 0.4;  ///< yaw

            odom.twist.twist.linear.x = vehicle_state.speed;
            odom.twist.twist.linear.y = 0.0;
            odom.twist.twist.angular.z = vehicle_state.imu_yaw_rate;

            this->odometry_publisher_->publish(odom);

            // Step 10: Publish odom -> base_link transform
            geometry_msgs::msg::TransformStamped tf;
            tf.header.frame_id = "odom";
            tf.header.stamp = this->now();
            tf.child_frame_id = "camera_link";
            tf.transform.translation.x = vehicle_state.pose.x;
            tf.transform.translation.y = vehicle_state.pose.y;
            tf.transform.translation.z = 0.0;
            tf.transform.rotation.x = 0.0;
            tf.transform.rotation.y = 0.0;
            tf.transform.rotation.z = sin(vehicle_state.pose.yaw / 2.0);
            tf.transform.rotation.w = cos(vehicle_state.pose.yaw / 2.0);

            this->tf_odom_base_link_publisher_->sendTransform(tf);

        } else {
            RCLCPP_ERROR(this->get_logger(), "Data corruption on ATmega SPI bus. CRC mismatch. After %i attempts.",
                         n_transmission_attempts);
        }
    }

public:
    MidLevelControllerNode() : Node("mid_level_controller_node") {
        // Declare parameter
        this->declare_parameter("vehicle_id", 0);

        // Get vehicle id and assert it is valid
        this->get_parameter("vehicle_id", this->vehicle_id_);
        assert(this->vehicle_id_ > 0 && this->vehicle_id_ <= 255);

        // Hardware setup
        if (!bcm2835_init()) {
            RCLCPP_ERROR(this->get_logger(), "bcm2835_init failed.");
            exit(EXIT_FAILURE);
        }
        spi_init();

        // Initialize components
        this->controller_ = std::make_shared<mid_level_controller::Controller>();
        this->localization_ = std::make_shared<mid_level_controller::Localization>();

        RCLCPP_INFO(this->get_logger(), "Initialised vehicle %i.", this->vehicle_id_);

        // Create publisher
        this->vehicle_state_publisher_ = this->create_publisher<lab_msgs::msg::VehicleState>(
                "vehicle_" + std::to_string(this->vehicle_id_) + "/vehicleState", 10);

        this->odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
                "vehicle_" + std::to_string(this->vehicle_id_) + "/odometry", 10);

        // Create subscriber
        this->vehicle_observation_subscriber_ = this->create_subscription<lab_msgs::msg::VehicleObservation>(
                "vehicle_" + std::to_string(this->vehicle_id_) + "/vehicleObservation", 10,
                std::bind(&MidLevelControllerNode::onVehicleObservation, this, std::placeholders::_1));
        this->vehicle_direct_command_subscriber_ = this->create_subscription<lab_msgs::msg::VehicleCommandDirect>(
                "vehicle_" + std::to_string(this->vehicle_id_) + "/vehicleCommandDirect", 10,
                std::bind(&MidLevelControllerNode::onVehicleCommandDirect, this, std::placeholders::_1));
        this->vehicle_speed_curvature_command_subscriber_ =
                this->create_subscription<lab_msgs::msg::VehicleCommandSpeedCurvature>(
                        "vehicle_" + std::to_string(this->vehicle_id_) + "/vehicleCommandSpeedCurvature", 10,
                        std::bind(&MidLevelControllerNode::onVehicleCommandSpeedCurvature, this,
                                  std::placeholders::_1));
        this->vehicle_trajectory_command_subscriber_ = this->create_subscription<lab_msgs::msg::VehicleCommandTrajectory>(
                "vehicle_" + std::to_string(this->vehicle_id_) + "/vehicleCommandTrajectory", 10,
                std::bind(&MidLevelControllerNode::onVehicleCommandTrajectory, this, std::placeholders::_1));
        this->vehicle_path_tracking_command_subscriber_ =
                this->create_subscription<lab_msgs::msg::VehicleCommandPathTracking>(
                        "vehicle_" + std::to_string(this->vehicle_id_) + "/vehicleCommandPathTracking", 10,
                        std::bind(&MidLevelControllerNode::onVehicleCommandPathTracking, this,
                                  std::placeholders::_1));

        // Create timer to update the vehicle
        this->timer_ = this->create_wall_timer(20ms, [this]() { this->onTimer(); });

        // Create tf publisher
        this->tf_odom_base_link_publisher_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MidLevelControllerNode>();

    for (int i = 0; i < argc; i++) {
        RCLCPP_INFO(node->get_logger(), "Parameter: %s", argv[i]);
    }

    RCLCPP_INFO(node->get_logger(), "Starting mid level controller.");
    rclcpp::spin(node);

    RCLCPP_INFO(node->get_logger(), "Stopping mid level controller.");
    rclcpp::shutdown();

    return 0;
}