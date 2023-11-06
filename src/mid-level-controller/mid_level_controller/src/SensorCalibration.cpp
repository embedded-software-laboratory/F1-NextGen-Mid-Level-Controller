#include "mid_level_controller/SensorCalibration.hpp"

lab_msgs::msg::VehicleState SensorCalibration::convert(spi_miso_data_t spi_miso_data) {
    const double odometer_meter_per_step = 0.0031225604996;
    const double speed_meter_per_second_per_step = odometer_meter_per_step * 0.2384185791;
    const double imu_yaw_radian_per_step = (0.00109083078249645598 / 50 * 32);
    const double imu_yaw_rate_radian_per_second_per_step = 0.00109083078249645598;
    const double battery_volt_per_step = 0.0116669;
    const double imu_meter_per_second_per_second_per_step = 0.01;

    const double motor_ampere_per_step = 0.01;


    lab_msgs::msg::VehicleState vehicleState;
    vehicleState.odometer_distance = spi_miso_data.odometer_steps * odometer_meter_per_step;
    vehicleState.pose.x = 0; // Not measured, TBD by the localization
    vehicleState.pose.y = 0;
    vehicleState.pose.yaw = 0;
    vehicleState.imu_acceleration_forward =
            spi_miso_data.imu_acceleration_forward * imu_meter_per_second_per_second_per_step;
    vehicleState.imu_acceleration_left = spi_miso_data.imu_acceleration_left * imu_meter_per_second_per_second_per_step;
    vehicleState.imu_acceleration_up = spi_miso_data.imu_acceleration_up * imu_meter_per_second_per_second_per_step;
    vehicleState.imu_yaw = spi_miso_data.imu_yaw * imu_yaw_radian_per_step;
    vehicleState.imu_yaw_rate = spi_miso_data.imu_yaw_rate * imu_yaw_rate_radian_per_second_per_step;
    vehicleState.speed = spi_miso_data.speed * speed_meter_per_second_per_step;
    vehicleState.battery_voltage = spi_miso_data.battery_voltage * battery_volt_per_step;
    vehicleState.motor_current = spi_miso_data.motor_current * motor_ampere_per_step;

    return vehicleState;
}